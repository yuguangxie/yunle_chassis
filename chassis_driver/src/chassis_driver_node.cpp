#include "chassis_driver/chassis_driver_node.hpp"

#include "chassis_driver/can_ethernet_codec.hpp"
#include "chassis_driver/control_command_bridge.hpp"
#include "chassis_driver/dbc_protocol.hpp"
#include "chassis_driver/frame_router.hpp"

#include <algorithm>
#include <chrono>
#include <sstream>

namespace chassis_driver
{

namespace
{
std::unordered_map<std::string, std::string> parseMap(const std::vector<std::string> & entries)
{
  std::unordered_map<std::string, std::string> out;
  for (const auto & e : entries) {
    const auto pos = e.find(':');
    if (pos == std::string::npos) {
      continue;
    }
    out[e.substr(0, pos)] = e.substr(pos + 1);
  }
  return out;
}

chassis_interfaces::CanFrame toRosFrame(const CanFrame & frame, const ros::Time & stamp)
{
  chassis_interfaces::CanFrame msg;
  msg.stamp = stamp;
  msg.can_id = frame.can_id;
  msg.is_extended = frame.is_extended;
  msg.is_remote = frame.is_remote;
  msg.dlc = frame.dlc;
  msg.data = frame.data;
  msg.channel = frame.channel;
  return msg;
}
}  // namespace

ChassisDriverNode::ChassisDriverNode()
: pnh_("~")
{
  loadParameters();

  if (isPublishTopicEnabled("can_rx_raw")) {
    raw_rx_pub_ = nh_.advertise<chassis_interfaces::CanFrame>(makeTopicName("can_rx/raw"), default_qos_depth_);
  }
  if (isPublishTopicEnabled("can_tx_raw")) {
    raw_tx_pub_ = nh_.advertise<chassis_interfaces::CanFrame>(makeTopicName("can_tx/raw"), default_qos_depth_);
  }
  if (isPublishTopicEnabled("debug_status")) {
    debug_status_pub_ = nh_.advertise<std_msgs::String>(makeTopicName("debug/status"), default_qos_depth_);
  }
  if (isPublishTopicEnabled("debug_unknown_frames")) {
    unknown_frame_pub_ = nh_.advertise<std_msgs::String>(makeTopicName("debug/unknown_frames"), default_qos_depth_);
  }

  if (isPublishTopicEnabled("feedback_bms_status")) {
    bms_status_pub_ = nh_.advertise<chassis_interfaces::BmsStatus>(makeTopicName("feedback/bms_status"), default_qos_depth_);
  }
  if (isPublishTopicEnabled("feedback_bms_realtime_status")) {
    bms_realtime_pub_ = nh_.advertise<chassis_interfaces::BmsRealtimeStatus>(
      makeTopicName("feedback/bms_realtime_status"), default_qos_depth_);
  }
  if (isPublishTopicEnabled("feedback_vcu_warning_level")) {
    vcu_warning_pub_ = nh_.advertise<chassis_interfaces::VcuWarningLevel>(
      makeTopicName("feedback/vcu_warning_level"), default_qos_depth_);
  }
  if (isPublishTopicEnabled("feedback_wheel_speed")) {
    wheel_speed_pub_ = nh_.advertise<chassis_interfaces::WheelSpeedFeedback>(
      makeTopicName("feedback/wheel_speed"), default_qos_depth_);
  }
  if (isPublishTopicEnabled("feedback_ccu_status")) {
    ccu_status_pub_ = nh_.advertise<chassis_interfaces::CcuStatus>(makeTopicName("feedback/ccu_status"), default_qos_depth_);
  }
  if (isPublishTopicEnabled("feedback_sas_angle")) {
    sas_angle_pub_ = nh_.advertise<chassis_interfaces::SasAngleFeedback>(makeTopicName("feedback/sas_angle"), default_qos_depth_);
  }
  if (isPublishTopicEnabled("feedback_target_speed_feedback")) {
    target_speed_pub_ = nh_.advertise<chassis_interfaces::ScuTargetSpeedFeedback>(
      makeTopicName("feedback/target_speed_feedback"), default_qos_depth_);
  }

  initializeChannels();

  frame_router_ = std::make_shared<FrameRouter>(*this);
  control_bridge_ = std::make_shared<ControlCommandBridge>(*this);

  startThreads();
}

ChassisDriverNode::~ChassisDriverNode()
{
  stopThreads();
}

void ChassisDriverNode::loadParameters()
{
  pnh_.param<std::string>("topic_prefix", topic_prefix_, "/yunle_chassis");
  pnh_.param<bool>("publish_raw_can", publish_raw_can_, true);
  pnh_.param<bool>("publish_unknown_frames", publish_unknown_frames_, true);
  pnh_.param<bool>("enable_debug_topics", enable_debug_topics_, true);
  pnh_.param<int>("default_qos_depth", default_qos_depth_, 10);
  pnh_.param<std::string>("local_ip", local_ip_, "192.168.1.102");
  pnh_.param<int>("can1_local_port", can1_local_port_, 8234);
  pnh_.param<int>("can2_local_port", can2_local_port_, 8235);
  pnh_.param<std::string>("can1_remote_ip", can1_remote_ip_, "192.168.1.98");
  pnh_.param<std::string>("can2_remote_ip", can2_remote_ip_, "192.168.1.99");
  pnh_.param<int>("remote_port", remote_port_, 1234);
  pnh_.param<int>("udp_buffer_size", udp_buffer_size_, 2048);
  pnh_.param<int>("socket_timeout_ms", socket_timeout_ms_, 200);

  if (topic_prefix_.empty()) {
    topic_prefix_ = "/yunle_chassis";
  }
  if (topic_prefix_.front() != '/') {
    topic_prefix_ = "/" + topic_prefix_;
  }
  while (topic_prefix_.size() > 1 && topic_prefix_.back() == '/') {
    topic_prefix_.pop_back();
  }

  std::vector<std::string> enabled_publish_topics;
  std::vector<std::string> enabled_subscribe_topics;
  std::vector<std::string> map_entries;
  std::vector<std::string> control_map_entries;

  pnh_.getParam("enabled_publish_topics", enabled_publish_topics);
  pnh_.getParam("enabled_subscribe_topics", enabled_subscribe_topics);
  pnh_.getParam("message_channel_map", map_entries);
  pnh_.getParam("control_message_channel_map", control_map_entries);

  if (enabled_publish_topics.empty()) {
    enabled_publish_topics = {"all"};
  }
  if (enabled_subscribe_topics.empty()) {
    enabled_subscribe_topics = {"all"};
  }

  enabled_publish_topics_ = std::set<std::string>(enabled_publish_topics.begin(), enabled_publish_topics.end());
  enabled_subscribe_topics_ = std::set<std::string>(enabled_subscribe_topics.begin(), enabled_subscribe_topics.end());
  feedback_channel_map_ = parseMap(map_entries);
  control_channel_map_ = parseMap(control_map_entries);
}

void ChassisDriverNode::initializeChannels()
{
  if (!can1_.open(local_ip_, static_cast<uint16_t>(can1_local_port_), can1_remote_ip_,
    static_cast<uint16_t>(remote_port_), udp_buffer_size_, socket_timeout_ms_)) {
    throw std::runtime_error("failed to initialize can1 udp channel");
  }
  if (!can2_.open(local_ip_, static_cast<uint16_t>(can2_local_port_), can2_remote_ip_,
    static_cast<uint16_t>(remote_port_), udp_buffer_size_, socket_timeout_ms_)) {
    throw std::runtime_error("failed to initialize can2 udp channel");
  }
}

void ChassisDriverNode::startThreads()
{
  running_.store(true);
  can1_rx_thread_ = std::thread(&ChassisDriverNode::rxLoop, this, 1U);
  can2_rx_thread_ = std::thread(&ChassisDriverNode::rxLoop, this, 2U);
}

void ChassisDriverNode::stopThreads()
{
  running_.store(false);
  if (can1_rx_thread_.joinable()) {
    can1_rx_thread_.join();
  }
  if (can2_rx_thread_.joinable()) {
    can2_rx_thread_.join();
  }
}

void ChassisDriverNode::rxLoop(uint8_t channel_id)
{
  UdpChannel & ch = (channel_id == 1U) ? can1_ : can2_;

  while (running_.load()) {
    std::vector<uint8_t> bytes;
    if (!ch.receive(bytes)) {
      continue;
    }

    auto maybe_frame = CanEthernetCodec::decode(bytes);
    if (!maybe_frame.has_value()) {
      continue;
    }

    CanFrame frame = maybe_frame.value();
    frame.channel = channel_id;

    publishRawRx(frame);
    if (frame_router_) {
      frame_router_->routeFrame(frame);
    }
  }
}

void ChassisDriverNode::sendControlFrame(const CanFrame & frame, const std::string & message_name)
{
  const uint8_t channel = resolveChannel(message_name, true);
  UdpChannel & ch = (channel == 1U) ? can1_ : can2_;
  const auto bytes = CanEthernetCodec::encode(frame);

  std::lock_guard<std::mutex> lock(tx_mutex_);
  if (!ch.send(bytes)) {
    ROS_ERROR("Failed to send frame %s on can%u", message_name.c_str(), channel);
    return;
  }
  CanFrame tx = frame;
  tx.channel = channel;
  publishRawTx(tx);
}

void ChassisDriverNode::publishDecoded(const CanFrame & frame)
{
  const ros::Time stamp = ros::Time::now();
  auto get = [&](const std::string & name) { return DbcProtocol::decodeSignal(frame, name).value_or(0.0); };

  switch (frame.can_id) {
    case 256U: {
      chassis_interfaces::BmsStatus msg;
      msg.stamp = stamp;
      msg.bms_voltage = static_cast<float>(get("BMS_Voltage"));
      msg.bms_current = static_cast<float>(get("BMS_Current"));
      msg.bms_soc = static_cast<float>(get("BMS_SOC"));
      if (bms_status_pub_) { bms_status_pub_.publish(msg); }
      break;
    }
    case 2542813185U: {
      chassis_interfaces::BmsRealtimeStatus msg;
      msg.stamp = stamp;
      msg.bms_total_battery_voltage = static_cast<float>(get("BMS_Total_Battery_Voltage"));
      msg.bms_pack_voltage = static_cast<float>(get("BMS_Pack_Voltage"));
      msg.bms_pack_current = static_cast<float>(get("BMS_Pack_Current"));
      msg.bms_realtime_soc = static_cast<float>(get("BMS_Realtime_SOC"));
      if (bms_realtime_pub_) { bms_realtime_pub_.publish(msg); }
      break;
    }
    case 119U: {
      chassis_interfaces::VcuWarningLevel msg;
      msg.stamp = stamp;
      msg.bms_charge_current_warning = static_cast<uint8_t>(get("BMS_Charge_Current_Warning"));
      msg.bms_discharge_current_warning = static_cast<uint8_t>(get("BMS_Discharge_Current_Warning"));
      msg.bms_soc_warning = static_cast<uint8_t>(get("BMS_SOC_Warning"));
      msg.bms_temperature_warning = static_cast<uint8_t>(get("BMS_Temperature_Warning"));
      msg.mcu_current_warning = static_cast<uint8_t>(get("MCU_Current_Warning"));
      msg.mcu_disconnect_warning = static_cast<uint8_t>(get("MCU_Disconnect_Warning"));
      msg.mcu_motor_warning = static_cast<uint8_t>(get("MCU_Motor_Warning"));
      msg.mcu_speed_warning = static_cast<uint8_t>(get("MCU_Speed_Warning"));
      msg.mcu_temperature_warning = static_cast<uint8_t>(get("MCU_Temperature_Warning"));
      msg.mcu_voltage_warning = static_cast<uint8_t>(get("MCU_Voltage_Warning"));
      msg.steering_disconnect_warning = static_cast<uint8_t>(get("Steering_Disconnect_Warning"));
      msg.steering_lock_warning = static_cast<uint8_t>(get("Steering_Lock_Warning"));
      msg.steering_uncontrollable_warning = static_cast<uint8_t>(get("Steering_Uncontrollable_Warning"));
      if (vcu_warning_pub_) { vcu_warning_pub_.publish(msg); }
      break;
    }
    case 360U: {
      chassis_interfaces::WheelSpeedFeedback msg;
      msg.stamp = stamp;
      msg.wheel_speed_front_left_rpm = static_cast<float>(get("Wheel_Speed_Front_Left_RPM"));
      msg.wheel_speed_front_right_rpm = static_cast<float>(get("Wheel_Speed_Front_Right_RPM"));
      msg.wheel_speed_rear_left_rpm = static_cast<float>(get("Wheel_Speed_Rear_Left_RPM"));
      msg.wheel_speed_rear_right_rpm = static_cast<float>(get("Wheel_Speed_Rear_Right_RPM"));
      if (wheel_speed_pub_) { wheel_speed_pub_.publish(msg); }
      break;
    }
    case 81U: {
      chassis_interfaces::CcuStatus msg;
      msg.stamp = stamp;
      msg.ccu_shift_level_status = static_cast<uint8_t>(get("CCU_Shift_Level_Status"));
      msg.ccu_parking_status = get("CCU_Parking_Status") > 0.5;
      msg.ccu_ignition_status = static_cast<uint8_t>(get("CCU_Ignition_Status"));
      msg.ccu_drive_mode_shift_button = get("CCU_Drive_Mode_Shift_Button") > 0.5;
      msg.steering_wheel_direction = get("Steering_Wheel_Direction") > 0.5;
      msg.ccu_steering_wheel_angle = static_cast<float>(get("CCU_Steering_Wheel_Angle"));
      msg.ccu_vehicle_speed = static_cast<float>(get("CCU_Vehicle_Speed"));
      msg.ccu_drive_mode = static_cast<uint8_t>(get("CCU_Drive_Mode"));
      msg.remote_brake_request_status = get("Remote_Brake_Request_Status") > 0.5;
      msg.emergency_brake_request_status = get("Emergency_Brake_Request_Status") > 0.5;
      msg.scu_brake_signal_status = get("SCU_Brake_Signal_Status") > 0.5;
      msg.left_turn_light_status = get("Left_Turn_Light_Status") > 0.5;
      msg.right_turn_light_status = get("Right_Turn_Light_Status") > 0.5;
      msg.position_light_status = get("Position_Light_Status") > 0.5;
      msg.low_beam_status = get("Low_Beam_Status") > 0.5;
      if (ccu_status_pub_) { ccu_status_pub_.publish(msg); }
      break;
    }
    case 225U: {
      chassis_interfaces::SasAngleFeedback msg;
      msg.stamp = stamp;
      msg.sas_front_angle = static_cast<float>(get("SAS_Front_Angle"));
      msg.sas_rear_angle = static_cast<float>(get("SAS_Rear_Angle"));
      if (sas_angle_pub_) { sas_angle_pub_.publish(msg); }
      break;
    }
    case 2033U: {
      chassis_interfaces::ScuTargetSpeedFeedback msg;
      msg.stamp = stamp;
      msg.hardware_target_speed = static_cast<float>(get("Hardware_Target_Speed"));
      msg.scu_target_speed_feedback = static_cast<float>(get("SCU_Target_Speed_Feedback"));
      msg.vehicle_target_speed = static_cast<float>(get("Vehicle_Target_Speed"));
      msg.vehicle_target_speed_rpm = static_cast<float>(get("Vehicle_Target_Speed_RPM"));
      if (target_speed_pub_) { target_speed_pub_.publish(msg); }
      break;
    }
    default:
      publishUnknownFrame(frame);
      break;
  }
}

void ChassisDriverNode::publishRawRx(const CanFrame & frame)
{
  if (!publish_raw_can_) return;
  if (raw_rx_pub_) { raw_rx_pub_.publish(toRosFrame(frame, ros::Time::now())); }
}

void ChassisDriverNode::publishRawTx(const CanFrame & frame)
{
  if (!publish_raw_can_) return;
  if (raw_tx_pub_) { raw_tx_pub_.publish(toRosFrame(frame, ros::Time::now())); }
}

void ChassisDriverNode::publishUnknownFrame(const CanFrame & frame)
{
  if (!publish_unknown_frames_ || !enable_debug_topics_) return;
  std_msgs::String msg;
  std::stringstream ss;
  ss << "Unknown frame: id=" << frame.can_id << " ext=" << frame.is_extended << " ch=" << static_cast<int>(frame.channel);
  msg.data = ss.str();
  if (unknown_frame_pub_) { unknown_frame_pub_.publish(msg); }
}

std::string ChassisDriverNode::makeTopicName(const std::string & suffix) const
{
  if (suffix.empty()) {
    return topic_prefix_;
  }
  if (suffix.front() == '/') {
    return topic_prefix_ + suffix;
  }
  return topic_prefix_ + "/" + suffix;
}

bool ChassisDriverNode::isPublishTopicEnabled(const std::string & topic_key) const
{
  return enabled_publish_topics_.count("all") > 0 || enabled_publish_topics_.count(topic_key) > 0;
}

bool ChassisDriverNode::isSubscribeTopicEnabled(const std::string & topic_key) const
{
  return enabled_subscribe_topics_.count("all") > 0 || enabled_subscribe_topics_.count(topic_key) > 0;
}

uint8_t ChassisDriverNode::resolveChannel(const std::string & message_name, bool tx) const
{
  const auto & map = tx ? control_channel_map_ : feedback_channel_map_;
  const auto it = map.find(message_name);
  if (it == map.end()) {
    throw std::runtime_error("missing channel mapping for " + message_name);
  }
  return (it->second == "can2") ? 2 : 1;
}

}  // namespace chassis_driver
