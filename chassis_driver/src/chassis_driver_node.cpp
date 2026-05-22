#include "chassis_driver/chassis_driver_node.hpp"

#include "chassis_driver/can_ethernet_codec.hpp"
#include "chassis_driver/control_command_bridge.hpp"
#include "chassis_driver/dbc_protocol.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace chassis_driver
{

namespace
{
constexpr double kSteeringAngleCodeAtMaxAngle = 120.0;

/** Parse k:v parameter entries into hash map. */
/** 将 k:v 形式的参数条目解析为哈希表。 */
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

/** Convert internal CAN frame to ROS message with timestamp. */
/** 将内部 CAN 帧转换为带时间戳的 ROS 消息。 */
chassis_interfaces::CanFrame toRosFrame(const CanFrame & frame, const ros::Time & stamp)
{
  chassis_interfaces::CanFrame msg;
  msg.stamp = stamp;
  msg.can_id = frame.can_id;
  msg.is_extended = frame.is_extended;
  msg.is_remote = frame.is_remote;
  msg.dlc = frame.dlc;
  for (size_t i = 0; i < frame.data.size(); ++i) {
    msg.data[i] = frame.data[i];
  }
  msg.channel = frame.channel;
  return msg;
}

/** Format a CAN frame as a compact hexadecimal string for operator logs. */
/** 将 CAN 帧格式化为紧凑的十六进制字符串，供操作日志使用。 */
std::string formatCanFrameHex(const CanFrame & frame, uint8_t channel, const std::string & message_name)
{
  std::ostringstream ss;
  ss << message_name << " can" << static_cast<int>(channel)
     << " id=0x" << std::uppercase << std::hex << frame.can_id
     << " dlc=" << std::dec << static_cast<int>(frame.dlc)
     << " data=[";
  const auto dlc = std::min<uint8_t>(frame.dlc, static_cast<uint8_t>(8U));
  for (uint8_t i = 0; i < dlc; ++i) {
    if (i > 0) {
      ss << ' ';
    }
    ss << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
       << static_cast<int>(frame.data[i]);
  }
  ss << "]";
  return ss.str();
}

/** Convert protocol steering-angle code to physical wheel angle in degrees. */
/** 将协议转角编码值换算为实际车轮转角，单位为度。 */
double steeringAngleCodeToDeg(double code, double max_steering_angle_deg)
{
  if (!std::isfinite(code)) {
    return 0.0;
  }
  return code / kSteeringAngleCodeAtMaxAngle * max_steering_angle_deg;
}
}  // namespace

/** Build node, setup publishers/subscribers, network channels and RX threads. */
/** 构建节点，配置发布者/订阅者、网络通道和接收线程。 */
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
  if (isPublishTopicEnabled("debug_unknown_frames")) {
    unknown_frame_pub_ = nh_.advertise<std_msgs::String>(makeTopicName("debug/unknown_frames"), default_qos_depth_);
  }

  if (isPublishTopicEnabled("feedback_bms_status")) {
    bms_status_pub_ = nh_.advertise<chassis_interfaces::BmsStatus>(makeTopicName("feedback/bms_status"), default_qos_depth_);
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
    sas_angle_pub_ = nh_.advertise<chassis_interfaces::SasAngleFeedback>(
      makeTopicName("feedback/sas_angle"), default_qos_depth_);
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

/** Ensure threads are stopped before destruction. */
/** 确保析构前停止线程。 */
ChassisDriverNode::~ChassisDriverNode()
{
  stopThreads();
}

/** Declare/read ROS parameters and build message-channel lookup tables. */
/** 声明/读取 ROS 参数，并构建报文通道查找表。 */
void ChassisDriverNode::loadParameters()
{
  pnh_.param<std::string>("topic_prefix", topic_prefix_, "/yunle_chassis");
  if (topic_prefix_.empty()) {
    topic_prefix_ = "/yunle_chassis";
  }
  if (topic_prefix_.front() != '/') {
    topic_prefix_ = "/" + topic_prefix_;
  }
  while (topic_prefix_.size() > 1 && topic_prefix_.back() == '/') {
    topic_prefix_.pop_back();
  }

  pnh_.param<bool>("publish_raw_can", publish_raw_can_, true);
  pnh_.param<bool>("publish_unknown_frames", publish_unknown_frames_, true);
  pnh_.param<bool>("enable_debug_topics", enable_debug_topics_, true);
  pnh_.param<bool>("log_control_can_frames", log_control_can_frames_, false);
  pnh_.param<int>("default_qos_depth", default_qos_depth_, 10);

  std::vector<std::string> enabled_publish_topics;
  std::vector<std::string> enabled_subscribe_topics;
  pnh_.getParam("enabled_publish_topics", enabled_publish_topics);
  pnh_.getParam("enabled_subscribe_topics", enabled_subscribe_topics);
  if (enabled_publish_topics.empty()) {
    enabled_publish_topics = {"all"};
  }
  if (enabled_subscribe_topics.empty()) {
    enabled_subscribe_topics = {"all"};
  }
  enabled_publish_topics_ = std::set<std::string>(enabled_publish_topics.begin(), enabled_publish_topics.end());
  enabled_subscribe_topics_ = std::set<std::string>(enabled_subscribe_topics.begin(), enabled_subscribe_topics.end());

  std::vector<std::string> map_entries;
  std::vector<std::string> control_map_entries;
  pnh_.getParam("message_channel_map", map_entries);
  pnh_.getParam("control_message_channel_map", control_map_entries);
  feedback_channel_map_ = parseMap(map_entries);
  control_channel_map_ = parseMap(control_map_entries);

  pnh_.param<std::string>("local_ip", local_ip_, "192.168.1.102");
  pnh_.param<int>("can1_local_port", can1_local_port_, 8234);
  pnh_.param<int>("can2_local_port", can2_local_port_, 8235);
  pnh_.param<std::string>("can1_remote_ip", can1_remote_ip_, "192.168.1.98");
  pnh_.param<std::string>("can2_remote_ip", can2_remote_ip_, "192.168.1.99");
  pnh_.param<int>("can1_remote_port", can1_remote_port_, 1234);
  pnh_.param<int>("can2_remote_port", can2_remote_port_, 1234);
  pnh_.param<int>("udp_buffer_size", udp_buffer_size_, 2048);
  pnh_.param<int>("socket_timeout_ms", socket_timeout_ms_, 200);
  pnh_.param<double>("scu_control_max_steering_angle_deg", scu_control_max_steering_angle_deg_, 27.0);
  pnh_.param<double>("scu_control_max_target_speed_kmh", scu_control_max_target_speed_kmh_, 15.0);

  if (!std::isfinite(scu_control_max_steering_angle_deg_) || scu_control_max_steering_angle_deg_ <= 0.0) {
    ROS_WARN(
      "Invalid scu_control_max_steering_angle_deg %.3f, fallback to 27.0", scu_control_max_steering_angle_deg_);
    scu_control_max_steering_angle_deg_ = 27.0;
  }
  if (!std::isfinite(scu_control_max_target_speed_kmh_) || scu_control_max_target_speed_kmh_ <= 0.0) {
    ROS_WARN(
      "Invalid scu_control_max_target_speed_kmh %.3f, fallback to 15.0", scu_control_max_target_speed_kmh_);
    scu_control_max_target_speed_kmh_ = 15.0;
  }
  for (const auto & required : {"SCU_Control_Command", "SCU_Chassis_Command", "SCU_Torque_Command",
      "VCU_Debug_Enable", "VCU_Drive_Debug"}) {
    if (control_channel_map_.find(required) == control_channel_map_.end()) {
      ROS_FATAL("Missing required TX mapping: %s", required);
      throw std::runtime_error("missing tx mapping");
    }
  }
}

/** Open CAN1/CAN2 UDP channels using configured addresses and ports. */
/** 使用配置的地址和端口打开 CAN1/CAN2 UDP 通道。 */
void ChassisDriverNode::initializeChannels()
{
  if (!can1_.open(local_ip_, static_cast<uint16_t>(can1_local_port_), can1_remote_ip_,
      static_cast<uint16_t>(can1_remote_port_), socket_timeout_ms_, udp_buffer_size_)) {
    ROS_FATAL("Failed to initialize CAN1 UDP channel");
    throw std::runtime_error("can1 open failed");
  }
  if (!can2_.open(local_ip_, static_cast<uint16_t>(can2_local_port_), can2_remote_ip_,
      static_cast<uint16_t>(can2_remote_port_), socket_timeout_ms_, udp_buffer_size_)) {
    ROS_FATAL("Failed to initialize CAN2 UDP channel");
    throw std::runtime_error("can2 open failed");
  }
}

/** Start background RX loops for both channels. */
/** 启动两个通道的后台接收循环。 */
void ChassisDriverNode::startThreads()
{
  running_.store(true);
  can1_rx_thread_ = std::thread([this]() { rxLoop(1); });
  can2_rx_thread_ = std::thread([this]() { rxLoop(2); });
}

/** Stop RX loops and join all worker threads. */
/** 停止接收循环并 join 所有工作线程。 */
void ChassisDriverNode::stopThreads()
{
  running_.store(false);
  can1_.close();
  can2_.close();
  if (can1_rx_thread_.joinable()) can1_rx_thread_.join();
  if (can2_rx_thread_.joinable()) can2_rx_thread_.join();
}

/** Channel receive loop: fetch UDP payload, decode frames, and route each frame. */
/** 通道接收循环：获取 UDP payload、解码帧并路由每一帧。 */
void ChassisDriverNode::rxLoop(uint8_t channel_id)
{
  auto & channel = channel_id == 1 ? can1_ : can2_;
  while (running_.load()) {
    std::vector<uint8_t> payload;
    if (!channel.receive(payload)) {
      continue;
    }
    bool trailing = false;
    size_t trailing_count = 0;
    auto frames = CanEthernetCodec::decodePayload(payload, channel_id, trailing, trailing_count);
    if (trailing) {
      ROS_WARN("UDP payload has %zu trailing bytes (not multiple of 13), dropped", trailing_count);
    }
    for (const auto & frame : frames) {
      publishRawRx(frame);
      frame_router_->routeFrame(frame);
    }
  }
}

/** Encode and send control frame over resolved channel and publish TX raw topic. */
/** 通过解析到的通道编码并发送控制帧，同时发布 TX 原始话题。 */
void ChassisDriverNode::sendControlFrame(const CanFrame & frame, const std::string & message_name)
{
  const auto packed = CanEthernetCodec::encodeFrame(frame);
  std::vector<uint8_t> payload(packed.begin(), packed.end());
  std::scoped_lock<std::mutex> lock(tx_mutex_);
  const uint8_t channel = resolveChannel(message_name, true);
  const bool ok = (channel == 1) ? can1_.send(payload) : can2_.send(payload);
  if (!ok) {
    ROS_ERROR("Failed to send frame %s on can%u", message_name.c_str(), channel);
    return;
  }
  CanFrame tx = frame;
  tx.channel = channel;
  if (log_control_can_frames_) {
    const auto formatted = formatCanFrameHex(tx, channel, message_name);
    ROS_INFO("TX control CAN: %s", formatted.c_str());
  }
  publishRawTx(tx);
}

/** Decode known CAN IDs and publish typed feedback topics. */
/** 解码已知 CAN ID，并发布类型化反馈话题。 */
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
    case 119U: {
      chassis_interfaces::VcuWarningLevel msg;
      msg.stamp = stamp;
      msg.bms_soc_warning = static_cast<uint8_t>(get("BMS_SOC_Warning"));
      msg.mcu_disconnect_warning = static_cast<uint8_t>(get("MCU_Disconnect_Warning"));
      msg.mcu_motor_warning = static_cast<uint8_t>(get("MCU_Motor_Warning"));
      msg.mcu_speed_warning = static_cast<uint8_t>(get("MCU_Speed_Warning"));
      msg.steering_disconnect_warning = static_cast<uint8_t>(get("Steering_Disconnect_Warning"));
      msg.steering_lock_warning = static_cast<uint8_t>(get("Steering_Lock_Warning"));
      msg.steering_uncontrollable_warning = static_cast<uint8_t>(get("Steering_Uncontrollable_Warning"));
      msg.steering_error_warning = static_cast<uint8_t>(get("Steering_Error_Warning"));
      msg.brake_error_warning = static_cast<uint8_t>(get("Brake_Error_Warning"));
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
      const double ccu_steering_code = get("CCU_Steering_Wheel_Angle");
      const double ccu_steering_angle = steeringAngleCodeToDeg(
        ccu_steering_code, scu_control_max_steering_angle_deg_);
      msg.ccu_steering_wheel_angle = static_cast<float>(
        msg.steering_wheel_direction ? ccu_steering_angle : -ccu_steering_angle);
      msg.ccu_vehicle_speed = static_cast<float>(get("CCU_Vehicle_Speed"));
      msg.ccu_drive_mode = static_cast<uint8_t>(get("CCU_Drive_Mode"));
      msg.remote_brake_request_status = get("Remote_Brake_Request_Status") > 0.5;
      msg.emergency_brake_request_status = get("Emergency_Brake_Request_Status") > 0.5;
      msg.scu_brake_signal_status = get("SCU_Brake_Signal_Status") > 0.5;
      msg.touch_brake_request_status = get("Touch_Brake_Request_Status") > 0.5;
      msg.handle_brake_request_status = get("Handle_Brake_Request_Status") > 0.5;
      msg.handle_mode_flag_status = get("Handle_Mode_Flag_Status") > 0.5;
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
      msg.sas_front_angle = static_cast<float>(steeringAngleCodeToDeg(
        get("SAS_Front_Angle"), scu_control_max_steering_angle_deg_));
      msg.sas_rear_angle = static_cast<float>(steeringAngleCodeToDeg(
        get("SAS_Rear_Angle"), scu_control_max_steering_angle_deg_));
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

/** Publish raw RX frame when enabled. */
/** 启用时发布原始 RX 帧。 */
void ChassisDriverNode::publishRawRx(const CanFrame & frame)
{
  if (!publish_raw_can_) return;
  if (raw_rx_pub_) { raw_rx_pub_.publish(toRosFrame(frame, ros::Time::now())); }
}

/** Publish raw TX frame when enabled. */
/** 启用时发布原始 TX 帧。 */
void ChassisDriverNode::publishRawTx(const CanFrame & frame)
{
  if (!publish_raw_can_) return;
  if (raw_tx_pub_) { raw_tx_pub_.publish(toRosFrame(frame, ros::Time::now())); }
}

/** Publish unknown-frame debug message when enabled. */
/** 启用时发布未知帧调试消息。 */
void ChassisDriverNode::publishUnknownFrame(const CanFrame & frame)
{
  if (!publish_unknown_frames_ || !enable_debug_topics_) return;
  std_msgs::String msg;
  std::stringstream ss;
  ss << "Unknown frame: id=" << frame.can_id << " ext=" << frame.is_extended << " ch=" << static_cast<int>(frame.channel);
  msg.data = ss.str();
  if (unknown_frame_pub_) { unknown_frame_pub_.publish(msg); }
}


/** Build topic name by joining configured prefix and suffix. */
/** 拼接配置前缀和后缀生成话题名。 */
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

/** Check whether publisher topic key is enabled by parameter list. */
/** 检查发布话题 key 是否被参数列表启用。 */
bool ChassisDriverNode::isPublishTopicEnabled(const std::string & topic_key) const
{
  return enabled_publish_topics_.count("all") > 0 || enabled_publish_topics_.count(topic_key) > 0;
}

/** Check whether subscriber topic key is enabled by parameter list. */
/** 检查订阅话题 key 是否被参数列表启用。 */
bool ChassisDriverNode::isSubscribeTopicEnabled(const std::string & topic_key) const
{
  return enabled_subscribe_topics_.count("all") > 0 || enabled_subscribe_topics_.count(topic_key) > 0;
}

/** Resolve channel ID from configured map with fallback to channel 1. */
/** 从配置映射中解析通道 ID。 */
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
