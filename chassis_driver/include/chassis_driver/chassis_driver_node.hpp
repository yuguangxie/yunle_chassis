#pragma once

#include "chassis_driver/can_types.hpp"
#include "chassis_driver/frame_router.hpp"
#include "chassis_driver/udp_channel.hpp"

#include "chassis_interfaces/msg/bms_realtime_status.hpp"
#include "chassis_interfaces/msg/bms_status.hpp"
#include "chassis_interfaces/msg/can_frame.hpp"
#include "chassis_interfaces/msg/ccu_status.hpp"
#include "chassis_interfaces/msg/sas_angle_feedback.hpp"
#include "chassis_interfaces/msg/scu_chassis_command.hpp"
#include "chassis_interfaces/msg/scu_control_command.hpp"
#include "chassis_interfaces/msg/scu_target_speed_feedback.hpp"
#include "chassis_interfaces/msg/scu_torque_command.hpp"
#include "chassis_interfaces/msg/vcu_warning_level.hpp"
#include "chassis_interfaces/msg/wheel_speed_feedback.hpp"

#include "std_msgs/msg/string.hpp"

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>

namespace chassis_driver
{

class ControlCommandBridge;

class ChassisDriverNode : public rclcpp::Node
{
public:
  ChassisDriverNode();
  ~ChassisDriverNode() override;

  void sendControlFrame(const CanFrame & frame, const std::string & message_name);
  void publishDecoded(const CanFrame & frame);
  void publishRawRx(const CanFrame & frame);
  void publishRawTx(const CanFrame & frame);
  void publishUnknownFrame(const CanFrame & frame);

  uint8_t resolveChannel(const std::string & message_name, bool tx) const;

private:
  void loadParameters();
  void initializeChannels();
  void startThreads();
  void stopThreads();
  void rxLoop(uint8_t channel_id);

  std::string topic_prefix_;
  bool publish_raw_can_{true};
  bool publish_unknown_frames_{true};
  bool enable_debug_topics_{true};
  int default_qos_depth_{10};
  std::unordered_map<std::string, std::string> control_channel_map_;
  std::unordered_map<std::string, std::string> feedback_channel_map_;

  std::string local_ip_;
  int can1_local_port_{8234};
  int can2_local_port_{8235};
  std::string can1_remote_ip_;
  std::string can2_remote_ip_;
  int remote_port_{1234};
  int udp_buffer_size_{2048};
  int socket_timeout_ms_{200};

  UdpChannel can1_;
  UdpChannel can2_;

  std::atomic<bool> running_{false};
  std::thread can1_rx_thread_;
  std::thread can2_rx_thread_;

  std::mutex tx_mutex_;

  rclcpp::Publisher<chassis_interfaces::msg::CanFrame>::SharedPtr raw_rx_pub_;
  rclcpp::Publisher<chassis_interfaces::msg::CanFrame>::SharedPtr raw_tx_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr unknown_frame_pub_;

  rclcpp::Publisher<chassis_interfaces::msg::BmsStatus>::SharedPtr bms_status_pub_;
  rclcpp::Publisher<chassis_interfaces::msg::BmsRealtimeStatus>::SharedPtr bms_realtime_pub_;
  rclcpp::Publisher<chassis_interfaces::msg::VcuWarningLevel>::SharedPtr vcu_warning_pub_;
  rclcpp::Publisher<chassis_interfaces::msg::WheelSpeedFeedback>::SharedPtr wheel_speed_pub_;
  rclcpp::Publisher<chassis_interfaces::msg::CcuStatus>::SharedPtr ccu_status_pub_;
  rclcpp::Publisher<chassis_interfaces::msg::SasAngleFeedback>::SharedPtr sas_angle_pub_;
  rclcpp::Publisher<chassis_interfaces::msg::ScuTargetSpeedFeedback>::SharedPtr target_speed_pub_;

  std::shared_ptr<FrameRouter> frame_router_;
  std::shared_ptr<ControlCommandBridge> control_bridge_;

  rclcpp::Subscription<chassis_interfaces::msg::ScuControlCommand>::SharedPtr scu_control_sub_;
  rclcpp::Subscription<chassis_interfaces::msg::ScuChassisCommand>::SharedPtr scu_chassis_sub_;
  rclcpp::Subscription<chassis_interfaces::msg::ScuTorqueCommand>::SharedPtr scu_torque_sub_;

  friend class FrameRouter;
  friend class ControlCommandBridge;
};

}  // namespace chassis_driver
