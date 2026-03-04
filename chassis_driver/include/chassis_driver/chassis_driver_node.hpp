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

/**
 * @brief ROS2 chassis driver node handling UDP-CAN bridging and topic I/O.
 */
class ChassisDriverNode : public rclcpp::Node
{
public:
  /** @brief Construct node, load parameters, initialize channels and worker threads. */
  ChassisDriverNode();

  /** @brief Stop worker threads and release resources. */
  ~ChassisDriverNode() override;

  /**
   * @brief Encode and send one control CAN frame to mapped channel.
   * @param frame CAN frame to transmit.
   * @param message_name DBC message name used for channel lookup.
   */
  void sendControlFrame(const CanFrame & frame, const std::string & message_name);

  /**
   * @brief Decode CAN frame by DBC and publish mapped ROS feedback message.
   * @param frame CAN frame received from chassis.
   */
  void publishDecoded(const CanFrame & frame);

  /** @brief Publish raw received CAN frame topic for diagnostics. */
  void publishRawRx(const CanFrame & frame);

  /** @brief Publish raw transmitted CAN frame topic for diagnostics. */
  void publishRawTx(const CanFrame & frame);

  /** @brief Publish unknown frame info when no DBC message mapping is found. */
  void publishUnknownFrame(const CanFrame & frame);

  /**
   * @brief Resolve configured channel ID for a message name.
   * @param message_name DBC message name.
   * @param tx True for control message map, false for feedback map.
   * @return Channel ID (1 or 2), defaults to 1 when map entry is missing.
   */
  uint8_t resolveChannel(const std::string & message_name, bool tx) const;

private:
  /** @brief Declare and fetch ROS parameters into member fields. */
  void loadParameters();

  /** @brief Open UDP channels according to loaded network parameters. */
  void initializeChannels();

  /** @brief Start RX worker threads for both CAN channels. */
  void startThreads();

  /** @brief Request stop and join RX worker threads. */
  void stopThreads();

  /**
   * @brief Receive loop of one channel: UDP receive -> decode -> publish/route.
   * @param channel_id Logical channel ID (1 or 2).
   */
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
