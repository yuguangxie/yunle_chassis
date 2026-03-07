#pragma once

#include "chassis_driver/can_types.hpp"
#include "chassis_driver/udp_channel.hpp"

#include "chassis_interfaces/BmsRealtimeStatus.h"
#include "chassis_interfaces/BmsStatus.h"
#include "chassis_interfaces/CanFrame.h"
#include "chassis_interfaces/CcuStatus.h"
#include "chassis_interfaces/SasAngleFeedback.h"
#include "chassis_interfaces/ScuChassisCommand.h"
#include "chassis_interfaces/ScuControlCommand.h"
#include "chassis_interfaces/ScuTargetSpeedFeedback.h"
#include "chassis_interfaces/ScuTorqueCommand.h"
#include "chassis_interfaces/VcuWarningLevel.h"
#include "chassis_interfaces/WheelSpeedFeedback.h"

#include "std_msgs/String.h"

#include <atomic>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>

#include <ros/ros.h>

namespace chassis_driver
{

class ControlCommandBridge;
class FrameRouter;

class ChassisDriverNode
{
public:
  ChassisDriverNode();
  ~ChassisDriverNode();

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

  std::string makeTopicName(const std::string & suffix) const;
  bool isPublishTopicEnabled(const std::string & topic_key) const;
  bool isSubscribeTopicEnabled(const std::string & topic_key) const;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  std::string topic_prefix_;
  bool publish_raw_can_{true};
  bool publish_unknown_frames_{true};
  bool enable_debug_topics_{true};
  int default_qos_depth_{10};
  std::unordered_map<std::string, std::string> control_channel_map_;
  std::unordered_map<std::string, std::string> feedback_channel_map_;
  std::set<std::string> enabled_publish_topics_;
  std::set<std::string> enabled_subscribe_topics_;

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

  ros::Publisher raw_rx_pub_;
  ros::Publisher raw_tx_pub_;
  ros::Publisher debug_status_pub_;
  ros::Publisher unknown_frame_pub_;

  ros::Publisher bms_status_pub_;
  ros::Publisher bms_realtime_pub_;
  ros::Publisher vcu_warning_pub_;
  ros::Publisher wheel_speed_pub_;
  ros::Publisher ccu_status_pub_;
  ros::Publisher sas_angle_pub_;
  ros::Publisher target_speed_pub_;

  std::shared_ptr<FrameRouter> frame_router_;
  std::shared_ptr<ControlCommandBridge> control_bridge_;

  ros::Subscriber scu_control_sub_;
  ros::Subscriber scu_chassis_sub_;
  ros::Subscriber scu_torque_sub_;

  friend class FrameRouter;
  friend class ControlCommandBridge;
};

}  // namespace chassis_driver
