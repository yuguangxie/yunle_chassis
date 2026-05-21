#pragma once

#include "chassis_driver/can_types.hpp"
#include "chassis_driver/frame_router.hpp"
#include "chassis_driver/udp_channel.hpp"

#include "chassis_interfaces/BmsStatus.h"
#include "chassis_interfaces/CanFrame.h"
#include "chassis_interfaces/CcuStatus.h"
#include "chassis_interfaces/SasAngleFeedback.h"
#include "chassis_interfaces/ScuChassisCommand.h"
#include "chassis_interfaces/ScuControlCommand.h"
#include "chassis_interfaces/ScuTargetSpeedFeedback.h"
#include "chassis_interfaces/ScuTorqueCommand.h"
#include "chassis_interfaces/VcuChassisDebug.h"
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

/**
 * @brief ROS1 chassis driver node handling UDP-CAN bridging and topic I/O.
 * @brief ROS1 底盘驱动节点，负责 UDP-CAN 桥接和话题输入输出。
 */
class ChassisDriverNode
{
public:
  /** @brief Construct node, load parameters, initialize channels and worker threads. */
  /** @brief 构造节点，加载参数，初始化通道和工作线程。 */
  ChassisDriverNode();

  /** @brief Stop worker threads and release resources. */
  /** @brief 停止工作线程并释放资源。 */
  ~ChassisDriverNode();

  /**
   * @brief Encode and send one control CAN frame to mapped channel.
   * @param frame CAN frame to transmit.
   * @param message_name DBC message name used for channel lookup.
   * @brief 编码并发送一帧控制 CAN 报文到映射通道。
   * @param frame 待发送的 CAN 帧。
   * @param message_name 用于查询通道映射的 DBC 报文名称。
   */
  void sendControlFrame(const CanFrame & frame, const std::string & message_name);

  /**
   * @brief Decode CAN frame by DBC and publish mapped ROS feedback message.
   * @param frame CAN frame received from chassis.
   * @brief 按 DBC 解码 CAN 帧，并发布对应 ROS 反馈消息。
   * @param frame 从底盘接收到的 CAN 帧。
   */
  void publishDecoded(const CanFrame & frame);

  /** @brief Publish raw received CAN frame topic for diagnostics. */
  /** @brief 发布原始接收 CAN 帧话题，用于诊断。 */
  void publishRawRx(const CanFrame & frame);

  /** @brief Publish raw transmitted CAN frame topic for diagnostics. */
  /** @brief 发布原始发送 CAN 帧话题，用于诊断。 */
  void publishRawTx(const CanFrame & frame);

  /** @brief Publish unknown frame info when no DBC message mapping is found. */
  /** @brief 当没有找到 DBC 报文映射时发布未知帧信息。 */
  void publishUnknownFrame(const CanFrame & frame);

  /**
   * @brief Resolve configured channel ID for a message name.
   * @param message_name DBC message name.
   * @param tx True for control message map, false for feedback map.
   * @return Channel ID (1 or 2), defaults to 1 when map entry is missing.
   * @brief 根据报文名称解析配置中的通道 ID。
   * @param message_name DBC 报文名称。
   * @param tx true 表示查询控制报文映射，false 表示查询反馈报文映射。
   * @return 通道 ID（1 或 2），映射缺失时默认返回 1。
   */
  uint8_t resolveChannel(const std::string & message_name, bool tx) const;

private:
  /** @brief Declare and fetch ROS parameters into member fields. */
  /** @brief 声明并读取 ROS 参数到成员变量。 */
  void loadParameters();

  /** @brief Open UDP channels according to loaded network parameters. */
  /** @brief 根据已加载的网络参数打开 UDP 通道。 */
  void initializeChannels();

  /** @brief Start RX worker threads for both CAN channels. */
  /** @brief 启动两个 CAN 通道的接收工作线程。 */
  void startThreads();

  /** @brief Request stop and join RX worker threads. */
  /** @brief 请求停止并 join 接收工作线程。 */
  void stopThreads();

  /**
   * @brief Receive loop of one channel: UDP receive -> decode -> publish/route.
   * @param channel_id Logical channel ID (1 or 2).
   * @brief 单个通道的接收循环：UDP 接收 -> 解码 -> 发布/路由。
   * @param channel_id 逻辑通道 ID（1 或 2）。
   */
  void rxLoop(uint8_t channel_id);

  /** @brief Build full topic name based on configured prefix and relative suffix. */
  /** @brief 根据配置前缀和相对后缀生成完整话题名。 */
  std::string makeTopicName(const std::string & suffix) const;

  /** @brief Check whether one logical publisher is enabled by configuration. */
  /** @brief 检查某个逻辑发布者是否被配置启用。 */
  bool isPublishTopicEnabled(const std::string & topic_key) const;

  /** @brief Check whether one logical subscriber is enabled by configuration. */
  /** @brief 检查某个逻辑订阅者是否被配置启用。 */
  bool isSubscribeTopicEnabled(const std::string & topic_key) const;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  std::string topic_prefix_;
  bool publish_raw_can_{true};
  bool publish_unknown_frames_{true};
  bool enable_debug_topics_{true};
  bool log_control_can_frames_{false};
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

  double scu_control_max_steering_angle_deg_{27.0};
  double scu_control_max_target_speed_kmh_{15.0};

  UdpChannel can1_;
  UdpChannel can2_;

  std::atomic<bool> running_{false};
  std::thread can1_rx_thread_;
  std::thread can2_rx_thread_;

  std::mutex tx_mutex_;

  ros::Publisher raw_rx_pub_;
  ros::Publisher raw_tx_pub_;
  ros::Publisher unknown_frame_pub_;

  ros::Publisher bms_status_pub_;
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
  ros::Subscriber vcu_chassis_debug_sub_;

  friend class FrameRouter;
  friend class ControlCommandBridge;
};

}  // namespace chassis_driver
