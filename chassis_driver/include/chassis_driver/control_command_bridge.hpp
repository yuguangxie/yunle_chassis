#pragma once

#include "chassis_driver/can_types.hpp"

namespace chassis_driver
{

class ChassisDriverNode;

/**
 * @brief Bridge ROS control command topics to encoded CAN frames.
 * @brief 将 ROS 控制指令话题桥接为编码后的 CAN 帧。
 */
class ControlCommandBridge
{
public:
  /**
   * @brief Register ROS subscribers and forward callbacks as CAN control messages.
   * @param node Owning node providing subscriptions and UDP transmit function.
   * @brief 注册 ROS 订阅者，并在回调中转发为 CAN 控制报文。
   * @param node 提供订阅和 UDP 发送能力的所属节点。
   */
  explicit ControlCommandBridge(ChassisDriverNode & node);

private:
  ChassisDriverNode & node_;
};

}  // namespace chassis_driver
