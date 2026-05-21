#pragma once

#include "chassis_driver/can_types.hpp"

namespace chassis_driver
{

class ChassisDriverNode;

/**
 * @brief Route received CAN frames to proper decode/publish pipeline.
 * @brief 将接收到的 CAN 帧路由到对应的解析/发布流程。
 */
class FrameRouter
{
public:
  /**
   * @brief Construct frame router with node callbacks.
   * @param node Parent node used to publish decoded feedback.
   * @brief 使用节点回调构造帧路由器。
   * @param node 用于发布解析后反馈的父节点。
   */
  explicit FrameRouter(ChassisDriverNode & node);

  /**
   * @brief Route one CAN frame to decoded publisher.
   * @param frame Received CAN frame.
   * @brief 将一帧 CAN 报文路由到解析发布器。
   * @param frame 接收到的 CAN 帧。
   */
  void routeFrame(const CanFrame & frame);

private:
  ChassisDriverNode & node_;
};

}  // namespace chassis_driver
