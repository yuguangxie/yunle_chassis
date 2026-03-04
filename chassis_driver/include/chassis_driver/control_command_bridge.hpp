#pragma once

#include "chassis_driver/can_types.hpp"

namespace chassis_driver
{

class ChassisDriverNode;

/**
 * @brief Bridge ROS control command topics to encoded CAN frames.
 */
class ControlCommandBridge
{
public:
  /**
   * @brief Register ROS subscribers and forward callbacks as CAN control messages.
   * @param node Owning node providing subscriptions and UDP transmit function.
   */
  explicit ControlCommandBridge(ChassisDriverNode & node);

private:
  ChassisDriverNode & node_;
};

}  // namespace chassis_driver
