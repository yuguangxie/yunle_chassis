#pragma once

#include "chassis_driver/can_types.hpp"

namespace chassis_driver
{

class ChassisDriverNode;

/**
 * @brief Route received CAN frames to proper decode/publish pipeline.
 */
class FrameRouter
{
public:
  /**
   * @brief Construct frame router with node callbacks.
   * @param node Parent node used to publish decoded feedback.
   */
  explicit FrameRouter(ChassisDriverNode & node);

  /**
   * @brief Route one CAN frame to decoded publisher.
   * @param frame Received CAN frame.
   */
  void routeFrame(const CanFrame & frame);

private:
  ChassisDriverNode & node_;
};

}  // namespace chassis_driver
