#pragma once

#include "chassis_driver/can_types.hpp"

namespace chassis_driver
{

class ChassisDriverNode;

class FrameRouter
{
public:
  explicit FrameRouter(ChassisDriverNode & node);
  void routeFrame(const CanFrame & frame);

private:
  ChassisDriverNode & node_;
};

}  // namespace chassis_driver
