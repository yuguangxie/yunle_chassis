#pragma once

#include "chassis_driver/can_types.hpp"

namespace chassis_driver
{

class ChassisDriverNode;

class ControlCommandBridge
{
public:
  explicit ControlCommandBridge(ChassisDriverNode & node);

private:
  ChassisDriverNode & node_;
};

}  // namespace chassis_driver
