#include "chassis_driver/frame_router.hpp"

#include "chassis_driver/chassis_driver_node.hpp"

namespace chassis_driver
{

FrameRouter::FrameRouter(ChassisDriverNode & node)
: node_(node)
{
}

void FrameRouter::routeFrame(const CanFrame & frame)
{
  node_.publishDecoded(frame);
}

}  // namespace chassis_driver
