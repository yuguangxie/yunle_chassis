#include "chassis_driver/frame_router.hpp"

#include "chassis_driver/chassis_driver_node.hpp"

namespace chassis_driver
{

/** Construct router with node callback target. */
FrameRouter::FrameRouter(ChassisDriverNode & node)
: node_(node)
{
}

/** Route one received frame to node decode/publish logic. */
void FrameRouter::routeFrame(const CanFrame & frame)
{
  node_.publishDecoded(frame);
}

}  // namespace chassis_driver
