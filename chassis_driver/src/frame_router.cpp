#include "chassis_driver/frame_router.hpp"

#include "chassis_driver/chassis_driver_node.hpp"

namespace chassis_driver
{

/** Construct router with node callback target. */
/** 使用节点回调目标构造路由器。 */
FrameRouter::FrameRouter(ChassisDriverNode & node)
: node_(node)
{
}

/** Route one received frame to node decode/publish logic. */
/** 将接收到的一帧报文路由到节点解析/发布逻辑。 */
void FrameRouter::routeFrame(const CanFrame & frame)
{
  node_.publishDecoded(frame);
}

}  // namespace chassis_driver
