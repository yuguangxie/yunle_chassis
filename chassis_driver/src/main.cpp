#include "chassis_driver/chassis_driver_node.hpp"

#include <ros/ros.h>

/** ROS1 process entrypoint: initialize, spin chassis node, shutdown. */
/** ROS1 进程入口：初始化、运行底盘节点并关闭。 */
int main(int argc, char ** argv)
{
  ros::init(argc, argv, "chassis_driver_node");
  chassis_driver::ChassisDriverNode node;
  ros::spin();
  ros::shutdown();
  return 0;
}
