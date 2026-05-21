#include "chassis_driver/chassis_driver_node.hpp"

#include <rclcpp/rclcpp.hpp>

/** ROS2 process entrypoint: initialize, spin chassis node, shutdown. */
/** ROS2 进程入口：初始化、运行底盘节点并关闭。 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<chassis_driver::ChassisDriverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
