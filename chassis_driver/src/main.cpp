#include "chassis_driver/chassis_driver_node.hpp"

#include <ros/ros.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "chassis_driver_node");
  chassis_driver::ChassisDriverNode node;
  ros::spin();
  return 0;
}
