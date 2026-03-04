cmake_minimum_required(VERSION 3.8)
project(chassis_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(chassis_interfaces REQUIRED)

include_directories(include)

add_executable(chassis_driver_node
  src/main.cpp
  src/chassis_driver_node.cpp
  src/control_command_bridge.cpp
  src/frame_router.cpp
  src/dbc_protocol.cpp
  src/can_ethernet_codec.cpp
  src/udp_channel.cpp
)

ament_target_dependencies(chassis_driver_node
  rclcpp
  std_msgs
  builtin_interfaces
  chassis_interfaces
)

install(TARGETS chassis_driver_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
