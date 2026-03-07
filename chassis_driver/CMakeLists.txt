cmake_minimum_required(VERSION 3.0.2)
project(chassis_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  chassis_interfaces
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs chassis_interfaces
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(chassis_driver_node
  src/main.cpp
  src/chassis_driver_node.cpp
  src/control_command_bridge.cpp
  src/frame_router.cpp
  src/dbc_protocol.cpp
  src/can_ethernet_codec.cpp
  src/udp_channel.cpp
)

add_dependencies(chassis_driver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(chassis_driver_node
  ${catkin_LIBRARIES}
)

install(TARGETS chassis_driver_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
