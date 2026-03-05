cmake_minimum_required(VERSION 3.0.2)
project(chassis_interfaces)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(FILES
  CanFrame.msg
  ScuControlCommand.msg
  ScuChassisCommand.msg
  ScuTorqueCommand.msg
  BmsStatus.msg
  BmsRealtimeStatus.msg
  VcuWarningLevel.msg
  WheelSpeedFeedback.msg
  CcuStatus.msg
  SasAngleFeedback.msg
  ScuTargetSpeedFeedback.msg
)

generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
