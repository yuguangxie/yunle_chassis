#include "chassis_driver/control_command_bridge.hpp"

#include "chassis_driver/chassis_driver_node.hpp"
#include "chassis_driver/dbc_protocol.hpp"

namespace chassis_driver
{

/** Subscribe control topics and convert each message into DBC-encoded CAN frame. */
ControlCommandBridge::ControlCommandBridge(ChassisDriverNode & node)
: node_(node)
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  node_.scu_control_sub_ = node_.create_subscription<chassis_interfaces::msg::ScuControlCommand>(
    "/chassis/control/scu_control_command", qos,
    [this](const chassis_interfaces::msg::ScuControlCommand::SharedPtr msg) {
      // Engineering assumption: although message is named SCU_*, ACU is allowed to send it.
      CanFrame frame;
      frame.can_id = 289U;
      frame.dlc = 8;
      DbcProtocol::encodeSignal(frame, "SCU_Shift_Level_Request", msg->scu_shift_level_request);
      DbcProtocol::encodeSignal(frame, "SCU_Drive_Mode_Request", msg->scu_drive_mode_request);
      DbcProtocol::encodeSignal(frame, "SCU_Steering_Angle_Front", msg->scu_steering_angle_front);
      DbcProtocol::encodeSignal(frame, "SCU_Steering_Angle_Rear", msg->scu_steering_angle_rear);
      DbcProtocol::encodeSignal(frame, "SCU_Target_Speed", msg->scu_target_speed);
      DbcProtocol::encodeSignal(frame, "SCU_Brake_Enable", msg->scu_brake_enable ? 1.0 : 0.0);
      DbcProtocol::encodeSignal(frame, "GW_Left_Turn_Light_Request", msg->gw_left_turn_light_request);
      DbcProtocol::encodeSignal(frame, "GW_Right_Turn_Light_Request", msg->gw_right_turn_light_request);
      DbcProtocol::encodeSignal(frame, "GW_Position_Light_Request", msg->gw_position_light_request);
      DbcProtocol::encodeSignal(frame, "GW_Low_Beam_Request", msg->gw_low_beam_request);
      DbcProtocol::encodeSignal(frame, "SCU_Torque_Or_Speed_Mode", msg->scu_torque_or_speed_mode);
      DbcProtocol::encodeSignal(frame, "Steering_Angle_Speed_Valid", msg->steering_angle_speed_valid ? 1.0 : 0.0);
      DbcProtocol::encodeSignal(frame, "Brake_Force_Command_Valid", msg->brake_force_command_valid ? 1.0 : 0.0);
      node_.sendControlFrame(frame, "SCU_Control_Command");
    });

  node_.scu_chassis_sub_ = node_.create_subscription<chassis_interfaces::msg::ScuChassisCommand>(
    "/chassis/control/scu_chassis_command", qos,
    [this](const chassis_interfaces::msg::ScuChassisCommand::SharedPtr msg) {
      CanFrame frame;
      frame.can_id = 294U;
      frame.dlc = 8;
      DbcProtocol::encodeSignal(frame, "VCU_Target_Steering_Angle_Speed", msg->vcu_target_steering_angle_speed);
      DbcProtocol::encodeSignal(frame, "Brake_Force_Front_Left", msg->brake_force_front_left);
      DbcProtocol::encodeSignal(frame, "Brake_Force_Front_Right", msg->brake_force_front_right);
      DbcProtocol::encodeSignal(frame, "Brake_Force_Rear_Left", msg->brake_force_rear_left);
      DbcProtocol::encodeSignal(frame, "Brake_Force_Rear_Right", msg->brake_force_rear_right);
      node_.sendControlFrame(frame, "SCU_Chassis_Command");
    });

  node_.scu_torque_sub_ = node_.create_subscription<chassis_interfaces::msg::ScuTorqueCommand>(
    "/chassis/control/scu_torque_command", qos,
    [this](const chassis_interfaces::msg::ScuTorqueCommand::SharedPtr msg) {
      CanFrame frame;
      frame.can_id = 291U;
      frame.dlc = 8;
      DbcProtocol::encodeSignal(frame, "Torque_Command_Front_Left", msg->torque_command_front_left);
      DbcProtocol::encodeSignal(frame, "Torque_Command_Front_Right", msg->torque_command_front_right);
      DbcProtocol::encodeSignal(frame, "Torque_Command_Rear_Left", msg->torque_command_rear_left);
      DbcProtocol::encodeSignal(frame, "Torque_Command_Rear_Right", msg->torque_command_rear_right);
      node_.sendControlFrame(frame, "SCU_Torque_Command");
    });
}

}  // namespace chassis_driver
