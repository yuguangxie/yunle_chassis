#include "chassis_driver/control_command_bridge.hpp"

#include "chassis_driver/chassis_driver_node.hpp"
#include "chassis_driver/dbc_protocol.hpp"

#include <cmath>

namespace chassis_driver
{

namespace
{
constexpr uint8_t kShiftD = 1U;
constexpr uint8_t kShiftN = 2U;
constexpr uint8_t kShiftR = 3U;
constexpr uint8_t kDriveAuto = 1U;
constexpr double kSteeringRawAtMaxAngle = 120.0;

bool isValidShiftRequest(uint8_t value)
{
  return value == kShiftD || value == kShiftN || value == kShiftR;
}

double steeringAngleDegToCanRaw(double angle_deg, double max_angle_deg)
{
  const int raw_signed = static_cast<int>(std::llround(angle_deg / max_angle_deg * kSteeringRawAtMaxAngle));
  return static_cast<double>(raw_signed < 0 ? 256 + raw_signed : raw_signed);
}
}  // namespace

/** Subscribe control topics and convert each message into DBC-encoded CAN frame. */
/** 订阅控制话题，并将每条消息转换为按 DBC 编码的 CAN 帧。 */
ControlCommandBridge::ControlCommandBridge(ChassisDriverNode & node)
: node_(node)
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(node_.default_qos_depth_));

  if (node_.isSubscribeTopicEnabled("control_scu_control_command")) {
    node_.scu_control_sub_ = node_.create_subscription<chassis_interfaces::msg::ScuControlCommand>(
      node_.makeTopicName("control/scu_control_command"), qos,
      [this](const chassis_interfaces::msg::ScuControlCommand::SharedPtr msg) {
      // Engineering assumption: although message is named SCU_*, ACU is allowed to send it.
      // 工程假设：虽然报文名为 SCU_*，但按当前需求 ACU 被允许发送该报文。
      if (!isValidShiftRequest(msg->scu_shift_level_request)) {
        RCLCPP_WARN(
          node_.get_logger(),
          "Drop SCU_Control_Command: scu_shift_level_request=%u is invalid, expected 1(D), 2(N), or 3(R)",
          static_cast<unsigned>(msg->scu_shift_level_request));
        return;
      }

      double speed_kmh = static_cast<double>(msg->scu_target_speed);
      if (!std::isfinite(speed_kmh) || speed_kmh < 0.0 || speed_kmh > node_.scu_control_max_target_speed_kmh_) {
        RCLCPP_WARN(
          node_.get_logger(),
          "SCU_Control_Command target speed %.3f km/h is outside [0, %.3f], send 0",
          static_cast<double>(msg->scu_target_speed), node_.scu_control_max_target_speed_kmh_);
        speed_kmh = 0.0;
      }

      double front_angle = static_cast<double>(msg->scu_steering_angle_front);
      if (!std::isfinite(front_angle) || std::abs(front_angle) > node_.scu_control_max_steering_angle_deg_) {
        RCLCPP_WARN(
          node_.get_logger(), "SCU_Control_Command front steering angle %.3f deg is outside [-%.3f, %.3f], send 0",
          front_angle, node_.scu_control_max_steering_angle_deg_, node_.scu_control_max_steering_angle_deg_);
        front_angle = 0.0;
      }

      double rear_angle = static_cast<double>(msg->scu_steering_angle_rear);
      if (!std::isfinite(rear_angle) || std::abs(rear_angle) > node_.scu_control_max_steering_angle_deg_) {
        RCLCPP_WARN(
          node_.get_logger(), "SCU_Control_Command rear steering angle %.3f deg is outside [-%.3f, %.3f], send 0",
          rear_angle, node_.scu_control_max_steering_angle_deg_, node_.scu_control_max_steering_angle_deg_);
        rear_angle = 0.0;
      }

      const double front_raw = steeringAngleDegToCanRaw(
        front_angle, node_.scu_control_max_steering_angle_deg_);
      const double rear_raw = steeringAngleDegToCanRaw(
        rear_angle, node_.scu_control_max_steering_angle_deg_);

      CanFrame frame;
      frame.can_id = 289U;
      frame.dlc = 8;
      DbcProtocol::encodeSignal(frame, "SCU_Shift_Level_Request", msg->scu_shift_level_request);
      DbcProtocol::encodeSignal(frame, "SCU_Drive_Mode_Request", kDriveAuto);
      DbcProtocol::encodeSignal(frame, "SCU_Steering_Angle_Front", front_raw);
      DbcProtocol::encodeSignal(frame, "SCU_Steering_Angle_Rear", rear_raw);
      DbcProtocol::encodeSignal(frame, "SCU_Target_Speed", speed_kmh);
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
  }

  if (node_.isSubscribeTopicEnabled("control_scu_chassis_command")) {
    node_.scu_chassis_sub_ = node_.create_subscription<chassis_interfaces::msg::ScuChassisCommand>(
      node_.makeTopicName("control/scu_chassis_command"), qos,
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
  }

  if (node_.isSubscribeTopicEnabled("control_scu_torque_command")) {
    node_.scu_torque_sub_ = node_.create_subscription<chassis_interfaces::msg::ScuTorqueCommand>(
      node_.makeTopicName("control/scu_torque_command"), qos,
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

  if (node_.isSubscribeTopicEnabled("control_vcu_chassis_debug")) {
    node_.vcu_chassis_debug_sub_ = node_.create_subscription<chassis_interfaces::msg::VcuChassisDebug>(
      node_.makeTopicName("control/vcu_chassis_debug"), qos,
      [this](const chassis_interfaces::msg::VcuChassisDebug::SharedPtr msg) {
      CanFrame enable_frame;
      enable_frame.can_id = 1808U;
      enable_frame.dlc = 8;
      DbcProtocol::encodeSignal(enable_frame, "PID_Debug_Enable", msg->pid_debug_enable ? 1.0 : 0.0);
      node_.sendControlFrame(enable_frame, "VCU_Debug_Enable");

      CanFrame debug_frame;
      debug_frame.can_id = 1813U;
      debug_frame.dlc = 8;
      DbcProtocol::encodeSignal(debug_frame, "Velocity_Kp", msg->velocity_kp);
      DbcProtocol::encodeSignal(debug_frame, "Velocity_Ki", msg->velocity_ki);
      DbcProtocol::encodeSignal(debug_frame, "Velocity_Kd", msg->velocity_kd);
      node_.sendControlFrame(debug_frame, "VCU_Drive_Debug");
      });
  }
}

}  // namespace chassis_driver
