# Yunle Chassis ROS2 Humble Driver

This repository provides a production-style, single-node ROS2 Humble C++17 CAN-over-Ethernet chassis driver.

## Engineering assumption

Although three outgoing command messages are named with `SCU_*`, this implementation explicitly treats them as **ACU-permitted transmit commands** per project requirement:

- `SCU_Control_Command` (ID 289)
- `SCU_Chassis_Command` (ID 294)
- `SCU_Torque_Command` (ID 291)

The DBC confirms `BO_TX_BU_` includes `ACU` for these message IDs, so the driver uses them as outgoing ACU control interface.

## Runtime DBC policy

No runtime DBC parsing is performed. All required DBC definitions are hardcoded in C++ (`DbcProtocol`) with static maps and manual bit encode/decode.

## Packages

- `chassis_interfaces`: custom `.msg` files.
- `chassis_driver`: single node `chassis_driver_node` with helper classes:
  - `UdpChannel`
  - `CanEthernetCodec`
  - `DbcProtocol`
  - `FrameRouter`
  - `ControlCommandBridge`

## Build

```bash
colcon build --packages-select chassis_interfaces chassis_driver
source install/setup.bash
```

## Run

```bash
ros2 launch chassis_driver chassis_driver.launch.py
```

## Topics

### Subscribed control topics
- `/chassis/control/scu_control_command`
- `/chassis/control/scu_chassis_command`
- `/chassis/control/scu_torque_command`

### Published feedback topics
- `/chassis/feedback/bms_status`
- `/chassis/feedback/bms_realtime_status`
- `/chassis/feedback/vcu_warning_level`
- `/chassis/feedback/wheel_speed`
- `/chassis/feedback/ccu_status`
- `/chassis/feedback/sas_angle`
- `/chassis/feedback/target_speed_feedback`

### Raw/debug topics
- `/chassis/can_rx/raw`
- `/chassis/can_tx/raw`
- `/chassis/debug/status`
- `/chassis/debug/unknown_frames`

## Control command examples

```bash
ros2 topic pub --once /chassis/control/scu_control_command chassis_interfaces/msg/ScuControlCommand "{
  scu_shift_level_request: 1,
  scu_drive_mode_request: 2,
  scu_steering_angle_jd01_front: 10.0,
  scu_steering_angle_jd01_rear: 0.0,
  scu_target_speed: 12.3,
  scu_brake_enable: false,
  gw_left_turn_light_request: 1,
  gw_right_turn_light_request: 0,
  gw_position_light_request: 1,
  gw_low_beam_request: 1,
  scu_torque_or_speed_mode: 1,
  steering_angle_speed_valid: true,
  brake_force_command_valid: true
}"

ros2 topic pub --once /chassis/control/scu_chassis_command chassis_interfaces/msg/ScuChassisCommand "{
  vcu_target_steering_angle_speed: 180.0,
  brake_force_front_left: 20.0,
  brake_force_front_right: 20.0,
  brake_force_rear_left: 10.0,
  brake_force_rear_right: 10.0
}"

ros2 topic pub --once /chassis/control/scu_torque_command chassis_interfaces/msg/ScuTorqueCommand "{
  torque_command_front_left: 100.0,
  torque_command_front_right: 100.0,
  torque_command_rear_left: 80.0,
  torque_command_rear_right: 80.0
}"
```
