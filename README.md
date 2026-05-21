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

The current hardcoded protocol is aligned with `Yunle_CAN_release.dbc`.

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

## Keyboard control node

The package also provides an optional keyboard helper node that publishes
`/yunle_chassis/control/scu_control_command` for manual low-speed checkout.
Run it directly in a terminal that has keyboard focus:

```bash
ros2 run chassis_driver keyboard_scu_control_node
```

An example launch file with the default parameters is also provided:

```bash
ros2 launch chassis_driver keyboard_scu_control.launch.py
```

For interactive keyboard input, `ros2 run` is recommended because launch
frontends may not attach stdin to child processes in every terminal setup.

Default key bindings:

- `w` / `s`: select D/R and increase forward/reverse speed
- `q` / `e`: decrease/increase target speed
- `a` / `d`: steer left/right
- `c`: center steering
- `x`: neutral zero command
- `Space`: brake and neutral
- `b`: toggle brake enable
- `j` / `k` / `u` / `i`: toggle left/right/position/low-beam light request
- `m`: toggle torque-or-speed mode
- `v`: toggle steering/brake valid flags
- `h`: print help
- `Ctrl-C`: exit

Important parameters in `keyboard_scu_control.launch.py`:

- `topic_name`: target topic, default `/yunle_chassis/control/scu_control_command`
- `publish_rate_hz`: command publish rate, default `20.0`
- `speed_step_kmh`: speed increment/decrement per key press, default `0.5`
- `default_speed_kmh`: first nonzero speed when pressing `w` or `s`, default `1.0`
- `max_speed_kmh`: keyboard-side speed limit, default `15.0`
- `steering_step_deg`: steering increment/decrement per key press, default `2.0`
- `max_steering_angle_deg`: keyboard-side steering limit, default `27.0`
- `rear_steering_ratio`: rear steering angle as a ratio of front steering, default `0.0`
- `auto_publish_zero_on_exit`: publish a brake/zero command when the node exits normally, default `true`

## Unified config file

All driver/network parameters are merged into one file:

- `chassis_driver/config/chassis_driver.yaml`

### Parameter notes (detailed)

- `topic_prefix`：统一的话题前缀，默认 `/yunle_chassis`。
- `publish_raw_can`：是否发布原始收发 CAN 帧。
- `publish_unknown_frames`：是否发布未知帧信息。
- `enable_debug_topics`：是否启用 debug 类话题。
- `log_control_can_frames`：是否在节点日志中输出成功下发到底盘的控制 CAN 帧十六进制内容。
- `default_qos_depth`：发布订阅队列深度。
- `enabled_publish_topics`：可选择启用发布话题（`all` 为全部）。
- `enabled_subscribe_topics`：可选择启用订阅话题（`all` 为全部）。
- `message_channel_map`：反馈消息走 `can1/can2` 的映射。
- `control_message_channel_map`：控制消息发送通道映射。
- `local_ip`：UDP 本地绑定 IP。
- `can1_local_port` / `can2_local_port`：CAN1/2 本地端口。
- `can1_remote_ip` / `can2_remote_ip`：CAN1/2 对端网关 IP。
- `remote_port`：对端 UDP 端口。
- `udp_buffer_size`：UDP 接收缓冲大小。
- `socket_timeout_ms`：UDP 接收超时。

## Topics (default prefix: `/yunle_chassis`)

### Subscribed control topics
- `/yunle_chassis/control/scu_control_command`
- `/yunle_chassis/control/scu_chassis_command`
- `/yunle_chassis/control/scu_torque_command`
- `/yunle_chassis/control/vcu_chassis_debug`

### Published feedback topics
- `/yunle_chassis/feedback/bms_status`
- `/yunle_chassis/feedback/vcu_warning_level`
- `/yunle_chassis/feedback/wheel_speed`
- `/yunle_chassis/feedback/ccu_status`
- `/yunle_chassis/feedback/sas_angle`
- `/yunle_chassis/feedback/target_speed_feedback`

### Raw/debug topics
- `/yunle_chassis/can_rx/raw`
- `/yunle_chassis/can_tx/raw`
- `/yunle_chassis/debug/unknown_frames`

## Control command examples

```bash
ros2 topic pub --once /yunle_chassis/control/scu_control_command chassis_interfaces/msg/ScuControlCommand "{
  scu_shift_level_request: 1,
  scu_steering_angle_front: 10.0,
  scu_steering_angle_rear: 0.0,
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

ros2 topic pub --once /yunle_chassis/control/scu_chassis_command chassis_interfaces/msg/ScuChassisCommand "{
  vcu_target_steering_angle_speed: 180.0,
  brake_force_front_left: 20.0,
  brake_force_front_right: 20.0,
  brake_force_rear_left: 10.0,
  brake_force_rear_right: 10.0
}"

ros2 topic pub --once /yunle_chassis/control/scu_torque_command chassis_interfaces/msg/ScuTorqueCommand "{
  torque_command_front_left: 100.0,
  torque_command_front_right: 100.0,
  torque_command_rear_left: 80.0,
  torque_command_rear_right: 80.0
}"

ros2 topic pub --once /yunle_chassis/control/vcu_chassis_debug chassis_interfaces/msg/VcuChassisDebug "{
  pid_debug_enable: true,
  velocity_kp: 10.0,
  velocity_ki: 0.5,
  velocity_kd: 0.2
}"
```
