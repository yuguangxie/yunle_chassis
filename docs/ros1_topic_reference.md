# ROS1 话题接口说明

项目归属：skywilling。

本文档说明 `chassis_driver_node` 对外提供的 ROS1 topic 接口。内容依据当前源码、`chassis_interfaces/msg/*.msg`、`chassis_driver/config/chassis_driver.yaml` 和 `Yunle_CAN_release.dbc` 整理。

## 1. 节点和命名空间

| 项目 | 当前值 |
|---|---|
| ROS1 package | `chassis_driver` |
| 节点可执行文件 | `chassis_driver_node` |
| 默认节点名 | `/chassis_driver_node` |
| 默认话题前缀 | `/yunle_chassis` |
| 启动文件 | `chassis_driver/launch/chassis_driver.launch` |
| 键盘控制节点 | `keyboard_scu_control_node` |
| 键盘控制启动文件 | `chassis_driver/launch/keyboard_scu_control.launch` |
| 配置文件 | `chassis_driver/config/chassis_driver.yaml` |

配置文件在 launch 中作为节点私有参数加载。`topic_prefix` 控制所有对外话题前缀。

## 2. 发布话题

| Topic | Message type | 触发方式 | 来源 CAN 报文 | 源码位置 |
|---|---|---|---|---|
| `/yunle_chassis/can_rx/raw` | `chassis_interfaces/CanFrame` | 每次 UDP payload 成功解码出 CAN 帧 | 原始 RX CAN 帧 | `ChassisDriverNode::publishRawRx()` |
| `/yunle_chassis/can_tx/raw` | `chassis_interfaces/CanFrame` | 每次控制 CAN 帧成功发送 | 原始 TX CAN 帧 | `ChassisDriverNode::publishRawTx()` |
| `/yunle_chassis/debug/unknown_frames` | `std_msgs/String` | 收到未实现解析的 CAN ID | 未知 CAN 帧 | `ChassisDriverNode::publishUnknownFrame()` |
| `/yunle_chassis/feedback/bms_status` | `chassis_interfaces/BmsStatus` | 收到 CAN ID `0x100` | `BMS_Status` | `ChassisDriverNode::publishDecoded()` |
| `/yunle_chassis/feedback/vcu_warning_level` | `chassis_interfaces/VcuWarningLevel` | 收到 CAN ID `0x077` | `VCU_Warning_Level` | `ChassisDriverNode::publishDecoded()` |
| `/yunle_chassis/feedback/wheel_speed` | `chassis_interfaces/WheelSpeedFeedback` | 收到 CAN ID `0x168` | `VCU_Wheel_Speed_Feedback` | `ChassisDriverNode::publishDecoded()` |
| `/yunle_chassis/feedback/ccu_status` | `chassis_interfaces/CcuStatus` | 收到 CAN ID `0x051` | `VCU_CCU_Status` | `ChassisDriverNode::publishDecoded()` |
| `/yunle_chassis/feedback/sas_angle` | `chassis_interfaces/SasAngleFeedback` | 收到 CAN ID `0x0E1` | `SAS_Angle_Feedback` | `ChassisDriverNode::publishDecoded()` |
| `/yunle_chassis/feedback/target_speed_feedback` | `chassis_interfaces/ScuTargetSpeedFeedback` | 收到 CAN ID `0x7F1` | `SCU_Target_Speed_Feedback` | `ChassisDriverNode::publishDecoded()` |

发布频率由底盘 CAN 反馈到达频率决定，当前代码没有对反馈 topic 增加 ROS timer。

## 3. 订阅话题

| Topic | Message type | 触发方式 | CAN 输出 | 源码位置 |
|---|---|---|---|---|
| `/yunle_chassis/control/scu_control_command` | `chassis_interfaces/ScuControlCommand` | 收到 ROS1 topic 消息 | CAN ID `0x121` `SCU_Control_Command` | `ControlCommandBridge::ControlCommandBridge()` |
| `/yunle_chassis/control/scu_chassis_command` | `chassis_interfaces/ScuChassisCommand` | 收到 ROS1 topic 消息 | CAN ID `0x126` `SCU_Chassis_Command` | `ControlCommandBridge::ControlCommandBridge()` |
| `/yunle_chassis/control/scu_torque_command` | `chassis_interfaces/ScuTorqueCommand` | 收到 ROS1 topic 消息 | CAN ID `0x123` `SCU_Torque_Command` | `ControlCommandBridge::ControlCommandBridge()` |
| `/yunle_chassis/control/vcu_chassis_debug` | `chassis_interfaces/VcuChassisDebug` | 收到 ROS1 topic 消息 | CAN ID `0x710` `VCU_Debug_Enable` 和 CAN ID `0x715` `VCU_Drive_Debug` | `ControlCommandBridge::ControlCommandBridge()` |

> **警示：** `/yunle_chassis/control/scu_chassis_command`、`/yunle_chassis/control/scu_torque_command` 和 `/yunle_chassis/control/vcu_chassis_debug` 会直接影响底盘制动力、转向响应、驱动扭矩或 VCU 调试参数，必须在专业人士指导下，于安全测试环境中操作。

## 4. `ScuControlCommand` 封装规则

`/yunle_chassis/control/scu_control_command` 是对 CAN ID `0x121` 的 ROS1 封装。

| ROS 字段 | CAN 信号 | 当前处理 |
|---|---|---|
| `scu_shift_level_request` | `SCU_Shift_Level_Request` | 只接受 `1=D`、`2=N`、`3=R`；其他值拒发本帧。 |
| 无 ROS 输入 | `SCU_Drive_Mode_Request` | 固定下发 `1`。 |
| `scu_steering_angle_front` | `SCU_Steering_Angle_Front` | 有效范围内按最大转角换算为 8 bit 补码 raw；非有限数或超范围值输出 warning 并按 0 下发。 |
| `scu_steering_angle_rear` | `SCU_Steering_Angle_Rear` | 有效范围内按最大转角换算为 8 bit 补码 raw；非有限数或超范围值输出 warning 并按 0 下发。 |
| `scu_target_speed` | `SCU_Target_Speed` | 只接受 `[0, scu_control_max_target_speed_kmh]`；非有限数、负值或超范围值输出 warning 并按 0 下发。 |
| `scu_brake_enable` | `SCU_Brake_Enable` | `true=1`，`false=0`。 |
| 灯光字段 | `GW_*_Light_Request` | 由 DBC 编码层写入对应 bit。 |
| `scu_torque_or_speed_mode` | `SCU_Torque_Or_Speed_Mode` | 由 DBC 编码层写入。 |
| `steering_angle_speed_valid` | `Steering_Angle_Speed_Valid` | `true=1`，`false=0`。 |
| `brake_force_command_valid` | `Brake_Force_Command_Valid` | `true=1`，`false=0`。 |

## 5. 转角反馈换算

`/yunle_chassis/feedback/ccu_status` 中的 `ccu_steering_wheel_angle`，以及 `/yunle_chassis/feedback/sas_angle` 中的 `sas_front_angle`、`sas_rear_angle`，当前发布值均为按配置换算后的实际转角，单位为 deg。

换算使用参数：

```yaml
scu_control_max_steering_angle_deg: 27.0
```

换算关系：

```text
实际角度 = 转角编码值 / 120 × scu_control_max_steering_angle_deg
```

其中 `ccu_steering_wheel_angle` 会结合 `Steering_Wheel_Direction` 决定正负号。

## 6. UDP 网关参数

CAN1 和 CAN2 的远端网关端口分别配置：

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `can1_remote_port` | `1234` | CAN1 网关 UDP 目标端口。 |
| `can2_remote_port` | `1234` | CAN2 网关 UDP 目标端口。 |

这两个参数分别对应 `can1_remote_ip` 和 `can2_remote_ip`，不再使用单一 `remote_port` 参数。

## 7. 使用示例

查看话题：

```bash
rostopic list
rostopic echo /yunle_chassis/feedback/ccu_status
rostopic echo /yunle_chassis/can_rx/raw
```

发送 SCU 控制指令：

```bash
rostopic pub -1 /yunle_chassis/control/scu_control_command chassis_interfaces/ScuControlCommand "{
  scu_shift_level_request: 1,
  scu_steering_angle_front: 10.0,
  scu_steering_angle_rear: 0.0,
  scu_target_speed: 3.0,
  scu_brake_enable: false,
  gw_left_turn_light_request: 0,
  gw_right_turn_light_request: 0,
  gw_position_light_request: 0,
  gw_low_beam_request: 0,
  scu_torque_or_speed_mode: 1,
  steering_angle_speed_valid: true,
  brake_force_command_valid: true
}"
```

发送停车制动指令：

```bash
rostopic pub -1 /yunle_chassis/control/scu_control_command chassis_interfaces/ScuControlCommand "{
  scu_shift_level_request: 1,
  scu_steering_angle_front: 0.0,
  scu_steering_angle_rear: 0.0,
  scu_target_speed: 0.0,
  scu_brake_enable: true,
  gw_left_turn_light_request: 0,
  gw_right_turn_light_request: 0,
  gw_position_light_request: 0,
  gw_low_beam_request: 0,
  scu_torque_or_speed_mode: 1,
  steering_angle_speed_valid: true,
  brake_force_command_valid: true
}"
```

发送底盘调试指令：

> **警示：VCU 调试控制会修改或使能速度环 PID 调试相关参数，可能改变车辆控制响应和稳定性。该操作仅供专业调试人员使用，必须在专业人士指导下，在安全测试环境中执行。**

```bash
rostopic pub -1 /yunle_chassis/control/vcu_chassis_debug chassis_interfaces/VcuChassisDebug "{
  pid_debug_enable: true,
  velocity_kp: 10.0,
  velocity_ki: 0.5,
  velocity_kd: 0.2
}"
```

## 8. 键盘控制节点

键盘控制节点发布 `/yunle_chassis/control/scu_control_command`：

```bash
rosrun chassis_driver keyboard_scu_control_node
```

也可以通过 launch 启动：

```bash
roslaunch chassis_driver keyboard_scu_control.launch
```

默认控制消息为 N 档、零速度、零转角、制动关闭、灯光关闭，且 `scu_torque_or_speed_mode=0`、`steering_angle_speed_valid=false`、`brake_force_command_valid=false`。按 `h` 可打印中英文帮助。

主要按键：

| 按键 | 功能 |
|---|---|
| `w` / `s` | 增加 / 降低目标速度，不改变档位。 |
| `1` / `2` / `3` | 选择 D / N / R 档。 |
| `a` / `d` | 左转 / 右转。 |
| `Space` | N 档制动。 |
| `x` | N 档零速命令。 |
| `m` | 切换扭矩/速度模式。 |
| `v` | 切换转向角速度和制动力有效位。 |
| `h` | 打印中英文帮助。 |

## 9. 备注

- 当前代码没有 service、action、TF 或 odom 发布。
- 当前控制报文由 topic callback 触发发送，没有独立周期发送 timer。
- 当前驱动没有控制超时自动停车和网关断线自动重连逻辑。
