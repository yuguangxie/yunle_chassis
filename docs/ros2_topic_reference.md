# ROS2 话题接口说明

生成日期：2026-05-21

本文档说明当前 `unit_protocol` 分支中 `chassis_driver_node` 对外提供的 ROS2 topic 接口。内容依据当前源码、`chassis_interfaces/msg/*.msg`、`chassis_driver/config/chassis_driver.yaml`、`Yunle_CAN_release.dbc`，以及 `docs/云乐线控底盘通信协议使用说明-2026.docx` 中对应 CAN 报文说明整理。

## 1. 适用范围

当前节点：

| 项目 | 当前值 |
|---|---|
| ROS2 package | `chassis_driver` |
| executable | `chassis_driver_node` |
| node name | `chassis_driver_node` |
| 参数文件 | `chassis_driver/config/chassis_driver.yaml` |
| 默认 topic 前缀 | `/yunle_chassis` |
| QoS 深度 | 参数 `default_qos_depth`，默认 `10` |

topic 名称由 `topic_prefix` 与相对 topic 拼接生成。本文档默认使用 `topic_prefix=/yunle_chassis`。

当前源码未发现 service、action、`nav_msgs/Odometry`、TF broadcaster、`/cmd_vel` 订阅或发布。

## 2. Topic 开关参数

publisher 是否创建由 `enabled_publish_topics` 控制；subscriber 是否创建由 `enabled_subscribe_topics` 控制。默认均为 `["all"]`。

控制 CAN 帧日志由参数 `log_control_can_frames` 控制，默认 `false`。当该参数为 `true` 时，每一帧成功发送到底盘的控制 CAN frame 会以十六进制输出到节点日志。该日志开关独立于 `/yunle_chassis/can_tx/raw` topic。

可用 publisher key：

| key | 默认 topic |
|---|---|
| `can_rx_raw` | `/yunle_chassis/can_rx/raw` |
| `can_tx_raw` | `/yunle_chassis/can_tx/raw` |
| `debug_unknown_frames` | `/yunle_chassis/debug/unknown_frames` |
| `feedback_bms_status` | `/yunle_chassis/feedback/bms_status` |
| `feedback_vcu_warning_level` | `/yunle_chassis/feedback/vcu_warning_level` |
| `feedback_wheel_speed` | `/yunle_chassis/feedback/wheel_speed` |
| `feedback_ccu_status` | `/yunle_chassis/feedback/ccu_status` |
| `feedback_sas_angle` | `/yunle_chassis/feedback/sas_angle` |
| `feedback_target_speed_feedback` | `/yunle_chassis/feedback/target_speed_feedback` |

可用 subscriber key：

| key | 默认 topic |
|---|---|
| `control_scu_control_command` | `/yunle_chassis/control/scu_control_command` |
| `control_scu_chassis_command` | `/yunle_chassis/control/scu_chassis_command` |
| `control_scu_torque_command` | `/yunle_chassis/control/scu_torque_command` |
| `control_vcu_chassis_debug` | `/yunle_chassis/control/vcu_chassis_debug` |

## 3. 总览

### 3.1 发布的话题

| Topic | Message type | 当前代码触发方式 | CAN 来源 |
|---|---|---|---|
| `/yunle_chassis/can_rx/raw` | `chassis_interfaces/msg/CanFrame` | 每个 UDP payload 被解析出 CAN frame 后发布，受 `publish_raw_can` 控制 | 所有收到的 CAN frame |
| `/yunle_chassis/can_tx/raw` | `chassis_interfaces/msg/CanFrame` | CAN 控制帧发送成功后发布，受 `publish_raw_can` 控制 | 所有成功发送的控制/debug CAN frame |
| `/yunle_chassis/debug/unknown_frames` | `std_msgs/msg/String` | 收到未在 `publishDecoded()` switch 中处理的 CAN ID 时发布 | 未识别 CAN frame |
| `/yunle_chassis/feedback/bms_status` | `chassis_interfaces/msg/BmsStatus` | 收到 CAN ID `0x100` 后发布 | `BMS_Status` |
| `/yunle_chassis/feedback/vcu_warning_level` | `chassis_interfaces/msg/VcuWarningLevel` | 收到 CAN ID `0x77` 后发布 | `VCU_Warning_Level` |
| `/yunle_chassis/feedback/wheel_speed` | `chassis_interfaces/msg/WheelSpeedFeedback` | 收到 CAN ID `0x168` 后发布 | `VCU_Wheel_Speed_Feedback` |
| `/yunle_chassis/feedback/ccu_status` | `chassis_interfaces/msg/CcuStatus` | 收到 CAN ID `0x51` 后发布 | `VCU_CCU_Status` |
| `/yunle_chassis/feedback/sas_angle` | `chassis_interfaces/msg/SasAngleFeedback` | 收到 CAN ID `0xE1` 后发布 | `SAS_Angle_Feedback` |
| `/yunle_chassis/feedback/target_speed_feedback` | `chassis_interfaces/msg/ScuTargetSpeedFeedback` | 收到 CAN ID `0x7F1` 后发布 | `SCU_Target_Speed_Feedback` |

当前代码没有使用 timer 主动发布反馈 topic。反馈 topic 的实际发布频率由底盘 CAN 报文到达频率决定。

### 3.2 订阅的话题

| Topic | Message type | 当前代码触发方式 | CAN 输出 |
|---|---|---|---|
| `/yunle_chassis/control/scu_control_command` | `chassis_interfaces/msg/ScuControlCommand` | 收到 ROS 消息后立即封装并发送一次 | CAN ID `0x121`，`SCU_Control_Command` |
| `/yunle_chassis/control/scu_chassis_command` | `chassis_interfaces/msg/ScuChassisCommand` | 收到 ROS 消息后立即封装并发送一次 | CAN ID `0x126`，`SCU_Chassis_Command` |
| `/yunle_chassis/control/scu_torque_command` | `chassis_interfaces/msg/ScuTorqueCommand` | 收到 ROS 消息后立即封装并发送一次 | CAN ID `0x123`，`SCU_Torque_Command` |
| `/yunle_chassis/control/vcu_chassis_debug` | `chassis_interfaces/msg/VcuChassisDebug` | 收到 ROS 消息后连续发送两帧 | CAN ID `0x710`，`VCU_Debug_Enable`；CAN ID `0x715`，`VCU_Drive_Debug` |

注意：协议文档中 0x121 要求 10 ms 周期持续发送，连续约 500 ms 未收到控制指令则切换回手动控制。当前代码仍是 ROS topic callback 触发式发送，没有实现 0x121 周期发送 timer。

## 4. Raw 与 Debug Topic

### 4.1 `/yunle_chassis/can_rx/raw`

| 项目 | 内容 |
|---|---|
| 类型 | `chassis_interfaces/msg/CanFrame` |
| 方向 | driver 发布 |
| 源码位置 | `ChassisDriverNode::publishRawRx()` |
| 触发 | `CanEthernetCodec::decodePayload()` 从 UDP payload 中解析出每个 CAN record 后 |
| 参数开关 | `publish_raw_can=true` 且 publisher key `can_rx_raw` 启用 |

消息字段：

| 字段 | 类型 | 含义 |
|---|---|---|
| `stamp` | `builtin_interfaces/Time` | driver 发布时刻 |
| `can_id` | `uint32` | CAN ID，标准帧会被 transport codec mask 到 11 bit |
| `is_extended` | `bool` | 是否扩展帧 |
| `is_remote` | `bool` | 是否远程帧 |
| `dlc` | `uint8` | CAN DLC，当前 codec 最大按 8 处理 |
| `data` | `uint8[8]` | CAN data bytes |
| `channel` | `uint8` | driver 标记的接收通道，`1` 或 `2` |

### 4.2 `/yunle_chassis/can_tx/raw`

| 项目 | 内容 |
|---|---|
| 类型 | `chassis_interfaces/msg/CanFrame` |
| 方向 | driver 发布 |
| 源码位置 | `ChassisDriverNode::publishRawTx()` |
| 触发 | `sendControlFrame()` 通过 UDP 成功发送 CAN frame 后 |
| 参数开关 | `publish_raw_can=true` 且 publisher key `can_tx_raw` 启用 |

该 topic 只在发送成功后发布；如果 `UdpChannel::send()` 失败，当前代码记录 error 并返回，不发布 raw TX。

### 4.3 `/yunle_chassis/debug/unknown_frames`

| 项目 | 内容 |
|---|---|
| 类型 | `std_msgs/msg/String` |
| 方向 | driver 发布 |
| 源码位置 | `ChassisDriverNode::publishUnknownFrame()` |
| 触发 | `publishDecoded()` 收到未处理的 CAN ID |
| 参数开关 | `publish_unknown_frames=true`、`enable_debug_topics=true` 且 publisher key `debug_unknown_frames` 启用 |

消息内容格式由代码拼接：

```text
Unknown frame: id=<can_id> ext=<is_extended> ch=<channel>
```

## 5. 反馈 Topic

### 5.1 `/yunle_chassis/feedback/bms_status`

| 项目 | 内容 |
|---|---|
| 类型 | `chassis_interfaces/msg/BmsStatus` |
| 方向 | driver 发布 |
| CAN 报文 | `BMS_Status` |
| CAN ID | `0x100` / `256` |
| 协议文档周期 | 1000 ms |
| 当前代码触发 | 收到 CAN ID `256` 即发布 |
| 当前配置通道 | `message_channel_map` 中为 `can2`；当前代码未按该 map 过滤 RX 来源通道 |

字段映射：

| ROS 字段 | 类型 | CAN 信号 | 单位 | 精度/说明 |
|---|---|---|---|---|
| `stamp` | `builtin_interfaces/Time` | 无 | - | driver 发布时刻 |
| `bms_voltage` | `float32` | `BMS_Voltage` | V | DBC/code factor `0.1` |
| `bms_current` | `float32` | `BMS_Current` | A | DBC/code factor `0.1`；协议文档说明正值为充电、负值为放电 |
| `bms_soc` | `float32` | `BMS_SOC` | % | DBC/code factor `1` |

协议文档还列出 `CRC_16` 预留字段；当前 `Yunle_CAN_release.dbc`、`DbcProtocol` 和 `BmsStatus.msg` 不发布该字段。

### 5.2 `/yunle_chassis/feedback/vcu_warning_level`

| 项目 | 内容 |
|---|---|
| 类型 | `chassis_interfaces/msg/VcuWarningLevel` |
| 方向 | driver 发布 |
| CAN 报文 | `VCU_Warning_Level` |
| CAN ID | `0x77` / `119` |
| 协议文档周期 | 10 ms |
| 当前代码触发 | 收到 CAN ID `119` 即发布 |
| 当前配置通道 | `message_channel_map` 中为 `can2`；当前代码未按该 map 过滤 RX 来源通道 |

协议文档定义的警告等级：`0` 未检测到异常，`1` 一级警告，`2` 二级警告，`3` 三级警告，等级越高越严重。当前 msg 字段类型均为 `uint8`。

| ROS 字段 | CAN 信号 | bit | 说明 |
|---|---|---:|---|
| `stamp` | 无 | - | driver 发布时刻 |
| `bms_soc_warning` | `BMS_SOC_Warning` | `0|3` | 电池 SOC 警告 |
| `mcu_disconnect_warning` | `MCU_Disconnect_Warning` | `3|3` | 控制器掉线警告 |
| `mcu_motor_warning` | `MCU_Motor_Warning` | `6|3` | 电机故障警告 |
| `mcu_speed_warning` | `MCU_Speed_Warning` | `9|3` | 车辆超速警告 |
| `steering_disconnect_warning` | `Steering_Disconnect_Warning` | `12|3` | 转向掉线警告 |
| `steering_lock_warning` | `Steering_Lock_Warning` | `15|3` | 转向卡死/失效警告 |
| `steering_uncontrollable_warning` | `Steering_Uncontrollable_Warning` | `18|3` | 转向失控警告 |
| `steering_error_warning` | `Steering_Error_Warning` | `21|3` | 转向角故障警告 |
| `brake_error_warning` | `Brake_Error_Warning` | `24|3` | 刹车故障警告 |

### 5.3 `/yunle_chassis/feedback/wheel_speed`

| 项目 | 内容 |
|---|---|
| 类型 | `chassis_interfaces/msg/WheelSpeedFeedback` |
| 方向 | driver 发布 |
| CAN 报文 | `VCU_Wheel_Speed_Feedback` |
| CAN ID | `0x168` / `360` |
| 协议文档周期 | 10 ms |
| 当前代码触发 | 收到 CAN ID `360` 即发布 |
| 当前配置通道 | `message_channel_map` 中为 `can2`；协议文档写 CAN1/CAN2 均可能存在；当前代码未按 map 过滤 RX 来源通道 |

| ROS 字段 | CAN 信号 | bit | 单位 | 精度/说明 |
|---|---|---:|---|---|
| `stamp` | 无 | - | - | driver 发布时刻 |
| `wheel_speed_front_left_rpm` | `Wheel_Speed_Front_Left_RPM` | `0|16` | rpm | factor `0.1` |
| `wheel_speed_front_right_rpm` | `Wheel_Speed_Front_Right_RPM` | `16|16` | rpm | factor `0.1` |
| `wheel_speed_rear_left_rpm` | `Wheel_Speed_Rear_Left_RPM` | `32|16` | rpm | factor `0.1` |
| `wheel_speed_rear_right_rpm` | `Wheel_Speed_Rear_Right_RPM` | `48|16` | rpm | factor `0.1` |

协议文档给出的轮速转线速度公式：

```text
速度值(m/s) = 2 × 3.1415 × 轮胎半径(m) × 轮速值(rpm) ÷ 60
```

文档中给出的轮胎半径：JD 约 `0.13 m`，WD 约 `0.22 m`，TD 约 `0.24 m`。当前代码不进行轮速到 `m/s` 的换算，只发布 rpm。

### 5.4 `/yunle_chassis/feedback/ccu_status`

| 项目 | 内容 |
|---|---|
| 类型 | `chassis_interfaces/msg/CcuStatus` |
| 方向 | driver 发布 |
| CAN 报文 | `VCU_CCU_Status` |
| CAN ID | `0x51` / `81` |
| 协议文档周期 | 10 ms |
| 当前代码触发 | 收到 CAN ID `81` 即发布 |
| 当前配置通道 | `message_channel_map` 中为 `can2`；协议文档写 CAN1/CAN2 均可能存在；当前代码未按 map 过滤 RX 来源通道 |

| ROS 字段 | CAN 信号 | bit | 类型 | 说明 |
|---|---|---:|---|---|
| `stamp` | 无 | - | time | driver 发布时刻 |
| `ccu_shift_level_status` | `CCU_Shift_Level_Status` | `0|2` | `uint8` | 当前实际工作档位；协议文档：`0` 未检测/初始，`1` D，`2` N，`3` R |
| `ccu_parking_status` | `CCU_Parking_Status` | `2|1` | `bool` | P 档状态；`0` 释放，`1` 拉紧 |
| `ccu_ignition_status` | `CCU_Ignition_Status` | `3|2` | `uint8` | VCU 点火信号状态；协议文档中说明 `0` 异常，`1` 正常 |
| `ccu_drive_mode_shift_button` | `CCU_Drive_Mode_Shift_Button` | `5|1` | `bool` | 自动驾驶切换按钮状态 |
| `steering_wheel_direction` | `Steering_Wheel_Direction` | `7|1` | `bool` | 实际转向方向；协议文档：`0` 左，`1` 右 |
| `ccu_steering_wheel_angle` | `CCU_Steering_Wheel_Angle` | `8|12` | `float32` | 当前代码按 `编码值 / 120 × scu_control_max_steering_angle_deg` 换算为实际角度，并用 `Steering_Wheel_Direction` 决定正负号 |
| `ccu_vehicle_speed` | `CCU_Vehicle_Speed` | `20|9` | `float32` | 当前车速，单位 km/h，factor `0.1` |
| `ccu_drive_mode` | `CCU_Drive_Mode` | `29|3` | `uint8` | 当前驾驶模式；协议文档：`1` 自动驾驶，`3` 遥控器，`0/2` 预留 |
| `remote_brake_request_status` | `Remote_Brake_Request_Status` | `32|1` | `bool` | 遥控刹车 |
| `emergency_brake_request_status` | `Emergency_Brake_Request_Status` | `33|1` | `bool` | 紧急按钮刹车 |
| `scu_brake_signal_status` | `SCU_Brake_Signal_Status` | `34|1` | `bool` | 自动驾驶模式刹车 |
| `touch_brake_request_status` | `Touch_Brake_Request_Status` | `35|1` | `bool` | 防撞条刹车 |
| `handle_brake_request_status` | `Handle_Brake_Request_Status` | `36|1` | `bool` | 手柄刹车 |
| `handle_mode_flag_status` | `Handle_Mode_Flag_Status` | `37|1` | `bool` | 手柄模式切换标志位 |
| `left_turn_light_status` | `Left_Turn_Light_Status` | `56|1` | `bool` | 左转向灯状态 |
| `right_turn_light_status` | `Right_Turn_Light_Status` | `57|1` | `bool` | 右转向灯状态 |
| `position_light_status` | `Position_Light_Status` | `59|1` | `bool` | 驻车灯状态 |
| `low_beam_status` | `Low_Beam_Status` | `60|1` | `bool` | 近光灯状态；该字段来自当前 DBC/code/msg |

### 5.5 `/yunle_chassis/feedback/sas_angle`

| 项目 | 内容 |
|---|---|
| 类型 | `chassis_interfaces/msg/SasAngleFeedback` |
| 方向 | driver 发布 |
| CAN 报文 | `SAS_Angle_Feedback` |
| CAN ID | `0xE1` / `225` |
| 协议文档周期 | 10 ms |
| 当前代码触发 | 收到 CAN ID `225` 即发布 |
| 当前配置通道 | `message_channel_map` 中为 `can2`；当前代码未按该 map 过滤 RX 来源通道 |

| ROS 字段 | CAN 信号 | bit | 单位/语义 | 精度 |
|---|---|---:|---|---|
| `stamp` | 无 | - | driver 发布时刻 | - |
| `sas_front_angle` | `SAS_Front_Angle` | `0|16` | 前轮实际转角，单位 deg | 由编码值按最大转角换算 |
| `sas_rear_angle` | `SAS_Rear_Angle` | `24|16` | 后轮实际转角，单位 deg | 由编码值按最大转角换算 |

协议文档说明：如果需要实际车轮转向角，应按底盘最大转角换算：

```text
车轮角度 = 转向角度编码值 / 120 × 底盘最大转角
```

文档中给出的底盘最大转角：JD `24 deg`，WD `27 deg`，TD `25 deg`。当前代码使用配置项 `scu_control_max_steering_angle_deg` 执行该换算。

### 5.6 `/yunle_chassis/feedback/target_speed_feedback`

| 项目 | 内容 |
|---|---|
| 类型 | `chassis_interfaces/msg/ScuTargetSpeedFeedback` |
| 方向 | driver 发布 |
| CAN 报文 | `SCU_Target_Speed_Feedback` |
| CAN ID | `0x7F1` / `2033` |
| 协议文档周期 | 10 ms |
| 当前代码触发 | 收到 CAN ID `2033` 即发布 |
| 当前配置通道 | `message_channel_map` 中为 `can2`；当前代码未按该 map 过滤 RX 来源通道 |

| ROS 字段 | CAN 信号 | bit | 单位 | 精度/说明 |
|---|---|---:|---|---|
| `stamp` | 无 | - | - | driver 发布时刻 |
| `hardware_target_speed` | `Hardware_Target_Speed` | `0|16` | km/h | factor `0.1`；协议文档说明为硬件目标速度 |
| `scu_target_speed_feedback` | `SCU_Target_Speed_Feedback` | `16|16` | km/h | factor `0.1`；协议文档说明为自动驾驶模式下发目标速度 |
| `vehicle_target_speed` | `Vehicle_Target_Speed` | `32|16` | km/h | factor `0.1`；协议文档说明为目标速度 |
| `vehicle_target_speed_rpm` | `Vehicle_Target_Speed_RPM` | `48|16` | rpm | factor `0.1`；协议文档说明为目标轮速 |

## 6. 控制 Topic

### 6.1 `/yunle_chassis/control/scu_control_command`

| 项目 | 内容 |
|---|---|
| 类型 | `chassis_interfaces/msg/ScuControlCommand` |
| 方向 | 上层 ROS 节点发布，driver 订阅 |
| CAN 报文 | `SCU_Control_Command` |
| CAN ID | `0x121` / `289` |
| 协议文档周期 | 10 ms 持续发送 |
| 当前代码触发 | 每收到一条 ROS 消息，立即发送一次 CAN frame |
| 当前配置通道 | `control_message_channel_map` 中为 `can2` |

当前封装参数：

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `scu_control_max_steering_angle_deg` | `27.0` | 将 ROS 侧前/后转向角度转换为 0x121 8 bit 补码 raw 时使用的车辆最大转角 |
| `scu_control_max_target_speed_kmh` | `15.0` | 允许下发的最大目标速度；超出 `[0, max]` 的值会记录 warning 并按 0 下发 |

字段说明：

| ROS 字段/常量 | 类型 | CAN 信号 | bit | ROS 侧含义与当前处理 |
|---|---|---|---:|---|
| `SHIFT_LEVEL_D=1` | constant | - | - | D 档 |
| `SHIFT_LEVEL_N=2` | constant | - | - | N 档 |
| `SHIFT_LEVEL_R=3` | constant | - | - | R 档 |
| `scu_shift_level_request` | `uint8` | `SCU_Shift_Level_Request` | `0|2` | 当前代码只接受 `1=D`、`2=N`、`3=R`；其他值拒发本帧 |
| 无 ROS 输入 | - | `SCU_Drive_Mode_Request` | `6|2` | 当前代码固定下发 `1`，即自动驾驶模式请求 |
| `scu_steering_angle_front` | `float32` | `SCU_Steering_Angle_Front` | `8|8` | ROS 侧单位 deg；有效范围内转换为 8 bit 补码 raw；非有限数或超范围值记录 warning 并按 0 下发 |
| `scu_steering_angle_rear` | `float32` | `SCU_Steering_Angle_Rear` | `16|8` | ROS 侧单位 deg；有效范围内转换为 8 bit 补码 raw；非有限数或超范围值记录 warning 并按 0 下发 |
| `scu_target_speed` | `float32` | `SCU_Target_Speed` | `24|9` | ROS 侧单位 km/h；只接受 `[0, max]`，非有限数、负值或超范围值记录 warning 并按 0 下发 |
| `scu_brake_enable` | `bool` | `SCU_Brake_Enable` | `33|1` | `true=1`，`false=0` |
| `gw_left_turn_light_request` | `uint8` | `GW_Left_Turn_Light_Request` | `40|2` | 协议文档：`0` 闭合，`1` 打开，`2/3` 预留 |
| `gw_right_turn_light_request` | `uint8` | `GW_Right_Turn_Light_Request` | `42|2` | 协议文档：`0` 闭合，`1` 打开，`2/3` 预留 |
| `gw_position_light_request` | `uint8` | `GW_Position_Light_Request` | `46|2` | 协议文档：`0` 闭合，`1` 打开，`2/3` 预留 |
| `gw_low_beam_request` | `uint8` | `GW_Low_Beam_Request` | `48|2` | 协议文档：`0` 闭合，`1` 打开，`2/3` 预留 |
| `scu_torque_or_speed_mode` | `uint8` | `SCU_Torque_Or_Speed_Mode` | `58|1` | DBC/code 字段；当前代码直接编码 |
| `steering_angle_speed_valid` | `bool` | `Steering_Angle_Speed_Valid` | `60|1` | 协议文档说明为转向速度标志位，仅针对 TD、NWD 车型 |
| `brake_force_command_valid` | `bool` | `Brake_Force_Command_Valid` | `61|1` | 协议文档说明为刹车强度标志位，仅针对配备 EMB 制动车辆 |

转向角换算公式来自协议文档并已在当前代码中实现：

```text
raw_signed = round(clamp(angle_deg, -max_angle_deg, +max_angle_deg) / max_angle_deg * 120)
if raw_signed < 0:
    CAN_raw = 256 + raw_signed
else:
    CAN_raw = raw_signed
```

当前代码不会自动以 10 ms 周期重复发送上一条 0x121 命令；需要上层控制节点按所需频率持续发布，或后续在 driver 内增加周期发送机制。

### 6.2 `/yunle_chassis/control/scu_chassis_command`

| 项目 | 内容 |
|---|---|
| 类型 | `chassis_interfaces/msg/ScuChassisCommand` |
| 方向 | 上层 ROS 节点发布，driver 订阅 |
| CAN 报文 | `SCU_Chassis_Command` |
| CAN ID | `0x126` / `294` |
| 协议文档章节 | “5.2 刹车强度和转向速度调节（0x121 / 0x126）” |
| 当前代码触发 | 每收到一条 ROS 消息，立即发送一次 CAN frame |
| 当前配置通道 | `control_message_channel_map` 中为 `can2` |

| ROS 字段 | 类型 | CAN 信号 | bit | 单位 | 当前处理 |
|---|---|---|---:|---|---|
| `vcu_target_steering_angle_speed` | `float32` | `VCU_Target_Steering_Angle_Speed` | `0|16` | deg/s | DBC/code 范围 `126-525`，`encodeSignal()` 默认 clamp |
| `brake_force_front_left` | `float32` | `Brake_Force_Front_Left` | `16|8` | % | DBC/code 范围 `0-100`，默认 clamp |
| `brake_force_front_right` | `float32` | `Brake_Force_Front_Right` | `24|8` | % | DBC/code 范围 `0-100`，默认 clamp |
| `brake_force_rear_left` | `float32` | `Brake_Force_Rear_Left` | `32|8` | % | DBC/code 范围 `0-100`，默认 clamp |
| `brake_force_rear_right` | `float32` | `Brake_Force_Rear_Right` | `40|8` | % | DBC/code 范围 `0-100`，默认 clamp |

协议文档说明该报文适用于 WD 和 TD 系列底盘的 EMB 制动和可调速转向电机。当前代码不根据车型自动启用或禁用该 topic。

### 6.3 `/yunle_chassis/control/scu_torque_command`

| 项目 | 内容 |
|---|---|
| 类型 | `chassis_interfaces/msg/ScuTorqueCommand` |
| 方向 | 上层 ROS 节点发布，driver 订阅 |
| CAN 报文 | `SCU_Torque_Command` |
| CAN ID | `0x123` / `291` |
| 当前代码触发 | 每收到一条 ROS 消息，立即发送一次 CAN frame |
| 当前配置通道 | `control_message_channel_map` 中为 `can2` |

| ROS 字段 | 类型 | CAN 信号 | bit | 单位 | 当前处理 |
|---|---|---|---:|---|---|
| `torque_command_front_left` | `float32` | `Torque_Command_Front_Left` | `0|16` | Nm | DBC/code factor `0.1`，范围 `-3276.8` 到 `3276.7`，默认 clamp |
| `torque_command_front_right` | `float32` | `Torque_Command_Front_Right` | `16|16` | Nm | DBC/code factor `0.1`，范围 `-3276.8` 到 `3276.7`，默认 clamp |
| `torque_command_rear_left` | `float32` | `Torque_Command_Rear_Left` | `32|16` | Nm | DBC/code factor `0.1`，范围 `-3276.8` 到 `3276.7`，默认 clamp |
| `torque_command_rear_right` | `float32` | `Torque_Command_Rear_Right` | `48|16` | Nm | DBC/code factor `0.1`，范围 `-3276.8` 到 `3276.7`，默认 clamp |

该控制 topic 的语义来自当前 `Yunle_CAN_release.dbc` 与代码映射；在已读取的 word 文档片段中未发现对 0x123 的详细章节说明。

### 6.4 `/yunle_chassis/control/vcu_chassis_debug`

| 项目 | 内容 |
|---|---|
| 类型 | `chassis_interfaces/msg/VcuChassisDebug` |
| 方向 | 上层 ROS 节点发布，driver 订阅 |
| CAN 输出 | 收到一条 ROS 消息后，当前代码先发送 `0x710`，再发送 `0x715` |
| 当前配置通道 | `control_message_channel_map` 中 `VCU_Debug_Enable` 和 `VCU_Drive_Debug` 均为 `can2` |
| 协议文档章节 | “5.1 台架试验 PID 参数控制（0x710 / 0x715）” |

| ROS 字段 | 类型 | CAN 报文 | CAN 信号 | bit | 当前处理 |
|---|---|---|---|---:|---|
| `pid_debug_enable` | `bool` | `VCU_Debug_Enable` / `0x710` | `PID_Debug_Enable` | 当前代码/DBC 为 `2|1` | `true=1`，`false=0` |
| `velocity_kp` | `float32` | `VCU_Drive_Debug` / `0x715` | `Velocity_Kp` | `34|10` | DBC/code factor `0.1`，范围 `0-102.3`，默认 clamp |
| `velocity_ki` | `float32` | `VCU_Drive_Debug` / `0x715` | `Velocity_Ki` | `44|10` | DBC/code factor `0.01`，范围 `0-10.23`，默认 clamp |
| `velocity_kd` | `float32` | `VCU_Drive_Debug` / `0x715` | `Velocity_Kd` | `54|10` | DBC/code factor `0.01`，范围 `0-10.23`，默认 clamp |

协议文档提示该功能仅供专业技术人员在安全环境下调试。当前代码不会对 PID 参数做除 DBC 范围 clamp 以外的安全策略。

## 7. 当前接口边界

以下内容是当前代码状态，不是协议能力的完整声明：

| 项目 | 当前状态 |
|---|---|
| 0x121 周期发送 | 未实现；当前由 `/control/scu_control_command` callback 触发一次发送 |
| 控制超时保护 | 未发现 driver 侧 watchdog 或超时停车逻辑 |
| `/cmd_vel` | 未实现 |
| Odometry / TF | 未实现 |
| service / action | 未实现 |
| RX 通道过滤 | `message_channel_map` 已加载，但当前 `publishDecoded()` 未按该 map 过滤来源 CAN 通道 |
| `debug/status` | 已删除；该 topic 原先没有发布路径 |

## 8. 常用命令示例

以下示例只展示 topic 消息结构；实际车辆联调前应确认急停、自动驾驶切换开关、车辆周围环境和通信链路状态。

发布 0x121 自动驾驶 D 档、3 km/h、零转角：

```bash
ros2 topic pub /yunle_chassis/control/scu_control_command chassis_interfaces/msg/ScuControlCommand "{
  scu_shift_level_request: 1,
  scu_steering_angle_front: 0.0,
  scu_steering_angle_rear: 0.0,
  scu_target_speed: 3.0,
  scu_brake_enable: false,
  gw_left_turn_light_request: 0,
  gw_right_turn_light_request: 0,
  gw_position_light_request: 0,
  gw_low_beam_request: 0,
  scu_torque_or_speed_mode: 1,
  steering_angle_speed_valid: false,
  brake_force_command_valid: false
}"
```

发布 0x121 刹车请求：

```bash
ros2 topic pub /yunle_chassis/control/scu_control_command chassis_interfaces/msg/ScuControlCommand "{
  scu_shift_level_request: 2,
  scu_steering_angle_front: 0.0,
  scu_steering_angle_rear: 0.0,
  scu_target_speed: 0.0,
  scu_brake_enable: true,
  gw_left_turn_light_request: 0,
  gw_right_turn_light_request: 0,
  gw_position_light_request: 0,
  gw_low_beam_request: 0,
  scu_torque_or_speed_mode: 1,
  steering_angle_speed_valid: false,
  brake_force_command_valid: false
}"
```

查看 raw TX：

```bash
ros2 topic echo /yunle_chassis/can_tx/raw
```

查看车辆状态反馈：

```bash
ros2 topic echo /yunle_chassis/feedback/ccu_status
```
