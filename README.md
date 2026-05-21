# 云乐线控底盘 ROS1 驱动使用说明

本项目是一个 ROS1 C++ 底盘驱动工作空间，用于通过以太网转 CAN 网关与云乐线控底盘通信。驱动节点接收底盘 CAN 反馈报文并发布为 ROS topic，也订阅 ROS 控制 topic，将控制指令编码为 CAN 报文后通过 UDP 下发到底盘。

当前代码中的 CAN 协议定义与 `Yunle_CAN_release.dbc` 对齐。运行时不解析 DBC 文件，DBC 报文和信号定义已经固化在 `chassis_driver/src/dbc_protocol.cpp` 中。

## 1. 项目组成

```text
yunle_chassis/
  chassis_interfaces/          ROS1 自定义消息包
    msg/                       底盘反馈、控制指令、原始 CAN 帧消息定义
  chassis_driver/              ROS1 C++ 底盘驱动包
    config/                    节点、话题、UDP 网关和控制封装参数
    launch/                    ROS1 XML launch 文件
    include/chassis_driver/    驱动节点、UDP、CAN 编解码、DBC 协议声明
    src/                       驱动节点、UDP、CAN 编解码、DBC 协议实现
  Yunle_CAN_release.dbc        当前协议参考 DBC
  docs/                        项目分析、话题说明和协议适配说明
```

| 包名 | 作用 |
|---|---|
| `chassis_interfaces` | 定义 ROS1 自定义消息，例如控制指令、底盘状态、原始 CAN 帧。 |
| `chassis_driver` | 提供单节点 `chassis_driver_node`，完成 UDP CAN 网关通信、DBC 信号编解码、ROS topic 发布和订阅。 |

## 2. 运行环境

推荐环境：

- ROS1 Noetic
- Ubuntu 20.04 或兼容 Linux 环境
- C++17 编译器
- `catkin_make` 或 `catkin build`

当前 UDP 通信实现使用 POSIX socket 头文件，例如 `arpa/inet.h`、`sys/socket.h`、`unistd.h`，因此不建议直接在原生 Windows 环境编译。

## 3. 构建

把本仓库放在 catkin 工作空间的 `src` 目录下，例如：

```text
catkin_ws/
  src/
    yunle_chassis/
      chassis_interfaces/
      chassis_driver/
```

使用 `catkin_make` 构建：

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

也可以只构建相关包：

```bash
catkin_make --pkg chassis_interfaces chassis_driver
```

## 4. 启动节点

使用默认配置启动：

```bash
roslaunch chassis_driver chassis_driver.launch
```

启动文件会加载：

```text
chassis_driver/config/chassis_driver.yaml
```

默认节点名称：

```text
/chassis_driver_node
```

默认话题前缀：

```text
/yunle_chassis
```

## 5. 配置文件说明

配置文件路径：

```text
chassis_driver/config/chassis_driver.yaml
```

该 YAML 在 ROS1 launch 中作为 `chassis_driver_node` 的私有参数加载，因此文件内容是扁平参数结构。

### 5.1 话题与调试参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `topic_prefix` | `/yunle_chassis` | 所有发布和订阅话题的统一前缀。 |
| `publish_raw_can` | `true` | 是否发布接收和发送方向的原始 CAN 帧。 |
| `publish_unknown_frames` | `true` | 是否发布未匹配到 DBC 映射的未知 CAN 帧。 |
| `enable_debug_topics` | `true` | 是否启用调试类话题。 |
| `log_control_can_frames` | `false` | 是否在节点日志中打印成功下发到底盘的控制 CAN 帧十六进制内容。 |
| `default_qos_depth` | `10` | ROS1 publisher/subscriber 队列长度。 |
| `enabled_publish_topics` | `["all"]` | 发布话题开关。使用 `all` 表示启用全部发布话题，也可以只列出需要启用的 key。 |
| `enabled_subscribe_topics` | `["all"]` | 订阅话题开关。使用 `all` 表示启用全部订阅话题，也可以只列出需要启用的 key。 |

### 5.2 CAN 通道映射

`message_channel_map` 用于记录反馈报文来源通道：

```yaml
message_channel_map:
  - "BMS_Status:can2"
  - "VCU_Warning_Level:can2"
  - "VCU_Wheel_Speed_Feedback:can2"
  - "VCU_CCU_Status:can2"
  - "SAS_Angle_Feedback:can2"
  - "SCU_Target_Speed_Feedback:can2"
```

`control_message_channel_map` 用于决定控制报文从哪个 CAN 通道发出：

```yaml
control_message_channel_map:
  - "SCU_Control_Command:can2"
  - "SCU_Chassis_Command:can2"
  - "SCU_Torque_Command:can2"
  - "VCU_Debug_Enable:can2"
  - "VCU_Drive_Debug:can2"
```

控制报文发送时会严格查找 `control_message_channel_map`。如果缺少必需控制报文映射，节点启动会失败。

### 5.3 UDP 网关参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `local_ip` | `192.168.1.102` | 本机绑定 IP。 |
| `can1_local_port` | `8234` | CAN1 的本地 UDP 端口。 |
| `can2_local_port` | `8235` | CAN2 的本地 UDP 端口。 |
| `can1_remote_ip` | `192.168.1.98` | CAN1 网关远端 IP。 |
| `can2_remote_ip` | `192.168.1.99` | CAN2 网关远端 IP。 |
| `remote_port` | `1234` | 网关远端 UDP 端口。 |
| `udp_buffer_size` | `2048` | UDP 接收缓冲区大小，单位 byte。 |
| `socket_timeout_ms` | `200` | UDP socket 接收超时时间，单位 ms。 |

### 5.4 SCU 控制封装参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `scu_control_max_steering_angle_deg` | `27.0` | 车辆实际最大转角，用于把 `/control/scu_control_command` 中的前后轮转角从度数换算为 0x121 报文 raw 值。 |
| `scu_control_max_target_speed_kmh` | `15.0` | `/control/scu_control_command` 目标速度最大值，单位 km/h。非有限数、负值或超出 `[0, max]` 的速度会记录 warning 并按 0 下发。 |

## 6. 以太网转 CAN 通信格式

当前驱动通过 UDP 与以太网转 CAN 网关通信，每个 CAN 通道对应一个 UDP socket。

每条 CAN 帧在 UDP payload 中使用固定 13 字节记录：

| 字节 | 含义 |
|---:|---|
| 0 | 信息字节。`0x80` 表示扩展帧，`0x40` 表示远程帧，低 4 bit 为 DLC。发送时会额外置位 `0x20`。 |
| 1-4 | CAN ID，网关传输格式为大端序。标准帧 ID 会被裁剪到 `0x7FF`。 |
| 5-12 | CAN data，共 8 byte。 |

一个 UDP 包可以包含多条 13 字节 CAN 记录。如果 UDP payload 长度不是 13 的整数倍，尾部多余字节会被丢弃并输出 warning。

## 7. 发布话题

以下话题名称基于默认 `topic_prefix=/yunle_chassis`。

### 7.1 原始 CAN 与调试话题

| 话题 | 消息类型 | 触发方式 | 说明 |
|---|---|---|---|
| `/yunle_chassis/can_rx/raw` | `chassis_interfaces/CanFrame` | 每次收到并解析 UDP CAN 记录 | 发布接收方向原始 CAN 帧。 |
| `/yunle_chassis/can_tx/raw` | `chassis_interfaces/CanFrame` | 每次控制 CAN 帧成功发送 | 发布发送方向原始 CAN 帧。 |
| `/yunle_chassis/debug/unknown_frames` | `std_msgs/String` | 收到未实现解析的 CAN ID | 输出未知 CAN ID、扩展帧标志和通道号。 |

### 7.2 底盘反馈话题

| 话题 | 消息类型 | 来源 CAN ID | 说明 |
|---|---|---:|---|
| `/yunle_chassis/feedback/bms_status` | `chassis_interfaces/BmsStatus` | `0x100` / 256 | 电池电压、电流、SOC。 |
| `/yunle_chassis/feedback/vcu_warning_level` | `chassis_interfaces/VcuWarningLevel` | `0x077` / 119 | BMS、MCU、转向、制动等告警等级。 |
| `/yunle_chassis/feedback/wheel_speed` | `chassis_interfaces/WheelSpeedFeedback` | `0x168` / 360 | 四轮轮速，单位 rpm。 |
| `/yunle_chassis/feedback/ccu_status` | `chassis_interfaces/CcuStatus` | `0x051` / 81 | 档位、驻车、点火、车速、驾驶模式、制动请求、灯光状态等。 |
| `/yunle_chassis/feedback/sas_angle` | `chassis_interfaces/SasAngleFeedback` | `0x0E1` / 225 | 前后转角反馈，单位 deg。 |
| `/yunle_chassis/feedback/target_speed_feedback` | `chassis_interfaces/ScuTargetSpeedFeedback` | `0x7F1` / 2033 | 硬件目标速度、SCU 目标速度反馈、车辆目标速度和目标转速。 |

查看反馈示例：

```bash
rostopic echo /yunle_chassis/feedback/ccu_status
rostopic echo /yunle_chassis/feedback/wheel_speed
rostopic echo /yunle_chassis/can_rx/raw
```

## 8. 订阅控制话题

### 8.1 `/yunle_chassis/control/scu_control_command`

消息类型：

```text
chassis_interfaces/ScuControlCommand
```

对应 CAN 报文：

```text
SCU_Control_Command, CAN ID 0x121 / 289
```

| 字段 | 类型 | 说明 |
|---|---|---|
| `scu_shift_level_request` | `uint8` | 档位请求。只接受 `1=D`、`2=N`、`3=R`，其他值拒发本帧。 |
| `scu_steering_angle_front` | `float32` | 前轮目标转角，单位 deg。超出 `±scu_control_max_steering_angle_deg` 或非有限数时按 0 下发并输出 warning。 |
| `scu_steering_angle_rear` | `float32` | 后轮目标转角，单位 deg。超出 `±scu_control_max_steering_angle_deg` 或非有限数时按 0 下发并输出 warning。 |
| `scu_target_speed` | `float32` | 目标速度，单位 km/h。只接受 `[0, scu_control_max_target_speed_kmh]`，超范围或非有限数时按 0 下发并输出 warning。 |
| `scu_brake_enable` | `bool` | 制动使能。 |
| `gw_left_turn_light_request` | `uint8` | 左转向灯请求。 |
| `gw_right_turn_light_request` | `uint8` | 右转向灯请求。 |
| `gw_position_light_request` | `uint8` | 位置灯请求。 |
| `gw_low_beam_request` | `uint8` | 近光灯请求。 |
| `scu_torque_or_speed_mode` | `uint8` | 扭矩/速度模式。 |
| `steering_angle_speed_valid` | `bool` | 转向角速度有效位。 |
| `brake_force_command_valid` | `bool` | 制动力命令有效位。 |

注意：

- `SCU_Drive_Mode_Request` 不作为 ROS 输入字段暴露。
- 只要收到并发送 `/yunle_chassis/control/scu_control_command`，驱动就固定把 `SCU_Drive_Mode_Request` 下发为 `1`。
- 前进和后退方向由 `scu_shift_level_request` 的 D/R 档决定，速度字段本身应为非负值。

示例：

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

### 8.2 `/yunle_chassis/control/scu_chassis_command`

消息类型：`chassis_interfaces/ScuChassisCommand`

对应 CAN 报文：`SCU_Chassis_Command, CAN ID 0x126 / 294`

```bash
rostopic pub -1 /yunle_chassis/control/scu_chassis_command chassis_interfaces/ScuChassisCommand "{
  vcu_target_steering_angle_speed: 180.0,
  brake_force_front_left: 20.0,
  brake_force_front_right: 20.0,
  brake_force_rear_left: 10.0,
  brake_force_rear_right: 10.0
}"
```

### 8.3 `/yunle_chassis/control/scu_torque_command`

消息类型：`chassis_interfaces/ScuTorqueCommand`

对应 CAN 报文：`SCU_Torque_Command, CAN ID 0x123 / 291`

```bash
rostopic pub -1 /yunle_chassis/control/scu_torque_command chassis_interfaces/ScuTorqueCommand "{
  torque_command_front_left: 100.0,
  torque_command_front_right: 100.0,
  torque_command_rear_left: 80.0,
  torque_command_rear_right: 80.0
}"
```

### 8.4 `/yunle_chassis/control/vcu_chassis_debug`

消息类型：`chassis_interfaces/VcuChassisDebug`

对应 CAN 报文：

```text
VCU_Debug_Enable, CAN ID 0x710 / 1808
VCU_Drive_Debug,  CAN ID 0x715 / 1813
```

该 ROS 话题会被驱动拆分并下发为两个 CAN 报文。

```bash
rostopic pub -1 /yunle_chassis/control/vcu_chassis_debug chassis_interfaces/VcuChassisDebug "{
  pid_debug_enable: true,
  velocity_kp: 10.0,
  velocity_ki: 0.5,
  velocity_kd: 0.2
}"
```

## 9. 原始 CAN 帧消息

`/yunle_chassis/can_rx/raw` 和 `/yunle_chassis/can_tx/raw` 使用：

```text
chassis_interfaces/CanFrame
```

| 字段 | 说明 |
|---|---|
| `stamp` | ROS1 `time` 时间戳。 |
| `can_id` | CAN ID。 |
| `is_extended` | 是否为扩展帧。 |
| `is_remote` | 是否为远程帧。 |
| `dlc` | CAN DLC。 |
| `data` | 8 字节 CAN 数据。 |
| `channel` | 逻辑 CAN 通道，`1` 或 `2`。 |

## 10. 控制 CAN 十六进制日志

如果需要在节点日志中查看实际下发到底盘的控制 CAN 帧，可以在配置文件中设置：

```yaml
log_control_can_frames: true
```

当控制 CAN 帧成功发送后，节点日志会输出类似内容：

```text
TX control CAN: SCU_Control_Command can2 id=0x121 dlc=8 data=[...]
```

## 11. 当前安全与实时性说明

当前驱动已经具备：

- DBC 信号编码时的范围限制。
- `/yunle_chassis/control/scu_control_command` 中档位合法性检查。
- `/yunle_chassis/control/scu_control_command` 中速度和转角超范围归零并输出 warning。
- 发送路径互斥锁保护。
- 两个 UDP CAN 通道的独立接收线程。
- UDP 接收超时，便于节点退出。

当前驱动未实现：

- 控制指令超时自动停车。
- 连接断开后自动停车或自动重连。
- 控制报文周期发送定时器。
- 独立急停 topic。
- 底盘反馈丢帧或超时检测。
- TF 或 `nav_msgs/Odometry` 里程计发布。

## 12. 常用排查命令

查看节点：

```bash
rosnode list
rosnode info /chassis_driver_node
```

查看话题：

```bash
rostopic list
rostopic echo /yunle_chassis/can_rx/raw
rostopic echo /yunle_chassis/debug/unknown_frames
```

查看参数：

```bash
rosparam list | grep chassis_driver_node
rosparam get /chassis_driver_node/topic_prefix
```

检查网关 UDP 端口：

```bash
ss -lunp | grep -E '8234|8235|1234'
```

## 13. 更多文档

- `docs/codex_project_analysis.md`：项目结构、通信链路、CAN 映射和扩展指南。
- `docs/codex_project_analysis_zh.md`：中文版项目分析。
- `docs/ros1_topic_reference.md`：ROS1 话题详细说明。
- `docs/scu_control_command_wrapper_2026.md`：`/yunle_chassis/control/scu_control_command` 与 2026 协议文档的封装说明和差异说明。
