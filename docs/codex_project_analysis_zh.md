# Yunle Chassis ROS1 项目分析

> 范围：当前 `ros1` 分支的持久上下文。CAN 信号映射仍与 `Yunle_CAN_release.dbc` 对齐。

## 1. 一句话概述

本分支是一个 ROS1 catkin C++17 底盘驱动工作空间，包含一个自定义消息包和一个 `roscpp` 驱动节点，用于将 UDP 以太网转 CAN 数据映射为 ROS 话题，并将 ROS 控制话题映射回 CAN 报文。

## 2. 目录结构

```text
yunle_chassis/
  chassis_interfaces/          # ROS1 自定义消息包
    msg/                       # CanFrame、控制命令、反馈消息
    CMakeLists.txt             # catkin message_generation 配置
    package.xml                # catkin 包元数据
  chassis_driver/              # ROS1 C++ 驱动包
    include/chassis_driver/    # 节点、UDP、codec、DBC、路由声明
    src/                       # 节点实现、UDP socket、codec、DBC 映射
    launch/                    # ROS1 XML launch 文件
    config/                    # 扁平 ROS1 私有参数 YAML
  Yunle_CAN_release.dbc        # DBC 参考文件；运行时不解析
  README.md                    # ROS1 构建、运行、话题使用说明
  docs/ros1_topic_reference.md # ROS1 话题接口说明
```

## 3. 核心节点

- 可执行文件：`chassis_driver_node`
- 节点类：`chassis_driver::ChassisDriverNode`
- 源码入口：`chassis_driver/src/main.cpp`
- ROS API：`roscpp`
- 启动文件：`chassis_driver/launch/chassis_driver.launch`
- 配置文件：`chassis_driver/config/chassis_driver.yaml`
- 线程模型：CAN1/CAN2 两个后台 RX 线程；控制 topic callback 直接发送 CAN 帧，发送路径由 `tx_mutex_` 保护。

当前没有 service、action、TF、odom 或 lifecycle 机制。

## 4. 参数清单

| 参数 | 默认值 | 用途 |
|---|---:|---|
| `topic_prefix` | `/yunle_chassis` | 所有话题前缀。 |
| `publish_raw_can` | `true` | 启用原始 RX/TX CAN 帧话题。 |
| `publish_unknown_frames` | `true` | 启用未知 CAN 帧调试话题。 |
| `enable_debug_topics` | `true` | 启用调试类发布。 |
| `log_control_can_frames` | `false` | 成功发送控制 CAN 帧后按十六进制输出日志。 |
| `default_qos_depth` | `10` | ROS1 publisher/subscriber 队列长度。 |
| `enabled_publish_topics` | `["all"]` | 发布话题开关。 |
| `enabled_subscribe_topics` | `["all"]` | 订阅话题开关。 |
| `message_channel_map` | 反馈报文默认 `can2` | 反馈报文通道元数据。 |
| `control_message_channel_map` | 控制报文默认 `can2` | 控制报文发送通道路由。 |
| `local_ip` | `192.168.1.102` | UDP 本地绑定 IP。 |
| `can1_local_port` | `8234` | CAN1 本地 UDP 端口。 |
| `can2_local_port` | `8235` | CAN2 本地 UDP 端口。 |
| `can1_remote_ip` | `192.168.1.98` | CAN1 网关 IP。 |
| `can2_remote_ip` | `192.168.1.99` | CAN2 网关 IP。 |
| `remote_port` | `1234` | 网关 UDP 目标端口。 |
| `udp_buffer_size` | `2048` | UDP 接收缓冲区大小。 |
| `socket_timeout_ms` | `200` | UDP 接收超时时间。 |
| `scu_control_max_steering_angle_deg` | `27.0` | 0x121 转角换算最大物理角度。 |
| `scu_control_max_target_speed_kmh` | `15.0` | 0x121 目标速度上限；超范围按 0 下发。 |

## 5. ROS1 接口

详见 `docs/ros1_topic_reference.md`。

发布话题：

- `/yunle_chassis/can_rx/raw`
- `/yunle_chassis/can_tx/raw`
- `/yunle_chassis/debug/unknown_frames`
- `/yunle_chassis/feedback/bms_status`
- `/yunle_chassis/feedback/vcu_warning_level`
- `/yunle_chassis/feedback/wheel_speed`
- `/yunle_chassis/feedback/ccu_status`
- `/yunle_chassis/feedback/sas_angle`
- `/yunle_chassis/feedback/target_speed_feedback`

订阅话题：

- `/yunle_chassis/control/scu_control_command`
- `/yunle_chassis/control/scu_chassis_command`
- `/yunle_chassis/control/scu_torque_command`
- `/yunle_chassis/control/vcu_chassis_debug`

## 6. 通信链路

反馈链路：

```text
底盘 CAN -> 以太网转 CAN 网关 -> UDP socket -> CanEthernetCodec -> FrameRouter -> DbcProtocol -> ROS1 feedback topic
```

控制链路：

```text
ROS1 control topic -> ControlCommandBridge -> DbcProtocol -> sendControlFrame -> UDP socket -> 以太网转 CAN 网关 -> 底盘 CAN
```

## 7. CAN 协议映射

| CAN ID | 方向 | 报文 | ROS1 topic |
|---:|---|---|---|
| 0x051 / 81 | RX | `VCU_CCU_Status` | `/feedback/ccu_status` |
| 0x077 / 119 | RX | `VCU_Warning_Level` | `/feedback/vcu_warning_level` |
| 0x0E1 / 225 | RX | `SAS_Angle_Feedback` | `/feedback/sas_angle` |
| 0x100 / 256 | RX | `BMS_Status` | `/feedback/bms_status` |
| 0x168 / 360 | RX | `VCU_Wheel_Speed_Feedback` | `/feedback/wheel_speed` |
| 0x7F1 / 2033 | RX | `SCU_Target_Speed_Feedback` | `/feedback/target_speed_feedback` |
| 0x121 / 289 | TX | `SCU_Control_Command` | `/control/scu_control_command` |
| 0x123 / 291 | TX | `SCU_Torque_Command` | `/control/scu_torque_command` |
| 0x126 / 294 | TX | `SCU_Chassis_Command` | `/control/scu_chassis_command` |
| 0x710 / 1808 | TX | `VCU_Debug_Enable` | `/control/vcu_chassis_debug` |
| 0x715 / 1813 | TX | `VCU_Drive_Debug` | `/control/vcu_chassis_debug` |

## 8. 安全和实时性

已有机制：

- DBC 编码范围限制。
- 0x121 档位合法性检查。
- 0x121 转角/速度超范围 warning 并按 0 下发。
- 发送路径 mutex 保护。
- CAN1/CAN2 独立接收线程。

未实现机制：

- 控制超时自动停车。
- 网关断线自动停车或重连。
- 控制报文周期发送 timer。
- 独立急停 topic。
- 反馈新鲜度检测。
- TF 或里程计发布。

## 9. 构建命令

推荐在 ROS1 Noetic catkin 工作空间中执行：

```bash
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
roslaunch chassis_driver chassis_driver.launch
```

当前 Windows shell 没有 ROS1/catkin 环境，因此需要在 ROS1 环境中补充真实构建验证。
