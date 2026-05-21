# unit_protocol 调试报文适配说明

## 1. 背景

`Yunle_CAN_release.dbc` 中包含两条此前代码未实现的调试类发送报文：

| CAN ID | DBC 消息名 | 方向 | 周期属性 | 信号 |
|---:|---|---|---:|---|
| 1808 | `VCU_Debug_Enable` | TX | 100 ms | `PID_Debug_Enable` |
| 1813 | `VCU_Drive_Debug` | TX | 100 ms | `Velocity_Kp`, `Velocity_Ki`, `Velocity_Kd` |

按本次需求，这两条底盘发送指令在代码的 ROS 接口层整合为一个逻辑消息 `VCU_Chassis_Debug`。实际接口文件名为 `VcuChassisDebug.msg`，对应 topic 为 `/yunle_chassis/control/vcu_chassis_debug`。

## 2. 接口变更

新增消息文件：

```text
chassis_interfaces/msg/VcuChassisDebug.msg
```

字段：

```text
bool pid_debug_enable
float32 velocity_kp
float32 velocity_ki
float32 velocity_kd
```

新增订阅 topic：

```text
/yunle_chassis/control/vcu_chassis_debug
```

收到该 topic 后，驱动会发送两帧 CAN 报文：

1. CAN ID 1808 `VCU_Debug_Enable`：写入 `PID_Debug_Enable`。
2. CAN ID 1813 `VCU_Drive_Debug`：写入 `Velocity_Kp`、`Velocity_Ki`、`Velocity_Kd`。

## 3. 删除的 DBC 外报文

新 DBC 中没有旧代码里的 `BMS_Realtime_Status`，因此本次删除：

- `chassis_interfaces/msg/BmsRealtimeStatus.msg`
- `chassis_interfaces/CMakeLists.txt` 中的 `BmsRealtimeStatus.msg` 注册
- `chassis_driver/src/dbc_protocol.cpp` 中 CAN ID `2542813185` 的 `BMS_Realtime_Status` 定义
- `chassis_driver/src/chassis_driver_node.cpp` 中 `/feedback/bms_realtime_status` publisher 和 decode 分支
- `chassis_driver/config/chassis_driver.yaml` 中 `feedback_bms_realtime_status` 和 `BMS_Realtime_Status:can2`
- README 和分析文档中的对应 topic/协议映射

## 4. 已覆盖的新 DBC 报文

本次调整后，代码中的 DBC 报文集合与 `Yunle_CAN_release.dbc` 对齐：

| CAN ID | DBC 消息名 | 当前代码处理 |
|---:|---|---|
| 81 | `VCU_CCU_Status` | RX 解码并发布 `/feedback/ccu_status` |
| 119 | `VCU_Warning_Level` | RX 解码并发布 `/feedback/vcu_warning_level` |
| 225 | `SAS_Angle_Feedback` | RX 解码并发布 `/feedback/sas_angle` |
| 256 | `BMS_Status` | RX 解码并发布 `/feedback/bms_status` |
| 360 | `VCU_Wheel_Speed_Feedback` | RX 解码并发布 `/feedback/wheel_speed` |
| 1808 | `VCU_Debug_Enable` | TX，由 `/control/vcu_chassis_debug` 触发 |
| 1813 | `VCU_Drive_Debug` | TX，由 `/control/vcu_chassis_debug` 触发 |
| 2033 | `SCU_Target_Speed_Feedback` | RX 解码并发布 `/feedback/target_speed_feedback` |
| 289 | `SCU_Control_Command` | TX，由 `/control/scu_control_command` 触发 |
| 291 | `SCU_Torque_Command` | TX，由 `/control/scu_torque_command` 触发 |
| 294 | `SCU_Chassis_Command` | TX，由 `/control/scu_chassis_command` 触发 |

## 5. 注意事项

- `VCU_Chassis_Debug` 是 ROS 层的整合逻辑名；实际 `.msg` 类型名为 `VcuChassisDebug`。CAN 层仍按 DBC 中的 `VCU_Debug_Enable` 和 `VCU_Drive_Debug` 两个报文发送。
- 当前发送仍由 topic callback 触发，不会自动按 DBC 的 100 ms 周期重复发送。
- 当前环境没有 ROS1/catkin，因此需要在 ROS1 环境中补跑 `catkin_make` 和相关测试。
