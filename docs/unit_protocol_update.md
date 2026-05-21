# unit_protocol DBC 适配说明

## 1. 背景

本次适配将驱动中的硬编码 CAN 协议从旧的 `Yunle_CAN_NTD_release.dbc` 对齐到新的 `Yunle_CAN_release.dbc`。

运行时策略保持不变：驱动不会在运行时解析 DBC 文件，仍由 `chassis_driver/src/dbc_protocol.cpp` 中的静态协议表完成 CAN 信号编解码。

## 2. 新 DBC 差异

### BMS_Status，CAN ID 256

新 DBC 保持字段、bit 位置、缩放系数不变，但更新物理量范围：

| 信号 | bit | factor | offset | 新范围 | 单位 |
|---|---|---:|---:|---|---|
| `BMS_Voltage` | `0|16` | 0.1 | 0 | 0 到 100 | V |
| `BMS_Current` | `16|16` | 0.1 | 0 | -500 到 500 | A |
| `BMS_SOC` | `32|16` | 1 | 0 | 0 到 100 | % |

### VCU_Warning_Level，CAN ID 119

新 DBC 将旧 warning 字段集合替换为 9 个 warning 字段：

| 信号 | bit | factor | offset | 范围 |
|---|---|---:|---:|---|
| `BMS_SOC_Warning` | `0|3` | 1 | 0 | 0 到 7 |
| `MCU_Disconnect_Warning` | `3|3` | 1 | 0 | 0 到 7 |
| `MCU_Motor_Warning` | `6|3` | 1 | 0 | 0 到 7 |
| `MCU_Speed_Warning` | `9|3` | 1 | 0 | 0 到 7 |
| `Steering_Disconnect_Warning` | `12|3` | 1 | 0 | 0 到 7 |
| `Steering_Lock_Warning` | `15|3` | 1 | 0 | 0 到 7 |
| `Steering_Uncontrollable_Warning` | `18|3` | 1 | 0 | 0 到 7 |
| `Steering_Error_Warning` | `21|3` | 1 | 0 | 0 到 7 |
| `Brake_Error_Warning` | `24|3` | 1 | 0 | 0 到 7 |

旧驱动中存在但新 DBC 已移除的字段：

- `BMS_Charge_Current_Warning`
- `BMS_Discharge_Current_Warning`
- `BMS_Temperature_Warning`
- `MCU_Current_Warning`
- `MCU_Temperature_Warning`
- `MCU_Voltage_Warning`

### VCU_CCU_Status，CAN ID 81

新 DBC 在原有状态字段基础上新增 3 个布尔状态：

| 信号 | bit | factor | offset | 范围 |
|---|---|---:|---:|---|
| `Touch_Brake_Request_Status` | `35|1` | 1 | 0 | 0 到 1 |
| `Handle_Brake_Request_Status` | `36|1` | 1 | 0 | 0 到 1 |
| `Handle_Mode_Flag_Status` | `37|1` | 1 | 0 | 0 到 1 |

### DBC 周期属性

`Yunle_CAN_release.dbc` 中包含 `GenMsgCycleTime`：

| CAN ID | 消息 | DBC 周期 |
|---:|---|---:|
| 256 | `BMS_Status` | 100 ms |
| 119 | `VCU_Warning_Level` | 100 ms |
| 1808 | `VCU_Debug_Enable` | 100 ms |
| 1813 | `VCU_Drive_Debug` | 100 ms |
| 2033 | `SCU_Target_Speed_Feedback` | 20 ms |
| 225 | `SAS_Angle_Feedback` | 20 ms |
| 360 | `VCU_Wheel_Speed_Feedback` | 20 ms |
| 81 | `VCU_CCU_Status` | 20 ms |
| 291 | `SCU_Torque_Command` | 20 ms |
| 294 | `SCU_Chassis_Command` | 20 ms |
| 289 | `SCU_Control_Command` | 20 ms |

注意：当前驱动代码仍按 UDP 接收或 ROS topic callback 事件触发发布/发送，没有实现按 DBC 周期主动发送控制帧的 timer。

## 3. 代码修改

### `chassis_driver/src/dbc_protocol.cpp`

- 更新 `BMS_Status` 中 `BMS_Voltage` 与 `BMS_Current` 的 min/max 范围。
- 重写 CAN ID 119 `VCU_Warning_Level` 的信号列表，使 bit 位置和字段名与新 DBC 一致。
- 为 CAN ID 81 `VCU_CCU_Status` 增加：
  - `Touch_Brake_Request_Status`
  - `Handle_Brake_Request_Status`
  - `Handle_Mode_Flag_Status`

### `chassis_driver/src/chassis_driver_node.cpp`

- 更新 CAN ID 119 的 `publishDecoded()` 映射，发布新 warning 字段。
- 更新 CAN ID 81 的 `publishDecoded()` 映射，发布新增的 3 个 CCU 状态字段。

### `chassis_interfaces/msg/VcuWarningLevel.msg`

- 删除新 DBC 中不存在的旧 warning 字段。
- 增加新 DBC 中的：
  - `steering_error_warning`
  - `brake_error_warning`

新的 ROS 消息字段为：

```text
builtin_interfaces/Time stamp
uint8 bms_soc_warning
uint8 mcu_disconnect_warning
uint8 mcu_motor_warning
uint8 mcu_speed_warning
uint8 steering_disconnect_warning
uint8 steering_lock_warning
uint8 steering_uncontrollable_warning
uint8 steering_error_warning
uint8 brake_error_warning
```

### `chassis_interfaces/msg/CcuStatus.msg`

新增：

```text
bool touch_brake_request_status
bool handle_brake_request_status
bool handle_mode_flag_status
```

### 文档

- `README.md` 标注当前硬编码协议对齐 `Yunle_CAN_release.dbc`。
- `docs/codex_project_analysis.md` 与 `docs/codex_project_analysis_zh.md` 已同步更新 DBC 文件名、CAN 映射表和周期属性说明。

## 4. 未实现但仍存在于 DBC 的报文

以下报文存在于 `Yunle_CAN_release.dbc`，但本次没有新增运行时 topic 或控制接口：

| CAN ID | 消息名 | 说明 |
|---:|---|---|
| 1808 | `VCU_Debug_Enable` | 诊断/调试使能命令 |
| 1813 | `VCU_Drive_Debug` | 纵向速度环调试参数 |

原因：当前驱动原先也未实现这两个调试类报文。本次目标是让现有运行接口按新 DBC 对齐，不额外扩展新的调试 topic。

## 5. 构建与测试状态

当前运行环境没有 ROS2 和 colcon 系统环境，因此无法完成真实 ROS2 构建验证。

已知可执行检查：

- `git diff --check`
- 源码字段残留扫描
- DBC 与硬编码字段的人工对照

建议在 ROS2 Humble/Linux 或项目指定 ROS2 环境中继续执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select chassis_interfaces chassis_driver
colcon test --packages-select chassis_interfaces chassis_driver
```

## 6. 后续注意事项

- `VcuWarningLevel.msg` 删除了旧字段，这是接口级变更；依赖旧字段的上层节点需要同步更新。
- `CcuStatus.msg` 新增字段后，依赖该消息的上层节点需要重新编译。
- DBC 的 TX 报文周期为 20 ms，但当前驱动仍只在收到 ROS control topic 时发送控制帧。如果底盘要求严格周期控制，应后续增加控制缓存、周期发送 timer、控制超时停车和急停联动逻辑。
- `Yunle_CAN_release.ini` 是 DBC 编辑器/视图配置，当前运行时不使用。
