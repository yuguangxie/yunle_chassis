# ScuControlCommand 0x121 封装修改说明

生成日期：2026-05-21
依据文档：`docs/云乐线控底盘通信协议使用说明-2026.docx` 中“3 自动驾驶控制报文（0x121）”章节。
涉及 ROS1 topic：`/yunle_chassis/control/scu_control_command`
涉及 ROS1 msg：`chassis_interfaces/msg/ScuControlCommand.msg`
涉及 CAN 报文：`SCU_Control_Command`，CAN ID `0x121` / 十进制 `289`。

## 1. 本次封装后的 ROS 侧语义

`ScuControlCommand.msg` 现在被视为 0x121 的上层物理量封装，而不是所有字段都直接等同 CAN raw。

| ROS 字段 | 封装后含义 | CAN 信号 | 当前处理 |
|---|---|---|---|
| `scu_shift_level_request` | 档位请求 | `SCU_Shift_Level_Request` | 只接受 `1=D`、`2=N`、`3=R`；其他值拒发本帧。 |
| 无 ROS 输入 | 固定自动驾驶模式 | `SCU_Drive_Mode_Request` | 只要发送 `/control/scu_control_command`，驱动固定下发 `1`。 |
| `scu_steering_angle_front` | 前轮目标角度，单位 deg | `SCU_Steering_Angle_Front` | 有效范围内按配置最大转角换算为 8 位补码 raw；非有限数或超范围值记录 warning 并按 0 下发。 |
| `scu_steering_angle_rear` | 后轮目标角度，单位 deg | `SCU_Steering_Angle_Rear` | 有效范围内按配置最大转角换算为 8 位补码 raw；非有限数或超范围值记录 warning 并按 0 下发。 |
| `scu_target_speed` | 目标速度，单位 km/h | `SCU_Target_Speed` | 只接受 `[0, scu_control_max_target_speed_kmh]`；非有限数、负值或超范围值记录 warning 并按 0 下发。 |
| `scu_brake_enable` | 刹车使能 | `SCU_Brake_Enable` | 沿用原逻辑，`true=1`，`false=0`。 |
| 灯光请求字段 | 灯光请求 | `GW_*_Light_Request` | 沿用原逻辑，由 DBC 编码层按范围 clamp。 |
| `scu_torque_or_speed_mode` | 扭矩/速度模式 | `SCU_Torque_Or_Speed_Mode` | 沿用原逻辑。 |
| `steering_angle_speed_valid` | 转向速度标志位 | `Steering_Angle_Speed_Valid` | 沿用原逻辑。 |
| `brake_force_command_valid` | 刹车强度标志位 | `Brake_Force_Command_Valid` | 沿用原逻辑。 |

## 2. 新增配置参数

配置文件：`chassis_driver/config/chassis_driver.yaml`

| 参数 | 默认值 | 用途 |
|---|---:|---|
| `scu_control_max_steering_angle_deg` | `27.0` | 车辆实际最大转角，用于把 ROS 侧角度值换算为 0x121 的 8 位补码 raw。协议文档举例：JD 24 deg，WD 27 deg，TD 25 deg。 |
| `scu_control_max_target_speed_kmh` | `15.0` | `/control/scu_control_command` 接受的最大目标速度。速度超出 `[0, max]` 时会按 0 下发。 |

## 3. 转向角换算

协议文档说明：`SCU_Steering_Angle_Front` 和 `SCU_Steering_Angle_Rear` 是 8 bit raw，按补码解释；以最大转角 27 deg 为例，`±120 raw` 对应 `±27 deg`。

当前驱动封装公式：

```text
raw_signed = round(angle_deg / max_angle_deg * 120)
if raw_signed < 0:
    can_raw = 256 + raw_signed
else:
    can_raw = raw_signed
```

如果 ROS 输入角度不是有限数，或超出 `[-max_angle_deg, +max_angle_deg]`，驱动会输出 warning，并将该角度按 `0 deg` 下发。

示例，当 `scu_control_max_steering_angle_deg=27.0`：

| ROS 输入角度 | raw_signed | CAN raw 十进制 | CAN raw 十六进制 |
|---:|---:|---:|---|
| `+9 deg` | `40` | `40` | `0x28` |
| `-9 deg` | `-40` | `216` | `0xD8` |
| `+27 deg` | `120` | `120` | `0x78` |
| `-27 deg` | `-120` | `136` | `0x88` |

源码位置：`chassis_driver/src/control_command_bridge.cpp` 中 `steeringAngleDegToCanRaw()`。

## 4. 速度处理

当前驱动将 `scu_target_speed` 作为非负速度处理：

1. 如果速度在 `[0, scu_control_max_target_speed_kmh]` 范围内，则按该值下发。
2. 如果速度不是有限数、为负值或超过 `scu_control_max_target_speed_kmh`，则输出 warning，并将 `SCU_Target_Speed` 按 0 下发。
3. 前进/后退方向由 `scu_shift_level_request` 的 D/R 档决定，不由速度正负号决定。

源码位置：`chassis_driver/src/control_command_bridge.cpp` 中 `/control/scu_control_command` subscription callback。

## 5. 与协议文档或 DBC 的差异

以下差异需要后续联调时重点确认。

| 项目 | 当前用户要求/本次实现 | `云乐线控底盘通信协议使用说明-2026.docx` | `Yunle_CAN_release.dbc` / 原代码 | 说明 |
|---|---|---|---|---|
| 驾驶模式输入 | ROS 接口不再暴露 `scu_drive_mode_request`；驱动固定下发 `SCU_Drive_Mode_Request=1` | 章节 3.3 写明 `SCU_Drive_Mode_Request=1` 为自动驾驶模式请求，`3` 为遥控器模式请求，并称遥控器模式为默认模式请求 | DBC `VAL_ 289 SCU_Drive_Mode_Request` 当前写的是 `0 Manual`、`1 Remote`、`2 Auto`、`3 Reserved` | 文档与 DBC 枚举不一致。本次按用户要求固定下发 1。 |
| 档位取值 | 只接受 `1=D`、`2=N`、`3=R`，其他值拒发 | 字段表写 `0` 为预留；但 3.7 的刹车/转向示例 Byte0 为 `0x40`，实际 shift bits 为 `0` | DBC 范围是 `0-3`，枚举中 `0=Invalid`、`1=D`、`2=N`、`3=R` | 文档示例使用了预留档位值。本次按用户要求禁止 0。 |
| 转向字段类型 | ROS 侧输入真实角度 deg，驱动换算为补码 raw | 文档写 `Unsigned / 补码`，`±120 raw` 对应车型最大转角 | DBC/C++ 当前定义为 unsigned、factor 1、范围 `0-255` | DBC 只描述 raw 容器；补码语义由本次 ROS 封装层实现。 |
| 最大转角 | 可配置，默认 `27 deg` | 文档写 JD 24 deg、WD 27 deg、TD 25 deg | 原代码直接把 ROS 字段当 raw，未使用车型最大转角 | 本次默认取用户示例 27 deg，车型切换时应改 YAML。 |
| 目标速度最大值 | 可配置，默认 `15 km/h`；超范围按 0 下发并输出 warning | 文档写 JD/WD 出厂限速默认 10 km/h，TD 默认 15 km/h；字段范围仍为 `0-51 km/h` | DBC 范围是 `0-51.1 km/h`，原代码由 DBC clamp | 本次按用户要求，超范围值不再 clamp，也不拒发整帧，而是按 0 下发。 |
| 速度符号 | ROS 输入必须在 `[0, max]`；负值按超范围处理，下发 0 | 文档示例前进/后退均发送正速度 `3 km/h`，通过 D/R 档区分方向 | 原代码直接编码 ROS 输入，负数会被 unsigned DBC 信号 clamp 到 0 | 本次显式对负值输出 warning 并下发 0。 |
| 发送周期 | 本次未实现周期发送，仍由 topic callback 触发 | 文档多处要求 0x121 以 `10 ms` 周期持续发送，500 ms 超时切回手动 | DBC `GenMsgCycleTime` 对 0x121 写 `20 ms`，原代码事件触发发送 | 周期发送/超时保护属于控制链路改造，本次仅完成 topic 封装。 |

## 6. 涉及源码位置

| 文件 | 修改内容 |
|---|---|
| `chassis_interfaces/msg/ScuControlCommand.msg` | 删除 `scu_drive_mode_request` 输入，并更新字段语义注释。 |
| `chassis_driver/include/chassis_driver/chassis_driver_node.hpp` | 增加 0x121 封装参数成员。 |
| `chassis_driver/src/chassis_driver_node.cpp` | 声明、读取并校验新增参数。 |
| `chassis_driver/src/control_command_bridge.cpp` | 固定下发 `SCU_Drive_Mode_Request=1`，并将超范围转角/速度按 0 下发且输出 warning。 |
| `chassis_driver/config/chassis_driver.yaml` | 增加 0x121 封装参数及注释。 |

## 7. 当前未实现但协议要求的事项

本次需求聚焦 `ScuControlCommand.msg` 与 `/yunle_chassis/control/scu_control_command` 的封装，因此以下事项只记录，未在本轮实现：

1. 0x121 按 10 ms 周期持续发送。
2. 500 ms 控制超时后的驱动侧保护策略。
3. 切换 D/R 档前检查当前车速接近 0。
4. 自动驾驶切换按钮状态与 0x121 发送策略联动。
5. 结合 0x126 报文设置 WD/TD EMB 制动强度。
