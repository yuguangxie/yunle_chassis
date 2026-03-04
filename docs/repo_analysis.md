# Yunle Chassis 仓库结构图与功能解析

## 1. 仓库整体结构（逻辑视图）

```mermaid
flowchart TD
  A[chassis_interfaces 包] -->|定义消息| B[chassis_driver 包]
  B --> C[chassis_driver_node]
  C --> D[UdpChannel CAN1]
  C --> E[UdpChannel CAN2]
  D --> F[CAN-over-Ethernet 设备]
  E --> F

  C --> G[FrameRouter]
  G --> H[DBC 解析 DbcProtocol]
  H --> I[/chassis/feedback/*]

  J[/chassis/control/* 订阅] --> K[ControlCommandBridge]
  K --> H
  H --> L[sendControlFrame]
  L --> D
  L --> E

  C --> M[/chassis/can_rx/raw]
  C --> N[/chassis/can_tx/raw]
  C --> O[/chassis/debug/*]
```

## 2. 分层说明

### 2.1 接口层：`chassis_interfaces`
- 负责定义所有 ROS2 消息，包括底盘控制命令、状态反馈、原始 CAN 帧消息。
- 该层不包含业务逻辑，仅提供跨节点通信的数据契约。

### 2.2 驱动层：`chassis_driver`
- 通过 `ChassisDriverNode` 组织所有运行时行为。
- `UdpChannel` 提供 UDP 收发封装。
- `CanEthernetCodec` 负责 CAN 与以太网传输格式互转（13 字节记录格式）。
- `DbcProtocol` 负责信号级编码/解码（bit-level + factor/offset + signed/unsigned）。
- `ControlCommandBridge` 将控制话题消息转成 CAN 帧。
- `FrameRouter` 统一入口把收帧路由到解码发布流程。

## 3. 关键数据流

### 3.1 下行控制链路（ROS -> CAN）
1. 上层发布 `/chassis/control/*`。
2. `ControlCommandBridge` 回调中构造 `CanFrame`。
3. `DbcProtocol::encodeSignal` 将物理量写入 frame.data bit 域。
4. `ChassisDriverNode::sendControlFrame` 根据消息名选择通道。
5. `CanEthernetCodec::encodeFrame` 打包为 13 字节。
6. `UdpChannel::send` 发往对应网关。

### 3.2 上行反馈链路（CAN -> ROS）
1. `rxLoop` 从 UDP 通道收包。
2. `CanEthernetCodec::decodePayload` 解析多条 CAN 帧。
3. 原始帧发布到 `/chassis/can_rx/raw`（可配置）。
4. `FrameRouter::routeFrame` 转发到 `publishDecoded`。
5. `DbcProtocol::decodeSignal` 解码并组装具体反馈消息。
6. 发布到 `/chassis/feedback/*`。

## 4. 主要模块详解

### 4.1 `ChassisDriverNode`
- 参数管理：网络参数、QoS、调试开关、消息到通道映射。
- 资源管理：2 个 UDP 通道、2 个接收线程、发布器/订阅器生命周期。
- 线程模型：每个 CAN 通道独立 RX 循环，发送路径互斥保护。
- 可观测性：支持 raw rx/tx 与 unknown frame debug 话题。

### 4.2 `DbcProtocol`
- 内置静态消息定义（frame_id -> signal list）。
- 支持 Intel/Motorola 两种位序。
- 支持有符号扩展、比例系数、偏移、范围裁剪。
- 是该仓库信号解释正确性的核心组件。

### 4.3 `ControlCommandBridge`
- 将 3 类控制命令映射到固定 CAN ID：
  - `SCU_Control_Command`
  - `SCU_Chassis_Command`
  - `SCU_Torque_Command`
- 每条命令通过多个 `encodeSignal` 写入 8 字节载荷。

### 4.4 `CanEthernetCodec` + `UdpChannel`
- 前者关注协议格式，后者关注网络 I/O。
- 形成清晰边界：编码器不依赖 socket，网络层不关心 DBC。

## 5. 当前设计优点
- 模块边界清晰，职责相对单一。
- DBC 解析与节点逻辑解耦，便于后续替换外部 DBC 解析库。
- 同时保留原始帧与解码话题，利于现场排障。
- 控制与反馈方向映射独立配置，可适配双通道网络拓扑。

## 6. 改进建议（优先级）

### P0（建议尽快）
1. **补充自动化测试**：
   - `DbcProtocol` 的边界值（min/max、signed 溢出、Motorola 位序）。
   - `CanEthernetCodec` 对 trailing bytes、多帧拆分。
2. **内存安全优化**：
   - `UdpChannel::remote_addr_ptr_` 使用值语义或 `std::unique_ptr`，避免手动 `new/delete`。
3. **错误可观测性增强**：
   - socket 初始化失败原因上报到日志（`errno`）。

### P1（中期）
1. **topic_prefix 一致性**：当前发布/订阅 topic 字面量写死，建议统一由参数拼接。
2. **配置健壮性**：对 `message_channel_map` 的非法值、重复键做告警。
3. **线程退出时延优化**：在 `stopThreads` 中加入更明确的中断机制或 wakeup 机制。

### P2（长期）
1. **DBC 外部化**：从硬编码转为加载 `.dbc` 或预编译描述文件。
2. **插件化解码器**：按车型/版本动态切换消息映射。
3. **指标化监控**：增加帧计数、丢包率、解码失败率等统计指标。

## 7. 函数注释规范化执行说明
- 已为驱动包中带函数定义/声明的 C++/Python 文件补充规范化函数注释（Doxygen/Docstring 风格）。
- 覆盖范围包括：`*.hpp` 中 public/private 方法声明、`*.cpp` 中函数定义与关键静态函数、`launch` 文件中的入口函数。
