# 当前项目不足分析（ROS1 分支）

本文档基于当前 `ros1` 分支源码、配置、消息定义和文档进行静态分析。未在真实 ROS1/catkin 环境、底盘硬件或以太网转 CAN 网关上运行验证。

## 1. 控制安全机制不足

当前驱动只在收到 ROS 控制 topic 时立即发送对应 CAN 报文，没有独立控制周期和控制超时保护。

主要风险：

- 没有上层控制指令超时后的自动停车逻辑。
- 没有网关断线后的自动停车或安全降级逻辑。
- 没有独立急停 topic。
- 没有控制报文周期发送 timer，发送频率完全依赖上游 `rostopic` 或控制节点发布频率。

建议：

- 增加控制指令最后更新时间戳。
- 增加安全定时器，超时后发送 0 速度、制动或停车报文。
- 增加急停 topic 或 service，并明确急停优先级。
- 将 0x121 等关键控制报文改为稳定周期发送。

## 2. 网关通信健壮性不足

当前 UDP 通道只在节点启动时初始化，运行过程中没有重连机制。

主要风险：

- `sendto()` 或 `recvfrom()` 异常后只记录或静默失败，不重新打开 socket。
- RX 数据来源没有校验远端 IP/端口。
- UDP payload 只按 13 字节记录切分，没有帧头、帧尾、校验、计数器或时间戳。

建议：

- 增加 socket 错误计数和重连状态机。
- 增加来源地址校验。
- 如网关支持更完整帧格式，建议抽象 transport codec 并显式处理帧头/校验/计数器。

## 3. DBC 协议维护成本较高

运行时不解析 DBC，所有报文和信号定义硬编码在 `chassis_driver/src/dbc_protocol.cpp`。

主要风险：

- DBC 更新后需要人工同步 C++ 映射。
- CAN ID、bit 位、缩放系数、范围、单位全部是硬编码。
- 缺少自动对照 DBC 的测试。

建议：

- 增加 DBC 到 C++ 映射的生成脚本，或增加 DBC 一致性检查工具。
- 为 `DbcProtocol::encodeSignal()` / `decodeSignal()` 增加单元测试。
- 每次修改 CAN 协议时同步更新 `docs/codex_project_analysis.md` 的协议映射。

## 4. ROS1 工程化能力不足

当前已迁移为 catkin 包，但仍缺少测试和诊断体系。

主要不足：

- 没有 `test/` 目录。
- 没有 gtest/rostest。
- 没有 diagnostics 发布。
- 没有 service 接口用于查询健康状态、重连、清故障或启停控制输出。

建议：

- 先为 `CanEthernetCodec` 和 `DbcProtocol` 增加 gtest。
- 再增加使用模拟 UDP payload 的 rostest。
- 增加 `/diagnostics` 或自定义健康状态 topic。

## 5. 线程模型需要实测

当前两个 RX 线程会直接调用 ROS publisher，控制 callback 会直接执行 UDP 发送。

风险：

- 高频反馈下，两个 RX 线程同时发布可能造成调度抖动。
- 控制 callback 内执行 UDP send，若系统网络栈异常可能影响 callback 响应时间。
- 没有独立 TX 队列或发送线程。

建议：

- 在目标 ROS1 环境下压测发布频率和 CPU 占用。
- 如控制实时性要求较高，增加独立 TX 线程和非阻塞发送队列。
- 对高频 warning 使用节流日志，避免日志刷屏。

## 6. 功能完整性不足

当前项目没有：

- TF 发布。
- `nav_msgs/Odometry` 发布。
- 轮速到里程计的积分逻辑。
- 参数动态重配置。
- 多车型配置模板。

建议：

- 若导航栈需要，增加 odom/TF 发布模块。
- 将车型最大转角、速度上限、通道映射等参数整理为多车型 YAML。
- 增加运行前配置校验，避免 IP、端口或通道映射错误导致实车风险。

## 7. 当前需要补充验证

在 ROS1 Noetic/Linux 环境中建议执行：

```bash
source /opt/ros/noetic/setup.bash
catkin_make
rostest 或 catkin_make run_tests
roslaunch chassis_driver chassis_driver.launch
```

当前 PowerShell 环境没有 ROS1/catkin，因此本轮无法完成真实构建和运行验证。
