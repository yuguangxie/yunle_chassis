# AGENTS.md

本项目当前 `ros1` 分支是一个 ROS1 移动机器人底盘驱动。驱动通过以太网转 CAN 网关与底盘通信，将 CAN 帧映射为 ROS1 topic，并将 ROS1 控制 topic 映射回 CAN 帧。

修改代码前，必须先阅读 `docs/codex_project_analysis.md`。

扩展 CAN 协议时，必须在同一次修改中更新 `docs/codex_project_analysis.md` 里的 CAN 协议映射表。

新增 topic、参数、CAN ID、消息类型、launch 行为或网关配置时，必须同步更新相关文档和 YAML 注释。

修改控制链路时，必须关注指令 clamp、控制超时、急停/制动、连接丢失行为和线程安全。

每次代码修改后，应在 ROS1/catkin 环境可用时运行 `catkin_make` 或相关 catkin 构建/测试命令。如果无法运行测试，必须记录原因。

网关 transport framing 应与 DBC 信号编解码、ROS topic 映射保持分离。
