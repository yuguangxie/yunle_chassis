# Yunle Chassis ROS1 Driver

本仓库已迁移为 ROS1（catkin + roscpp）版本，提供单节点 C++17 CAN-over-Ethernet 底盘驱动。

## Packages

- `chassis_interfaces`: 自定义 `.msg` 消息定义。
- `chassis_driver`: 主节点 `chassis_driver_node` 与辅助模块：
  - `UdpChannel`
  - `CanEthernetCodec`
  - `DbcProtocol`
  - `FrameRouter`
  - `ControlCommandBridge`

## Build

```bash
catkin_make
source devel/setup.bash
```

## Run

```bash
roslaunch chassis_driver chassis_driver.launch
```

## Config

统一参数文件：

- `chassis_driver/config/chassis_driver.yaml`

## Topics (默认前缀 `/yunle_chassis`)

### Subscribed control topics
- `/yunle_chassis/control/scu_control_command`
- `/yunle_chassis/control/scu_chassis_command`
- `/yunle_chassis/control/scu_torque_command`

### Published feedback topics
- `/yunle_chassis/feedback/bms_status`
- `/yunle_chassis/feedback/bms_realtime_status`
- `/yunle_chassis/feedback/vcu_warning_level`
- `/yunle_chassis/feedback/wheel_speed`
- `/yunle_chassis/feedback/ccu_status`
- `/yunle_chassis/feedback/sas_angle`
- `/yunle_chassis/feedback/target_speed_feedback`
