# Yunle Chassis ROS1 Project Analysis

> Scope: persistent context for the ROS1 branch of the Yunle CAN-over-Ethernet chassis driver. The CAN signal mapping is still aligned with `Yunle_CAN_release.dbc`.

## 1. One-line Summary

This branch is a ROS1 catkin C++17 chassis driver workspace with a custom message package and one `roscpp` node that bridges UDP CAN-over-Ethernet frames to ROS topics and ROS control topics back to CAN frames.

## 2. Repository Layout

```text
yunle_chassis/
  chassis_interfaces/          # ROS1 custom message package
    msg/                       # CanFrame, control commands, feedback messages
    CMakeLists.txt             # catkin message_generation setup
    package.xml                # catkin package metadata
  chassis_driver/              # ROS1 C++ driver package
    include/chassis_driver/    # node, UDP, codec, DBC, routing declarations
    src/                       # node implementation, UDP socket I/O, codec, DBC maps
    launch/                    # ROS1 XML launch file
    config/                    # flat ROS1 private-parameter YAML
    CMakeLists.txt             # catkin executable build
    package.xml                # roscpp/std_msgs/chassis_interfaces deps
  Yunle_CAN_release.dbc        # source DBC reference; runtime code does not parse it
  README.md                    # ROS1 build/run/topic usage
  docs/ros1_topic_reference.md # ROS1 topic interface reference
```

Package model: this is a multi-package ROS1 catkin workspace. The two packages are `chassis_interfaces` and `chassis_driver`.

## 3. Core Node and Runtime Entry Points

- Executable: `chassis_driver_node`.
- Source entry: `chassis_driver/src/main.cpp`.
- Runtime API: `roscpp`.
- Node class: `chassis_driver::ChassisDriverNode` in `chassis_driver/include/chassis_driver/chassis_driver_node.hpp`.
- Launch: `chassis_driver/launch/chassis_driver.launch`.
- Config: `chassis_driver/config/chassis_driver.yaml`, loaded as private parameters under `/chassis_driver_node`.
- Threading: two background RX threads, one for CAN1 and one for CAN2; ROS callbacks send control frames directly and guard TX with `tx_mutex_`.

No lifecycle node, component node, service, action, TF, or odometry publisher exists in the current source.

## 4. Parameters

| Parameter | Default | Purpose |
|---|---:|---|
| `topic_prefix` | `/yunle_chassis` | Prefix for all driver topics. |
| `publish_raw_can` | `true` | Enables raw RX/TX CAN frame topics. |
| `publish_unknown_frames` | `true` | Enables unknown-frame debug publishing. |
| `enable_debug_topics` | `true` | Enables debug publishers. |
| `log_control_can_frames` | `false` | Logs each successfully transmitted control CAN frame as hexadecimal text. |
| `default_qos_depth` | `10` | ROS1 publisher/subscriber queue size. |
| `enabled_publish_topics` | `["all"]` | Fine-grained publisher enable list. |
| `enabled_subscribe_topics` | `["all"]` | Fine-grained subscriber enable list. |
| `message_channel_map` | feedback messages on `can2` | Feedback message channel metadata. |
| `control_message_channel_map` | control messages on `can2` | Required outgoing control-message channel routing. |
| `local_ip` | `192.168.1.102` | Local bind IP for UDP sockets. |
| `can1_local_port` | `8234` | Local UDP port for CAN1. |
| `can2_local_port` | `8235` | Local UDP port for CAN2. |
| `can1_remote_ip` | `192.168.1.98` | Remote CAN1 gateway IP. |
| `can2_remote_ip` | `192.168.1.99` | Remote CAN2 gateway IP. |
| `remote_port` | `1234` | Remote UDP destination port. |
| `udp_buffer_size` | `2048` | Receive buffer size. |
| `socket_timeout_ms` | `200` | UDP receive timeout. |
| `scu_control_max_steering_angle_deg` | `27.0` | Maximum steering angle used for 0x121 steering raw conversion. |
| `scu_control_max_target_speed_kmh` | `15.0` | Allowed target speed range is `[0, max]`; values outside the range are logged and sent as 0. |

## 5. ROS1 Interfaces

See `docs/ros1_topic_reference.md` for full field-level details.

Published topics:

- `/yunle_chassis/can_rx/raw` (`chassis_interfaces/CanFrame`)
- `/yunle_chassis/can_tx/raw` (`chassis_interfaces/CanFrame`)
- `/yunle_chassis/debug/unknown_frames` (`std_msgs/String`)
- `/yunle_chassis/feedback/bms_status` (`chassis_interfaces/BmsStatus`)
- `/yunle_chassis/feedback/vcu_warning_level` (`chassis_interfaces/VcuWarningLevel`)
- `/yunle_chassis/feedback/wheel_speed` (`chassis_interfaces/WheelSpeedFeedback`)
- `/yunle_chassis/feedback/ccu_status` (`chassis_interfaces/CcuStatus`)
- `/yunle_chassis/feedback/sas_angle` (`chassis_interfaces/SasAngleFeedback`)
- `/yunle_chassis/feedback/target_speed_feedback` (`chassis_interfaces/ScuTargetSpeedFeedback`)

Subscribed topics:

- `/yunle_chassis/control/scu_control_command` (`chassis_interfaces/ScuControlCommand`) -> CAN ID 0x121.
- `/yunle_chassis/control/scu_chassis_command` (`chassis_interfaces/ScuChassisCommand`) -> CAN ID 0x126.
- `/yunle_chassis/control/scu_torque_command` (`chassis_interfaces/ScuTorqueCommand`) -> CAN ID 0x123.
- `/yunle_chassis/control/vcu_chassis_debug` (`chassis_interfaces/VcuChassisDebug`) -> CAN IDs 0x710 and 0x715.

## 6. Ethernet-to-CAN Link

- Transport: UDP socket per CAN channel.
- Codec: `CanEthernetCodec` uses fixed 13-byte CAN-over-Ethernet records.
- RX path: UDP receive -> `CanEthernetCodec::decodePayload()` -> raw RX topic -> `FrameRouter::routeFrame()` -> `ChassisDriverNode::publishDecoded()`.
- TX path: ROS control callback -> `DbcProtocol::encodeSignal()` -> `ChassisDriverNode::sendControlFrame()` -> UDP send -> raw TX topic.

The gateway record format is:

- byte 0: flags and DLC.
- bytes 1-4: CAN ID in big-endian transport order.
- bytes 5-12: eight CAN data bytes.

## 7. CAN Protocol Mapping

The in-code DBC map lives in `chassis_driver/src/dbc_protocol.cpp`.

| CAN ID | Direction | Message | ROS1 topic |
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

## 8. Safety and Real-time Notes

Implemented:

- DBC min/max clamping in `DbcProtocol::encodeSignal()`.
- Shift validation for `ScuControlCommand`.
- Steering and target-speed range handling for CAN ID 0x121.
- TX mutex protection.
- Separate RX threads for CAN1 and CAN2.

Not implemented:

- Command timeout auto-stop.
- Gateway disconnect auto-stop or reconnect.
- Periodic TX timer.
- Dedicated emergency-stop topic.
- Feedback freshness checks.
- TF or odometry publishing.

## 9. Build and Test Commands

Recommended in a ROS1 Noetic catkin workspace:

```bash
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
roslaunch chassis_driver chassis_driver.launch
```

The current Windows shell used for this analysis does not provide ROS1/catkin, so runtime build verification must be completed in a ROS1 environment.

## 10. Extension Guide

- Add feedback CAN IDs in `DbcProtocol::kMessageById`, `ChassisDriverNode::publishDecoded()`, message definitions, publisher members, YAML keys, and this mapping.
- Add control topics in `chassis_interfaces/msg`, `chassis_interfaces/CMakeLists.txt`, `ControlCommandBridge`, `DbcProtocol`, and `control_message_channel_map`.
- Add parameters in `ChassisDriverNode::loadParameters()` and `chassis_driver/config/chassis_driver.yaml`.
- Keep gateway transport framing separate from DBC signal encode/decode and ROS topic mapping.
