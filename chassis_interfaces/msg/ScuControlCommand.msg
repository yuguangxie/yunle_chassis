# Shift request values accepted by the wrapper for CAN ID 0x121.
# CAN ID 0x121 封装层接受的档位请求取值。
uint8 SHIFT_LEVEL_D=1
uint8 SHIFT_LEVEL_N=2
uint8 SHIFT_LEVEL_R=3

# D/N/R only. Values outside 1, 2, 3 are rejected by the driver wrapper.
# 仅支持 D/N/R。除 1、2、3 之外的取值会被驱动封装层拒发。
uint8 scu_shift_level_request

# Wheel steering angles in degrees. Values outside
# [-scu_control_max_steering_angle_deg, +scu_control_max_steering_angle_deg]
# are sent as 0 with a warning. Valid values are converted to the 8-bit
# two's-complement raw values required by 0x121.
# 车轮转角，单位 deg。超出
# [-scu_control_max_steering_angle_deg, +scu_control_max_steering_angle_deg]
# 的值会输出 warning 并按 0 下发。有效值会被转换为 0x121 所需的
# 8 位补码 raw 值。
float32 scu_steering_angle_front
float32 scu_steering_angle_rear

# Speed magnitude in km/h. Direction is selected by scu_shift_level_request.
# Values outside [0, scu_control_max_target_speed_kmh] are sent as 0 with a warning.
# 速度大小，单位 km/h。行驶方向由 scu_shift_level_request 选择。
# 超出 [0, scu_control_max_target_speed_kmh] 的值会输出 warning 并按 0 下发。
float32 scu_target_speed

bool scu_brake_enable

uint8 gw_left_turn_light_request
uint8 gw_right_turn_light_request
uint8 gw_position_light_request
uint8 gw_low_beam_request
uint8 scu_torque_or_speed_mode
bool steering_angle_speed_valid
bool brake_force_command_valid
