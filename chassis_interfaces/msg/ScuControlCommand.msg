# Shift request values accepted by the wrapper for CAN ID 0x121.
uint8 SHIFT_LEVEL_D=1
uint8 SHIFT_LEVEL_N=2
uint8 SHIFT_LEVEL_R=3

# D/N/R only. Values outside 1, 2, 3 are rejected by the driver wrapper.
uint8 scu_shift_level_request

# Wheel steering angles in degrees. Values outside
# [-scu_control_max_steering_angle_deg, +scu_control_max_steering_angle_deg]
# are sent as 0 with a warning. Valid values are converted to the 8-bit
# two's-complement raw values required by 0x121.
float32 scu_steering_angle_front
float32 scu_steering_angle_rear

# Speed magnitude in km/h. Direction is selected by scu_shift_level_request.
# Values outside [0, scu_control_max_target_speed_kmh] are sent as 0 with a warning.
float32 scu_target_speed

bool scu_brake_enable

uint8 gw_left_turn_light_request
uint8 gw_right_turn_light_request
uint8 gw_position_light_request
uint8 gw_low_beam_request
uint8 scu_torque_or_speed_mode
bool steering_angle_speed_valid
bool brake_force_command_valid
