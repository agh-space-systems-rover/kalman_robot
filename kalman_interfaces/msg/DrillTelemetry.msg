uint8 STATE_IDLE = 0
uint8 STATE_HOMING = 1
uint8 STATE_LOWER_DRILL = 2
uint8 STATE_DRILLING = 3
uint8 STATE_MOTOR_STOP = 4
uint8 STATE_RETRACTING = 5
uint8 STATE_MEASURING = 6
uint8 STATE_DONE = 7
uint8 STATE_ERROR = 8

float32 depth_mm
float32 rack_velocity_mmps
float32 weight_g
float32 rack_current_a
float32 drill_current_a

# Raw status byte: limit switch, autonomy enable and base detection in bits 0-2,
# autonomy state in bits 3-6.
uint8 flags
bool upper_limit_pressed
bool autonomy_active
bool based
uint8 autonomy_state
