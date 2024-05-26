CAN_CMD_SET_DIGITAL_OUTPUT = 0x63 # value = 0...145 servo angle // UART = 0x63
CAN_CMD_SET_PWM_OUTPUT     = 0x64 # value = 0-100               // UART = 0x64
CAN_CMD_SET_LED_DRIVER     = 0x65 # value = 0-100               // UART = 0x65

# typedef struct _packed_ {
#     uint8_t board_id;       //  0,1,2
#     uint8_t channel_id;     //  0,1,2,3
#     uint8_t value;          // depends on command
# } PWMFrame;
PWNFRAME_Format = "<BBB" # board_id, channel_id, value

CAN_CMD_SET_HBRIDGE = 0x66  # UART = 0x66
# typedef struct _packed_ {
#     uint8_t board_id;       //  0,1,2
#     uint8_t channel_id;     //  0, 1
#     uint8_t speed;          //  0-100
#     uint8_t direction;      //  0, 1
# } HBridgeFrame;
HBridgeFrame_Format = "<BBBB"  # board_id, channel_id, speed, direction

CAN_CMD_SET_STEPPER_POSITION = 0x67  # UART = 0x67
# typedef struct _packed_ {
#     uint8_t board_id;           // 0, 1, 2
#     uint8_t channel_id;         // 0, 1
#     int32_t target_position;    // signed position in stepper motor steps - station must be responssible for converting angle to steps based on current configuration,
#                                 // relative to homing position (either starting position or limit switch)
# } StepperFrame;
StepperFrame_Format = "<BBi" # board_id, channel_id, target_position


CAN_CMD_STEPPER_HOMING_REQUEST   = 0x68  # UART = 0x68
CAN_CMD_WEIGHT_REQUEST           = 0x69  # UART = 0x69
CAN_CMD_ANALOG_INPUT_REQUEST     = 0x6A  # UART = 0x6A
CAN_CMD_STEPPER_POSITION_REQUEST = 0x6B  # UART = 0x6B
# typedef struct _packed_ {
#     uint8_t board_id;   // 0, 1, 2
#     uint8_t channel_id; // 0, 1
# } GetFrame;
GetFrame_Format = "<BB"  # board_id, channel_id


CAN_CMD_WEIGHT_RESPONSE = 0x6C  # UART = 0x6C
# typedef struct _packed_ {
#     uint8_t board_id;   // 0, 1, 2
#     uint8_t channel_id; // 0, 1
#     uint32_t adc_value; // 24 bit value from tensometer - station must be responssible for converting it to weight units
# } WeightValueFrame;
WeightValueFrame_Format = "<BBI"  # board_id, channel_id, adc_value

CAN_CMD_ANALOG_INPUT_RESPONSE = 0x6D  # UART = 0x6D
# typedef struct _packed_ {
#     uint8_t board_id;   // 0, 1, 2
#     uint8_t channel_id; // 0, 1, 2, 3, 4
#     uint16_t adc_value; // 12 bit value from analog input (0-4095)
# } AnalogInputValueFrame;
AnalogInputValueFrame_Format = "<BBH"  # board_id, channel_id, adc_value

CAN_CMD_STEPPER_POSITION_RESPONSE = 0x6E  # UART = 0x6E
# typedef struct _packed_ {
#     uint8_t board_id;           // 0, 1, 2
#     uint8_t channel_id;         // 0, 1
#     int32_t current_position;   // current signed position in stepper motor steps relative to home position (either start position or limit switch)
# } StepperPositionFrame;
StepperPositionFrame_Format = "<BBi"  # board_id, channel_id, current_position
