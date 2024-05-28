import cstruct
CAN_CMD_SET_DIGITAL_OUTPUT = 0x50  # value = 0...145 servo angle // UART = 0x50
CAN_CMD_SET_PWM_OUTPUT     = 0x51  # value = 0-255               // UART = 0x51
CAN_CMD_SET_LED_DRIVER     = 0x52  # value = 0-255               // UART = 0x52
# typedef struct _packed_ {
#     uint8_t board_id;       //  0,1,2
#     uint8_t channel_id;     //  0,1,2,3
#     uint8_t value;          // depends on command
# } PWMFrame;
PWMFrame_Format = "<BBB"
class PWMFrame(cstruct.MemCStruct):
    __byte_order__ = cstruct.LITTLE_ENDIAN
    __def__ = """
        struct {
            uint8_t board_id;       //  0,1,2
            uint8_t channel_id;     //  0,1,2,3
            uint8_t value;          // depends on command
        };
    """

def create_PWMFrame(board_id: int, channel_id: int, value: int) -> PWMFrame:
    frame = PWMFrame()
    frame.board_id = board_id
    frame.channel_id = channel_id
    frame.value = value
    return frame

CAN_CMD_SET_HBRIDGE        = 0x53  # UART = 0x53
# typedef struct _packed_ {
#     uint8_t board_id;       //  0,1,2
#     uint8_t channel_id;     //  0, 1
#     uint8_t speed;          //  0-100
#     uint8_t direction;      //  0, 1
# } HBridgeFrame;
HBridgeFrame_Format = "<BBBB"

class HBridgeFrame(cstruct.MemCStruct):
    __byte_order__ = cstruct.LITTLE_ENDIAN
    __def__ = """
        struct {
            uint8_t board_id;       //  0,1,2
            uint8_t channel_id;     //  0, 1
            uint8_t speed;          //  0-100
            uint8_t direction;      //  0, 1
        };
    """
def create_HBridgeFrame(board_id: int, channel_id: int, speed: int, direction: int) -> HBridgeFrame:
    frame = HBridgeFrame()
    frame.board_id = board_id
    frame.channel_id = channel_id
    frame.speed = speed
    frame.direction = direction
    return frame

CAN_CMD_SET_STEPPER_POSITION = 0x54  # UART = 0x54
# typedef struct _packed_ {
#     uint8_t board_id;          // 0, 1, 2
#     uint8_t channel_id;        // 0, 1
#     float target_position;     // signed position in degrees relative to homing position (either starting position or limit switch)
# } StepperFrame;
StepperFrame_Format = "<BBf"

class StepperFrame(cstruct.MemCStruct):
    __byte_order__ = cstruct.LITTLE_ENDIAN
    __def__ = """
        struct {
            uint8_t board_id;          // 0, 1, 2
            uint8_t channel_id;        // 0, 1
            float target_position;     // signed position in degrees relative to homing position (either starting position or limit switch)
        };
    """
def create_StepperFrame(board_id: int, channel_id: int, target_position: float) -> StepperFrame:
    frame = StepperFrame()
    frame.board_id = board_id
    frame.channel_id = channel_id
    frame.target_position = target_position
    return frame


CAN_CMD_SET_RESPONSE  = 0x55  # UART = 0x55
# typedef struct {
#     uint8_t board_id;       // 0, 1, 2
#     uint8_t channel_id;     // 0, 1, 2, 3
#     uint8_t command_id;     // id of the response is comming from, ie: if we received CAN_CMD_SET_LED_DRIVER(0x50), respose.command_id will be 0x50
#     uint8_t type;           // type of state payload (0 = FLOAT, 1 = byte, 2 = speed)
#     union {
#         float float_state;
#         uint8_t byte_state;
#         struct {
#             uint8_t speed;
#             uint8_t direction;
#         };
#     } state;                // payload with current state
# } SetResponseFrame;
# SetResponseFrame_Format_type  = "<XXXBXXXX"
# SetResponseFrame_Format_float = "<BBBBf"
# SetResponseFrame_Format_byte  = "<BBBBBXXX"
# SetResponseFrame_Format_speed = "<BBBBBBXX"

class SetResponseFrame(cstruct.MemCStruct):
    __byte_order__ = cstruct.LITTLE_ENDIAN
    __def__ = """
        struct {
            uint8_t board_id;       // 0, 1, 2
            uint8_t channel_id;     // 0, 1, 2, 3
            uint8_t command_id;     // id of the response is comming from, ie: if we received CAN_CMD_SET_LED_DRIVER(0x50), respose.command_id will be 0x50
            uint8_t type;           // type of state payload (0 = FLOAT, 1 = byte, 2 = speed)
            union {
                float float_state;
                uint8_t byte_state;
                struct {
                    uint8_t speed;
                    uint8_t direction;
                };
            } state;                // payload with current state
        };
    """
def create_SetResponseFrame_float(board_id: int, channel_id: int, command_id: int, float_state: float) -> SetResponseFrame:
    frame = SetResponseFrame()
    frame.board_id = board_id
    frame.channel_id = channel_id
    frame.command_id = command_id
    frame.type = 0
    frame.float_state = float_state
    return frame
# This is never constructed, so I won't create more constructors


CAN_CMD_STEPPER_HOMING_REQUEST   = 0x56  # UART = 0x56
CAN_CMD_WEIGHT_REQUEST           = 0x57  # UART = 0x57
CAN_CMD_INPUT_REQUEST            = 0x58  # UART = 0x58
CAN_CMD_STEPPER_POSITION_REQUEST = 0x59  # UART = 0x59
# typedef struct _packed_ {
#     uint8_t board_id;   // 0, 1, 2
#     uint8_t channel_id; // 0, 1
# } RequestFrame;
RequestFrame_Format = "<BB"

class RequestFrame(cstruct.MemCStruct):
    __byte_order__ = cstruct.LITTLE_ENDIAN
    __def__ = """
        struct {
            uint8_t board_id;   // 0, 1, 2
            uint8_t channel_id; // 0, 1
        };
    """
def create_RequestFrame(board_id: int, channel_id: int) -> RequestFrame:
    frame = RequestFrame()
    frame.board_id = board_id
    frame.channel_id = channel_id
    return frame

CAN_CMD_WEIGHT_RESPONSE  = 0x5A  # UART = 0x5A
# typedef struct _packed_ {
#     uint8_t board_id;   // 0, 1, 2
#     uint8_t channel_id; // 0, 1
#     uint32_t adc_value; // 24 bit value from tensometer - station must be responssible for converting it to weight units
# } WeightValueFrame;
WeightValueFrame_Format = "<BBI"

class WeightValueFrame(cstruct.MemCStruct):
    __byte_order__ = cstruct.LITTLE_ENDIAN
    __def__ = """
        struct {
            uint8_t board_id;   // 0, 1, 2
            uint8_t channel_id; // 0, 1
            uint32_t adc_value; // 24 bit value from tensometer - station must be responssible for converting it to weight units
        };
    """

CAN_CMD_INPUT_RESPONSE  = 0x5B  # UART = 0x5B
# typedef struct _packed_ {
#     uint8_t board_id;   // 0, 1, 2
#     uint8_t channel_id; // 0, 1, 2, 3, 4
#     uint16_t value; // 12 bit value from analog input (0-4095), or 0 - 1 value from digital input
# } InputResponseFrame;
InputResponseFrame_Format = "<BBH"

class InputResponseFrame(cstruct.MemCStruct):
    __byte_order__ = cstruct.LITTLE_ENDIAN
    __def__ = """
        struct {
            uint8_t board_id;   // 0, 1, 2
            uint8_t channel_id; // 0, 1, 2, 3, 4
            uint16_t value; // 12 bit value from analog input (0-4095), or 0 - 1 value from digital input
        };
    """

CAN_CMD_STEPPER_POSITION_RESPONSE  = 0x5C  # UART = 0x5C
# typedef struct _packed_ {
#     uint8_t board_id;           // 0, 1, 2
#     uint8_t channel_id;         // 0, 1
#     float current_position;    // current signed position in degrees relative to home position (either start position or limit switch)
# } StepperPositionFrame;
StepperPositionFrame_Format = "<BBf"

class StepperPositionFrame(cstruct.MemCStruct):
    __byte_order__ = cstruct.LITTLE_ENDIAN
    __def__ = """
        struct {
            uint8_t board_id;           // 0, 1, 2
            uint8_t channel_id;         // 0, 1
            float current_position;    // current signed position in degrees relative to home position (either start position or limit switch)
        };
    """

CAN_CMD_AUTOMATION_SEQUENCE_BEGIN_REQUEST = 0x5D  # UART = 0x5D
CAN_CMD_AUTOMATION_SEQUENCE_STATE_REQUEST = 0x5E  # UART = 0x5E
# typedef struct _packed_ {
#     uint8_t board_id;           // 0, 1, 2
#     uint8_t sequence_id;        // id of the sequence to begin execution - probably 0, 1, 2, ...
# } SequenceManagerFrame;
SequenceManagerFrame_Format = "<BB"
class SequenceManagerFrame(cstruct.MemCStruct):
    __byte_order__ = cstruct.LITTLE_ENDIAN
    __def__ = """
        struct {
            uint8_t board_id;           // 0, 1, 2
            uint8_t sequence_id;        // id of the sequence to begin execution - probably 0, 1, 2, ...
        };
    """
def create_SequenceManagerFrame(board_id: int, sequence_id: int) -> SequenceManagerFrame:
    frame = SequenceManagerFrame()
    frame.board_id = board_id
    frame.sequence_id = sequence_id
    return frame

CAN_CMD_AUTOMATION_SEQUENCE_STATE_RESPONSE = 0x5F  # UART = 0x5F
# typedef struct _packed_ {
#     uint8_t board_id;   // 0, 1, 2
#     uint8_t sequence_id; // id of the sequence to begin execution - probably 0, 1, 2, ...
#     uint8_t sequence_state; // state of the frame
#                    // 0 = NOT_STARTED_YET,
#                    // 1 = RUNNING,
#                    // 2 = DONE
#     uint8_t current_stage; // if sequence_state == RUNNING it tells which stage is currenly executed
# } SequenceManagerStateFrame;
SequenceManagerStateFrame_Format = "<BBBB"

class SequenceManagerStateFrame(cstruct.MemCStruct):
    __byte_order__ = cstruct.LITTLE_ENDIAN
    __def__ = """
        struct {
            uint8_t board_id;   // 0, 1, 2
            uint8_t sequence_id; // id of the sequence to begin execution - probably 0, 1, 2, ...
            uint8_t sequence_state; // state of the frame
                            // 0 = NOT_STARTED_YET,
                            // 1 = RUNNING,
                            // 2 = DONE
            uint8_t current_stage; // if sequence_state == RUNNING it tells which stage is currenly executed
        };
    """
