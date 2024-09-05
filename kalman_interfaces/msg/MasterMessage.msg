uint8 AUTONOMY_SWITCH = 0x20

uint8 MOTOR_SET_WHEELS = 0x40
uint8 MOTOR_GET_TEMPERATURE = 0x42
uint8 MOTOR_GET_WHEELS = 0x4F

uint8 UEUOS_SET_STATE = 0x70
uint8 UEUOS_SET_COLOR = 0x71
uint8 UEUOS_SET_EFFECT = 0x72

uint8 GS_TO_PC = 0x80
uint8 PC_TO_GS = 0x81
uint8 GS_TO_ARM = 0x86
uint8 ARM_TO_GS = 0x87
# No ARM_TO_PC and PC_TO_ARM because they are not needed.

uint8 FEED_SET_CAMERA = 0xB1
uint8 FEED_SET_CHANNEL = 0xB2
uint8 FEED_SET_POWER = 0xB3

uint8 ARM_SEND_SPACEMOUSE = 0xB4

uint8 DRILL_ARM = 0xD0
uint8 DRILL_RACK = 0xD1
uint8 DRILL_DRILL = 0xD2


# command number
uint8 cmd

# command data
uint8[] data
