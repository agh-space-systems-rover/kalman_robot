# input device connected to the computer that runs kalman_gamepad

uint8 TYPE_GAMEPAD = 0 # controlled by kalman_gamepad
uint8 TYPE_CUSTOM = 1 # our custom Arduino controller
uint8 TYPE_SPACEMOUSE = 2
uint8 TYPE_UNSUPPORTED = 3 # unknown button layout, e.g. a gamepad in DirectInput mode

uint8 MODE_NONE = 0
uint8 MODE_WHEELS = 1
uint8 MODE_ARM = 2
uint8 MODE_DRILL = 3

string id # e.g. js0
string name # name reported by the device
uint8 type
uint8 mode # always MODE_NONE if type is not TYPE_GAMEPAD
# remember to add new types and modes in gamepads.ts
