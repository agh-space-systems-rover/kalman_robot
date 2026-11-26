import errno
import fcntl
import math
import os
import struct

# Linux joystick API, see linux/joystick.h and linux/input-event-codes.h.
JS_EVENT = struct.Struct("=IhBB")  # time, value, type, number
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80


def _ior(nr: int, size: int) -> int:
    return (2 << 30) | (size << 16) | (ord("j") << 8) | nr


JSIOCGAXES = _ior(0x11, 1)
JSIOCGBUTTONS = _ior(0x12, 1)
JSIOCGAXMAP = _ior(0x32, 64)  # __u8[ABS_CNT]
JSIOCGBTNMAP = _ior(0x34, 1024)  # __u16[KEY_MAX - BTN_MISC + 1]

ABS_X = 0x00
ABS_Y = 0x01
ABS_Z = 0x02
ABS_RX = 0x03
ABS_RY = 0x04
ABS_RZ = 0x05
ABS_HAT0X = 0x10
ABS_HAT0Y = 0x11

BTN_A = 0x130
BTN_B = 0x131
BTN_X = 0x133
BTN_Y = 0x134
BTN_TL = 0x136
BTN_TR = 0x137
BTN_TL2 = 0x138
BTN_TR2 = 0x139
BTN_SELECT = 0x13A
BTN_START = 0x13B
BTN_MODE = 0x13C
BTN_THUMBL = 0x13D
BTN_THUMBR = 0x13E
BTN_DPAD_UP = 0x220
BTN_DPAD_DOWN = 0x221
BTN_DPAD_LEFT = 0x222
BTN_DPAD_RIGHT = 0x223

DEADZONE = 0.04

# Input names and value ranges are the same as in the old kalman_gs gamepad code.
STICKS = {
    "left-x": (ABS_X, 1),  # right is positive
    "left-y": (ABS_Y, -1),  # up is positive
    "right-x": (ABS_RX, 1),
    "right-y": (ABS_RY, -1),
}
TRIGGERS = {  # 0 to 1; digital on some gamepads
    "left-trigger": (ABS_Z, BTN_TL2),
    "right-trigger": (ABS_RZ, BTN_TR2),
}
DPAD = {  # hat axis, its direction or a button on some gamepads
    "dpad-up": (ABS_HAT0Y, -1, BTN_DPAD_UP),
    "dpad-down": (ABS_HAT0Y, 1, BTN_DPAD_DOWN),
    "dpad-left": (ABS_HAT0X, -1, BTN_DPAD_LEFT),
    "dpad-right": (ABS_HAT0X, 1, BTN_DPAD_RIGHT),
}
BUTTONS = {
    "a-button": BTN_A,
    "b-button": BTN_B,
    "x-button": BTN_X,
    "y-button": BTN_Y,
    "left-shoulder": BTN_TL,
    "right-shoulder": BTN_TR,
    "left-stick": BTN_THUMBL,
    "right-stick": BTN_THUMBR,
    "select": BTN_SELECT,
    "start": BTN_START,
    "mode": BTN_MODE,
}
BUTTON_NAMES = {code: name for name, code in BUTTONS.items()}


def list_input_devices() -> list[tuple[str, list[str]]]:
    """Returns names and handlers (e.g. ["event5", "js0"]) of all input devices."""
    devices = []
    with open("/proc/bus/input/devices") as f:
        for block in f.read().split("\n\n"):
            name = None
            handlers = []
            for line in block.splitlines():
                if line.startswith('N: Name="'):
                    name = line[len('N: Name="') : -1]
                elif line.startswith("H: Handlers="):
                    handlers = line[len("H: Handlers=") :].split()
            if name is not None:
                devices.append((name, handlers))
    return devices


def apply_deadzone(value: float) -> float:
    magnitude = max(abs(value) - DEADZONE, 0.0) / (1 - DEADZONE)
    return math.copysign(magnitude, value)


class Gamepad:
    def __init__(self, path: str, name: str):
        self.path = path
        self.name = name
        self.mode = 0  # GamepadState.MODE_*, managed by GamepadNode

        self.fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        try:
            num_axes = fcntl.ioctl(self.fd, JSIOCGAXES, bytes(1))[0]
            num_buttons = fcntl.ioctl(self.fd, JSIOCGBUTTONS, bytes(1))[0]
            axis_codes = fcntl.ioctl(self.fd, JSIOCGAXMAP, bytes(64))
            button_codes = struct.unpack(
                "512H", fcntl.ioctl(self.fd, JSIOCGBTNMAP, bytes(1024))
            )
        except OSError:
            os.close(self.fd)
            raise

        self.button_codes = button_codes[:num_buttons]
        self.axis_numbers = {code: i for i, code in enumerate(axis_codes[:num_axes])}
        self.button_numbers = {code: i for i, code in enumerate(self.button_codes)}
        self.axes = [0.0] * num_axes
        self.buttons = [0] * num_buttons

        # The initial state reported by the kernel can be stale, so inputs are
        # ignored until they send their first real event.
        self.moved_axes: set[int] = set()
        self.moved_buttons: set[int] = set()

    def is_standard(self) -> bool:
        """Whether the device uses the standard Linux gamepad layout, e.g. XInput."""
        sticks = all(
            code in self.axis_numbers for code in (ABS_X, ABS_Y, ABS_RX, ABS_RY)
        )
        return sticks and BTN_A in self.button_numbers

    def close(self):
        os.close(self.fd)

    def update(self) -> list[str]:
        """Reads pending events and returns the names of newly pressed buttons.

        Raises OSError if the gamepad was disconnected.
        """
        pressed = []
        while True:
            try:
                data = os.read(self.fd, JS_EVENT.size * 64)
            except BlockingIOError:
                return pressed
            if not data:
                raise OSError(errno.ENODEV, os.strerror(errno.ENODEV), self.path)

            for _, value, event_type, number in JS_EVENT.iter_unpack(data):
                is_init = event_type & JS_EVENT_INIT
                event_type &= ~JS_EVENT_INIT
                if event_type == JS_EVENT_AXIS and number < len(self.axes):
                    self.axes[number] = max(-1.0, min(value / 32767, 1.0))
                    if not is_init:
                        self.moved_axes.add(number)
                elif event_type == JS_EVENT_BUTTON and number < len(self.buttons):
                    self.buttons[number] = value
                    if not is_init:
                        self.moved_buttons.add(number)
                        name = BUTTON_NAMES.get(self.button_codes[number])
                        if value and name is not None:
                            pressed.append(name)

    def read(self, name: str) -> float:
        """Reads an input named like in STICKS, TRIGGERS, DPAD or BUTTONS."""
        if name in STICKS:
            code, sign = STICKS[name]
            return apply_deadzone(sign * self._axis(code, 0.0))

        if name in TRIGGERS:
            code, button = TRIGGERS[name]
            if code not in self.axis_numbers:
                return float(self._button(button))
            return apply_deadzone(self._axis(code, -1.0) * 0.5 + 0.5)

        if name in DPAD:
            code, direction, button = DPAD[name]
            if code not in self.axis_numbers:
                return float(self._button(button))
            return 1.0 if self._axis(code, 0.0) * direction > 0.5 else 0.0

        return float(self._button(BUTTONS[name]))

    def _axis(self, code: int, neutral: float) -> float:
        number = self.axis_numbers.get(code)
        if number not in self.moved_axes:
            return neutral
        return self.axes[number]

    def _button(self, code: int) -> int:
        number = self.button_numbers.get(code)
        if number not in self.moved_buttons:
            return 0
        return self.buttons[number]
