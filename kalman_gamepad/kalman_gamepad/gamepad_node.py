import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Int8, String, UInt8

from kalman_gamepad.gamepad import Gamepad, list_input_devices
from kalman_interfaces.msg import ArmFkCommand, Drive, GamepadState, GamepadStates
from kalman_interfaces.srv import SetGamepadMode

WHEELS = GamepadState.MODE_WHEELS
ARM = GamepadState.MODE_ARM
DRILL = GamepadState.MODE_DRILL
MODE_NAMES = {
    GamepadState.MODE_NONE: "none",
    WHEELS: "wheels",
    ARM: "arm",
    DRILL: "drill",
}
MODE_CYCLE = list(MODE_NAMES)

# Devices only displayed on the station. They are handled by other nodes.
CUSTOM_NAMES = ["Arduino"]
SPACEMOUSE_NAMES = ["3dconnexion", "spacemouse"]

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

WHEELS_RATE = 30
SPEED_FACTOR_RATE_OF_CHANGE = 0.5
MIN_SPEED_FACTOR = 0.2

ARM_RATE = 10
# Zero commands sent after the last gamepad leaves the arm mode.
# joint_republisher drops some of them, because it throttles to 10 Hz too.
ARM_STOP_TICKS = 10

DRILL_RATE = 10
DRILL_MAX_SPEED = 100


class DrillState:
    STOP = 0
    DRILLING_SITE_1 = 1
    DRILLING_SITE_2 = 2
    HOME = 3
    CLOSING_TUBES_SITE_1 = 4
    CLOSING_TUBES_SITE_2 = 5
    CLEANING_DRILL = 6
    OPENING_TUBES_SITE_1 = 7
    OPENING_TUBES_SITE_2 = 8
    OPENING_TUBES_BOTH_SITES = 9


# If multiple inputs are pressed at once, the first one selects the state.
DRILL_STATE_INPUTS = [
    ("b-button", DrillState.STOP),
    ("x-button", DrillState.HOME),
    ("y-button", DrillState.OPENING_TUBES_BOTH_SITES),
    ("a-button", DrillState.CLEANING_DRILL),
    ("left-stick", DrillState.DRILLING_SITE_1),
    ("left-trigger", DrillState.OPENING_TUBES_SITE_1),
    ("left-shoulder", DrillState.CLOSING_TUBES_SITE_1),
    ("right-stick", DrillState.DRILLING_SITE_2),
    ("right-trigger", DrillState.OPENING_TUBES_SITE_2),
    ("right-shoulder", DrillState.CLOSING_TUBES_SITE_2),
]


def get_display_only_type(name: str) -> int | None:
    if any(s in name for s in CUSTOM_NAMES):
        return GamepadState.TYPE_CUSTOM
    if any(s in name.lower() for s in SPACEMOUSE_NAMES):
        return GamepadState.TYPE_SPACEMOUSE
    return None


class GamepadNode(Node):
    def __init__(self):
        super().__init__("gamepad")

        self.rotation_speed = self.declare_parameter("rotation_speed", math.pi / 2)
        self.turn_radius = self.declare_parameter("turn_radius", 0.75)

        self.pads: dict[str, Gamepad] = {}  # by device ID, e.g. js0
        self.unsupported: dict[str, GamepadState] = {}
        self.other_devices: list[GamepadState] = []
        self.open_failures: dict[str, int] = {}
        self.last_values: dict[tuple[str, int], float] = {}

        self.speed_factor = MIN_SPEED_FACTOR
        self.last_drive = (0.0, 0.0, 0.0, 0.0)
        self.joint_locks = 0  # bit 0 is joint 1
        self.lock_focus = 1
        self.arm_stop_ticks = 0
        self.last_bridge_b = 0
        self.last_bridge_c = 0

        self.state_pub = self.create_publisher(
            GamepadStates, "/station/gamepads/state", LATCHED
        )
        self.sound_pub = self.create_publisher(String, "/station/gamepads/sound", 10)
        self.create_service(
            SetGamepadMode, "/station/gamepads/set_mode", self.set_mode_cb
        )

        self.drive_pub = self.create_publisher(Drive, "/drive", 10)

        self.fk_pub = self.create_publisher(ArmFkCommand, "/station/arm/fk/command", 10)
        self.fastclick_pub = self.create_publisher(UInt8, "/fastclick", 10)
        self.lock_focus_pub = self.create_publisher(
            UInt8, "/station/arm/joint_lock_focus", LATCHED
        )
        self.create_subscription(
            UInt8, "/station/arm/joint_locks", self.joint_locks_cb, LATCHED
        )

        # Latched, because rosbridge subscribes with transient local durability
        # after the station has published on these topics too.
        self.drill_b_pub = self.create_publisher(Int8, "/science/drill/b", LATCHED)
        self.drill_c_pub = self.create_publisher(Int8, "/science/drill/c", LATCHED)
        self.drill_state_pub = self.create_publisher(
            UInt8, "/science/drill/state", LATCHED
        )

        self.create_timer(1.0, self.scan_devices)
        self.create_timer(0.01, self.read_gamepads)
        self.create_timer(1 / WHEELS_RATE, self.update_wheels)
        self.create_timer(1 / ARM_RATE, self.update_arm)
        self.create_timer(1 / DRILL_RATE, self.update_drill)
        self.scan_devices()

    def scan_devices(self):
        joysticks = {}  # ID -> name
        others = {}  # name -> state
        for name, handlers in list_input_devices():
            js_id = next((h for h in handlers if h.startswith("js")), None)
            device_type = get_display_only_type(name)
            if device_type is not None:
                device_id = js_id or next(iter(handlers), name)
                others.setdefault(
                    name, GamepadState(id=device_id, name=name, type=device_type)
                )
            elif js_id is not None:
                joysticks[js_id] = name

        for js_id, name in joysticks.items():
            if js_id not in self.pads and js_id not in self.unsupported:
                self.open_gamepad(js_id, name)

        self.unsupported = {k: v for k, v in self.unsupported.items() if k in joysticks}
        self.open_failures = {
            k: v for k, v in self.open_failures.items() if k in joysticks
        }
        self.other_devices = list(self.unsupported.values()) + list(others.values())

        # Also lets the station notice when this node stops.
        self.publish_state()
        self.lock_focus_pub.publish(UInt8(data=self.lock_focus))

    def open_gamepad(self, js_id: str, name: str):
        try:
            pad = Gamepad(f"/dev/input/{js_id}", name)
        except OSError as e:
            # Right after connecting, udev may not have set the permissions yet.
            self.open_failures[js_id] = self.open_failures.get(js_id, 0) + 1
            if self.open_failures[js_id] == 2:
                self.get_logger().error(f"Cannot open {name} ({js_id}): {e}")
            return
        self.open_failures.pop(js_id, None)

        if not pad.is_standard():
            pad.close()
            self.unsupported[js_id] = GamepadState(
                id=js_id, name=name, type=GamepadState.TYPE_UNSUPPORTED
            )
            self.get_logger().warn(
                f"{name} ({js_id}) is not supported. "
                "If it is a gamepad, switch it to XInput mode."
            )
            return

        self.pads[js_id] = pad
        self.get_logger().info(f"Connected {name} ({js_id}).")
        self.publish_state()

    def read_gamepads(self):
        for js_id, pad in list(self.pads.items()):
            try:
                pressed = pad.update()
            except OSError:
                pad.close()
                del self.pads[js_id]
                self.get_logger().info(f"Disconnected {pad.name} ({js_id}).")
                self.publish_state()
                continue

            mode = pad.mode
            for name in pressed:
                if name in ("select", "mode"):
                    mode = MODE_CYCLE[(MODE_CYCLE.index(mode) + 1) % len(MODE_CYCLE)]
                elif name == "start":
                    mode = WHEELS
            if mode != pad.mode:
                self.set_mode(js_id, mode)

    def set_mode(self, js_id: str, mode: int):
        pad = self.pads[js_id]
        pad.mode = mode
        self.get_logger().info(f"{pad.name} ({js_id}) mode: {MODE_NAMES[mode]}")
        self.publish_state()

    def set_mode_cb(
        self, request: SetGamepadMode.Request, response: SetGamepadMode.Response
    ):
        response.success = request.id in self.pads and request.mode in MODE_NAMES
        if response.success:
            self.set_mode(request.id, request.mode)
        return response

    def publish_state(self):
        pads = [
            GamepadState(
                id=js_id, name=pad.name, type=GamepadState.TYPE_GAMEPAD, mode=pad.mode
            )
            for js_id, pad in self.pads.items()
        ]
        self.state_pub.publish(GamepadStates(gamepads=pads + self.other_devices))

    def joint_locks_cb(self, msg: UInt8):
        self.joint_locks = msg.data

    def read(self, name: str, mode: int) -> float:
        """Averages an input of all gamepads in the given mode."""
        values = [pad.read(name) for pad in self.pads.values() if pad.mode == mode]
        return sum(values) / max(len(values), 1)

    def pressed(self, name: str, mode: int) -> bool:
        """Whether the input went over 0.5 since the last call with the same args."""
        value = self.read(name, mode)
        last_value = self.last_values.get((name, mode), 0.0)
        self.last_values[(name, mode)] = value
        return value > 0.5 and last_value <= 0.5

    def update_wheels(self):
        speed = self.read("right-trigger", WHEELS) - self.read("left-trigger", WHEELS)
        turn = self.read("left-x", WHEELS)
        angle = self.read("right-x", WHEELS)
        shoulder = (
            self.read("left-shoulder", WHEELS) > 0
            or self.read("right-shoulder", WHEELS) > 0
        )
        change_speed = self.read("y-button", WHEELS) - self.read("x-button", WHEELS)

        if self.pressed("left-stick", WHEELS):
            self.sound_pub.publish(String(data="horn_left"))
        if self.pressed("right-stick", WHEELS):
            self.sound_pub.publish(String(data="horn_right"))

        self.speed_factor += change_speed * SPEED_FACTOR_RATE_OF_CHANGE / WHEELS_RATE
        self.speed_factor = max(MIN_SPEED_FACTOR, min(1.0, self.speed_factor))
        speed *= self.speed_factor

        msg = Drive(
            speed=speed,
            inv_radius=-turn / self.turn_radius.value,
            sin_angle=-angle,
            rotation=-speed * self.rotation_speed.value if shoulder else 0.0,
        )
        if shoulder and msg.rotation == 0:
            msg.rotation = 0.0001

        values = (msg.speed, msg.inv_radius, msg.sin_angle, msg.rotation)
        if not any(values) and not any(self.last_drive):
            return
        self.last_drive = values
        self.drive_pub.publish(msg)

    def update_arm(self):
        joints = [
            self.read("left-x", ARM),
            self.read("left-y", ARM),
            self.read("right-y", ARM),
            self.read("right-x", ARM),
            self.read("dpad-up", ARM) - self.read("dpad-down", ARM),
            self.read("right-trigger", ARM) - self.read("left-trigger", ARM),
        ]
        joints = [
            0.0 if self.joint_locks & (1 << i) else joint
            for i, joint in enumerate(joints)
        ]
        gripper = self.read("y-button", ARM) - self.read("a-button", ARM)

        focus = self.lock_focus
        if self.pressed("right-shoulder", ARM):
            focus = focus % 6 + 1
        if self.pressed("left-shoulder", ARM):
            focus = (focus + 4) % 6 + 1
        if focus != self.lock_focus:
            self.lock_focus = focus
            self.lock_focus_pub.publish(UInt8(data=focus))

        if self.pressed("x-button", ARM):
            self.fastclick_pub.publish(UInt8(data=0))
        if self.pressed("b-button", ARM):
            self.fastclick_pub.publish(UInt8(data=255))
        if self.pressed("dpad-left", ARM):
            self.sound_pub.publish(String(data="interstellar"))

        if any(pad.mode == ARM for pad in self.pads.values()):
            self.arm_stop_ticks = ARM_STOP_TICKS
        elif self.arm_stop_ticks > 0:
            self.arm_stop_ticks -= 1
        else:
            return
        self.fk_pub.publish(
            ArmFkCommand(
                joint_1=joints[0],
                joint_2=joints[1],
                joint_3=joints[2],
                joint_4=joints[3],
                joint_5=joints[4],
                joint_6=joints[5],
                gripper=gripper,
            )
        )

    def update_drill(self):
        bridge_b = self.dpad_speed("dpad-up", "dpad-down")
        if bridge_b is None:
            bridge_b = self.stick_speed("left-y")
        bridge_c = self.dpad_speed("dpad-right", "dpad-left")
        if bridge_c is None:
            bridge_c = self.stick_speed("left-x")

        if bridge_b != self.last_bridge_b:
            self.drill_b_pub.publish(Int8(data=bridge_b))
            self.last_bridge_b = bridge_b
        if bridge_c != self.last_bridge_c:
            self.drill_c_pub.publish(Int8(data=bridge_c))
            self.last_bridge_c = bridge_c

        requested_state = None
        for name, state in DRILL_STATE_INPUTS:
            if self.pressed(name, DRILL) and requested_state is None:
                requested_state = state
        if requested_state is not None:
            self.drill_state_pub.publish(UInt8(data=requested_state))

    def dpad_speed(self, positive: str, negative: str) -> int | None:
        positive_pressed = self.read(positive, DRILL) > 0
        negative_pressed = self.read(negative, DRILL) > 0
        if positive_pressed == negative_pressed:
            return None
        return DRILL_MAX_SPEED if positive_pressed else -DRILL_MAX_SPEED

    def stick_speed(self, name: str) -> int:
        speed = math.floor(self.read(name, DRILL) * DRILL_MAX_SPEED + 0.5)
        return max(-DRILL_MAX_SPEED, min(DRILL_MAX_SPEED, speed))


def main():
    try:
        rclpy.init()
        node = GamepadNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
