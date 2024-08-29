import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Joy
from kalman_interfaces.msg import Drive

LEFT_X = 0
RIGHT_X = 2
RIGHT_Y = 1

LEFT_POTENTIOMETER = 3
MIDDLE_POTENTIOMETER = 4
RIGHT_POTENTIOMETER = 5

LEFT_STICK_BUTTON = 4
RIGHT_STICK_BUTTON = 5

def lerp(a, b, t):
    return a + (b - a) * t


class ArduinoDriving(Node):
    def __init__(self):
        super().__init__("arduino_driving")

        self.rotation_speed = self.declare_parameter("rotation_speed", np.pi / 2)
        self.min_turn_radius = self.declare_parameter("min_turn_radius", 0.5)
        self.max_turn_radius = self.declare_parameter("max_turn_radius", 1.5)
        self.max_translation_angle = self.declare_parameter("max_translation_angle", 110 * np.pi / 180)

        self.joy_sub = self.create_subscription(Joy, "joy", self.joy_cb, 10)
        self.drive_pub = self.create_publisher(Drive, "drive", 10)

        self.last_drive_msg = Drive()

    def joy_cb(self, msg: Joy):
        max_speed = -msg.axes[LEFT_POTENTIOMETER] * 0.5 + 0.5
        max_turn = -msg.axes[MIDDLE_POTENTIOMETER] * 0.5 + 0.5
        max_translation = -msg.axes[RIGHT_POTENTIOMETER] * 0.5 + 0.5
        max_translation **= 0.5

        drive = Drive()

        rotate_in_place = msg.buttons[LEFT_STICK_BUTTON] or msg.buttons[RIGHT_STICK_BUTTON]
        if rotate_in_place:
            rotation_input = np.clip(msg.axes[LEFT_X] + msg.axes[RIGHT_X], -1, 1)
            drive.rotation = rotation_input * self.rotation_speed.value * max_speed
            if drive.rotation == 0:
                drive.rotation = 0.0001
        else:
            drive.speed = msg.axes[RIGHT_Y] * max_speed
            drive.inv_radius = msg.axes[LEFT_X] / lerp(self.max_turn_radius.value, self.min_turn_radius.value, max_turn)
            drive.sin_angle = msg.axes[RIGHT_X] * max_translation

            # Scale sin(angle) to reach max_translation_angle.
            if self.max_translation_angle.value < np.pi / 2:
                max_sin = np.sin(self.max_translation_angle.value)
            else:
                max_sin = 2 - np.sin(self.max_translation_angle.value)
                # Interpreted as over-translation. Explained in Drive.msg.
            drive.sin_angle *= max_sin
                
        if self.drive_msg_is_zero(drive) and self.drive_msg_is_zero(self.last_drive_msg):
            return

        self.last_drive_msg = drive
        self.drive_pub.publish(drive)

    def drive_msg_is_zero(self, msg: Drive):
        return msg.speed == 0 and msg.sin_angle == 0 and msg.inv_radius == 0 and msg.rotation == 0

def main():
    try:
        rclpy.init()
        node = ArduinoDriving()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
