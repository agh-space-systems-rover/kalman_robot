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

MIN_TURN_RADIUS = 0.5
MAX_TURN_RADIUS = 1.5

def lerp(a, b, t):
    return a + (b - a) * t


class JoyDriving(Node):
    def __init__(self):
        super().__init__("joy_driving")

        self.rotation_speed = self.declare_parameter("rotation_speed", np.pi / 2).value
        self.turn_radius = self.declare_parameter("turn_radius", 0.75).value

        self.joy_sub = self.create_subscription(
            Joy, "joy", self.joy_cb, qos_profile=10
        )
        self.drive_pub = self.create_publisher(
            Drive, "drive", qos_profile=10
        )

        self.last_drive_msg = Drive()

    def joy_cb(self, msg: Joy):
        max_speed = -msg.axes[LEFT_POTENTIOMETER] * 0.5 + 0.5
        max_turn = msg.axes[MIDDLE_POTENTIOMETER] * 0.5 + 0.5
        max_translation = -msg.axes[RIGHT_POTENTIOMETER] * 0.5 + 0.5
        
        speed = msg.axes[RIGHT_Y] * max_speed
        inv_radius = msg.axes[LEFT_X] / lerp(MIN_TURN_RADIUS, MAX_TURN_RADIUS, max_turn)
        sin_angle = msg.axes[RIGHT_X] * max_translation
        
        rotate_in_place = msg.buttons[LEFT_STICK_BUTTON] or msg.buttons[RIGHT_STICK_BUTTON]
        
        rotate_value = msg.axes[LEFT_X] if abs(msg.axes[LEFT_X]) > abs(msg.axes[RIGHT_X]) else msg.axes[RIGHT_X]
        rotation =  rotate_value * max_speed if rotate_in_place else 0.0
                        
        drive = Drive()
        if not rotate_in_place:
            drive.speed = speed
            drive.inv_radius = inv_radius
            drive.sin_angle = sin_angle
        else:
            drive.rotation = rotation
                
        if self.drive_msg_is_zero(drive) and self.drive_msg_is_zero(self.last_drive_msg):
            return

        self.last_drive_msg = drive
        self.drive_pub.publish(drive)

    def drive_msg_is_zero(self, msg: Drive):
        return msg.speed == 0 and msg.sin_angle == 0 and msg.inv_radius == 0 and msg.rotation == 0

def main():
    try:
        rclpy.init()
        node = JoyDriving()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
