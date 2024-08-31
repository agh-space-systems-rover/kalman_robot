import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Joy
from kalman_interfaces.msg import Drive

LEFT_X = 0
LEFT_Y = 1
LEFT_TRIGGER = 2
RIGHT_X = 3
RIGHT_Y = 4
RIGHT_TRIGGER = 5

LEFT_SHOULDER = 4
RIGHT_SHOULDER = 5

class GamepadDriving(Node):
    def __init__(self):
        super().__init__("gamepad_driving")

        self.rotation_speed = self.declare_parameter("rotation_speed", np.pi / 2)
        self.turn_radius = self.declare_parameter("turn_radius", 0.75)

        self.joy_sub = self.create_subscription(
            Joy, "joy", self.joy_cb, qos_profile=10
        )
        self.drive_pub = self.create_publisher(
            Drive, "drive", qos_profile=10
        )

        self.last_drive_msg = Drive()

    def joy_cb(self, msg: Joy):
        # Detect if turning in place.
        if msg.buttons[LEFT_SHOULDER] or msg.buttons[RIGHT_SHOULDER]:
            # Read input.
            speed = (-msg.axes[LEFT_TRIGGER] * 0.5 + 0.5) - (-msg.axes[RIGHT_TRIGGER] * 0.5 + 0.5)
            speed *= min(msg.buttons[LEFT_SHOULDER] + msg.buttons[RIGHT_SHOULDER], 1)
            if speed == 0:
                speed = 0.0001

            # Compute and send velocities.
            drive = Drive()
            drive.rotation = speed * self.rotation_speed.value
        else:
            drive = Drive()
            drive.sin_angle = msg.axes[RIGHT_X]
            drive.inv_radius = msg.axes[LEFT_X] / self.turn_radius.value
            drive.speed = (-msg.axes[RIGHT_TRIGGER] * 0.5 + 0.5) - (-msg.axes[LEFT_TRIGGER] * 0.5 + 0.5)
        
        if self.drive_msg_is_zero(drive) and self.drive_msg_is_zero(self.last_drive_msg):
            return

        self.last_drive_msg = drive
        self.drive_pub.publish(drive)

    def drive_msg_is_zero(self, msg: Drive):
        return msg.speed == 0 and msg.sin_angle == 0 and msg.inv_radius == 0 and msg.rotation == 0

def main():
    try:
        rclpy.init()
        node = GamepadDriving()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
