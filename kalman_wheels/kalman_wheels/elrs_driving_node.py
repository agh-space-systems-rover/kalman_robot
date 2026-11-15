import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from kalman_interfaces.msg import Drive


class ElrsDriving(Node):
    def __init__(self):
        super().__init__("elrs_driving")
        self.rate = self.declare_parameter("rate", 30).value
        self.deadband = self.declare_parameter("deadband", 0.05).value
        self.max_speed = self.declare_parameter("max_speed", 1.0).value
        self.rotation_speed = self.declare_parameter("rotation_speed", np.pi / 2).value
        self.turn_radius = self.declare_parameter("turn_radius", 0.75).value
        self.joy_timeout = self.declare_parameter("joy_timeout", 0.3).value

        self.ch_steer = self.declare_parameter("steer_channel", 1).value  # right stick X
        self.ch_speed = self.declare_parameter("speed_channel", 2).value  # right stick Y
        self.ch_crab = self.declare_parameter("crab_channel", 4).value    # left stick X
        self.ch_rotate = self.declare_parameter("rotate_channel", 6).value

        self.joy = None
        self.last_joy_time = 0.0
        self.create_subscription(Joy, "joy", self.joy_cb, 10)
        self.drive_pub = self.create_publisher(Drive, "drive", 10)
        self.create_timer(1.0 / self.rate, self.tick)

    def joy_cb(self, msg: Joy):
        self.joy = msg
        self.last_joy_time = time.monotonic()

    def channel(self, ch: int) -> float:
        i = ch - 1
        if self.joy is None or not 0 <= i < len(self.joy.axes):
            return 0.0
        return float(self.joy.axes[i])

    def stick(self, ch: int) -> float:
        x = self.channel(ch)
        if abs(x) < self.deadband:
            return 0.0
        return float(np.sign(x) * (abs(x) - self.deadband) / (1.0 - self.deadband))

    def switch(self, ch: int) -> bool:
        return self.channel(ch) > 0.5

    def tick(self):
        if self.joy is None:
            return
        if time.monotonic() - self.last_joy_time > self.joy_timeout:
            # Link lost: send a single stop, then go silent until joy comes back.
            self.get_logger().warn("No joy input, stopping")
            self.joy = None
            self.drive_pub.publish(Drive())
            return

        # Sticks read + when pushed right, Drive uses + = left, hence the negations.
        drive = Drive()
        if self.switch(self.ch_rotate):
            drive.rotation = -self.stick(self.ch_steer) * self.rotation_speed
            if drive.rotation == 0:
                drive.rotation = 0.0001  # keep drive_controller in rotate mode
        else:
            drive.speed = self.stick(self.ch_speed) * self.max_speed
            drive.inv_radius = -self.stick(self.ch_steer) / self.turn_radius
            drive.sin_angle = -self.stick(self.ch_crab)
        self.drive_pub.publish(drive)


def main():
    try:
        rclpy.init()
        rclpy.spin(ElrsDriving())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()