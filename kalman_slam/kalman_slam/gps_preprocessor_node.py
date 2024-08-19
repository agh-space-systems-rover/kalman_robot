import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import time


class GpsNanRemoval(Node):
    def __init__(self) -> None:
        super().__init__("gps_preprocessor")

        self.initial_covariance_duration = self.declare_parameter(
            "initial_covariance_duration", 5.0
        ).value
        self.initial_covariance = self.declare_parameter(
            "initial_covariance", 0.0
        ).value

        self.sub = self.create_subscription(
            NavSatFix,
            "fix",
            self.fix_callback,
            10,
        )

        self.pub = self.create_publisher(NavSatFix, "fix/filtered", 10)

        self.first_fix_time = None
        self.logged_switch = False

    def fix_callback(self, msg: NavSatFix) -> None:
        # Check if altitude is NaN and replace it with zero.
        if math.isnan(msg.altitude):
            msg.altitude = 0.0
        for i in range(len(msg.position_covariance)):
            if math.isnan(msg.position_covariance[i]):
                msg.position_covariance[i] = 0.0

        # If this is the first fix, remember the time.
        if self.first_fix_time is None:
            self.first_fix_time = time.time()

        # Set initial covariance for the first few seconds.
        if time.time() - self.first_fix_time < self.initial_covariance_duration:
            for i in range(0, len(msg.position_covariance), 4):
                msg.position_covariance[i] = self.initial_covariance

        self.pub.publish(msg)


def main():
    try:
        rclpy.init()
        node = GpsNanRemoval()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
