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
        self.covariance_override = self.declare_parameter(
            "covariance_override", 0.0
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
        # Check if any of the fields are NaN and replace them with zeros.
        if math.isnan(msg.latitude):
            msg.latitude = 0.0
        if math.isnan(msg.longitude):
            msg.longitude = 0.0
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
        elif self.covariance_override > 0:
            # If the first few seconds have passed, override the covariance.
            # DO not do that if the override is set to 0.
            for i in range(0, len(msg.position_covariance), 4):
                msg.position_covariance[i] = self.covariance_override

            if not self.logged_switch:
                self.get_logger().info(
                    f"Switching GPS covariance from {self.initial_covariance} to {self.covariance_override}."
                )
                self.logged_switch = True

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
