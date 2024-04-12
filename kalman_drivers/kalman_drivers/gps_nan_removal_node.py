import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

from kalman_interfaces.srv import CalibrateCompass

class GpsNanRemoval(Node):
    def __init__(self) -> None:
        super().__init__("gps_nan_removal")

        self.sub = self.create_subscription(
            NavSatFix,
            "fix",
            self.fix_callback,
            10,
        )

        self.pub = self.create_publisher(NavSatFix, "fix/filtered", 10)

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
