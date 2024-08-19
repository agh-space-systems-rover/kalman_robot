import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from kalman_interfaces.srv import SpoofGps

# This node creates a service that can be used to send a fake GPS fix.
# In practice, this service is forwarded over RF to be used on the ground station.
class GpsSpoofer(Node):
    def __init__(self) -> None:
        super().__init__("gps_spoofer")

        self.frame_id = self.declare_parameter(
            "frame_id", "base_link"
        ).value
        self.covariance = self.declare_parameter(
            "covariance", 0.0
        ).value

        self.pub = self.create_publisher(NavSatFix, "fix", 10)

        self.srv = self.create_service(SpoofGps, "spoof_gps", self.spoof_callback)

    def spoof_callback(self, req: SpoofGps.Request, res: SpoofGps.Response):
        fix = NavSatFix()
        fix.header.frame_id = self.frame_id
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.latitude = req.location.latitude
        fix.longitude = req.location.longitude
        fix.altitude = req.location.altitude
        fix.status.status = 0
        fix.status.service = 1
        fix.position_covariance_type = 0
        fix.position_covariance = [self.covariance] * 9
        self.pub.publish(fix)
        return res
    

def main():
    try:
        rclpy.init()
        node = GpsSpoofer()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
