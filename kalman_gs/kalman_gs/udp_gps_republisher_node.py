import json
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from udp_msgs.msg import UdpPacket


UDP_TOPIC = "/gps/udp_packets"
GPS_TOPIC = "/gps/fix"
GPS_FRAME_ID = "gps_link"


def number(payload: dict, key: str) -> float:
    try:
        return float(payload.get(key, float("nan")))
    except (TypeError, ValueError, OverflowError):
        return float("nan")


class UdpGpsRepublisher(Node):
    def __init__(self) -> None:
        super().__init__("udp_gps_republisher")
        self.fix_pub = self.create_publisher(NavSatFix, GPS_TOPIC, 10)
        self.create_subscription(UdpPacket, UDP_TOPIC, self.on_packet, 10)

    def on_packet(self, packet: UdpPacket) -> None:
        try:
            payload = json.loads(bytes(packet.data).decode())
        except (UnicodeDecodeError, json.JSONDecodeError):
            return

        if not isinstance(payload, dict):
            return

        latitude = number(payload, "lat")
        longitude = number(payload, "lon")

        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = GPS_FRAME_ID
        fix.latitude = latitude
        fix.longitude = longitude
        altitude = number(payload, "alt_m")
        fix.altitude = altitude if math.isfinite(altitude) else float("nan")
        fix.status.status = (
            NavSatStatus.STATUS_FIX
            if number(payload, "fix_quality") > 0
            else NavSatStatus.STATUS_NO_FIX
        )
        fix.status.service = NavSatStatus.SERVICE_GPS

        hdop = number(payload, "hdop")
        if math.isfinite(hdop) and 0 <= hdop <= 100:
            variance = hdop * hdop
            fix.position_covariance = [
                variance,
                0.0,
                0.0,
                0.0,
                variance,
                0.0,
                0.0,
                0.0,
                variance * 4,
            ]
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        self.fix_pub.publish(fix)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UdpGpsRepublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
