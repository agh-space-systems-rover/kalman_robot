import utm

import rclpy
import rclpy.node
import rclpy.qos
import tf2_ros
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPoint
from tf2_geometry_msgs import do_transform_pose

from kalman_interfaces.msg import GeoPath


# Converts nav_msgs/Path to kalman_interfaces/GeoPath.
class GeoPathConverter(rclpy.node.Node):
    def __init__(self):
        super().__init__("geo_path_converter")

        self.utm_frame_id = self.declare_parameter("utm_frame_id", "utm")

        # Initialize TF2 buffer and listener.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Create a service compatible with service_based_nav2_controller.
        self.fix_sub = self.create_subscription(NavSatFix, "fix", self.fix_cb, 10)
        self.path_sub = self.create_subscription(Path, "path", self.path_cb, 10)
        self.geo_path_pub = self.create_publisher(GeoPath, "geo_path", 10)

        self.zone_number = None
        self.zone_letter = None

    def fix_cb(self, msg: NavSatFix):
        _, _, self.zone_number, self.zone_letter = utm.from_latlon(
            msg.latitude, msg.longitude
        )

    def path_cb(self, msg: Path):
        if self.zone_number is None:
            self.get_logger().warn("No fix received yet.")
            return

        # Get transform to utm
        try:
            transform = self.tf_buffer.lookup_transform(
                self.utm_frame_id.value, msg.header.frame_id, msg.header.stamp
            )
        except tf2_ros.LookupException as e:
            self.get_logger().warn("Failed to lookup transform: %s", e)
            return

        geo_path: GeoPath = GeoPath()

        pose: PoseStamped
        for pose in msg.poses:
            pose = pose.pose

            # Transform pose to utm
            pose = do_transform_pose(pose, transform)

            # Convert to lat/lon
            lat, lon = utm.to_latlon(
                pose.position.x, pose.position.y, self.zone_number, self.zone_letter
            )

            geo_point = GeoPoint()
            geo_point.latitude = lat
            geo_point.longitude = lon
            geo_path.points.append(geo_point)

        self.geo_path_pub.publish(geo_path)


def main():
    try:
        rclpy.init()
        node = GeoPathConverter()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
