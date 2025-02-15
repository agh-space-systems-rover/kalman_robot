import utm
import numpy as np

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


# P is at 0,0
def closest_point_on_segment(v1, v2):
    if np.dot(v1, v1 - v2) < 0:
        return v1
    if np.dot(v2, v2 - v1) < 0:
        return v2
    # https://gdbooks.gitbooks.io/3dcollisions/content/Chapter1/closest_point_on_line.html
    return v1 + np.dot(-v1, v2 - v1) / np.dot(v2 - v1, v2 - v1) * (v2 - v1)


def dist_to_segment(p, v1, v2):
    return np.linalg.norm(closest_point_on_segment(v1 - p, v2 - p))


# Converts nav_msgs/Path to kalman_interfaces/GeoPath.
class GeoPathConverter(rclpy.node.Node):
    def __init__(self):
        super().__init__("geo_path_converter")

        self.utm_frame_id = self.declare_parameter("utm_frame_id", "utm")
        self.min_deviation = self.declare_parameter("min_deviation", 1.0)
        self.max_points = self.declare_parameter("max_points", 10)

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

        # Optimize path
        msg = self.optimize_path(msg, self.min_deviation.value, self.max_points.value)

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

    # Removes unnecessary points from the path.
    def optimize_path(self, path: Path, min_dist=0.5, max_points=10):
        if len(path.poses) < 2:
            return path

        optimized_path = Path()
        optimized_path.header = path.header
        optimized_path.poses.append(path.poses[0])

        last_appended_i = 0
        for i in range(1, len(path.poses)):
            last_optim_pose = optimized_path.poses[-1].pose
            cur_pose = path.poses[i].pose
            # Take a segment from last_optim_pose to cur_pose
            # Check the maximum distance between the segment line and the points in between
            # If the distance is greater than max_dist, add the point to the optimized path
            # Otherwise, skip the point
            for j in range(last_appended_i + 1, i):
                compare_pose = path.poses[j].pose
                dist = dist_to_segment(
                    np.array([compare_pose.position.x, compare_pose.position.y]),
                    np.array([last_optim_pose.position.x, last_optim_pose.position.y]),
                    np.array([cur_pose.position.x, cur_pose.position.y]),
                )

                if dist > min_dist:
                    optimized_path.poses.append(path.poses[j])
                    last_appended_i = j
                    break

            # Exit early if we have too many nodes.
            if len(optimized_path.poses) >= max_points:
                return optimized_path

        # Add the last point
        if last_appended_i != len(path.poses) - 1:
            optimized_path.poses.append(path.poses[-1])

        return optimized_path


def main():
    try:
        rclpy.init()
        node = GeoPathConverter()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
