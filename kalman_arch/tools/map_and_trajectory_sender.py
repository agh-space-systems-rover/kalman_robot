import argparse
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import yaml
from plyfile import PlyData


class MapAndTrajectoryBridge(Node):
    def __init__(self, ply_file_path, yaml_file_path):
        super().__init__("map_and_trajectory_bridge")
        # Initialize the bridge with PLY and YAML file paths
        self.points = self.read_ply_file(ply_file_path)
        self.path_data = self.read_yaml_file(yaml_file_path)

        self.pc_pub = self.create_publisher(PointCloud2, "/map", 10)
        self.path_pub = self.create_publisher(Path, "/path", 10)

        # Create timer for periodic publishing (10Hz)

        self.points_cloud_msg = self.get_point_cloud_msg(self.points)
        self.path_msg = self.get_path_msg(self.path_data)

        self.get_logger().info("Map and Trajectory Bridge initialized")
        self.timer = self.create_timer(1, self.publish_pc_and_path)

    def read_ply_file(self, ply_file_path):
        cloud = PlyData.read(ply_file_path)
        points = []

        # Extract points
        x = cloud["vertex"]["x"]
        y = cloud["vertex"]["y"]
        z = cloud["vertex"]["z"]

        # Extract colors (if available)
        r = cloud["vertex"]["red"]
        g = cloud["vertex"]["green"]
        b = cloud["vertex"]["blue"]

        for i in range(len(x)):
            points.append(
                [float(x[i]), float(y[i]), float(z[i]), int(r[i]), int(g[i]), int(b[i])]
            )

        self.get_logger().info(f"Loaded {len(points)} points from PLY file")
        return points

    def read_yaml_file(self, yaml_file_path):
        # Read path data from a YAML file
        path_data = []
        try:
            with open(yaml_file_path, "r") as file:
                data = yaml.safe_load(file)

            for entry in data["poses"]:
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = entry["position"]["x"]
                pose.pose.position.y = entry["position"]["y"]
                pose.pose.position.z = entry["position"]["z"]
                pose.pose.orientation.x = entry["orientation"]["x"]
                pose.pose.orientation.y = entry["orientation"]["y"]
                pose.pose.orientation.z = entry["orientation"]["z"]
                pose.pose.orientation.w = entry["orientation"]["w"]
                path_data.append(pose)

            self.get_logger().info(f"Loaded {len(path_data)} poses from YAML file")
        except Exception as e:
            self.get_logger().error(f"Error reading YAML file: {e}")

        return path_data

    def get_point_cloud_msg(self, points):
        # Create a PointCloud2 message
        msg = PointCloud2()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        # Define fields
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        msg.is_bigendian = False
        msg.point_step = 16
        msg.height = 1
        msg.width = len(points)
        msg.row_step = msg.point_step * len(points)
        msg.is_dense = True

        # Convert points to binary data
        buffer = bytearray()
        for point in points:
            float_rgb = struct.unpack(
                "f", struct.pack("i", point[3] * 2**16 + point[4] * 2**8 + point[5])
            )[0]
            buffer.extend(struct.pack("ffff", point[0], point[1], point[2], float_rgb))

        msg.data = buffer

        self.get_logger().info(f"Created PointCloud2 message with {len(points)} points")

        return msg

    def get_path_msg(self, path_data):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.poses = path_data

        self.get_logger().info(f"Created Path message with {len(path_data)} poses")

        return path_msg

    def publish_pc_and_path(self):
        self.get_logger().info("Publishing point cloud and path")
        # Update timestamps
        self.points_cloud_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_msg.header.stamp = self.get_clock().now().to_msg()

        # Publish messages
        self.get_logger().info("Publishing point cloud")
        self.pc_pub.publish(self.points_cloud_msg)
        self.get_logger().info("Publishing path")
        self.path_pub.publish(self.path_msg)
        self.get_logger().debug("Published point cloud and path")


def main(args=None):

    parser = argparse.ArgumentParser(description="Map and Trajectory Bridge")
    parser.add_argument("--ply", type=str, required=True, help="Path to PLY file")
    parser.add_argument("--yaml", type=str, required=True, help="Path to YAML file")

    rclpy.init(args=args)
    parsed_args = parser.parse_args()

    bridge = MapAndTrajectoryBridge(parsed_args.ply, parsed_args.yaml)

    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
