import rclpy
import numpy as np
from rclpy.node import Node
import time
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import do_transform_pose_stamped

import sensor_msgs_py.point_cloud2 as pc2


def join_point_clouds(cloud1: PointCloud2, cloud2: PointCloud2) -> PointCloud2:

    array1 = pc2.read_points_numpy(cloud1)
    array2 = pc2.read_points_numpy(cloud2)

    combined_array = np.concatenate((array1, array2))
    output_cloud = pc2.create_cloud(
        header=cloud1.header, fields=cloud1.fields, points=combined_array
    )

    return output_cloud


class CubeClouds(Node):
    def __init__(self):
        super().__init__("cube_clouds_node")
        self.time_elapsed = self.declare_parameter("last_detection_time", 0.5)
        self.obst_rad = self.declare_parameter("obstacle_rad", 0.2).value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        img_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            Detection2DArray, "/yolo_detections", self.detection_cb, img_qos
        )
        self.pub = self.create_publisher(PointCloud2, "/yolo_clouds", 10)
        self.empty_det_timer = self.create_timer(0.1, self.timer_callback)
        self.end = time.time()
        self.last_msg = None
    def detection_cb(self, msg: Detection2DArray):
        self.last_msg = msg
        self.end = time.time()

    def timer_callback(self):
        if time.time() - self.end >= 0.5 or self.last_msg is None:
            header = std_msgs.msg.Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = "base_link"
            empty_cloud = pc2.create_cloud_xyz32(header, [])
            self.pub.publish(empty_cloud)
        else:
            to_frame = "base_link"
            from_frame = self.last_msg.header.frame_id
            try:
                transform = self.tf_buffer.lookup_transform(
                    to_frame, from_frame, rclpy.time.Time()
                )
                pc2_data = None
                det: Detection2D
                for det in self.last_msg.detections:
                    hyp: ObjectHypothesisWithPose = det.results[0]
                    # cube_color = hyp.hypothesis.class_id
                    pose_stamped = PoseStamped()
                    pose_stamped.header = det.header
                    pose_stamped.pose = hyp.pose.pose
                    det_transformed: PoseStamped = do_transform_pose_stamped(
                        pose_stamped, transform=transform
                    )
                    if (
                        det_transformed.pose.position.x == 0
                        and det_transformed.pose.position.y == 0
                        and det_transformed.pose.position.z == 0
                    ):
                        continue

                    cube_cloud: PointCloud2 = self.generate_cube_cloud(
                        det_transformed.pose.position.x,
                        det_transformed.pose.position.y,
                        det_transformed.pose.position.z,
                        self.obst_rad,
                    )
                    pc2_header = det_transformed.header
                    new_pc2_data = pc2.create_cloud_xyz32(pc2_header, cube_cloud)
                    if pc2_data is None:
                        pc2_data = new_pc2_data
                    else:
                        pc2_data = join_point_clouds(pc2_data, new_pc2_data)

                self.pub.publish(pc2_data)
            except Exception as ex:
                self.get_logger().info(f"Error: {ex}")

    def generate_cube_cloud(self, cx, cy, cz, rad):
        points = []
        resolution = 0.1
        height_min = -0.1
        height_max = 0.1

        # generate fake cloud
        for x in np.arange(-rad, rad + 0.01, resolution):
            for y in np.arange(-rad, rad + 0.01, resolution):
                for z in np.arange(height_min, height_max, resolution):
                    points.append([cx + x, cy + y, cz + z])

        return points


def main():
    try:
        rclpy.init()
        node = CubeClouds()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
