from typing import Tuple
import tf2_ros
from rclpy.duration import Duration
from rclpy.clock import Clock
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import tf2_geometry_msgs
import utm

Vec2 = Tuple[float, float]

class Transformer:
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def transform(self, pose: PoseStamped, target_frame: str) -> PoseStamped:
        return self.tf_buffer.transform(
            tf2_geometry_msgs.PoseStamped(header=pose.header, pose=pose.pose),
            target_frame, Duration(seconds=10)
        )

    def transform2D(self, position: Vec2, input_frame: str, target_frame: str) -> Vec2:
        header = Header()
        header.frame_id = input_frame
        header.stamp = Clock().now()

        pose = PoseStamped()
        pose.header = header
        pose.pose = Pose()
        pose.pose.position = Point(x=position[0], y=position[1], z=0.0)
        pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        output = self.transform(pose, target_frame)

        return (output.pose.position.x, output.pose.position.y)
        
    def gps2utm(self, gps: Vec2) -> Vec2:
        x, y, *_ = utm.from_latlon(*gps)
        return (x, y)