from launch_ros.actions import Node
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from aruco_opencv_msgs.msg import ArucoDetection, MarkerPose
import tf2_ros
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import do_transform_pose_stamped
import rclpy.time


class TagRepublisher(Node):
    def __init__(self):
        super().__init__("tag_republisher_node")
        self.pub_ = self.create_publisher(PoseStamped, "/goal_update", 10)

        self.marker_subscription = self.create_subscription(
            ArucoDetection, '/d455_front/aruco_detections', self.callback, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def callback(self, msg: ArucoDetection):

        to_frame = "map"
        from_frame = msg.header.frame_id
        try:
            transform = self.tf_buffer.lookup_transform(
                to_frame,
                from_frame,
                rclpy.time.Time())

            markers: list[MarkerPose] = msg.markers
            for marker in markers:
                if(marker.marker_id == 55):
                    pose_stamped = PoseStamped()
                    pose_stamped.header.stamp = msg.header.stamp
                    pose_stamped.header.frame_id = msg.header.frame_id

                    pose_stamped.pose.position.x = marker.pose.position.x
                    pose_stamped.pose.position.y = marker.pose.position.y
                    pose_stamped.pose.position.z = 0.0

                    pose_stamped.pose.orientation.x = 0.0
                    pose_stamped.pose.orientation.y = 0.0
                    pose_stamped.pose.orientation.z = 0.0
                    pose_stamped.pose.orientation.w = 1.0

                    detection_transformed: PoseStamped = do_transform_pose_stamped(
                        pose_stamped, transform=transform
                    )
                    self.pub_.publish(detection_transformed)

        except Exception as ex:
            self.get_logger().info(f"Error:{ex}")


def main():
    try:
        rclpy.init()
        node = TagRepublisher()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
