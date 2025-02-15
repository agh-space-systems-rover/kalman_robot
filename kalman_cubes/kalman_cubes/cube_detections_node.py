import cv2
import os
import rclpy
from rclpy.node import Node
import rclpy.time
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros.buffer import Buffer
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from tf2_ros.transform_listener import TransformListener


class CubeNode(Node):
    def __init__(self):
        super().__init__("kalman_cubes")
        self.declare_parameter("camera_no", "d455_front")
        self.camera_no = self.get_parameter("camera_no").value
        img_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            Detection2DArray, "/yolo_detections", self.transform_detection, img_qos)
        self.create_subscription(
            CompressedImage, f"/d455_front/yolo_annotated/compressed", self.save_image, img_qos)
        self.tf_buffer = Buffer()
        self.bridge = CvBridge()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.image = None
        self.create_directory()

    def save_image(self, msg: CompressedImage):
        camera_id = msg.header.frame_id[:-len("_color_optical_frame")]
        millis = msg.header.stamp.sec * 1000 + \
            round(msg.header.stamp.nanosec / 1e6)
        name = f"{camera_id}-{millis}"
        cv_image = self.bridge.compressed_imgmsg_to_cv2(
            msg, desired_encoding="bgr8")
        cv2.imwrite(
            f"{self.path}/{name}.jpg", cv_image)

    def save_position(self, msg: PoseStamped):
        camera_id = msg.header.frame_id[:-len("_color_optical_frame")]
        millis = msg.header.stamp.sec * 1000 + \
            round(msg.header.stamp.nanosec / 1e6)
        name = f"{camera_id}-{millis}"
        file = open(f"{self.path}/{name}.txt", "w")
        pos_dict = {
            "x": round(msg.pose.position.x, 3),
            "y": round(msg.pose.position.y, 3),
            "z": round(msg.pose.position.z, 3),
        }
        file.write(str(pos_dict))

    def transform_detection(self, msg: Detection2DArray):
        to_frame = "base_link"
        from_frame = msg.header.frame_id
        try:
            transform = self.tf_buffer.lookup_transform(
                to_frame, from_frame, rclpy.time.Time())
            det: Detection2D
            for det in msg.detections:
                hyp: ObjectHypothesisWithPose = det.results[0]
                pose_stamped = PoseStamped()
                pose_stamped.header = msg.header
                pose_stamped.pose = hyp.pose.pose
                detection_transformed: PoseStamped = do_transform_pose_stamped(
                    pose_stamped, transform=transform)
                detection_transformed.header = det.header
                self.save_position(detection_transformed)

        except Exception as ex:
            self.get_logger().info(
                f'Error: {ex}')

    def create_directory(self):
        image_dir = "arch-images"
        # home_dir = os.path.expanduser("~")
        home_dir = "/home/rafal"
        self.path = os.path.join(home_dir, image_dir)
        try:
            os.makedirs(self.path, exist_ok=False)
            print(f"Directory {self.path} created successfully ")
        except OSError as err:
            print(f"Could not create a directory: {err}")


def main():
    try:
        rclpy.init()
        node = CubeNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
