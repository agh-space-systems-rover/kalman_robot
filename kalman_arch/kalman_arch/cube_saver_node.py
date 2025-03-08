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
from collections import deque
from dataclasses import dataclass
from typing import Dict
import time


@dataclass
class ImageData:
    camera_id: str
    timestamp: int
    image: cv2.Mat


@dataclass
class DetectionData:
    camera_id: str
    timestamp: int
    position: Dict[str, float]


class CubeNode(Node):
    def __init__(self):
        super().__init__("cube_saver")
        REALSENSE_CAMERAS = ["d455_front",
                             "d455_left", "d455_right", "d455_back"]

        self.declare_parameter("buffer_size", 100)
        self.declare_parameter("buffer_cleanup_time", 5000)

        self.BUFFER_SIZE = self.get_parameter("buffer_size").value
        self.BUFFER_CLEANUP_TIME = self.get_parameter(
            "buffer_cleanup_time").value

        self.image_buffer = deque(maxlen=self.BUFFER_SIZE)
        self.detection_buffer = deque(maxlen=self.BUFFER_SIZE)

        img_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(
            Detection2DArray, "/yolo_detections", self.detection_cb, img_qos)
        for rgbd_ids in REALSENSE_CAMERAS:
            self.create_subscription(
                CompressedImage, f"/{rgbd_ids}/yolo_annotated/compressed", self.image_cb, img_qos)

        # Create timer for periodic matching
        self.create_timer(1, self.match_and_save_timer_cb)
        # Convert to milliseconds
        self.tf_buffer = Buffer()
        self.bridge = CvBridge()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_directory()

    def image_cb(self, msg: CompressedImage):
        camera_id = msg.header.frame_id[:-len("_color_optical_frame")]
        millis = msg.header.stamp.sec * 1000 + \
            round(msg.header.stamp.nanosec / 1e6)
        cv_image = self.bridge.compressed_imgmsg_to_cv2(
            msg, desired_encoding="bgr8")

        image_data = ImageData(
            camera_id=camera_id,
            timestamp=millis,
            image=cv_image
        )
        self.image_buffer.append(image_data)

    def detection_cb(self, msg: Detection2DArray):
        to_frame = "base_link"
        from_frame = msg.header.frame_id
        try:
            transform = self.tf_buffer.lookup_transform(
                to_frame, from_frame, rclpy.time.Time())

            det: Detection2D
            for det in msg.detections:
                hyp: ObjectHypothesisWithPose = det.results[0]
                confidence = hyp.hypothesis.score
                cube_color = hyp.hypothesis.class_id
                pose_stamped = PoseStamped()
                pose_stamped.header = det.header
                pose_stamped.pose = hyp.pose.pose
                detection_transformed: PoseStamped = do_transform_pose_stamped(
                    pose_stamped, transform=transform)

                millis = det.header.stamp.sec * 1000 + \
                    round(det.header.stamp.nanosec / 1e6)
                camera_id = det.header.frame_id[:-len("_color_optical_frame")]

                pos_list = " ".join([
                    cube_color,
                    str(round(confidence, 3)),
                    str(round(detection_transformed.pose.position.x, 3)),
                    str(round(detection_transformed.pose.position.y, 3)),
                    str(round(detection_transformed.pose.position.z, 3))
                ])

                detection_data = DetectionData(
                    camera_id=camera_id,
                    timestamp=millis,
                    position=pos_list
                )
                self.detection_buffer.append(detection_data)

        except Exception as ex:
            self.get_logger().info(f'Error: {ex}')

    def match_and_save_timer_cb(self):
        """Match and save images with their corresponding detections only if timestamps exactly match."""
        # Create sets of timestamps for quick lookup
        img_timestamps = {(img.camera_id, img.timestamp)                          : img for img in self.image_buffer}
        det_timestamps = {(det.camera_id, det.timestamp)                          : det for det in self.detection_buffer}

        # Find common timestamps
        common_keys = set(img_timestamps.keys()) & set(det_timestamps.keys())

        # Save matched pairs
        for key in common_keys:
            img_data = img_timestamps[key]
            det_data = det_timestamps[key]
            self.save_matched_data(img_data, det_data)

            # Remove processed items from buffers
            self.image_buffer = deque(
                [x for x in self.image_buffer if (
                    x.camera_id, x.timestamp) != key],
                maxlen=self.BUFFER_SIZE
            )
            self.detection_buffer = deque(
                [x for x in self.detection_buffer if (  # Convert to milliseconds
                    x.camera_id, x.timestamp) != key],
                maxlen=self.BUFFER_SIZE
            )

        # Clean up old entries
        self.cleanup_buffers()

    def save_matched_data(self, img_data: ImageData, det_data: DetectionData):
        """Save matched image and detection data."""
        name = f"{img_data.camera_id}-{img_data.timestamp}"

        # Save image
        cv2.imwrite(
            f"{self.path}/{name}.jpg",
            img_data.image,
            [cv2.IMWRITE_JPEG_QUALITY, 30]
        )

        # Save position
        with open(f"{self.path}/{name}.txt", "w") as file:
            file.write(str(det_data.position))

    def cleanup_buffers(self):
        """Remove old entries from buffers."""
        current_time = time.time() * 1000  # Convert to milliseconds

        # Remove entries older than 5 seconds
        cutoff_time = current_time - self.BUFFER_CLEANUP_TIME

        self.image_buffer = deque(
            [x for x in self.image_buffer if x.timestamp > cutoff_time],
            maxlen=self.BUFFER_SIZE
        )
        self.detection_buffer = deque(
            [x for x in self.detection_buffer if x.timestamp > cutoff_time],
            maxlen=self.BUFFER_SIZE
        )

    def create_directory(self):
        self.path = os.path.expanduser("~/arch/cube-saves")
        try:
            os.makedirs(self.path, exist_ok=True)
            self.get_logger().info(
                f"Directory {self.path} created successfully")
        except OSError as err:
            self.get_logger().info(f"Could not create a directory: {err}")


def main():
    try:
        rclpy.init()
        node = CubeNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
