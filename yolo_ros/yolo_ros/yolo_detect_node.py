import os.path
import rclpy
from rclpy import Parameter
from rclpy.lifecycle import Node, State, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from cv_bridge import CvBridge
from vision_msgs.msg import Detection2DArray
from ultralytics import YOLO
from tf2_ros import Buffer, TransformListener, TransformBroadcaster

import yolo_ros.detection_msg_utils as msg_utils
import yolo_ros.yolo_detect_impl as node_impl
from yolo_ros.detection_msg_utils import DetectionGroup


class YOLODetect(Node):
    def __init__(self) -> None:
        super().__init__("yolo_detect")

        self.declare_parameter("num_cameras", 1)
        self.declare_parameter("subscribe_depth", False)
        self.declare_parameter("use_compression", True)
        self.declare_parameter("rate", 10.0)
        self.declare_parameter("model", "")
        self.declare_parameter("confidence_threshold", 0.8)
        self.declare_parameter(
            "class_names",
            Parameter(
                "class_names", type_=Parameter.Type.STRING_ARRAY, value=[]
            ).get_parameter_value(),
        )
        self.declare_parameter(
            "class_radii",
            Parameter(
                "class_names", type_=Parameter.Type.DOUBLE_ARRAY, value=[]
            ).get_parameter_value(),
        )
        self.declare_parameter("world_frame", "odom")
        self.declare_parameter("merge_radius", 0.5)
        self.declare_parameter("temporal_window", 5)
        self.declare_parameter("temporal_threshold", 3)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("publish_annotated", True)

        result = self.trigger_configure()
        if result != TransitionCallbackReturn.SUCCESS:
            self.get_logger().error("Failed to auto-configure.")
            return

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.loaded_model = ""  # path to the currently loaded model
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.temporal_history: list[DetectionGroup] = []  # See: detection_msg_utils.py

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        try:
            # Read parameters.
            self.num_cameras = self.get_parameter("num_cameras").value
            self.subscribe_depth = self.get_parameter("subscribe_depth").value
            self.use_compression = self.get_parameter("use_compression").value
            self.rate = self.get_parameter("rate").value
            self.model = os.path.abspath(self.get_parameter("model").value)
            self.confidence_threshold = self.get_parameter("confidence_threshold").value
            self.class_names = self.get_parameter("class_names").value
            self.class_radii = self.get_parameter("class_radii").value
            self.world_frame = self.get_parameter("world_frame").value
            self.merge_radius = self.get_parameter("merge_radius").value
            self.temporal_window = self.get_parameter("temporal_window").value
            self.temporal_threshold = self.get_parameter("temporal_threshold").value
            self.publish_tf = self.get_parameter("publish_tf").value
            self.publish_annotated = self.get_parameter("publish_annotated").value

            # Validate parameters.
            if self.num_cameras < 1:
                self.get_logger().error(
                    "num_cameras must be at least 1, got " + str(self.num_cameras)
                )
                return TransitionCallbackReturn.ERROR
            if self.subscribe_depth and self.use_compression:
                self.get_logger().error(
                    "Compression of depth images is currently unsupported."
                )
                return TransitionCallbackReturn.ERROR
            if not os.path.isfile(self.model):
                self.get_logger().error("Model file not found: " + self.model)
                return TransitionCallbackReturn.ERROR
            if self.confidence_threshold < 0 or self.confidence_threshold > 1:
                self.get_logger().error(
                    "confidence_threshold must be between 0 and 1. Got: "
                    + str(self.confidence_threshold)
                )
                return TransitionCallbackReturn.ERROR
            if len(self.class_names) == 0:
                self.get_logger().error("class_names must not be empty.")
                return TransitionCallbackReturn.ERROR
            if len(self.class_radii) == 0:
                self.get_logger().error("class_radii must not be empty.")
                return TransitionCallbackReturn.ERROR
            if len(self.class_names) != len(self.class_radii):
                self.get_logger().error(
                    "class_names and class_radii must have the same length. Got: "
                    + str(len(self.class_names))
                    + " != "
                    + str(len(self.class_radii))
                )
                return TransitionCallbackReturn.ERROR
            if self.merge_radius < 0:
                self.get_logger().error(
                    "merge_radius must be non-negative. Got: " + str(self.merge_radius)
                )
                return TransitionCallbackReturn.ERROR
            if self.temporal_window < 0:
                self.get_logger().error(
                    "temporal_window must be non-negative. Got: "
                    + str(self.temporal_window)
                )
                return TransitionCallbackReturn.ERROR
            if self.temporal_threshold < 0:
                self.get_logger().error(
                    "temporal_threshold must be non-negative. Got: "
                    + str(self.temporal_threshold)
                )
                return TransitionCallbackReturn.ERROR
            if self.temporal_threshold > self.temporal_window:
                self.get_logger().error(
                    "temporal_threshold must not be greater than temporal_window. Got: "
                    + str(self.temporal_threshold)
                    + " > "
                    + str(self.temporal_window)
                )
                return TransitionCallbackReturn.ERROR

            # Load YOLO model if it was changed.
            if self.model != self.loaded_model:
                self.get_logger().info("Loading YOLO model: " + self.model)
                self.yolo = YOLO(self.model)
                self.loaded_model = self.model
                self.get_logger().info("YOLO is ready.")

            # Store latest image message on each topic.
            self.last_color_msgs = [None] * self.num_cameras
            self.last_depth_msgs = [None] * self.num_cameras
            self.last_info_msgs = [None] * self.num_cameras
            self.new_colors = [False] * self.num_cameras

            # Create subscribers.
            img_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.color_subs = []
            self.depth_subs = []
            self.info_subs = []
            for i in range(self.num_cameras):
                self.color_subs.append(
                    self.create_subscription(
                        CompressedImage if self.use_compression else Image,
                        f"color{i}/compressed" if self.use_compression else f"color{i}",
                        lambda color_msg, i=i: self.on_color(color_msg, i),
                        img_qos,
                    )
                )
                if self.subscribe_depth:
                    self.depth_subs.append(
                        self.create_subscription(
                            CompressedImage if self.use_compression else Image,
                            (
                                f"depth{i}/compressed"
                                if self.use_compression
                                else f"depth{i}"
                            ),
                            lambda depth_msg, i=i: self.on_depth(depth_msg, i),
                            img_qos,
                        )
                    )
                self.info_subs.append(
                    self.create_subscription(
                        CameraInfo,
                        f"info{i}",
                        lambda msg, i=i: self.on_info(msg, i),
                        10,
                    )
                )

            # Create detection publisher.
            self.detection_pub = self.create_publisher(
                Detection2DArray, "detections", 10
            )

            # Optionally create a TF broadcaster.
            if self.publish_tf:
                self.tf_broadcaster = TransformBroadcaster(self)

            # Optionally create annotated image publisher.
            if self.publish_annotated:
                self.annotated_pubs = []
                for i in range(self.num_cameras):
                    if self.use_compression:
                        self.annotated_pubs.append(
                            self.create_publisher(
                                CompressedImage, f"annotated{i}/compressed", img_qos
                            )
                        )
                    else:
                        self.annotated_pubs.append(
                            self.create_publisher(Image, f"annotated{i}", img_qos)
                        )

            # Start processing images at the specified rate.
            self.create_timer(1.0 / self.rate, self.detect)
        except Exception as e:
            self.get_logger().error("Error during activation: " + str(e))
            return TransitionCallbackReturn.ERROR

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.destroy_timer(self.detect)
        if self.publish_tf:
            self.destroy_publisher(self.tf_broadcaster.pub_tf)
        self.destroy_publisher(self.detection_pub)
        for sub in self.color_subs:
            self.destroy_subscription(sub)
        for sub in self.depth_subs:
            self.destroy_subscription(sub)
        for sub in self.info_subs:
            self.destroy_subscription(sub)
        # Never unload the YOLO model because it is expensive to load again.

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def on_color(self, color_msg: Image | CompressedImage, i: int) -> None:
        self.last_color_msgs[i] = color_msg
        self.new_colors[i] = True

    def on_depth(self, depth_msg: Image | CompressedImage, i: int) -> None:
        self.last_depth_msgs[i] = depth_msg

    def on_info(self, msg: CameraInfo, i: int) -> None:
        self.last_info_msgs[i] = msg

    # Reads cached image messages, processes them and publishes data on topics.
    def detect(self) -> None:
        # Get the latest messages.
        color_msgs = [
            self.last_color_msgs[i]
            for i in range(self.num_cameras)
            if self.new_colors[i]
        ]
        depth_msgs = [
            self.last_depth_msgs[i]
            for i in range(self.num_cameras)
            if self.new_colors[i]
        ]
        info_msgs = [
            self.last_info_msgs[i]
            for i in range(self.num_cameras)
            if self.new_colors[i]
        ]
        msg_camera_indices = [i for i in range(self.num_cameras) if self.new_colors[i]]
        self.new_colors = [False] * self.num_cameras

        if (
            len(color_msgs) == 0
            or not all(color_msgs)
            or (self.subscribe_depth and not all(depth_msgs))
            or not all(info_msgs)
        ):
            return

        # Decode images.
        depth_images = None
        if self.use_compression:
            color_images = [
                self.bridge.compressed_imgmsg_to_cv2(msg) for msg in color_msgs
            ]
            if self.subscribe_depth:
                # depth_images = []
                # for msg in depth_msgs:
                #     # [:12] removes header inserted by compressed_depth_image_transport.
                #     data = np.frombuffer(msg.data[12:], np.uint8)
                #     img = cv2.imdecode(data, cv2.IMREAD_UNCHANGED)
                #     img *= 255 # Convert quantized 8-bit to 16-bit.
                #     depth_images.append(img)
                #     self.get_logger().info(f"Depth image msg: {msg}")
                #     data = np.frombuffer(msg.data, np.uint8)
                #     self.get_logger().info(f"Depth image data: {data.shape}")
                raise NotImplementedError(
                    "Depth image decompression is not implemented."
                )

        else:
            color_images = [self.bridge.imgmsg_to_cv2(msg) for msg in color_msgs]
            if self.subscribe_depth:
                depth_images = [self.bridge.imgmsg_to_cv2(msg) for msg in depth_msgs]

        # Run YOLO.
        results = self.yolo.predict(color_images, conf=self.confidence_threshold)

        # Convert YOLO results to Detection2DArray.
        stamps = [msg.header.stamp for msg in color_msgs]
        detections: Detection2DArray = node_impl.detection_array_from_yolo_results(
            self, results, stamps
        )

        # If anything was detected...
        if len(detections.detections) > 0:
            # Process detections.
            detections = node_impl.add_3d_positions_to_detections(
                self, detections, depth_images, info_msgs
            )
            detections = msg_utils.merge_detections_by_distance(
                detections, self.merge_radius
            )
            detections = msg_utils.temporal_filter(
                detections,
                self.temporal_history,
                self.temporal_window,
                self.temporal_threshold,
                self.merge_radius,
            )

            # Publish detections.
            self.detection_pub.publish(detections)

            # Optionally publish TFs.
            if self.publish_tf:
                transforms = msg_utils.detections_to_transforms(detections)
                self.tf_broadcaster.sendTransform(transforms)

        # Regardless of whether there are any detections, optionally publish annotated images.
        if self.publish_annotated:
            for i, result in enumerate(results):
                annotated = result.plot()

                if self.use_compression:
                    annotated_msg = self.bridge.cv2_to_compressed_imgmsg(annotated)
                else:
                    annotated_msg = self.bridge.cv2_to_imgmsg(annotated, "bgr8")

                camera_index = msg_camera_indices[i]
                annotated_msg.header = color_msgs[i].header
                self.annotated_pubs[camera_index].publish(annotated_msg)


def main():
    try:
        rclpy.init()
        node = YOLODetect()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
