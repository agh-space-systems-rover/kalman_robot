"""Select the darkest boulder detection using synchronized camera images and TF2."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
import yolo_ros.detection_msg_utils as msg_utils
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection2D, Detection2DArray
from visualization_msgs.msg import Marker


@dataclass(frozen=True)
class BoulderCandidate:
    """A boulder that passed geometric filters and has a measured luma score."""

    luma: float
    position: PointStamped


@dataclass(frozen=True)
class CameraIntrinsics:
    """Pinhole camera focal lengths extracted from CameraInfo.K."""

    fx: float
    fy: float


def parse_rgbd_ids(rgbd_ids_param: str) -> list[str]:
    """Split a space-separated camera id parameter into a clean list."""
    return [camera_id for camera_id in rgbd_ids_param.split(" ") if camera_id]


def extract_camera_intrinsics(camera_info: CameraInfo) -> CameraIntrinsics | None:
    """Extract fx and fy from the 3x3 camera matrix K."""
    if len(camera_info.k) < 5:
        return None

    fx = float(camera_info.k[0])
    fy = float(camera_info.k[4])
    if fx <= 0.0 or fy <= 0.0:
        return None

    return CameraIntrinsics(fx=fx, fy=fy)


def resolve_camera_intrinsics(
    camera_id: str,
    camera_intrinsics: dict[str, CameraIntrinsics],
    default_focal_length_px: float,
    warned_cameras: set[str],
    logger,
) -> CameraIntrinsics | None:
    """Return stored intrinsics, a configured fallback, or None."""
    stored = camera_intrinsics.get(camera_id)
    if stored is not None:
        return stored

    if default_focal_length_px > 0.0:
        if camera_id not in warned_cameras:
            logger.warning(
                "CameraInfo not yet received for '%s'; using default focal length %.1f px.",
                camera_id,
                default_focal_length_px,
            )
            warned_cameras.add(camera_id)
        return CameraIntrinsics(
            fx=default_focal_length_px,
            fy=default_focal_length_px,
        )

    if camera_id not in warned_cameras:
        logger.warning(
            "Skipping detections for camera '%s': CameraInfo not received and "
            "'default_focal_length_px' is unset.",
            camera_id,
        )
        warned_cameras.add(camera_id)
    return None


def detection_camera_depth(detection: Detection2D) -> float | None:
    """Return positive Z-depth of the detection in its native camera frame."""
    if not detection.results:
        return None

    depth = float(detection.results[0].pose.pose.position.z)
    if depth <= 0.0:
        return None

    return depth


def compute_physical_bbox_size_m(
    detection: Detection2D,
    depth_m: float,
    intrinsics: CameraIntrinsics,
) -> tuple[float, float] | None:
    """Estimate physical bbox width and height in meters using pinhole geometry."""
    if depth_m <= 0.0 or intrinsics.fx <= 0.0 or intrinsics.fy <= 0.0:
        return None

    physical_width_m = (float(detection.bbox.size_x) * depth_m) / intrinsics.fx
    physical_height_m = (float(detection.bbox.size_y) * depth_m) / intrinsics.fy
    return physical_width_m, physical_height_m


def boulder_passes_physical_size_filter(
    physical_width_m: float,
    physical_height_m: float,
    min_boulder_size_m: float,
    max_boulder_size_m: float,
) -> bool:
    """Return True when both physical bbox dimensions are within configured bounds."""
    return (
        min_boulder_size_m <= physical_width_m <= max_boulder_size_m
        and min_boulder_size_m <= physical_height_m <= max_boulder_size_m
    )


def match_camera_id(detection: Detection2D, rgbd_ids: Sequence[str]) -> str | None:
    """Map a detection header frame to one of the configured camera ids."""
    frame_id = detection.header.frame_id
    for camera_id in rgbd_ids:
        if camera_id in frame_id:
            return camera_id
    return None


def compute_inner_roi_bounds(
    center_x: int,
    center_y: int,
    bbox_width: float,
    bbox_height: float,
    image_width: int,
    image_height: int,
) -> tuple[int, int, int, int] | None:
    """Return safe [x1, y1, x2, y2) bounds for the inner 50% of a bounding box."""
    if image_width <= 0 or image_height <= 0:
        return None

    half_width = max(int(bbox_width / 4), 0)
    half_height = max(int(bbox_height / 4), 0)
    if half_width == 0 or half_height == 0:
        return None

    x1 = max(0, min(image_width, center_x - half_width))
    y1 = max(0, min(image_height, center_y - half_height))
    x2 = max(0, min(image_width, center_x + half_width))
    y2 = max(0, min(image_height, center_y + half_height))

    if x2 <= x1 or y2 <= y1:
        return None

    return x1, y1, x2, y2


def extract_inner_roi_luma(
    cv_image: np.ndarray,
    center_x: int,
    center_y: int,
    bbox_width: float,
    bbox_height: float,
) -> float | None:
    """Compute mean grayscale luma for the inner 50% crop of a bounding box."""
    image_height, image_width = cv_image.shape[:2]
    bounds = compute_inner_roi_bounds(
        center_x,
        center_y,
        bbox_width,
        bbox_height,
        image_width,
        image_height,
    )
    if bounds is None:
        return None

    x1, y1, x2, y2 = bounds
    inner_roi = cv_image[y1:y2, x1:x2]
    if inner_roi.size == 0:
        return None

    if inner_roi.ndim == 2:
        gray_roi = inner_roi
    elif inner_roi.shape[2] >= 3:
        gray_roi = cv2.cvtColor(inner_roi, cv2.COLOR_BGR2GRAY)
    else:
        return None

    return float(np.mean(gray_roi))


def transform_detection_position(
    detection: Detection2D,
    target_frame: str,
    tf_buffer: Buffer,
    logger,
) -> PointStamped | None:
    """Transform a detection pose into the configured global frame."""
    pose_stamped = PoseStamped()
    pose_stamped.header = detection.header
    pose_stamped.pose = detection.results[0].pose.pose

    if pose_stamped.header.frame_id == target_frame:
        point = PointStamped()
        point.header = pose_stamped.header
        point.point = pose_stamped.pose.position
        return point

    try:
        transform = tf_buffer.lookup_transform(
            target_frame,
            pose_stamped.header.frame_id,
            pose_stamped.header.stamp,
        )
        transformed_pose = do_transform_pose_stamped(pose_stamped, transform)
    except TransformException as exc:
        logger.warning(
            "Skipping detection in frame '%s': failed to transform to '%s': %s",
            pose_stamped.header.frame_id,
            target_frame,
            exc,
        )
        return None

    point = PointStamped()
    point.header = transformed_pose.header
    point.point = transformed_pose.pose.position
    return point


def point_distance(point: PointStamped) -> float:
    """Euclidean distance of a point from the origin of its frame."""
    return math.hypot(point.point.x, point.point.y, point.point.z)


def evaluate_boulder_candidate(
    detection: Detection2D,
    camera_id: str,
    cv_image: np.ndarray,
    camera_intrinsics: dict[str, CameraIntrinsics],
    default_focal_length_px: float,
    warned_cameras: set[str],
    min_boulder_size_m: float,
    max_boulder_size_m: float,
    max_distance: float,
    global_frame: str,
    tf_buffer: Buffer,
    logger,
) -> BoulderCandidate | None:
    """Apply physical-size, distance, and luma checks for a single detection."""
    if not detection.results:
        logger.debug("Skipping detection without pose results.")
        return None

    intrinsics = resolve_camera_intrinsics(
        camera_id,
        camera_intrinsics,
        default_focal_length_px,
        warned_cameras,
        logger,
    )
    if intrinsics is None:
        return None

    depth_m = detection_camera_depth(detection)
    if depth_m is None:
        logger.debug(
            "Skipping detection in frame '%s': invalid or non-positive camera depth.",
            detection.header.frame_id,
        )
        return None

    physical_size = compute_physical_bbox_size_m(detection, depth_m, intrinsics)
    if physical_size is None:
        logger.debug(
            "Skipping detection in frame '%s': failed to compute physical bbox size.",
            detection.header.frame_id,
        )
        return None

    physical_width_m, physical_height_m = physical_size
    if not boulder_passes_physical_size_filter(
        physical_width_m,
        physical_height_m,
        min_boulder_size_m,
        max_boulder_size_m,
    ):
        logger.debug(
            "Skipping detection in frame '%s': physical size %.3fm x %.3fm outside "
            "[%.3f, %.3f]m.",
            detection.header.frame_id,
            physical_width_m,
            physical_height_m,
            min_boulder_size_m,
            max_boulder_size_m,
        )
        return None

    transformed_point = transform_detection_position(
        detection, global_frame, tf_buffer, logger
    )
    if transformed_point is None:
        return None

    if point_distance(transformed_point) > max_distance:
        return None

    center_x = int(detection.bbox.center.position.x)
    center_y = int(detection.bbox.center.position.y)

    try:
        luma = extract_inner_roi_luma(
            cv_image,
            center_x,
            center_y,
            detection.bbox.size_x,
            detection.bbox.size_y,
        )
    except (cv2.error, ValueError, TypeError) as exc:
        logger.error(
            "Failed to compute luma for detection in frame '%s': %s",
            detection.header.frame_id,
            exc,
        )
        return None

    if luma is None:
        logger.debug(
            "Skipping detection in frame '%s': ROI outside image bounds.",
            detection.header.frame_id,
        )
        return None

    return BoulderCandidate(luma=luma, position=transformed_point)


def select_darkest_candidate(
    detections: Sequence[Detection2D],
    images_by_camera: dict[str, np.ndarray],
    rgbd_ids: Sequence[str],
    camera_intrinsics: dict[str, CameraIntrinsics],
    default_focal_length_px: float,
    warned_cameras: set[str],
    min_boulder_size_m: float,
    max_boulder_size_m: float,
    max_distance: float,
    global_frame: str,
    tf_buffer: Buffer,
    logger,
) -> BoulderCandidate | None:
    """Return the darkest valid boulder candidate across all detections."""
    darkest: BoulderCandidate | None = None

    for detection in detections:
        camera_id = match_camera_id(detection, rgbd_ids)
        if camera_id is None:
            logger.debug(
                "Skipping detection in frame '%s': no matching camera id.",
                detection.header.frame_id,
            )
            continue

        cv_image = images_by_camera.get(camera_id)
        if cv_image is None:
            logger.debug(
                "Skipping detection for camera '%s': synchronized image missing.",
                camera_id,
            )
            continue

        candidate = evaluate_boulder_candidate(
            detection,
            camera_id,
            cv_image,
            camera_intrinsics,
            default_focal_length_px,
            warned_cameras,
            min_boulder_size_m,
            max_boulder_size_m,
            max_distance,
            global_frame,
            tf_buffer,
            logger,
        )
        if candidate is None:
            continue

        if darkest is None or candidate.luma < darkest.luma:
            darkest = candidate

    return darkest


def apply_detection_filters(
    detections_msg: Detection2DArray,
    temporal_history: list[msg_utils.DetectionGroup],
    temporal_window: int,
    temporal_threshold: int,
    group_radius: float,
    merge_radius: float,
) -> Detection2DArray:
    """Merge nearby detections and apply temporal stabilization."""
    if not detections_msg.detections:
        empty_msg = Detection2DArray()
        empty_msg.header = detections_msg.header
        return msg_utils.temporal_filter(
            empty_msg,
            temporal_history,
            temporal_window,
            temporal_threshold,
            group_radius,
        )

    merged_msg = msg_utils.merge_detections_by_distance(detections_msg, merge_radius)
    return msg_utils.temporal_filter(
        merged_msg,
        temporal_history,
        temporal_window,
        temporal_threshold,
        group_radius,
    )


class DarkestBoulderFilter(Node):
    """Find the darkest boulder among temporally filtered YOLO detections."""

    def __init__(self) -> None:
        super().__init__("darkest_boulder_filter")

        self.declare_parameter("temporal_window", 4)
        self.declare_parameter("temporal_threshold", 2)
        self.declare_parameter("group_radius", 0.5)
        self.declare_parameter("merge_radius", 0.5)
        self.declare_parameter("min_boulder_size_m", 0.1)
        self.declare_parameter("max_boulder_size_m", 0.5)
        self.declare_parameter("max_distance", 5.0)
        self.declare_parameter("global_frame", "base_link")
        self.declare_parameter("rgbd_ids", "d455_front")
        self.declare_parameter("detections_topic", "yolo_detections")
        self.declare_parameter("sync_queue_size", 10)
        self.declare_parameter("sync_slop", 0.1)
        self.declare_parameter("default_focal_length_px", 0.0)

        self.temporal_window = int(self.get_parameter("temporal_window").value)
        self.temporal_threshold = int(self.get_parameter("temporal_threshold").value)
        self.group_radius = float(self.get_parameter("group_radius").value)
        self.merge_radius = float(self.get_parameter("merge_radius").value)
        self.min_boulder_size_m = float(self.get_parameter("min_boulder_size_m").value)
        self.max_boulder_size_m = float(self.get_parameter("max_boulder_size_m").value)
        self.max_distance = float(self.get_parameter("max_distance").value)
        self.global_frame = str(self.get_parameter("global_frame").value)
        self.rgbd_ids = parse_rgbd_ids(str(self.get_parameter("rgbd_ids").value))
        self.default_focal_length_px = float(
            self.get_parameter("default_focal_length_px").value
        )
        detections_topic = str(self.get_parameter("detections_topic").value)
        sync_queue_size = int(self.get_parameter("sync_queue_size").value)
        sync_slop = float(self.get_parameter("sync_slop").value)

        self.temporal_history: list[msg_utils.DetectionGroup] = []
        self.camera_intrinsics: dict[str, CameraIntrinsics] = {}
        self.missing_intrinsics_warned: set[str] = set()
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        camera_info_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.target_pub = self.create_publisher(PointStamped, "darkest_boulder_target", 10)
        self.filtered_pub = self.create_publisher(
            Detection2DArray, "boulder_detections_filtered", 10
        )
        self.marker_pub = self.create_publisher(Marker, "darkest_boulder_marker", 10)

        for camera_id in self.rgbd_ids:
            self.create_subscription(
                CameraInfo,
                f"/{camera_id}/color/camera_info",
                lambda msg, cid=camera_id: self._camera_info_callback(msg, cid),
                camera_info_qos,
            )

        if not self.rgbd_ids:
            self.get_logger().warn(
                "Parameter 'rgbd_ids' is empty; luma scoring is disabled until cameras are configured."
            )
            self.create_subscription(
                Detection2DArray,
                detections_topic,
                self.detections_only_callback,
                sensor_qos,
            )
        else:
            self._setup_synchronized_subscribers(
                detections_topic, sensor_qos, sync_queue_size, sync_slop
            )

        self.get_logger().info(
            f"Darkest boulder filter ready (global_frame='{self.global_frame}', cameras={self.rgbd_ids}, ",
        )
        self.get_logger().info(
            f"physical size=[{self.min_boulder_size_m}, {self.max_boulder_size_m}] m)."
        )

    def _camera_info_callback(self, msg: CameraInfo, camera_id: str) -> None:
        intrinsics = extract_camera_intrinsics(msg)
        if intrinsics is None:
            self.get_logger().error(
                "Received invalid CameraInfo for '%s'; fx/fy must be positive.",
                camera_id,
            )
            return

        self.camera_intrinsics[camera_id] = intrinsics
        self.missing_intrinsics_warned.discard(camera_id)
        self.get_logger().info(
            f"Updated intrinsics for '{camera_id}': fx={intrinsics.fx}, fy={intrinsics.fy}.",
        )

    def _setup_synchronized_subscribers(
        self,
        detections_topic: str,
        qos: QoSProfile,
        queue_size: int,
        slop: float,
    ) -> None:
        detection_sub = message_filters.Subscriber(
            self, Detection2DArray, detections_topic, qos_profile=qos
        )
        image_subs = [
            message_filters.Subscriber(
                self,
                CompressedImage,
                f"/{camera_id}/color/image_raw/compressed",
                qos_profile=qos,
            )
            for camera_id in self.rgbd_ids
        ]

        synchronizer = message_filters.ApproximateTimeSynchronizer(
            [detection_sub, *image_subs],
            queue_size=queue_size,
            slop=slop,
        )
        synchronizer.registerCallback(self.synchronized_callback)

    def detections_only_callback(self, detections_msg: Detection2DArray) -> None:
        """Handle the no-camera configuration by publishing filtered detections only."""
        filtered_msg = apply_detection_filters(
            detections_msg,
            self.temporal_history,
            self.temporal_window,
            self.temporal_threshold,
            self.group_radius,
            self.merge_radius,
        )
        self.filtered_pub.publish(filtered_msg)

    def synchronized_callback(
        self,
        detections_msg: Detection2DArray,
        *image_msgs: CompressedImage,
    ) -> None:
        filtered_msg = apply_detection_filters(
            detections_msg,
            self.temporal_history,
            self.temporal_window,
            self.temporal_threshold,
            self.group_radius,
            self.merge_radius,
        )
        self.filtered_pub.publish(filtered_msg)

        if not filtered_msg.detections:
            return

        images_by_camera: dict[str, np.ndarray] = {}
        for camera_id, image_msg in zip(self.rgbd_ids, image_msgs, strict=True):
            try:
                images_by_camera[camera_id] = self.bridge.compressed_imgmsg_to_cv2(
                    image_msg, desired_encoding="bgr8"
                )
            except CvBridgeError as exc:
                self.get_logger().error(
                    "Failed to decode synchronized image for camera '%s': %s",
                    camera_id,
                    exc,
                )
                return

        darkest = select_darkest_candidate(
            filtered_msg.detections,
            images_by_camera,
            self.rgbd_ids,
            self.camera_intrinsics,
            self.default_focal_length_px,
            self.missing_intrinsics_warned,
            self.min_boulder_size_m,
            self.max_boulder_size_m,
            self.max_distance,
            self.global_frame,
            self.tf_buffer,
            self.get_logger(),
        )
        if darkest is None:
            return

        self.publish_rviz_marker(darkest.position)
        self.publish_target(darkest.position)

    def publish_rviz_marker(self, point: PointStamped) -> None:
        marker = Marker()
        marker.header.frame_id = point.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "boulder_target"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = point.point
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        self.marker_pub.publish(marker)

    def publish_target(self, point: PointStamped) -> None:
        target_msg = PointStamped()
        target_msg.header.frame_id = point.header.frame_id
        target_msg.header.stamp = self.get_clock().now().to_msg()
        target_msg.point = point.point
        self.target_pub.publish(target_msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DarkestBoulderFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
