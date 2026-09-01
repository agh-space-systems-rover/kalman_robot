#!/usr/bin/env python3
"""Locate a YOLO-detected rock using depth inside its bounding box."""

from collections import deque
import math

import cv2
import numpy as np
import rclpy
import tf2_geometry_msgs  # noqa: F401 - registers stamped geometry conversions
import tf2_ros
from geometry_msgs.msg import (
    PointStamped,
    Pose,
    PoseArray,
    PoseStamped,
)
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import Empty
from std_srvs.srv import Trigger
from vision_msgs.msg import Detection2DArray


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class RockDetector(Node):
    """Fuse YOLO bounding boxes with RealSense depth."""

    def __init__(self):
        super().__init__("rock_detector")

        self.declare_parameter(
            "depth_topic", "/d455_arm_wheel/depth/image_raw"
        )
        self.declare_parameter(
            "camera_info_topic", "/d455_arm_wheel/color/camera_info"
        )
        self.declare_parameter("detections_topic", "/yolo_detections")
        self.declare_parameter("rock_class", "stone")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("end_effector_frame", "arm_link_end")
        self.declare_parameter("min_depth", 0.2)
        self.declare_parameter("max_depth", 1.5)
        self.declare_parameter("bbox_scale", 1.0)
        self.declare_parameter("depth_band", 0.04)
        self.declare_parameter("min_depth_pixels", 20)
        self.declare_parameter("min_depth_fraction", 0.08)
        self.declare_parameter("detection_timeout", 0.6)
        self.declare_parameter("sample_jump_tolerance", 0.05)
        self.declare_parameter("max_position_spread", 0.025)
        self.declare_parameter("track_timeout", 2.0)
        self.declare_parameter("track_match_distance", 1.5)
        self.declare_parameter("averaging_window", 3.0)
        self.declare_parameter("min_samples", 10)
        # Vertical distance between the stone and arm_link_end.
        self.declare_parameter("standoff", 0.15)
        # The goal is an arm_link_end position expressed in base_link.
        self.declare_parameter("min_target_distance_from_base", 0.40)
        self.declare_parameter("target_publish_rate", 10.0)
        self.declare_parameter("arrival_tolerance", 0.02)
        self.declare_parameter("approach_timeout", 15.0)

        self.depth_topic = self.get_parameter("depth_topic").value.rstrip("/")
        self.rock_class = self.get_parameter("rock_class").value
        self.base_frame = self.get_parameter("base_frame").value
        self.end_effector_frame = self.get_parameter("end_effector_frame").value
        self.min_depth = float(self.get_parameter("min_depth").value)
        self.max_depth = float(self.get_parameter("max_depth").value)
        self.bbox_scale = float(self.get_parameter("bbox_scale").value)
        self.depth_band = float(self.get_parameter("depth_band").value)
        self.min_depth_pixels = int(self.get_parameter("min_depth_pixels").value)
        self.min_depth_fraction = float(
            self.get_parameter("min_depth_fraction").value
        )
        self.detection_timeout = float(
            self.get_parameter("detection_timeout").value
        )
        self.sample_jump_tolerance = float(
            self.get_parameter("sample_jump_tolerance").value
        )
        self.max_position_spread = float(
            self.get_parameter("max_position_spread").value
        )
        self.track_timeout = float(
            self.get_parameter("track_timeout").value
        )
        self.track_match_distance = float(
            self.get_parameter("track_match_distance").value
        )
        self.averaging_window = float(
            self.get_parameter("averaging_window").value
        )
        self.min_samples = int(self.get_parameter("min_samples").value)
        self.standoff = float(self.get_parameter("standoff").value)
        self.min_target_distance_from_base = float(
            self.get_parameter("min_target_distance_from_base").value
        )
        self.arrival_tolerance = float(
            self.get_parameter("arrival_tolerance").value
        )
        self.approach_timeout = float(
            self.get_parameter("approach_timeout").value
        )

        self.camera_info = None
        self.latest_bbox = None
        self.latest_bboxes = []
        self.latest_detection_stamp = None
        self.last_sample_detection_stamp = None
        self.samples = deque()
        self.rock_tracks = []
        self.latest_tracks = []
        self.approach_target = None
        self.approach_start_time = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.target_pose_pub = self.create_publisher(
            PoseStamped, "target_pose", 10
        )
        self.rock_pose_pub = self.create_publisher(
            PoseStamped, "rock_pose", 10
        )
        self.rock_positions_pub = self.create_publisher(
            PoseArray, "rock_positions", 10
        )
        self.rock_detections_pub = self.create_publisher(
            Detection2DArray, "rock_detections", 10
        )

        depth_type = (
            CompressedImage
            if self.depth_topic.endswith("/compressedDepth")
            else Image
        )
        depth_callback = (
            self.on_compressed_depth
            if depth_type is CompressedImage
            else self.on_raw_depth
        )
        self.depth_sub = self.create_subscription(
            depth_type,
            self.depth_topic,
            depth_callback,
            qos_profile_sensor_data,
        )
        self.info_sub = self.create_subscription(
            CameraInfo,
            self.get_parameter("camera_info_topic").value,
            self.on_camera_info,
            qos_profile_sensor_data,
        )
        self.detections_sub = self.create_subscription(
            Detection2DArray,
            self.get_parameter("detections_topic").value,
            self.on_detections,
            10,
        )
        self.selected_target_sub = self.create_subscription(
            PoseStamped,
            "selected_rock_target",
            self.on_selected_target,
            10,
        )
        self.stop_target_sub = self.create_subscription(
            Empty,
            "stop_rock_target",
            self.on_stop_target,
            10,
        )

        self.approach_srv = self.create_service(
            Trigger, "approach_rock", self.on_approach_rock
        )
        self.stop_srv = self.create_service(
            Trigger, "stop_approach", self.on_stop_approach
        )
        publish_rate = float(
            self.get_parameter("target_publish_rate").value
        )
        self.target_timer = self.create_timer(
            1.0 / publish_rate, self.publish_target
        )

        self.get_logger().info(
            f"Rock detector using {self.depth_topic} and "
            f"{self.get_parameter('detections_topic').value}"
        )

    def on_camera_info(self, msg):
        self.camera_info = msg

    def on_detections(self, msg):
        candidates = []
        for detection in msg.detections:
            if (
                self.camera_info is not None
                and detection.header.frame_id
                and self.camera_info.header.frame_id
                and detection.header.frame_id != self.camera_info.header.frame_id
            ):
                continue
            if not detection.results:
                continue
            hypothesis = detection.results[0].hypothesis
            if hypothesis.class_id != self.rock_class:
                continue
            bbox = detection.bbox
            if bbox.size_x <= 1.0 or bbox.size_y <= 1.0:
                continue
            candidates.append(
                (
                    float(hypothesis.score),
                    float(bbox.center.position.x),
                    float(bbox.center.position.y),
                    float(bbox.size_x),
                    float(bbox.size_y),
                    detection.header,
                    detection,
                )
            )

        if not candidates:
            self.latest_bbox = None
            self.latest_bboxes = []
            self.latest_tracks = []
            self.latest_detection_stamp = None
            self.prune_rock_tracks(self.get_clock().now())
            self.publish_empty_rock_results()
            return

        # Stable left-to-right ordering associates each YOLO box with the
        # pose at the same index in rock_positions.
        self.latest_bboxes = sorted(
            candidates, key=lambda item: (item[1], item[2])
        )
        self.associate_rock_tracks(self.latest_bboxes)
        # Keep the highest-confidence rock as the legacy service target.
        self.latest_bbox = max(
            candidates, key=lambda item: (item[0], item[3] * item[4])
        )
        detection_header = self.latest_bbox[5]
        self.latest_detection_stamp = stamp_seconds(detection_header.stamp)

    def prune_rock_tracks(self, now):
        self.rock_tracks = [
            track
            for track in self.rock_tracks
            if (now - track["last_seen"]).nanoseconds * 1e-9
            <= self.track_timeout
        ]

    def associate_rock_tracks(self, records):
        now = self.get_clock().now()
        self.prune_rock_tracks(now)
        unmatched = set(range(len(self.rock_tracks)))
        latest_tracks = []
        for record in records:
            best_index = None
            best_distance = math.inf
            for index in unmatched:
                previous = self.rock_tracks[index]["bbox"]
                scale = max(
                    record[3],
                    record[4],
                    previous[3],
                    previous[4],
                    1.0,
                )
                distance = math.hypot(
                    record[1] - previous[1],
                    record[2] - previous[2],
                ) / scale
                if distance < best_distance:
                    best_index = index
                    best_distance = distance

            if (
                best_index is not None
                and best_distance <= self.track_match_distance
            ):
                track = self.rock_tracks[best_index]
                unmatched.remove(best_index)
                track["bbox"] = record
                track["last_seen"] = now
            else:
                track = {
                    "bbox": record,
                    "samples": deque(),
                    "last_sample_stamp": None,
                    "last_seen": now,
                }
                self.rock_tracks.append(track)
            latest_tracks.append(track)
        self.latest_tracks = latest_tracks

    def publish_empty_rock_results(self):
        now = self.get_clock().now().to_msg()
        positions = PoseArray()
        positions.header.stamp = now
        positions.header.frame_id = self.base_frame
        detections = Detection2DArray()
        detections.header.stamp = now
        detections.header.frame_id = self.base_frame
        self.rock_detections_pub.publish(detections)
        self.rock_positions_pub.publish(positions)

    def decode_raw_depth(self, msg):
        enc = msg.encoding.upper()
        if enc in ("16UC1", "MONO16"):
            dtype, scale = np.dtype("<u2"), 1e-3
        elif enc == "32FC1":
            dtype, scale = np.dtype("<f4"), 1.0
        else:
            self.get_logger().error(
                f"Unsupported depth encoding '{msg.encoding}'",
                throttle_duration_sec=5.0,
            )
            return None
        if msg.is_bigendian:
            dtype = dtype.newbyteorder(">")
        stride = msg.step // dtype.itemsize
        data = np.frombuffer(msg.data, dtype=dtype)
        if data.size < msg.height * stride:
            return None
        return (
            data.reshape(msg.height, stride)[:, : msg.width].astype(np.float32)
            * scale
        )

    def decode_compressed_depth(self, msg):
        if "rvl" in msg.format.lower():
            self.get_logger().error(
                "RVL compressedDepth is unsupported; use the raw depth topic",
                throttle_duration_sec=5.0,
            )
            return None
        # compressed_depth_image_transport prepends a 12-byte header.
        image = cv2.imdecode(
            np.frombuffer(msg.data[12:], dtype=np.uint8),
            cv2.IMREAD_UNCHANGED,
        )
        if image is None:
            return None
        if "32FC1" in msg.format:
            quant_a, quant_b = np.frombuffer(
                bytes(msg.data[4:12]), dtype="<f4", count=2
            )
            values = image.astype(np.float32)
            with np.errstate(divide="ignore", invalid="ignore"):
                return np.where(
                    values > 0.0, quant_a / (values - quant_b), 0.0
                )
        return image.astype(np.float32) * 1e-3

    def on_raw_depth(self, msg):
        depth = self.decode_raw_depth(msg)
        if depth is not None:
            self.process_depth(depth, msg.header)

    def on_compressed_depth(self, msg):
        depth = self.decode_compressed_depth(msg)
        if depth is not None:
            self.process_depth(depth, msg.header)

    def scaled_bbox(self, depth_shape, bbox_record=None):
        if self.camera_info is None:
            return None
        record = bbox_record if bbox_record is not None else self.latest_bbox
        if record is None:
            return None
        _, center_x, center_y, width, height, _, _ = record
        source_width = max(1, int(self.camera_info.width))
        source_height = max(1, int(self.camera_info.height))
        depth_height, depth_width = depth_shape
        sx = depth_width / source_width
        sy = depth_height / source_height

        half_w = 0.5 * width * sx * self.bbox_scale
        half_h = 0.5 * height * sy * self.bbox_scale
        cx = center_x * sx
        cy = center_y * sy
        x0 = max(0, int(math.floor(cx - half_w)))
        y0 = max(0, int(math.floor(cy - half_h)))
        x1 = min(depth_width, int(math.ceil(cx + half_w)))
        y1 = min(depth_height, int(math.ceil(cy + half_h)))
        if x1 <= x0 or y1 <= y0:
            return None
        return x0, y0, x1, y1, cx, cy, depth_width, depth_height

    def depth_position_in_bbox(self, depth, bbox):
        x0, y0, x1, y1, cx, cy, _, _ = bbox
        roi = depth[y0:y1, x0:x1]
        yy, xx = np.ogrid[y0:y1, x0:x1]
        rx = max((x1 - x0) * 0.5, 1.0)
        ry = max((y1 - y0) * 0.5, 1.0)
        ellipse = ((xx - cx) / rx) ** 2 + ((yy - cy) / ry) ** 2 <= 1.0
        valid = (
            ellipse
            & np.isfinite(roi)
            & (roi >= self.min_depth)
            & (roi <= self.max_depth)
        )
        values = roi[valid]
        required_pixels = max(
            self.min_depth_pixels,
            int(math.ceil(np.count_nonzero(ellipse) * self.min_depth_fraction)),
        )
        if values.size < required_pixels:
            return None

        # Estimate depth from the central half of the YOLO box. At high
        # resolution this rejects the unstable silhouette and background
        # pixels while retaining many samples from the rock surface.
        core = ((xx - cx) / (0.5 * rx)) ** 2 + (
            (yy - cy) / (0.5 * ry)
        ) ** 2 <= 1.0
        core_values = roi[valid & core]
        depth_values = (
            core_values
            if core_values.size >= self.min_depth_pixels
            else values
        )
        center_depth = float(np.median(depth_values))
        inlier = valid & (np.abs(roi - center_depth) <= self.depth_band)
        if np.count_nonzero(inlier) < required_pixels:
            return None

        z = float(np.median(roi[inlier]))
        # YOLO's bbox center is less sensitive to depth-edge speckle than the
        # centroid of a changing inlier mask.
        return float(cx), float(cy), z, inlier

    def transform_camera_point(self, u, v, z, header, depth_shape):
        info = self.camera_info
        fx, fy = float(info.k[0]), float(info.k[4])
        cx, cy = float(info.k[2]), float(info.k[5])
        if fx <= 0.0 or fy <= 0.0:
            return None
        depth_height, depth_width = depth_shape
        # The box is expressed in color pixels. Convert a potentially
        # differently sized registered depth image back to color coordinates.
        u *= float(info.width) / max(1, depth_width)
        v *= float(info.height) / max(1, depth_height)

        point = PointStamped()
        point.header = header
        # The depth stream is registered to the color image in this stack.
        point.header.frame_id = info.header.frame_id or header.frame_id
        point.point.x = (u - cx) * z / fx
        point.point.y = (v - cy) * z / fy
        point.point.z = z
        try:
            transformed = self.tf_buffer.transform(
                point, self.base_frame, timeout=Duration(seconds=0.1)
            )
        except tf2_ros.TransformException:
            # robot_state_publisher may not publish another dynamic transform
            # while the robot is stationary. The camera message then has a
            # newer timestamp than the newest TF and an exact-time lookup
            # fails with "extrapolation into the future". Retry with time zero,
            # which asks tf2 for the latest available transform.
            point.header.stamp = rclpy.time.Time().to_msg()
            try:
                transformed = self.tf_buffer.transform(
                    point, self.base_frame, timeout=Duration(seconds=0.1)
                )
            except tf2_ros.TransformException as ex:
                self.get_logger().warn(
                    f"Rock TF {point.header.frame_id} -> {self.base_frame}: {ex}",
                    throttle_duration_sec=2.0,
                )
                return None
        return np.array(
            [
                transformed.point.x,
                transformed.point.y,
                transformed.point.z,
            ],
            dtype=np.float64,
        )

    def process_depth(self, depth, header):
        if not self.latest_bboxes or self.camera_info is None:
            return
        image_time = stamp_seconds(header.stamp)
        if (
            self.latest_detection_stamp is None
            or abs(image_time - self.latest_detection_stamp)
            > self.detection_timeout
        ):
            return

        measured = {}
        primary_position = None
        for index, record in enumerate(self.latest_bboxes):
            bbox = self.scaled_bbox(depth.shape, record)
            if bbox is None:
                continue
            measurement = self.depth_position_in_bbox(depth, bbox)
            if measurement is None:
                continue
            u, v, surface_depth, _ = measurement
            position = self.transform_camera_point(
                u, v, surface_depth, header, depth.shape
            )
            if position is None:
                continue
            measured[index] = position
            if record == self.latest_bbox:
                primary_position = position

        if not measured:
            self.publish_empty_rock_results()
            return

        now = self.get_clock().now()
        for index, position in measured.items():
            track = self.latest_tracks[index]
            detection_stamp = stamp_seconds(
                self.latest_bboxes[index][5].stamp
            )
            if detection_stamp == track["last_sample_stamp"]:
                continue
            self.prune_rock_samples(track, now)
            samples = track["samples"]
            if samples:
                previous = np.median(
                    np.array([sample[1] for sample in samples]), axis=0
                )
                if (
                    np.linalg.norm(position - previous)
                    > self.sample_jump_tolerance
                ):
                    samples.clear()
            samples.append((now, position))
            track["last_sample_stamp"] = detection_stamp

        rock_positions = PoseArray()
        rock_positions.header.stamp = now.to_msg()
        rock_positions.header.frame_id = self.base_frame
        rock_detections = Detection2DArray()
        rock_detections.header = rock_positions.header
        # Publish only stable, depth-fused detections. The detection and pose
        # arrays are compact and have exact one-to-one index correspondence.
        for index in range(len(self.latest_bboxes)):
            position = self.averaged_rock_position(
                self.latest_tracks[index], now
            )
            target_pose = (
                self.standoff_pose(position) if position is not None else None
            )
            if target_pose is None:
                continue
            rock_detections.detections.append(self.latest_bboxes[index][6])
            rock_positions.poses.append(target_pose)
        self.rock_detections_pub.publish(rock_detections)
        self.rock_positions_pub.publish(rock_positions)

        # Preserve the existing single-target averaging/service behavior using
        # only the highest-confidence detection.
        if (
            primary_position is None
            or self.latest_detection_stamp == self.last_sample_detection_stamp
        ):
            return
        self.last_sample_detection_stamp = self.latest_detection_stamp
        self.prune_samples(now)
        if self.samples:
            previous = np.median(
                np.array([sample[1] for sample in self.samples]), axis=0
            )
            if (
                np.linalg.norm(primary_position - previous)
                > self.sample_jump_tolerance
            ):
                # A new/inconsistent YOLO target must establish a fresh track.
                self.samples.clear()
        self.samples.append((now, primary_position))

        average = self.averaged_position()
        if average is not None:
            self.publish_rock_pose(average, now)

    def standoff_pose(self, rock):
        """Return the ready-to-send arm_link_end pose for a detected rock."""
        target_position = np.array(
            [rock[0], rock[1], rock[2] + self.standoff],
            dtype=np.float64,
        )
        horizontal_distance = float(np.linalg.norm(target_position[:2]))
        if horizontal_distance < self.min_target_distance_from_base:
            return None
        pose = Pose()
        pose.position.x = float(target_position[0])
        pose.position.y = float(target_position[1])
        pose.position.z = float(target_position[2])
        self.set_canonical_approach_orientation(pose)
        return pose

    @staticmethod
    def set_canonical_approach_orientation(pose):
        # R_y(+pi/2) expressed in base_link:
        #   arm_link_end +X -> base_link -Z (tool points down)
        #   arm_link_end +Z -> base_link +X (EE Z always faces robot front)
        pose.orientation.x = 0.0
        pose.orientation.y = math.sqrt(0.5)
        pose.orientation.z = 0.0
        pose.orientation.w = math.sqrt(0.5)

    def prune_rock_samples(self, track, now):
        samples = track["samples"]
        while samples and (
            (now - samples[0][0]).nanoseconds * 1e-9
            > self.averaging_window
        ):
            samples.popleft()

    def averaged_rock_position(self, track, now):
        self.prune_rock_samples(track, now)
        return self.stable_position(track["samples"])

    def prune_samples(self, now):
        while self.samples and (
            (now - self.samples[0][0]).nanoseconds * 1e-9
            > self.averaging_window
        ):
            self.samples.popleft()

    def averaged_position(self):
        self.prune_samples(self.get_clock().now())
        return self.stable_position(self.samples)

    def stable_position(self, samples):
        if len(samples) < self.min_samples:
            return None
        positions = np.array([sample[1] for sample in samples])
        median = np.median(positions, axis=0)
        distances = np.linalg.norm(positions - median, axis=1)
        inliers = positions[distances <= self.sample_jump_tolerance]
        if len(inliers) < self.min_samples:
            return None
        center = np.median(inliers, axis=0)
        spread = np.percentile(
            np.linalg.norm(inliers - center, axis=1), 90.0
        )
        if spread > self.max_position_spread:
            return None
        return center

    def publish_rock_pose(self, position, now):
        pose = PoseStamped()
        pose.header.stamp = now.to_msg()
        pose.header.frame_id = self.base_frame
        pose.pose.position.x = float(position[0])
        pose.pose.position.y = float(position[1])
        pose.pose.position.z = float(position[2])
        pose.pose.orientation.w = 1.0
        self.rock_pose_pub.publish(pose)

        # transform = TransformStamped()
        # transform.header = pose.header
        # transform.child_frame_id = "rock"
        # transform.transform.translation.x = float(position[0])
        # transform.transform.translation.y = float(position[1])
        # transform.transform.translation.z = float(position[2])
        # transform.transform.rotation.w = 1.0
        # self.tf_broadcaster.sendTransform(transform)

    def current_ee_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.end_effector_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
        except tf2_ros.TransformException:
            return None
        return transform.transform

    def on_selected_target(self, target):
        if target.header.frame_id != self.base_frame:
            self.get_logger().warn(
                "Rejected selected rock target in frame "
                f"'{target.header.frame_id}'; expected '{self.base_frame}'"
            )
            return
        position = target.pose.position
        self.set_canonical_approach_orientation(target.pose)
        orientation = target.pose.orientation
        values = (
            position.x,
            position.y,
            position.z,
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        quaternion_norm = math.sqrt(
            orientation.x**2
            + orientation.y**2
            + orientation.z**2
            + orientation.w**2
        )
        if not all(math.isfinite(value) for value in values):
            self.get_logger().warn("Rejected non-finite selected rock target")
            return
        if quaternion_norm < 0.5:
            self.get_logger().warn(
                "Rejected selected rock target with invalid orientation"
            )
            return
        if (
            math.hypot(position.x, position.y)
            < self.min_target_distance_from_base
        ):
            self.get_logger().warn(
                "Rejected selected rock target inside the base safety radius"
            )
            return

        target.header.stamp = self.get_clock().now().to_msg()
        self.approach_target = target
        self.approach_start_time = self.get_clock().now()
        self.get_logger().info(
            "Accepted frozen GS rock target "
            f"({position.x:.3f}, {position.y:.3f}, {position.z:.3f})"
        )

    def on_stop_target(self, _message):
        self.approach_target = None
        self.get_logger().info("Stopped GS rock target")

    def on_approach_rock(self, request, response):
        rock = self.averaged_position()
        if rock is None:
            response.success = False
            response.message = (
                f"No stable YOLO rock estimate ({len(self.samples)} samples, "
                f"need {self.min_samples})"
            )
            return response

        # Approach vertically from above: XY is centered on the stone and
        # standoff controls only the positive base_link Z offset.
        target_pose = self.standoff_pose(rock)
        if target_pose is None:
            unsafe_position = np.array(
                [rock[0], rock[1], rock[2] + self.standoff],
                dtype=np.float64,
            )
            target_horizontal_distance = float(
                np.linalg.norm(unsafe_position[:2])
            )
            response.success = False
            response.message = (
                "Unsafe target: arm_link_end horizontal distance would be "
                f"{target_horizontal_distance:.3f} m from base_link (minimum "
                f"{self.min_target_distance_from_base:.3f} m)"
            )
            return response

        target_position = np.array(
            [
                target_pose.position.x,
                target_pose.position.y,
                target_pose.position.z,
            ]
        )
        target_radius = float(np.linalg.norm(target_position))
        target = PoseStamped()
        target.header.frame_id = self.base_frame
        target.pose = target_pose
        self.approach_target = target
        self.approach_start_time = self.get_clock().now()

        response.success = True
        response.message = (
            f"Approaching rock at ({rock[0]:.3f}, {rock[1]:.3f}, "
            f"{rock[2]:.3f}) from above; target z="
            f"{target_position[2]:.3f} (standoff={self.standoff:.3f} m), "
            f"target radius={target_radius:.3f} m"
        )
        return response

    def on_stop_approach(self, request, response):
        self.approach_target = None
        response.success = True
        response.message = "Approach stopped"
        return response

    def publish_target(self):
        if self.approach_target is None:
            return
        now = self.get_clock().now()
        if (
            self.approach_timeout > 0.0
            and (now - self.approach_start_time).nanoseconds * 1e-9
            > self.approach_timeout
        ):
            self.approach_target = None
            self.get_logger().warn("Rock approach timed out")
            return

        ee = self.current_ee_pose()
        if ee is not None:
            error = np.linalg.norm(
                np.array(
                    [ee.translation.x, ee.translation.y, ee.translation.z]
                )
                - np.array(
                    [
                        self.approach_target.pose.position.x,
                        self.approach_target.pose.position.y,
                        self.approach_target.pose.position.z,
                    ]
                )
            )
            if error <= self.arrival_tolerance:
                self.approach_target = None
                self.get_logger().info("Rock approach target reached")
                return

        self.approach_target.header.stamp = now.to_msg()
        self.target_pose_pub.publish(self.approach_target)

def main():
    rclpy.init()
    node = RockDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
