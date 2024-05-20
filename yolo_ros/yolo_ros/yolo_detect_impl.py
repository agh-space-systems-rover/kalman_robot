import random
import numpy as np
import copy
from typing import Any
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from rclpy.time import Time
from tf2_ros import Buffer, LookupException, ConnectivityException

import yolo_ros.detection_msg_utils as msg_utils
from yolo_ros.detection_msg_utils import DetectionGroup

POSITION_ESTIMATION_DEPTH_SAMPLES = 100

# helper functions


def quat_to_basis(q: np.ndarray) -> np.ndarray:
    return (
        np.array(
            [
                1 - 2 * (q[2] * q[2] + q[3] * q[3]),
                2 * (q[1] * q[2] + q[3] * q[0]),
                2 * (q[1] * q[3] - q[2] * q[0]),
                2 * (q[1] * q[2] - q[3] * q[0]),
                1 - 2 * (q[1] * q[1] + q[3] * q[3]),
                2 * (q[2] * q[3] + q[1] * q[0]),
                2 * (q[1] * q[3] + q[2] * q[0]),
                2 * (q[2] * q[3] - q[1] * q[0]),
                1 - 2 * (q[1] * q[1] + q[2] * q[2]),
            ]
        )
        .reshape(3, 3)
        .T
    )


def lookup_tf_as_matrix(
    tf_buffer: Buffer, target_frame: str, source_frame: str, time: Time
) -> np.ndarray:
    # await self.tf_buffer.wait_for_transform_async(target_frame, source_frame, time)
    # This often results in TF errors.
    transform = tf_buffer.lookup_transform(target_frame, source_frame, time).transform
    pos = np.array(
        [transform.translation.x, transform.translation.y, transform.translation.z]
    )
    matrix = np.eye(4)
    matrix[:3, :3] = quat_to_basis(
        [
            transform.rotation.w,
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
        ]
    )
    matrix[:3, 3] = pos
    return matrix


# API Below


def detection_array_from_yolo_results(
    node: Any, results: list[Any], stamps: list[Any]
) -> Detection2DArray:
    detections = Detection2DArray()
    detections.header.stamp = node.get_clock().now().to_msg()
    detections.header.frame_id = node.world_frame
    for i, result in enumerate(results):
        for j in range(len(result.boxes)):
            # Set header.
            detection = Detection2D()
            detection.header.stamp = stamps[i]
            detection.header.frame_id = node.world_frame

            # Set hypothesis.
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = node.class_names[
                int(result.boxes.cls[j] + 0.5)
            ]
            hypothesis.hypothesis.score = result.boxes.conf[j].item()
            # Pose will be added later using add_3d_positions_to_detections.
            # For now, let's store the camera index in the pose.
            hypothesis.pose.pose.position.x = float(i)
            detection.results = [hypothesis]

            # Set BBox.
            bb = result.boxes[j].xyxy[0].tolist()
            detection.bbox.center.position.x = (bb[0] + bb[2]) / 2
            detection.bbox.center.position.y = (bb[1] + bb[3]) / 2
            detection.bbox.size_x = bb[2] - bb[0]
            detection.bbox.size_y = bb[3] - bb[1]

            # Add detection.
            detections.detections.append(detection)
    return detections


# Approximates the 3D positions of the detections in a Detection2DArray and appends them to Detection2D elements.
# If depth image is unavailable, self.class_radii should be used to compute the approximate 3D position.
def add_3d_positions_to_detections(
    node: Any,
    detections: Detection2DArray,
    depth_images: list[Any],
    info_msgs: list[Any],
) -> Detection2DArray:
    for i, detection in enumerate(detections.detections):
        # Get camera index encoded in pose.
        camera_index = int(detection.results[0].pose.pose.position.x + 0.5)

        depth_image = depth_images[camera_index] if depth_images is not None else None
        camera_info = info_msgs[camera_index]

        # At first attempt to use depth_image to compute the 3D position.
        bb_radius = ((detection.bbox.size_x + detection.bbox.size_y) / 2) / 2
        z = None
        if depth_image is not None:
            # Sample pixels in bb_radius around the center of the bounding box.
            # Skip black pixels because they have no depth information.
            total_depth = 0
            samples = 0
            for _ in range(POSITION_ESTIMATION_DEPTH_SAMPLES):
                # Generate non-uniformly distributed random points around BB center.
                t = random.uniform(0, 2 * np.pi)
                r = random.uniform(0, 1) * bb_radius
                # (Without sqrt more points are sampled closer to the center.)
                dx = np.cos(t) * r
                dy = np.sin(t) * r
                x = int(detection.bbox.center.position.x + dx)
                y = int(detection.bbox.center.position.y + dy)

                # Skip points outside the image.
                if not (
                    0 <= y < depth_image.shape[0] and 0 <= x < depth_image.shape[1]
                ):
                    continue

                depth = depth_image[y, x]

                # Skip black pixels because they have no depth information.
                if depth < 100:  # < 10 cm
                    continue

                total_depth += depth
                samples += 1

            # Set the final z value if successful.
            if samples > 0:
                z = total_depth / samples / 1000  # mm -> m

        # If computation using depth image failed, use class_radii.
        if z is None:
            class_id = detection.results[0].hypothesis.class_id
            class_index = node.class_names.index(class_id)
            z = (
                node.class_radii[class_index] / bb_radius * camera_info.k[0]
            )  # fx = fy (always)

        # x = (col - cx) * z / fx
        # fx = (width / 2) / tan(fov / 2)
        x = z * (detection.bbox.center.position.x - camera_info.k[2]) / camera_info.k[0]
        y = z * (detection.bbox.center.position.y - camera_info.k[5]) / camera_info.k[4]

        # Transform xyz from camera frame to odom.
        # This is needed to merge and filter detections between multiple cameras and during movement.
        try:
            to_odom = lookup_tf_as_matrix(
                node.tf_buffer, node.world_frame, camera_info.header.frame_id, Time()
            )
            xyz = (to_odom @ np.array([x, y, z, 1]))[:3]
        except (LookupException, ConnectivityException):
            node.get_logger().error(
                f"Failed to lookup transform from {camera_info.header.frame_id} to odom. Detection positions won't be in the right frame."
            )
            xyz = np.array([x, y, z])

        # Update the detection.
        detection = copy.deepcopy(detection)
        detection.results[0].pose.pose.position.x = xyz[0]
        detection.results[0].pose.pose.position.y = xyz[1]
        detection.results[0].pose.pose.position.z = xyz[2]
        detections.detections[i] = detection
    return detections
