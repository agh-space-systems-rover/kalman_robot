import numpy as np
import copy
import random
import cv2
from collections import deque
from vision_msgs.msg import Detection2D, Detection2DArray
from geometry_msgs.msg import TransformStamped


# Converts detection to its 3D position as a numpy array.
def detection_pos(detection: Detection2D) -> np.ndarray:
    return np.array(
        [
            detection.results[0].pose.pose.position.x,
            detection.results[0].pose.pose.position.y,
            detection.results[0].pose.pose.position.z,
        ]
    )


# 3D distance between two Detection2D instances
def detection_distance_3d(detection1: Detection2D, detection2: Detection2D) -> float:
    # Compute 2D distance between the centers of the two detections.
    # (This is a simple optimization to reduce the number of distance calculations.)
    pos1 = detection_pos(detection1)
    pos2 = detection_pos(detection2)
    return np.linalg.norm(pos1 - pos2)


# Merge multiple Detection2D instances together.
# All instances should come from the same Detection2DArray and should have the same class.
# 3D distance must be provided in each instance.
def merge_detections(detections: list[Detection2D]) -> Detection2D:
    # Copy detection1 to avoid modifying the original data.
    merged_detection = copy.deepcopy(detections[0])

    # Merge the two detections.
    # The score of the merged detection is the maximum of the scores of the two detections.
    # The position of the merged detection is the average of the positions of the two detections.
    # A new bounding box is created with the average width and height, x and y position of the two detections.

    # score
    merged_detection.results[0].hypothesis.score = np.max(
        [detection.results[0].hypothesis.score for detection in detections]
    )

    # 3D position
    merged_detection.results[0].pose.pose.position.x = np.mean(
        [detection.results[0].pose.pose.position.x for detection in detections]
    )
    merged_detection.results[0].pose.pose.position.y = np.mean(
        [detection.results[0].pose.pose.position.y for detection in detections]
    )
    merged_detection.results[0].pose.pose.position.z = np.mean(
        [detection.results[0].pose.pose.position.z for detection in detections]
    )

    # bounding box
    merged_detection.bbox.center.position.x = np.mean(
        [detection.bbox.center.position.x for detection in detections]
    )
    merged_detection.bbox.center.position.y = np.mean(
        [detection.bbox.center.position.y for detection in detections]
    )
    merged_detection.bbox.size_x = np.mean(
        [detection.bbox.size_x for detection in detections]
    )
    merged_detection.bbox.size_y = np.mean(
        [detection.bbox.size_y for detection in detections]
    )

    return merged_detection


# Merge detections of the same class that are close to each other.
# 3D position must be provided for all detections prior to calling this function.
def merge_detections_by_distance(
    detections: Detection2DArray, merge_radius: float
) -> Detection2DArray:
    # Create a deep copy of the input detections to avoid modifying the original data.
    merged_detections = copy.deepcopy(detections)

    # Merge detections of the same class that are close to each other.
    # This is done by iterating over all pairs of detections until a
    # pair is found that is eligible for merging.
    # The process is repeated until no more eligible pairs are found.
    while True:
        # Find a pair of detections that is eligible for merging.
        # If no such pair is found, stop the process.
        pair_found = False
        for i in range(len(merged_detections.detections)):
            for j in range(i + 1, len(merged_detections.detections)):
                # Skip detections of different classes.
                if (
                    merged_detections.detections[i].results[0].hypothesis.class_id
                    != merged_detections.detections[j].results[0].hypothesis.class_id
                ):
                    continue

                # If the pair is eligible for merging, merge the detections and stop the process.
                # (This is a simple optimization to reduce the number of distance calculations.)
                if (
                    detection_distance_3d(
                        merged_detections.detections[i], merged_detections.detections[j]
                    )
                    < merge_radius
                ):
                    merged_detections.detections[i] = merge_detections(
                        [
                            merged_detections.detections[i],
                            merged_detections.detections[j],
                        ]
                    )
                    merged_detections.detections.pop(j)
                    pair_found = True
                    break

            if pair_found:
                break

        if not pair_found:
            break

    return merged_detections


# Group of detections of the same object.
class DetectionGroup:
    def __init__(self, detection: Detection2D, max_length: int) -> None:
        if len(detection.id) == 0:
            detection.id = str(random.randint(0, 2**31 - 1))

        self.id = detection.id
        self.detections: deque[Detection2D] = deque([detection], maxlen=max_length)

    # Adds a recent detection to the group.
    def push(self, detection: Detection2D) -> None:
        detection.id = self.id
        self.detections.append(detection)  # right side

    # average of the positions of all detections in the group
    def center(self):
        positions = [detection_pos(detection) for detection in self.detections]
        return np.mean(positions, axis=0)

    # whether the group is empty
    def empty(self) -> bool:
        return len(self.detections) == 0

    # Returns merge_detections() of all detections in the group.
    def as_single_detection(self) -> Detection2D:
        return merge_detections(self.detections)

    # Removes the oldest detection from the group.
    def pop(self) -> None:
        self.detections.popleft()  # left side

    # group size
    def count(self) -> int:
        return len(self.detections)

    def __repr__(self) -> str:
        return f"DetectionGroup(id={self.id}, detections={self.detections})"


# Stabilizes detection array using temporal history.
# This filter will prevent any short-lived false positives from being reported.
# It should also prevent existing detections from jumping around and disappearing.
def temporal_filter(
    detections: Detection2DArray,
    temporal_history: list[DetectionGroup],
    temporal_window: int,
    temporal_threshold: int,
    group_radius: float,
) -> Detection2DArray:
    # List of Detection2Ds that are to be pushed onto the groups.
    incoming_detections = copy.deepcopy(detections.detections)

    # For each detection group in the temporal history.
    # Temporal history is a deque of DetectionGroup objects.
    # Each group contains Detections2Ds of the same object at different points in time.
    detection_group: DetectionGroup
    # copy to allow removal during iteration
    for detection_group in copy.copy(temporal_history):
        group_was_updated = False
        # For each incoming detection.
        # copy to allow removal during iteration
        for incoming_detection in copy.copy(incoming_detections):
            # Use merge_radius to determine if it should be added to this group.
            incoming_pos = detection_pos(incoming_detection)
            if (
                np.linalg.norm(detection_group.center() - incoming_pos) < group_radius
                and detection_group.detections[-1].results[0].hypothesis.class_id
                == incoming_detection.results[0].hypothesis.class_id
            ):
                # Append the incoming detection to the group.
                detection_group.push(incoming_detection)
                group_was_updated = True
                incoming_detections.remove(incoming_detection)
        if not group_was_updated:
            # If the group was not updated, it means that nothing was added to it and the window did not move.
            # Move it by removing the oldest detection.
            detection_group.pop()
            if detection_group.empty():
                # If the group is empty, remove it from the history.
                temporal_history.remove(detection_group)

    # Create a new group for each incoming detection that did not match any of the existing groups.
    for incoming_detection in incoming_detections:
        temporal_history.append(DetectionGroup(incoming_detection, temporal_window))
    incoming_detections = []

    # Create a new Detection2DArray to store the filtered detections.
    filtered_detections = copy.deepcopy(detections)
    filtered_detections.detections = []

    # Add all detections from the temporal history to filtered detections.
    for detection_group in temporal_history:
        # Skip groups that are too short.
        if detection_group.count() < temporal_threshold:
            continue
        # Add the detection to filtered detections.
        filtered_detections.detections.append(detection_group.as_single_detection())

    return filtered_detections


def detections_to_transforms(detections: Detection2DArray) -> list[TransformStamped]:
    transforms = []

    for detection in detections.detections:
        # Create a name for the TF.
        class_id = detection.results[0].hypothesis.class_id
        object_id = detection.id
        frame_name = f"yolo_{class_id}_{object_id}"

        # Create the transform.
        transform = TransformStamped()
        transform.header = detection.header
        transform.child_frame_id = frame_name
        transform.transform.translation.x = detection.results[0].pose.pose.position.x
        transform.transform.translation.y = detection.results[0].pose.pose.position.y
        transform.transform.translation.z = detection.results[0].pose.pose.position.z
        transform.transform.rotation.w = 1.0

        transforms.append(transform)

    return transforms


# def annotate_detections_on_image(
#     img: np.ndarray,
#     detections: Detection2DArray,
#     color: tuple[int, int, int] = (0, 255, 0),
#     thickness: int = 1,
# ) -> np.ndarray:
#     for detection in detections.detections:
#         # Draw the bounding box.
#         top_left = (
#             int(detection.bbox.center.position.x - detection.bbox.size_x / 2),
#             int(detection.bbox.center.position.y - detection.bbox.size_y / 2),
#         )
#         bottom_right = (
#             int(detection.bbox.center.position.x + detection.bbox.size_x / 2),
#             int(detection.bbox.center.position.y + detection.bbox.size_y / 2),
#         )
#         cv2.rectangle(img, top_left, bottom_right, color, thickness)

#         # Draw the class name and score.
#         text = f"{detection.results[0].hypothesis.class_id}: {detection.results[0].hypothesis.score:.2f}"
#         cv2.putText(
#             img, text, top_left, cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, thickness
#         )

#     return img
