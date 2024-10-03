# YOLO-ROS

ROS 2 package that integrates YOLOv8 object detection as a Python node.

## Output Specification

The objects detected by the node are published to `/detections` as `vision_msgs/Detection2DArray` messages. Before publishing similar detections are merged together and then filtered temporally to eliminate flickering false positives. The node can also publish additional visualizations of the detections to `/annotatedX` (`sensor_msgs/Image`) and `/tf` (`tf2_msgs/TFMessage`).

## How to Launch

```py
from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="yolo_ros",
                executable="yolo_detect_auto_activate",
                parameters=[
                    ParameterFile(
                        str(get_package_share_path("kalman_yolo") / "config" / "urc2024.yaml"),
                        allow_substs=True,
                    ),
                    {"num_cameras": len(camera_names)},
                ],
                remappings=remappings = sum(
                    [
                        [
                            (f"color{i}", f"{camera_name}/color/image_raw"),
                            (f"color{i}/compressed", f"{camera_name}/color/image_raw/compressed"),
                            (f"depth{i}", f"{camera_name}/aligned_depth_to_color/image_raw"),
                            (
                                f"depth{i}/compressed",
                                f"{camera_name}/aligned_depth_to_color/image_raw/compressed",
                            ),
                            (f"info{i}", f"{camera_name}/color/camera_info"),
                            (f"detections", f"yolo_detections"),
                            (f"annotated{i}", f"{camera_name}/yolo_annotated"),
                            (f"annotated{i}/compressed", f"{camera_name}/yolo_annotated/compressed"),
                        ]
                        for i, camera_name in enumerate(camera_names)
                    ],
                    [],
                ),
            )
        ]
    )
```

## Example Config

```yaml
/**:
  ros__parameters:
    # number of cameras to use
    # num_cameras: 4
    # NOTE: This parameter is specified in the launch file.
    # subscribe to depth topics
    subscribe_depth: false
    # Use custom image transport for color and depth images
    color_transport: compressed
    depth_transport: compressedDepth
    # detection frequency; Hz
    rate: 10.0
    # path to YOLO model weights in PT format (required)2
    model: $(find-pkg-share kalman_yolo)/models/urc2024.pt
    # whether to convert incoming images to grayscale before inference
    grayscale: false
    # detection threshold
    confidence_threshold: 0.7
    # list of class names
    class_names:
    - bottle
    - mallet
    # average real life half-width of each class in meters
    class_radii:
    - 0.1
    - 0.13
    # maximum 3D distance between two detections of the same class to consider them the same
    merge_radius: 2.0
    # number of history frames to use for temporal filtering
    temporal_window: 20
    # number of times a detection must be present in the history to pass through the filter; Must be less than or equal to temporal_window. Setting it to 0 will result in lack of entry filtering and setting it to temporal_window value will provide no exit filtering. Entry filtering is useful for preventing false positives from being reported, while exit filtering is useful for preventing existing detections from jumping around and disappearing. If you are not sure what value to use, choose temporal_window / 2 which corresponds to a detection being present half of the time.
    temporal_threshold: 15
    # Publish detection results as TF2 transforms.
    publish_tf: true
    # Publish annotated images on annotatedX topics.
    publish_annotated: true
    annotated_transport: compressed
```
