from rclpy.node import Node
from typing import Tuple, List, Optional
from dataclasses import dataclass
from fiducial_msgs.msg import FiducialTransformArray, FiducialTransform
from geometry_msgs.msg import PoseStamped, Pose, Point
from .control import Control
from .transformer import Transformer

Vec2 = Tuple[float, float]
node = Node()

@dataclass
class Tag:
    frame_id: str
    position: Vec2


class TagDetector:
    def __init__(self):
        self.__control = Control()
        self.__transformer = Transformer()
        self.__subscriber = node.create_subscription(FiducialTransformArray, "/fiducial_transforms", self.__callback)
        self.__found: List[Optional[Tag]] = [None, None]

    @property
    def tags(self) -> Tuple[Optional[Tag], Optional[Tag]]:
        return tuple(self.__found)

    def reset(self) -> None:
        self.__found = [None, None]

    def __callback(self, msg: FiducialTransformArray):

        tag_ids = self.__control.get_tags()

        transform: FiducialTransform
        for transform in msg.transforms:
            for i, tag_id in enumerate(tag_ids):
                if tag_id != transform.fiducial_id:
                    continue

                tag_pose = PoseStamped(
                    header=msg.header,
                    pose=Pose(
                        position=Point(
                            transform.transform.translation.x,
                            transform.transform.translation.y,
                            transform.transform.translation.z,
                        ),
                        orientation=transform.transform.rotation,
                    ),
                )

                transformed: PoseStamped = self.__transformer.transform(
                    tag_pose, "odom"
                )

                self.__found[i] = Tag(
                    frame_id="odom",
                    position=(transformed.pose.position.x, transformed.pose.position.y),
                )