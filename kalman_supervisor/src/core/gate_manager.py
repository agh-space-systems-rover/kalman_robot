import numpy as np
from rclpy.node import Node
from typing import Tuple, Optional
from enum import IntEnum
from .tag_detector import Tag
from .transformer import Transformer

Vec2 = Tuple[float, float]
node = Node()

# Mini State Machine for gate passing
class GateManager:
    class State(IntEnum):
        LOOKING_FOR_GATE = 0
        GO_FAR_FRONT = 1
        POS_FAR_FRONT = 2
        CM_CLEAR = 3
        GO_CLOSE_FRONT = 4
        POS_CLOSE_FRONT = 5
        GO_BACK = 6
        CROSSED = 7

    # How far in front should we be from the gate
    FRONT_LONG = 5
    FRONT_SHORT = 3

    # How far should we go after passing the gate
    BACK_DISTANCE = 3.5

    def __init__(self):
        self.front: Optional[Vec2] = None
        self.front_close: Optional[Vec2] = None
        self.back: Optional[Vec2] = None
        self.__state = self.State.LOOKING_FOR_GATE
        self.__transformer = Transformer()

    @property
    def state(self) -> State:
        return self.__state

    def calculate(self, tags: Tuple[Optional[Tag], Optional[Tag]]):
        if tags[0] is None or tags[1] is None:
            node.get_logger().info("Calculating gate goals without finding both tags first.")
            return

        tag1 = np.array(
            self.__transformer.transform2D(tags[0].position, tags[0].frame_id, "odom")
        )
        tag2 = np.array(
            self.__transformer.transform2D(tags[1].position, tags[1].frame_id, "odom")
        )

        center = (tag1 + tag2) / 2
        tag_to_center = center - tag1

        center_to_front = np.array([tag_to_center[1], -tag_to_center[0]])
        center_to_back = -center_to_front

        center_to_front /= np.linalg.norm(center_to_front)
        center_to_back /= np.linalg.norm(center_to_back)

        tag_to_front = tag_to_center + self.FRONT_LONG * center_to_front
        tag_to_front_short = tag_to_center + self.FRONT_SHORT * center_to_front
        tag_to_back = tag_to_center + self.BACK_DISTANCE * center_to_back

        self.front = tuple(tag1 + tag_to_front)
        self.front_close = tuple(tag1 + tag_to_front_short)
        self.back = tuple(tag1 + tag_to_back)

    def next(self):
        self.__state = self.State(
            (self.__state.value + 1) % (self.State.CROSSED.value + 1)
        )