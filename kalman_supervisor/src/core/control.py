from typing import Tuple, Optional
from rclpy import Node
from enum import IntEnum
from std_msgs.msg import String
from .transformer import Transformer
from .mode import Mode
from .position import Position


Vec2 = Tuple[float, float]
# sync with UnitedSupervisingProtocol
class Control(Node):
    class Frame(IntEnum):
        NONE = 0
        BASE_LINK = 1
        MAP = 2
        ODOM = 3
        GPS = 4

        UTM = 10  # this state will never come as input

        def __str__(self):
            return self.name.lower()

    def __init__(self):
        self.__transformer = Transformer()
        self.__debug_publisher = node.create_publisher(String,  "/supervisor/debug", 10)
        self._x = self.declare_parameter('x')
        self._y = self.declare_parameter('y')
        self._frame = self.declare_parameter('frame')
        self._autonomus_drv = self.declare_parameter('autonomus_driving')
        self._mode = self.declare_parameter('mode')
        self._tag1 = self.declare_parameter('tag1')
        self._tag2 = self.declare_parameter('tag2')
        self._goal_set = self.declare_parameter('goal_set')

    # returns a Tuple[goal: Vec2, frame_id: str]
    def get_goal(self) -> Tuple[Vec2, str]:
        x: float = self.get_parameter('x').value
        y: float = self.get_parameter('y').value
        frame: self.Frame = self.Frame(self.get_parameter('frame').value)

        if frame == self.Frame.NONE:
            self.get_logger().info("Got NONE as the input frame, this should never happen!")

        if frame == self.Frame.GPS:
            self.__debug_publisher.publish("Got GPS goal, converting to UTM.")
            x, y = self.__transformer.gps2utm((x, y))
            frame = self.Frame.UTM

        if frame == self.Frame.BASE_LINK:
            x, y = self.__transformer.transform2D((x, y), "base_link", "odom")
            frame = self.Frame.ODOM

        self.__debug_publisher.publish(f"Getting goal: {x} {y} {str(frame)}")

        return (x, y), str(frame)

    def is_autonomous(self) -> bool:
        return self.get_parameter('autonomus_driving').value
    
    def is_goal_set(self) -> bool:
        return self.get_parameter('goal_set').value
    
    def has_multiple_waypoints(self) -> bool:
        return self.get_parameter('multiple_waypoints').value

    def get_tags(self) -> Tuple[Optional[int], Optional[int]]:
        tag1 = self.get_parameter('tag1').value
        tag2 = self.get_parameter('tag2').value

        # if tag id is zero, convert it to none
        tag1 = tag1 or None
        tag2 = tag2 or None

        return (tag1, tag2)

    def get_mode(self) -> Mode:
        mode = Mode(self.get_parameter('mode').value)

        if mode == Mode.NONE:
            self.get_logger().info("Got NONE as mode, this should never happen!")

        return mode
    
    def clear(self):
        self._autonomus_drv.value = False
        self._x.value = 0
        self._y.value = 0
        self._tag1.value = -1
        self._tag2.value = -1
        self._goal_set = False
        self._mode.value = int(Mode.NONE)

    def completed(self):
        self._autonomus_drv.value
        self._x.value = 0
        self._y.value = 0
        self._tag1.value = -1
        self._tag2.value = -1
        self._goal_set.value = False
        self._mode.value = int(Mode.NONE)
        