from typing import Tuple, Optional
from rclpy import Node
from enum import IntEnum
#import dynamic_reconfigure.client
from std_msgs.msg import String
from .transformer import Transformer
from .mode import Mode
from .position import Position

Vec2 = Tuple[float, float]
node = Node()
# sync with UnitedSupervisingProtocol
class Control:
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
        self.__client = dynamic_reconfigure.client.Client(
            "united_supervising_protocol", timeout=30, config_callback=None
        )
        self.__debug_publisher = node.create_publisher(String,  "/supervisor/debug", 10)

    # returns a Tuple[goal: Vec2, frame_id: str]
    def get_goal(self) -> Tuple[Vec2, str]:
        x: float = node.declare_parameter("/united_supervising_protocol/x").value
        y: float = node.declare_parameter("/united_supervising_protocol/y").value
        frame: self.Frame = self.Frame(node.declare_parameter("/united_supervising_protocol/frame").value)

        if frame == self.Frame.NONE:
            node.get_logger().info("Got NONE as the input frame, this should never happen!")
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
        return node.declare_parameter("/united_supervising_protocol/autonomous_driving").value
    
    def is_goal_set(self) -> bool:
        return node.declare_parameter("/united_supervising_protocol/goal_set").value
    
    def has_multiple_waypoints(self) -> bool:
        return node.declare_parameter("/united_supervising_protocol/multiple_waypoints").value

    def get_tags(self) -> Tuple[Optional[int], Optional[int]]:
        tag1 = node.declare_parameter("/united_supervising_protocol/tag1").value
        tag2 = node.declare_parameter("/united_supervising_prot)ocol/tag2").value

        # if tag id is zero, convert it to none
        tag1 = tag1 or None
        tag2 = tag2 or None

        return (tag1, tag2)

    def get_mode(self) -> Mode:
        mode = Mode(node.declare_parameter("/united_supervising_protocol/mode").value)

        if mode == Mode.NONE:
            node.get_logger().info("Got NONE as mode, this should never happen!")

        return mode

    def clear(self):
        self.__client.update_configuration(
            {
                "autonomous_driving": False,
                "x": 0,
                "y": 0,
                "mode": int(Mode.NONE),
                "goal_set": False,
                "tag1": -1,
                "tag2": -1,
            }
        )

    def completed(self):
        self.__client.update_configuration(
            {
                "autonomous_driving": True,
                "x": 0,
                "y": 0,
                "mode": int(Mode.NONE),
                "goal_set": False,
                "tag1": -1,
                "tag2": -1,
            }
        )