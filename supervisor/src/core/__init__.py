import rclpy
from std_msgs.msg import String
from kalman_interfaces.msg import SupervisorStatus
from rclpy.node import Node
from dataclasses import dataclass, field
from .move_base import MoveBase
from .transformer import Transformer
from .ueuos import Ueuos
from .position import Position
from .control import Control
from .mode import Mode
from .tag_detector import TagDetector
from .gate_manager import GateManager
from .long_goal_manager import LongGoalManager


@dataclass
class Core:
    position: Position = field(default_factory=Position)
    ueuos: Ueuos = field(default_factory=Ueuos)
    move_base: MoveBase = field(default_factory=MoveBase)
    control: Control = field(default_factory=Control)
    transformer: Transformer = field(default_factory=Transformer)
    tag_detector: TagDetector = field(default_factory=TagDetector)
    gate_manager: GateManager = field(default_factory=GateManager)
    long_goal_manager: LongGoalManager = field(default_factory=LongGoalManager)

    debug_publisher: rclpy.Node().create_publisher(String, "/supervisor/debug", 10)
    status_publisher: rclpy.Node().create_publisher(SupervisorStatus, "/supervisor/debug", 10)

    
    