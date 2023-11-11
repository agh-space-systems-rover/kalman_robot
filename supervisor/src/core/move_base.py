from typing import Tuple, Optional
from enum import IntEnum
from rclpy import Node
from rclpy.client import Client
from geometry_msgs.msg import Twist
from rclpy.action import ActionServer, CancelResponse, GoalResponse
import numpy as np

#import actionlib
#from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult

from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
import dynamic_reconfigure.client
from .transformer import Transformer

Vec2 = Tuple[float, float]
EPS = 1e-6
node = Node()
client = Client()
class MoveBase:
    class Status(IntEnum):
        PENDING = 0
        ACTIVE = 1
        PREEMPTED = 2
        SUCCEEDED = 3
        ABORTED = 4  # When we get really close to the goal but it's in a obstacle
        REJECTED = 5
        PREEMPTING = 6
        RECALLING = 7
        RECALLED = 8
        LOST = 9

    def __init__(self):
        self.__transformer = Transformer()
        self.__client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.__costmap_client = dynamic_reconfigure.client.Client(
            "/move_base/global_costmap/obstacles"
        )
        self.__current_goal: Optional[MoveBaseGoal] = None
        self.__status = self.Status.PENDING
        self.__status_subscriber = node.create_subscription(MoveBaseActionResult, "/move_base/result", self.__status_callback)
        self.__cached_id: Optional[str] = None  # Prevent duplicate messages
        client.wait_for_service("/move_base/clear_costmaps")
        self.__clear_costmap_handle = node.create_client(Empty, "/move_base/clear_costmaps")
        self.__costmap_client.update_configuration({"enabled": True})
        self.clear_costmap()

    def is_goal_reached(self) -> bool:
        return (
            self.__status == self.Status.SUCCEEDED
            or self.__status == self.Status.ABORTED
        )

    def send_goal(self, frame_id: str, position: Vec2):

        if frame_id == "gps":
            node.get_logger().info("Got GPS as the goal frame, this should never happen, convert it to UTM!")

        if frame_id == "base_link" and position[0] > EPS and position[1] > EPS:
            node.get_logger().info("Got non-zero BASE_LINK goal, this should never happen!")

        self.__client.wait_for_server()
        goal: MoveBaseGoal = self.__convert_goal(frame_id, position)
        self.__client.send_goal(goal)
        self.__status = self.Status.ACTIVE
        self.__current_goal = goal

    def send_goal_in_dir(self, odom: Vec2, direction: Vec2, length: float):
        np_odom = np.array(odom)
        np_direction = np.array(direction)
        np_direction = np_direction / np.linalg.norm(np_direction)
        np_direction *= length
        goal = np.add(np_odom, np_direction)
        goal = (goal[0], goal[1])
        self.send_goal("odom", goal)

    def cancel_goal(self):
        self.__client.wait_for_server()
        self.__client.cancel_all_goals()
        self.send_goal("base_link", (0.0, 0.0))
        self.__current_goal = None

    def __convert_goal(self, frame_id: str, position: Vec2) -> MoveBaseGoal:
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = frame_id

        goal.target_pose.pose.position.x = position[0]
        goal.target_pose.pose.position.y = position[1]
        goal.target_pose.pose.orientation.w = 1.0

        return goal

    def disable_costmap(self):
        self.__costmap_client.update_configuration({"enabled": False})

    def enable_costmap(self):
        self.__costmap_client.update_configuration({"enabled": True})

    def clear_costmap(self):
        try:
            self.__clear_costmap_handle()
        except rospy.ServiceException as exc:
        #############################################3
            node.get_logger().info("Service did not process request: " + str(exc))

    def __status_callback(self, msg: MoveBaseActionResult):
        if msg.status.goal_id.id != self.__cached_id:
            self.__status = self.Status(msg.status.status)
            self.__cached_id = msg.status.goal_id.id

    # Returns distance to the current goal in meters
    def distance_to_goal(self, odom: Vec2) -> float:
        if self.__current_goal is None:
            return float("inf")

        self.__current_goal: MoveBaseGoal

        odom_pose: PoseStamped = self.__transformer.transform(
            self.__current_goal.target_pose, "odom"
        )

        if odom_pose is None:
            return float("inf")

        goal = (odom_pose.pose.position.x, odom_pose.pose.position.y)
        goal_vector = np.subtract(odom, goal)
        return np.linalg.norm(goal_vector)

    @property
    def current_goal(self) -> MoveBaseGoal:
        return self.__current_goal

    @property
    def status(self) -> Status:
        return self.__status