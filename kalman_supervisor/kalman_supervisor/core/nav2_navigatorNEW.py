from typing import Tuple, Optional
from enum import IntEnum
from rclpy import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import numpy as np
from .transformer import Transformer
from nav2_msgs.srv import ClearEntireCostmap
from functools import partial
Vec2 = Tuple[float, float]
EPS = 1e-6

class MoveBase(Node):
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
        self.__nav = BasicNavigator()
        self.__current_goal: Optional[PoseStamped] = None
        self.__status = self.Status.PENDING
        #self.__cached_id: Optional[str] = None  # Prevent duplicate messages
        self.clear_costmap_service()
        self.clear_costmap()

    def is_goal_reached(self) -> bool:
        return (
            self.__status == self.Status.SUCCEEDED
            or self.__status == self.Status.ABORTED
        )
    
    def send_goal(self, frame_id: str, position: Vec2, order): 
        if frame_id == "gps":
            self.get_logger().info("Got GPS as the goal frame, this should never happen, convert it to UTM!")

        if frame_id == "base_link" and position[0] > EPS and position[1] > EPS:
            self.get_logger().info("Got non-zero BASE_LINK goal, this should never happen!")

        self.get_logger().info("Sending goal... ")
        goal_pose = self.__convert_goal(frame_id, position)
        self.__status = self.Status.ACTIVE
        self.__current_goal = goal_pose

    def send_goal_in_dir(self, odom: Vec2, direction: Vec2, length: float):
        np_odom = np.array(odom)
        np_direction = np.array(direction)
        np_direction = np_direction / np.linalg.norm(np_direction)
        np_direction *= length
        goal = np.add(np_odom, np_direction)
        goal = (goal[0], goal[1])
        self.send_goal("odom", goal)

    def __convert_goal(self, frame_id: str, position: Vec2) -> PoseStamped:
        goal = PoseStamped()
        goal.header.frame_id = frame_id
        goal.pose.position.x = position[0]
        goal.pose.position.y = position[1]
        goal.pose.orientation.w = 1.0
       
        return goal
    
    def cancel_goal(self):
        self.__nav.cancelTask()
        self.send_goal("base_link", (0.0, 0.0))
        self.__current_goal = None

    ## handling costmap services
    def clear_costmap_service(self):
        self.client = self.create_client(ClearEntireCostmap, 
            "/global_costmap/clear_entirely_global_costmap")
        while not self.client.wait_for_service(1.0):
            self.get_logger().warn("Clear_Costmap waiting for service...")
    
    def clear_costmap(self):
        req = ClearEntireCostmap.Request()
        request = self.client.call_async(req)
        request.add_done_callback(partial(self._callback_clear_costmap))

    def _callback_clear_costmap(self, request):
        try:
            response = request.result()
        except Exception as e:
            self.get_logger().error("Clear_Costmap call failed: %r" % (e,))
            
    ##
    def distance_to_goal(self, odom: Vec2) -> float:
        if self.__current_goal is None:
            return float("inf")

        self.__current_goal: PoseStamped

        odom_pose: PoseStamped = self.__transformer.transform(
            self.__current_goal.pose, "odom"
        )

        if odom_pose is None:
            return float("inf")

        goal = (odom_pose.pose.position.x, odom_pose.pose.position.y)
        goal_vector = np.subtract(odom, goal)
        return np.linalg.norm(goal_vector)

    @property
    def current_goal(self) -> PoseStamped:
        return self.__current_goal

    @property
    def status(self) -> Status:
        return self.__status
