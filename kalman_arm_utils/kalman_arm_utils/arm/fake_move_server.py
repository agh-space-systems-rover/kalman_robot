import rclpy
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from rosidl_runtime_py import message_to_yaml
import sys
from moveit_msgs.action import MoveGroup


class FakeMoveServer(Node):

    def __init__(self, file_name: str = None):
        super().__init__("fake_move_server")
        self._action_server = ActionServer(
            self, MoveGroup, "/move_action", self.execute_callback
        )
        self.file_name = file_name

    def execute_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Executing goal...")
        goal: MoveGroup.Goal = goal_handle.request
        yaml_msg = message_to_yaml(goal)
        self.get_logger().info(yaml_msg)

        with open(
            f"/home/kiwi/Programming/arm_monorepo/arm_ws/{self.file_name}.yaml", "w"
        ) as f:
            f.write(yaml_msg)
        # print(goal.trajectory.)
        result = MoveGroup.Result()
        return result


def main(args=None):
    rclpy.init(args=args)

    fts = FakeMoveServer(args[1])

    rclpy.spin(fts)


main(sys.argv)
