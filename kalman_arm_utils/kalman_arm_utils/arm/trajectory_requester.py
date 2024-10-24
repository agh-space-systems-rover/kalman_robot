import rclpy
from rclpy.action import ActionServer, ActionClient
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from rclpy.task import Future
from rosidl_runtime_py import set_message_fields
import yaml
from ament_index_python.packages import get_package_share_directory

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory
from example_interfaces.msg import Empty
from kalman_interfaces.msg import ArmTrajectorySelect, ArmGoalStatus, ArmState
from collections import namedtuple
from std_msgs.msg import UInt8

MAX_DISTANCE_RAD = 0.05
STOP_TRAJECTORY_TIMEOUT = 0.5

Trajectory = namedtuple("Trajectory", ["name", "path"])
arm_config = get_package_share_directory("kalman_arm_config")

PREDEFINED_TRAJECTORIES: dict[int, Trajectory] = {}

try:
    with open(f"{arm_config}/config/predefined_trajectories.yaml", "r") as f:
        predefined_trajectories = yaml.safe_load(f)
        MAX_DISTANCE_RAD = predefined_trajectories["max_distance_rad"]
        STOP_TRAJECTORY_TIMEOUT = predefined_trajectories["stop_trajectory_timeout"]
        for pose in predefined_trajectories["trajectories"]:
            PREDEFINED_TRAJECTORIES[int(pose["id"])] = Trajectory(
                pose["name"],
                f"{arm_config}/{pose['path']}",
            )
except:
    print("Error loading predefined trajectories configuration")


class TrajectoryClient(Node):

    def __init__(self):
        super().__init__("trajectory_sender")
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            "joint_trajectory_controller/follow_joint_trajectory",
        )

        self.joints: dict[str, float] = {}
        self._send_goal_future = None
        self._timer = None
        self._goal_handle = None
        self._cancel_future: None | Future = None

        self._status_pub = self.create_publisher(
            ArmGoalStatus, "/trajectory/status", 10
        )
        self._status_pub.publish(ArmGoalStatus(status=ArmGoalStatus.IDLE))

        self.joint_states_sub = self.create_subscription(
            ArmState, "/arm_state", self.update_state, 10
        )

        self._keep_alive_sub = self.create_subscription(
            Empty, "/trajectory/keep_alive", self.keep_alive, 10
        )

        self._abort_sub = self.create_subscription(
            Empty, "/trajectory/abort", self.abort, 10
        )

        self._execute_trajectory_sub = self.create_subscription(
            ArmTrajectorySelect, "/trajectory/execute", self.start_trajectory, 10
        )

        self._change_control_pub = self.create_publisher(
            UInt8, "/change_control_type", 10
        )

        self._send_goal_future = None
        self._goal_handle = None
        self._result_handle = None
        self._timer = None
        self._goal_sent = False

    def update_state(self, msg: ArmState):
        self.joints = {
            "joint_1": msg.joint_1,
            "joint_2": msg.joint_2,
            "joint_3": msg.joint_3,
            "joint_4": msg.joint_4,
            "joint_5": msg.joint_5,
            "joint_6": msg.joint_6,
        }

    def keep_alive(self, msg: Empty):
        if self._timer:
            self._timer.reset()

    def abort(self, msg: Empty):
        self._status_pub.publish(ArmGoalStatus(status=ArmGoalStatus.ABORT_RECEIVED))
        self.timer_callback()

    def start_trajectory(self, msg: ArmTrajectorySelect):
        if msg.trajectory_id not in PREDEFINED_TRAJECTORIES:
            self.get_logger().error("Invalid trajectory ID")
            self._status_pub.publish(ArmGoalStatus(status=ArmGoalStatus.INVALID_ID))
            return
        filename = PREDEFINED_TRAJECTORIES[msg.trajectory_id].path
        while len(self.joints.keys()) == 0:
            rclpy.spin_once(self)

        if self._goal_handle:
            self.get_logger().info("Canceling previous goal")
            self._status_pub.publish(ArmGoalStatus(status=ArmGoalStatus.PREEMPTING))
            self.timer_callback()
            self._cancel_future.add_done_callback(lambda x: self.send_goal(filename))
        else:
            self.send_goal(filename)

    def send_goal(self, filename: str):
        try:
            with open(filename, "r") as f:
                traj_dict = yaml.safe_load(f)

                request = FollowJointTrajectory.Goal()

                set_message_fields(request, traj_dict)

                joint_names = request.trajectory.joint_names

                close_enough = True
                for i in range(len(joint_names)):
                    if (
                        abs(
                            self.joints[joint_names[i]]
                            - request.trajectory.points[0].positions[i]
                        )
                        > MAX_DISTANCE_RAD
                    ):
                        close_enough = False
                        break

                if close_enough:
                    self.get_logger().info("Sending goal...")
                    self._status_pub.publish(
                        ArmGoalStatus(status=ArmGoalStatus.GOAL_SENDING)
                    )
                    self._action_client.wait_for_server()
                    
                    if self._goal_sent:
                        self.get_logger().info("Goal already being sent")
                        return
                    
                    self._goal_sent = True

                    self._send_goal_future = self._action_client.send_goal_async(
                        request
                    )
                    self._send_goal_future.add_done_callback(
                        self.goal_response_callback
                    )
                else:
                    self.get_logger().warn("Too far away for pose...")
                    self._status_pub.publish(
                        ArmGoalStatus(status=ArmGoalStatus.TOO_FAR)
                    )

        except FileNotFoundError:
            self._status_pub.publish(ArmGoalStatus(status=ArmGoalStatus.EXCEPTION))
            self.get_logger().error(
                f"Error loading predefined trajectories from {filename}"
            )

    def goal_response_callback(self, future):
        self._goal_sent = False
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            self._status_pub.publish(ArmGoalStatus(status=ArmGoalStatus.GOAL_REJECTED))
            return

        self._goal_handle = goal_handle

        self.get_logger().info("Goal accepted :)")
        self._status_pub.publish(ArmGoalStatus(status=ArmGoalStatus.GOAL_ACCEPTED))
        self._change_control_pub.publish(UInt8(data=0))

        # Start abort timer
        if not self._timer:
            self._timer = self.create_timer(
                STOP_TRAJECTORY_TIMEOUT, self.timer_callback
            )
        else:
            self._timer.reset()

        self._result_handle = goal_handle.get_result_async()
        self._result_handle.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result: FollowJointTrajectory.Result = future.result().result
        status: int = result.error_code

        if status == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info("Goal succeeded!")
            self._status_pub.publish(ArmGoalStatus(status=ArmGoalStatus.SUCCESS))
        else:
            self.get_logger().warn(f"Goal did not succeed with error code: {status}")
            self._status_pub.publish(ArmGoalStatus(status=ArmGoalStatus.FAILED))
        self._goal_handle = None

    def timer_callback(self):
        self._change_control_pub.publish(UInt8(data=1))

        # Cancel the timer
        if self._timer:
            self._timer.cancel()

        if self._goal_handle:
            self.get_logger().info("Canceling goal")
            self._status_pub.publish(ArmGoalStatus(status=ArmGoalStatus.CANCELLING))
            # Cancel the goal
            self._cancel_future = self._goal_handle.cancel_goal_async()
            self._cancel_future.add_done_callback(self.cancel_done)

    def cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info("Goal successfully canceled")
            self._status_pub.publish(ArmGoalStatus(status=ArmGoalStatus.CANCEL_SUCCESS))
        else:
            self.get_logger().info("Goal failed to cancel")
            self._status_pub.publish(ArmGoalStatus(status=ArmGoalStatus.CANCEL_FAILED))
        self._goal_handle = None


def main(args=None):
    rclpy.init(args=args)

    action_client = TrajectoryClient()

    rclpy.spin(action_client)


if __name__ == "__main__":
    main()
