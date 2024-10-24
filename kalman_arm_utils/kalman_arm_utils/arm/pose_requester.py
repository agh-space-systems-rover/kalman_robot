import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Future
from rosidl_runtime_py import set_message_fields
import yaml
from ament_index_python.packages import get_package_share_directory

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from example_interfaces.msg import Empty
from kalman_interfaces.msg import ArmPoseSelect, ArmGoalStatus, ArmState
from collections import namedtuple
from std_msgs.msg import UInt8

MAX_DISTANCE_RAD = 0.35  # about 20

Pose = namedtuple("Pose", ["name", "path", "joints_set", "joints_checked", "joints_reversed", "safe_previous_poses"])

arm_config = get_package_share_directory("kalman_arm_config")

PREDEFINED_POSES: dict[int, Pose] = {}
try:
    with open(f"{arm_config}/config/predefined_poses.yaml", "r") as f:
        predefined_poses = yaml.safe_load(f)
        MAX_DISTANCE_RAD = predefined_poses["max_distance_rad"]
        STOP_TRAJECTORY_TIMEOUT = predefined_poses["stop_trajectory_timeout"]
        for pose in predefined_poses["poses"]:
            PREDEFINED_POSES[int(pose["id"])] = Pose(
                pose["name"],
                f"{arm_config}/{pose['path']}",
                pose["joints_set"],
                pose["joints_checked"],
                pose["joints_reversed"],
                pose["safe_previous_poses"],
            )
except:
    print("Error loading predefined poses configuration")


class PoseRequestSender(Node):

    def __init__(self):
        super().__init__("pose_request_sender")
        self._action_client = ActionClient(self, MoveGroup, "/move_action")

        self.joints: dict[str, float] = {}
        self._send_goal_future = None
        self._timer = None
        self._goal_handle = None
        self._cancel_future: None | Future = None
        self._goal_sent = False

        self.joint_states_sub = self.create_subscription(
            ArmState, "/arm_state", self.update_state, 10
        )

        self._change_control_pub = self.create_publisher(
            UInt8, "/change_control_type", 10
        )

        self._execute_sub = self.create_subscription(
            ArmPoseSelect,
            "/pose_request/execute",
            self.send_goal,
            10,
        )

        self._status_pub = self.create_publisher(
            ArmGoalStatus, "/pose_request/status", 10
        )
        self._status_pub.publish(ArmGoalStatus(status=ArmGoalStatus.IDLE))

        self._keep_alive_sub = self.create_subscription(
            Empty, "/pose_request/keep_alive", self.keep_alive, 10
        )

        self._abort_sub = self.create_subscription(
            Empty, "/pose_request/abort", self.abort, 10
        )

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

    def send_goal(self, msg: ArmPoseSelect):
        if msg.pose_id not in PREDEFINED_POSES:
            self.get_logger().error("Invalid pose ID")
            self._status_pub.publish(ArmGoalStatus(status=ArmGoalStatus.INVALID_ID))
            return
        
        pose = PREDEFINED_POSES[msg.pose_id]
        while len(self.joints.keys()) == 0:
            rclpy.spin_once(self)

        if self._goal_handle:
            self.get_logger().info("Canceling previous goal")
            self._status_pub.publish(ArmGoalStatus(status=ArmGoalStatus.PREEMPTING))
            self.timer_callback()
            self._cancel_future.add_done_callback(lambda x: self.publish_pose_goal(pose))
        else:
            self.publish_pose_goal(pose)

    def publish_pose_goal(self, pose: Pose):
        request = self.get_request_from_file(pose.path)
        self.reset_not_set_joints(request, pose.joints_set)
        self.reverse_joints(request, pose.joints_reversed)
        
        if self.check_joints_too_far(request, pose.joints_checked) or self.check_is_from_safe_previous_poses(pose):
            self.get_logger().info("Sending pose goal...")
            self._status_pub.publish(ArmGoalStatus(status=ArmGoalStatus.GOAL_SENDING))
            self.send_request(request)
        else:
            self.get_logger().warn("Too far away for pose...")
            self._status_pub.publish(ArmGoalStatus(status=ArmGoalStatus.TOO_FAR))
            
    def get_request_from_file(self, filename: str) -> MoveGroup.Goal:
        try:
            predefined_pose = None
            with open(filename, "r") as f:
                predefined_pose = yaml.safe_load(f)

            request = MoveGroup.Goal()

            set_message_fields(request, predefined_pose)

            return request

        except FileNotFoundError:
            self.get_logger().error(f"Error loading predefined pose from {filename}")
            self._status_pub.publish(ArmGoalStatus(status=ArmGoalStatus.EXCEPTION))

            return None
        
    
    def reset_not_set_joints(self, request: MoveGroup.Goal, joints_to_set: list[str]):
        goal_constraint: Constraints = request.request.goal_constraints[0]
        joint_constraints: list[JointConstraint] = sorted(
            goal_constraint.joint_constraints, key=lambda x: x.joint_name
        )

        for i in range(6):
            if joint_constraints[i].joint_name not in joints_to_set:
                joint_constraints[i].position = self.joints[
                    joint_constraints[i].joint_name
                ]

        goal_constraint.joint_constraints = joint_constraints
        request.request.goal_constraints = [goal_constraint]
        
        
    def reverse_joints(self, request: MoveGroup.Goal, joints_to_reverse: list[str]):
        goal_constraint: Constraints = request.request.goal_constraints[0]
        joint_constraints: list[JointConstraint] = sorted(
            goal_constraint.joint_constraints, key=lambda x: x.joint_name
        )

        for i in range(6):
            if joint_constraints[i].joint_name in joints_to_reverse:
                joint_constraints[i].position = -self.joints[
                    joint_constraints[i].joint_name
                ]

        goal_constraint.joint_constraints = joint_constraints
        request.request.goal_constraints = [goal_constraint]

        
    def check_joints_too_far(self, request: MoveGroup.Goal, joints_to_check: list[str]) -> bool:
        goal_constraint: Constraints = request.request.goal_constraints[0]
        joint_constraints: list[JointConstraint] = sorted(
            goal_constraint.joint_constraints, key=lambda x: x.joint_name
        )

        close_enough = True
        for i in range(6):
            if (joint_constraints[i].joint_name in joints_to_check) and (
                abs(
                    self.joints[joint_constraints[i].joint_name]
                    - joint_constraints[i].position
                )
                > MAX_DISTANCE_RAD
            ):
                close_enough = False
                
        return close_enough
    
    def check_is_from_safe_previous_poses(self, pose: Pose) -> bool:
        for safe_pose in pose.safe_previous_poses:
            if safe_pose in PREDEFINED_POSES:
                safe_pose = PREDEFINED_POSES[safe_pose]
                if self.check_joints_too_far(self.get_request_from_file(safe_pose.path), safe_pose.joints_checked):
                    return True
        return False
        

    def send_request(self, request, check=True):
    
        self._action_client.wait_for_server()

        if self._goal_sent:
            self.get_logger().info("Goal already being sent")
            return
        
        self._goal_sent = True
        self._send_goal_future = self._action_client.send_goal_async(request)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    

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
        result: MoveGroup.Result = future.result().result
        status: MoveItErrorCodes = result.error_code

        if status.val == MoveItErrorCodes.SUCCESS:
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

    action_client = PoseRequestSender()

    rclpy.spin(action_client)


if __name__ == "__main__":
    main()
