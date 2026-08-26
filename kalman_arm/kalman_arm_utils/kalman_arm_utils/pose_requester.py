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
from std_msgs.msg import UInt8
from sensor_msgs.msg import JointState

# Threshold distance - how much each joint can be away from predefined position
MAX_DISTANCE_RAD = 0.35  # about 20
# Time [in seconds] after which goal is aborted if no keep_alive message is received
STOP_TRAJECTORY_TIMEOUT = 1.5

arm_config = get_package_share_directory("kalman_arm")

class PoseRequestSender(Node):

    def __init__(self):
        super().__init__("pose_request_sender")
        self._action_client = ActionClient(
            self, MoveGroup, "/arm_controllers/move_action"
        )

        self.default_request: MoveGroup.Goal = None
        self.load_default_request(f"{arm_config}/config/pose_config.yaml")

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
            JointState,
            "/pose_request/execute",
            self.send_goal,
            10,
        )
        self._execute_sub_special = self.create_subscription(
            ArmPoseSelect,
            "/pose_request/execute_special",
            self.send_special_goal,
            10,
        )
        self.special_poses: dict[int,dict] = {}
        self.load_special_poses(f"{arm_config}/config/special_poses.yaml")

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
    def load_special_poses(self, filename: str):
        try:
            with open(filename, "r") as f:
                data = yaml.safe_load(f)
                for pose in data.get("special_poses", []):
                    self.special_poses[int(pose['id'])] = pose
        except Exception as e:
            self.get_logger().error(f"Failed to load special poses from {filename}: {e}")

    def send_special_goal(self, msg: ArmPoseSelect):
        if not self.joints:
            self.get_logger().warn("Target rejected: No current joint positions (arm_state not yet received).")
            return
        if msg.pose_id not in self.special_poses:
            self.get_logger().warn(f"Target rejected: Pose ID {msg.pose_id} not found in special poses.")
            self._status_pub.publish(ArmGoalStatus(status=ArmGoalStatus.INVALID_ID))
            return

        pose_config = self.special_poses[msg.pose_id]
        target_state = JointState()
        names = []
        positions = []
        for joint_name, rule in pose_config.get("joints", {}).items():
            if joint_name not in self.joints:
                continue
            action = rule.get("action", "hold")
            current_val = self.joints[joint_name]
            match action:
                case "hold":
                    target_value = current_val
                case "fixed":
                    target_value = rule.get("value", 0.0)
                case "reversed":
                    target_value = -current_val
                case "offset":
                    target_value = current_val + rule.get("offset", 0.0)
            names.append(joint_name)
            positions.append(target_value)

        target_state.name = names
        target_state.position = positions
        self.send_goal(target_state)

    def load_default_request(self, filename: str):
        try:
            default_config = None
            with open(filename, "r") as f:
                default_config = yaml.safe_load(f)
            request = MoveGroup.Goal()
            set_message_fields(request, default_config)
            self.default_request = request

        except FileNotFoundError:
            self.get_logger().error(f"Error loading default pose from {filename}")
            self._status_pub.publish(ArmGoalStatus(status=ArmGoalStatus.EXCEPTION))

    def update_state(self, msg: ArmState):
        self.joints = {
            "arm_joint_1": msg.joint_1,
            "arm_joint_2": msg.joint_2,
            "arm_joint_3": msg.joint_3,
            "arm_joint_4": msg.joint_4,
            "arm_joint_5": msg.joint_5,
            "arm_joint_6": msg.joint_6,
        }

    def keep_alive(self, msg: Empty):
        if self._timer:
            self._timer.reset()

    def abort(self, msg: Empty):
        self._status_pub.publish(ArmGoalStatus(status=ArmGoalStatus.ABORT_RECEIVED))
        self.timer_callback()

    def send_goal(self, msg: JointState):
        if not self.joints:
            self.get_logger().warn("Target rejected: No current joint positions (arm_state not yet received).")
            return

        if self._goal_handle:
            self.get_logger().info("Canceling previous goal")
            self._status_pub.publish(ArmGoalStatus(status=ArmGoalStatus.PREEMPTING))
            self.timer_callback()
            self._cancel_future.add_done_callback(
                lambda x: self.publish_pose_goal(msg)
            )
        else:
            self.publish_pose_goal(msg)

    def publish_pose_goal(self, msg: JointState):
        request = self.default_request
        request.request.goal_constraints = [self.build_goal_constraints(msg)]

        #TODO here should be collision detection and checking if the goal is reachable, if not then publish ArmGoalStatus with status=ArmGoalStatus.GOAL_UNREACHABLE but for what we now in given space probably all poses should be accesibly
        self.get_logger().info("Sending pose goal...")
        self._status_pub.publish(ArmGoalStatus(status=ArmGoalStatus.GOAL_SENDING))
        self.send_request(request)

    def build_goal_constraints(self, msg: JointState) -> Constraints:
        goal_constraints: Constraints = Constraints(name="pose")
        joints_names = sorted(self.joints.keys(), key=lambda x: x)
        joint_constraints: list[JointConstraint] = [JointConstraint(joint_name=name) for name in joints_names]

        request_joint = dict(zip(msg.name, msg.position))
        for i, constrain in enumerate(joint_constraints):
            name = constrain.joint_name
            if name in request_joint:
                constrain.position = float(request_joint[name])
            else:
                constrain.position = float(self.joints[name])
            constrain.tolerance_above = 0.01
            constrain.tolerance_below = 0.01

        goal_constraints.joint_constraints = joint_constraints
        return goal_constraints
    
    # def check_joints_too_far(
    #     self, request: MoveGroup.Goal, joints_to_check: list[str]
    # ) -> bool:
    #     goal_constraint: Constraints = request.request.goal_constraints[0]
    #     joint_constraints: list[JointConstraint] = sorted(
    #         goal_constraint.joint_constraints, key=lambda x: x.joint_name
    #     )

    #     close_enough = True
    #     for i in range(6):
    #         if (joint_constraints[i].joint_name in joints_to_check) and (
    #             abs(
    #                 self.joints[joint_constraints[i].joint_name]
    #                 - joint_constraints[i].position
    #             )
    #             > MAX_DISTANCE_RAD
    #         ):
    #             close_enough = False

    #     return close_enough

    # def check_is_from_safe_previous_poses(self, pose: Pose) -> bool:
    #     for safe_pose in pose.safe_previous_poses:
    #         if safe_pose in PREDEFINED_POSES:
    #             safe_pose = PREDEFINED_POSES[safe_pose]
    #             if self.check_joints_too_far(
    #                 self.get_request_from_file(safe_pose.path), safe_pose.joints_checked
    #             ):
    #                 return True
    #     return False
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
