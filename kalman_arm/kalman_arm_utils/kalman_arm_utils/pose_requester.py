import copy
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
MAX_DISTANCE_RAD = 0.35  # about 20 deg
# Time [in seconds] after which goal is aborted if no keep_alive message is received
STOP_TRAJECTORY_TIMEOUT = 1.5

arm_config = get_package_share_directory("kalman_arm")


class PoseRequestSender(Node):

    def __init__(self):
        super().__init__("pose_request_sender")
        self._action_client = ActionClient(
            self, MoveGroup, "/arm_controllers/move_action"
        )

        self.default_request: MoveGroup.Goal | None = None
        self.load_default_request(f"{arm_config}/config/pose_config.yaml")

        self.joints: dict[str, float] = {}

        # Stan cyklu życia akcji MoveGroup (ochrona przed wyścigami wątków)
        self._goal_handle = None
        self._send_goal_future: Future | None = None
        self._result_future: Future | None = None
        self._cancel_future: Future | None = None

        self._is_sending: bool = False
        self._is_canceling: bool = False
        self._pending_goal_msg: JointState | None = None

        self._timer = None

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
        self.special_poses: dict[int, dict] = {}
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
                    self.special_poses[int(pose["id"])] = pose
        except Exception as e:
            self.get_logger().error(f"Failed to load special poses from {filename}: {e}")

    def send_special_goal(self, msg: ArmPoseSelect):
        if not self.joints:
            self.get_logger().warn(
                "Target rejected: No current joint positions (arm_state not yet received)."
            )
            return
        if msg.pose_id not in self.special_poses:
            self.get_logger().warn(
                f"Target rejected: Pose ID {msg.pose_id} not found in special poses."
            )
            self._status_pub.publish(ArmGoalStatus(status=ArmGoalStatus.INVALID_ID))
            return

        pose_config = self.special_poses[msg.pose_id]
        target_state = JointState()
        names = []
        positions = []

        for joint_name in [
            "arm_joint_1",
            "arm_joint_2",
            "arm_joint_3",
            "arm_joint_4",
            "arm_joint_5",
            "arm_joint_6",
        ]:
            current_val = self.joints.get(joint_name, 0.0)
            rule = pose_config.get("joints", {}).get(joint_name, {"action": "hold"})
            action = rule.get("action", "hold")

            match action:
                case "hold":
                    target_value = current_val
                case "fixed":
                    target_value = rule.get("value", 0.0)
                case "reversed":
                    target_value = -current_val
                case "offset":
                    target_value = current_val + rule.get("offset", 0.0)
                case _:
                    target_value = current_val

            names.append(joint_name)
            positions.append(target_value)

        target_state.name = names
        target_state.position = positions
        self.send_goal(target_state)

    def load_default_request(self, filename: str):
        try:
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
        if self._timer is not None and not self._timer.is_canceled():
            self._timer.reset()

    def abort(self, msg: Empty):
        # Czyścimy też ewentualny cel czekający w kolejce
        self._pending_goal_msg = None
        self._status_pub.publish(ArmGoalStatus(status=ArmGoalStatus.ABORT_RECEIVED))
        self.cancel_active_goal()

    def send_goal(self, msg: JointState):
        if not self.joints:
            self.get_logger().warn(
                "Target rejected: No current joint positions (arm_state not yet received)."
            )
            return

        if len(msg.position) != 6:
            self.get_logger().warn("Target rejected: Invalid number of joints (expected 6).")
            return

        # 1. Jeśli cel jest właśnie wysyłany (czekamy na akceptację serwera),
        # zapisz nowy cel jako oczekujący — anulujemy bieżący natychmiast po akceptacji.
        if self._is_sending:
            self.get_logger().info("Goal is currently being sent; queueing latest target.")
            self._pending_goal_msg = msg
            return

        # 2. Jeśli aktywny cel trwa, zapamiętaj najnowszy cel i zleć bezpieczne anulowanie.
        # Nowy cel zostanie wysłany dopiero w get_result_callback() po zamknięciu poprzedniego!
        if self._goal_handle is not None:
            self.get_logger().info("Preempting active goal; waiting for full termination.")
            self._pending_goal_msg = msg
            self._status_pub.publish(ArmGoalStatus(status=ArmGoalStatus.PREEMPTING))
            self.cancel_active_goal()
            return

        # 3. Brak aktywnego celu — wysyłamy od razu
        self.publish_pose_goal(msg)

    def publish_pose_goal(self, msg: JointState):
        if self.default_request is None:
            self.get_logger().error("Default request not loaded; cannot send goal.")
            return

        request = copy.deepcopy(self.default_request)
        request.request.goal_constraints = [self.build_goal_constraints(msg)]

        self.get_logger().info("Sending pose goal...")
        self._status_pub.publish(ArmGoalStatus(status=ArmGoalStatus.GOAL_SENDING))
        self.send_request(request)

    def build_goal_constraints(self, msg: JointState) -> Constraints:
        goal_constraints = Constraints(name="pose")
        joints_names = [f"arm_joint_{i}" for i in range(1, 7)]
        goal_constraints.joint_constraints = [
            JointConstraint(
                joint_name=name,
                position=float(pos),
                tolerance_above=0.01,
                tolerance_below=0.01,
            )
            for name, pos in zip(joints_names, msg.position)
        ]
        return goal_constraints

    def send_request(self, request: MoveGroup.Goal):
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("MoveGroup action server not available!")
            self._status_pub.publish(ArmGoalStatus(status=ArmGoalStatus.EXCEPTION))
            return

        if self._is_sending:
            self.get_logger().warn("Goal already being sent.")
            return

        self._is_sending = True
        self._send_goal_future = self._action_client.send_goal_async(request)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future: Future):
        self._is_sending = False
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            self._status_pub.publish(ArmGoalStatus(status=ArmGoalStatus.GOAL_REJECTED))
            self._dispatch_pending_goal()
            return

        self._goal_handle = goal_handle
        self._is_canceling = False

        self.get_logger().info("Goal accepted :)")
        self._status_pub.publish(ArmGoalStatus(status=ArmGoalStatus.GOAL_ACCEPTED))
        self._change_control_pub.publish(UInt8(data=0))

        # Rejestrujemy oczekiwanie na ostateczny wynik (SUCCEEDED / CANCELED / ABORTED)
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self.get_result_callback)

        # Jeśli w trakcie czekania na akceptację przyszedł nowy cel, od razu anulujemy ten świeżo przyjęty
        if self._pending_goal_msg is not None:
            self.get_logger().info("New target arrived while goal was being accepted; canceling immediately.")
            self.cancel_active_goal()
            return

        # Start abort timer
        if self._timer is None:
            self._timer = self.create_timer(
                STOP_TRAJECTORY_TIMEOUT, self.timer_callback
            )
        else:
            self._timer.reset()

    def timer_callback(self):
        self.cancel_active_goal()

    def cancel_active_goal(self):
        self._change_control_pub.publish(UInt8(data=1))

        if self._timer is not None:
            self._timer.cancel()

        # Wysyłamy cancel_goal_async TYLKO RAZ dla danego goal_handle
        if self._goal_handle is not None and not self._is_canceling:
            self._is_canceling = True
            self.get_logger().info("Canceling active goal...")
            self._status_pub.publish(ArmGoalStatus(status=ArmGoalStatus.CANCELLING))
            self._cancel_future = self._goal_handle.cancel_goal_async()
            self._cancel_future.add_done_callback(self.cancel_done)

    def cancel_done(self, future: Future):
        # UWAGA: To potwierdza tylko przyjęcie żądania anulowania (stan CANCELING),
        # a NIE koniec wykonania trajektorii! Nie zerujemy tutaj self._goal_handle.
        try:
            cancel_response = future.result()
            if len(cancel_response.goals_canceling) > 0:
                self.get_logger().info("Cancel request accepted by server.")
                self._status_pub.publish(ArmGoalStatus(status=ArmGoalStatus.CANCEL_SUCCESS))
            else:
                self.get_logger().info("Goal could not be canceled (may have already finished).")
                self._status_pub.publish(ArmGoalStatus(status=ArmGoalStatus.CANCEL_FAILED))
        except Exception as e:
            self.get_logger().error(f"Exception in cancel_done: {e}")

    def get_result_callback(self, future: Future):
        # Dopiero tutaj cel w MoveIt całkowicie zakończył swój cykl życia!
        try:
            wrapped_result = future.result()
            result: MoveGroup.Result = wrapped_result.result
            status: MoveItErrorCodes = result.error_code

            if status.val == MoveItErrorCodes.SUCCESS:
                self.get_logger().info("Goal succeeded!")
                self._status_pub.publish(ArmGoalStatus(status=ArmGoalStatus.SUCCESS))
            elif status.val == MoveItErrorCodes.PREEMPTED:
                self.get_logger().info("Goal preempted / canceled cleanly.")
            else:
                self.get_logger().warn(f"Goal did not succeed with error code: {status.val}")
                self._status_pub.publish(ArmGoalStatus(status=ArmGoalStatus.FAILED))
        except Exception as e:
            self.get_logger().error(f"Exception in get_result_callback: {e}")
        finally:
            if self._timer is not None:
                self._timer.cancel()
            self._goal_handle = None
            self._is_canceling = False
            self._dispatch_pending_goal()

    def _dispatch_pending_goal(self):
        if self._pending_goal_msg is not None:
            next_msg = self._pending_goal_msg
            self._pending_goal_msg = None
            self.get_logger().info("Dispatching queued pose goal...")
            self.publish_pose_goal(next_msg)


def main(args=None):
    rclpy.init(args=args)
    action_client = PoseRequestSender()
    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        pass
    finally:
        action_client.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
