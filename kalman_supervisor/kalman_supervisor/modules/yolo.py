import numpy as np

from rclpy import Future
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition, State
from vision_msgs.msg import Detection2DArray, Detection2D

from kalman_supervisor.module import Module


class Yolo(Module):
    def __init__(self):
        super().__init__("yolo")

    def configure(self) -> None:
        self.supervisor.declare_parameter("yolo.max_detection_distance", 5.0)

    def activate(self) -> None:
        self.__get_state_client = self.supervisor.create_client(
            GetState, f"yolo/get_state"
        )
        self.__change_state_client = self.supervisor.create_client(
            ChangeState, f"yolo/change_state"
        )
        self.__detection_sub = self.supervisor.create_subscription(
            Detection2DArray, f"yolo/detections", self.__detection_callback, 10
        )

        self.__enabled = True
        self.__should_be_enabled = False  # forces disable on first tick
        self.__get_state_future: Future | None = None
        self.__change_state_future: Future | None = None
        self.__detections: dict[str, tuple[np.ndarray, str]] = {}

    def tick(self) -> None:
        # If state change is needed and state requests are not pending, send the get state request.
        if (
            self.__enabled != self.__should_be_enabled
            and self.__change_state_future is None
            and self.__get_state_future is None
        ):
            # Wait for get state service to be ready.
            if not self.__get_state_client.service_is_ready():
                self.supervisor.get_logger().info("Waiting for yolo/get_state...")
                self.__get_state_client.wait_for_service()

            request = GetState.Request()
            self.__get_state_future = self.__get_state_client.call_async(request)

        # If a get state request is pending and it is done, optionally follow with a state change request.
        if self.__get_state_future is not None and self.__get_state_future.done():
            response = self.__get_state_future.result()
            self.__get_state_future = None

            # Wait for state change service to be ready.
            if not self.__change_state_client.service_is_ready():
                self.supervisor.get_logger().info("Waiting for yolo/change_state...")
                self.__change_state_client.wait_for_service()

            if (
                response.current_state.id == State.PRIMARY_STATE_INACTIVE
                and self.__should_be_enabled
            ):
                request = ChangeState.Request()
                request.transition.id = Transition.TRANSITION_ACTIVATE
                self.__change_state_future = self.__change_state_client.call_async(
                    request
                )
            elif (
                response.current_state.id == State.PRIMARY_STATE_ACTIVE
                and not self.__should_be_enabled
            ):
                request = ChangeState.Request()
                request.transition.id = Transition.TRANSITION_DEACTIVATE
                self.__change_state_future = self.__change_state_client.call_async(
                    request
                )
            else:
                # No state change is needed, clear the request and update the internal state to match the request.
                self.__enabled = self.__should_be_enabled
                self.__detections.clear()

        # If a state change request is pending and it is done, update the state and clear request.
        if self.__change_state_future is not None and self.__change_state_future.done():
            self.__change_state_future = None

            self.__enabled = not self.__enabled
            self.__detections.clear()
            self.supervisor.get_logger().info(
                f"[YOLO] Detection is now {'enabled' if self.__enabled else 'disabled'}."
            )

    def deactivate(self) -> None:
        self.supervisor.destroy_subscription(self.__detection_sub)
        self.supervisor.destroy_client(self.__change_state_client)
        self.supervisor.destroy_client(self.__get_state_client)

    def __detection_callback(self, msg: Detection2DArray) -> None:
        max_detection_distance = self.supervisor.get_parameter(
            "yolo.max_detection_distance"
        ).value

        detection_msg: Detection2D
        for detection_msg in msg.detections:
            frame = detection_msg.header.frame_id
            # time = Time.from_msg(detection_msg.header.stamp)

            # This should not happen, but just in case.
            if not self.supervisor.tf.can_transform(
                self.supervisor.tf.world_frame(), frame
            ) or not self.supervisor.tf.can_transform(
                self.supervisor.tf.robot_frame(), frame
            ):
                return

            class_ = detection_msg.results[0].hypothesis.class_id
            pos = np.array(
                [
                    detection_msg.results[0].pose.pose.position.x,
                    detection_msg.results[0].pose.pose.position.y,
                    detection_msg.results[0].pose.pose.position.z,
                ]
            )

            # Transform to a frame in which markers should stay static
            world_pos = self.supervisor.tf.transform_numpy(
                pos, self.supervisor.tf.world_frame(), frame
            )

            # Compare the distance to the maximum detection distance
            robot_pos = self.supervisor.tf.robot_pos()
            distance = np.linalg.norm(world_pos - robot_pos)
            if distance > max_detection_distance:
                return

            # Log the detection
            if class_ not in self.__detections:
                self.supervisor.get_logger().info(f"[YOLO] Detected a {class_}.")

            self.__detections[class_] = (world_pos, self.supervisor.tf.world_frame())

    def clear_detections(self) -> None:
        self.__detections.clear()

    def class_pos(self, class_: str, frame: str = "") -> np.ndarray | None:
        if not class_ in self.__detections:
            return None

        if frame == "":
            frame = self.supervisor.tf.world_frame()

        pos, pos_frame = self.__detections[class_]
        if frame != pos_frame:
            pos = self.supervisor.tf.transform_numpy(pos, frame, pos_frame)

        return pos

    def enabled(self) -> bool:
        return self.__enabled

    def enable(self) -> None:
        self.__should_be_enabled = True

    def disable(self) -> None:
        self.__should_be_enabled = False
