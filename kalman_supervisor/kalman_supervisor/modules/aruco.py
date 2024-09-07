import numpy as np

from rclpy import Future
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition, State
from aruco_opencv_msgs.msg import ArucoDetection, MarkerPose

from kalman_supervisor.module import Module


class ArUco(Module):
    def __init__(self):
        super().__init__("aruco")

    def configure(self) -> None:
        self.supervisor.declare_parameter("aruco.enabled", True)
        self.supervisor.declare_parameter("aruco.deactivate_unused", False)
        self.supervisor.declare_parameter("aruco.num_cameras", 0)
        self.supervisor.declare_parameter("aruco.max_detection_distance", 5.0)

    def activate(self) -> None:
        self.module_enabled = self.supervisor.get_parameter("aruco.enabled").value
        self.deactivate_unused = self.supervisor.get_parameter("aruco.deactivate_unused").value
        if not self.module_enabled:
            return

        num_cameras = self.supervisor.get_parameter("aruco.num_cameras").value

        self.__get_state_clients = []
        self.__change_state_clients = []
        self.__detection_subs = []
        for i in range(num_cameras):
            self.__get_state_clients.append(
                self.supervisor.create_client(GetState, f"aruco/get_state{i}")
            )
            self.__change_state_clients.append(
                self.supervisor.create_client(ChangeState, f"aruco/change_state{i}")
            )
            self.__detection_subs.append(
                self.supervisor.create_subscription(
                    ArucoDetection, f"aruco/detection{i}", self.__detection_callback, 10
                )
            )

        self.__enabled = True
        self.__should_be_enabled = False  # forces disable on first tick
        self.__get_state_futures: list[Future] = []
        self.__change_state_futures: list[Future] = []
        self.__detections: dict[int, tuple[np.ndarray, str]] = {}

    def tick(self) -> None:
        if not self.module_enabled or not self.deactivate_unused:
            return

        # If state change is needed and state requests are not pending, send the get state request.
        if (
            self.__enabled != self.__should_be_enabled
            and len(self.__change_state_futures) == 0
            and len(self.__get_state_futures) == 0
        ):
            # Wait for get state services to be ready.
            for i, client in enumerate(self.__get_state_clients):
                if not client.service_is_ready():
                    self.supervisor.get_logger().info(
                        f"Waiting for aruco/get_state{i}..."
                    )
                    client.wait_for_service()

            request = GetState.Request()
            self.__get_state_futures = [
                client.call_async(request) for client in self.__get_state_clients
            ]

        # If a get state request is pending and it is done, optionally follow with a state change request.
        if len(self.__get_state_futures) != 0 and all(
            [future.done() for future in self.__get_state_futures]
        ):
            responses = [future.result() for future in self.__get_state_futures]
            self.__get_state_futures.clear()

            # Wait for state change services to be ready.
            for i, client in enumerate(self.__change_state_clients):
                if not client.service_is_ready():
                    self.supervisor.get_logger().info(
                        f"Waiting for aruco/change_state{i}..."
                    )
                    client.wait_for_service()

            # Process the responses and send state change requests if needed.
            for response, change_state_client in zip(
                responses, self.__change_state_clients
            ):
                if (
                    response.current_state.id == State.PRIMARY_STATE_INACTIVE
                    and self.__should_be_enabled
                ):
                    request = ChangeState.Request()
                    request.transition.id = Transition.TRANSITION_ACTIVATE
                    self.__change_state_futures.append(
                        change_state_client.call_async(request)
                    )
                elif (
                    response.current_state.id == State.PRIMARY_STATE_ACTIVE
                    and not self.__should_be_enabled
                ):
                    request = ChangeState.Request()
                    request.transition.id = Transition.TRANSITION_DEACTIVATE
                    self.__change_state_futures.append(
                        change_state_client.call_async(request)
                    )

            # If no state change is needed, clear the request and update the internal state.
            if len(self.__change_state_futures) == 0:
                self.__enabled = self.__should_be_enabled
                self.__detections.clear()

        # If a state change request is pending and it is done, update the state and clear request.
        if len(self.__change_state_futures) != 0 and all(
            [future.done() for future in self.__change_state_futures]
        ):
            self.__change_state_futures.clear()

            self.__enabled = not self.__enabled
            self.__detections.clear()
            self.supervisor.get_logger().info(
                f"[ArUco] Detection is now {'enabled' if self.__enabled else 'disabled'}."
            )

    def deactivate(self) -> None:
        if not self.module_enabled:
            return

        for sub in self.__detection_subs:
            self.supervisor.destroy_subscription(sub)
        for client in self.__change_state_clients:
            self.supervisor.destroy_client(client)

    def __detection_callback(self, msg: ArucoDetection) -> None:
        max_detection_distance = self.supervisor.get_parameter(
            "aruco.max_detection_distance"
        ).value

        frame = msg.header.frame_id

        # This should not happen, but just in case.
        if not self.supervisor.tf.can_transform(
            self.supervisor.tf.world_frame(), frame
        ) or not self.supervisor.tf.can_transform(
            self.supervisor.tf.robot_frame(), frame
        ):
            return

        marker_msg: MarkerPose
        for marker_msg in msg.markers:
            id = marker_msg.marker_id
            pos = np.array(
                [
                    marker_msg.pose.position.x,
                    marker_msg.pose.position.y,
                    marker_msg.pose.position.z,
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
            if id not in self.__detections:
                self.supervisor.get_logger().info(
                    f"[ArUco] Detected marker #{id} in frame {frame}."
                )

            self.__detections[id] = (world_pos, self.supervisor.tf.world_frame())

    def clear_detections(self) -> None:
        self.__detections.clear()

    def marker_pos(self, id: int, frame: str = "") -> np.ndarray | None:
        if not id in self.__detections:
            return None

        if frame == "":
            frame = self.supervisor.tf.world_frame()

        pos, pos_frame = self.__detections[id]
        if frame != pos_frame:
            pos = self.supervisor.tf.transform_numpy(pos, frame, pos_frame)

        return pos

    def enabled(self) -> bool:
        return self.__enabled

    def enable(self) -> None:
        self.__should_be_enabled = True

    def disable(self) -> None:
        self.__should_be_enabled = False
