from threading import Condition, Lock
from typing import Literal

import numpy as np

from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionServer, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup

from kalman_supervisor.module import Module
from kalman_interfaces.action import (
    SupervisorTfGoal,
    SupervisorGpsGoal,
    SupervisorGpsArUcoSearch,
    SupervisorGpsYoloSearch,
    SupervisorMappingGoals,
)
from kalman_interfaces.msg import SupervisorMappingGoal


class Missions(Module):
    class Mission:
        __id_counter = 1
        __id_counter_lock = Lock()

        def __init__(self):
            with Missions.Mission.__id_counter_lock:
                self.id = Missions.Mission.__id_counter
                Missions.Mission.__id_counter += 1

    class TfGoal(Mission):
        def __init__(self, x: float, y: float, frame: str):
            super().__init__()
            self.x = x
            self.y = y
            self.frame = frame

    class GpsGoal(Mission):
        def __init__(self, lat: float, lon: float):
            super().__init__()
            self.lat = lat
            self.lon = lon

    class GpsArUcoSearch(Mission):
        def __init__(self, init_lat: float, init_lon: float, marker_id: int):
            super().__init__()
            self.init_lat = init_lat
            self.init_lon = init_lon
            self.marker_id = marker_id

            self.marker_found = False
            self.marker_lat = 0.0
            self.marker_lon = 0.0

            self.marker_world_frame_pos = np.array(
                [0.0, 0.0, 0.0]
            )  # not reported as feedback, used for approach goal

    class GpsYoloSearch(Mission):
        def __init__(self, init_lat: float, init_lon: float, obj_class: str):
            super().__init__()
            self.init_lat = init_lat
            self.init_lon = init_lon
            self.obj_class = obj_class

            self.obj_found = False
            self.obj_lat = 0.0
            self.obj_lon = 0.0

            self.obj_world_frame_pos = np.array([0.0, 0.0, 0.0])

    class MappingGoals(Mission):
        class Goal:
            GoalType = Literal[
                "NAVIGATE_TO_PRECISE_LOCATION",
                "NAVIGATE_TO_ROUGH_LOCATION",
                "ATTEMPT_LOOP_CLOSURE",
                "TAKE_PHOTOS",
            ]
            NAVIGATE_TO_PRECISE_LOCATION = "NAVIGATE_TO_PRECISE_LOCATION"
            NAVIGATE_TO_ROUGH_LOCATION = "NAVIGATE_TO_ROUGH_LOCATION"
            ATTEMPT_LOOP_CLOSURE = "ATTEMPT_LOOP_CLOSURE"
            TAKE_PHOTOS = "TAKE_PHOTOS"
            types = (
                NAVIGATE_TO_PRECISE_LOCATION,
                NAVIGATE_TO_ROUGH_LOCATION,
                ATTEMPT_LOOP_CLOSURE,
                TAKE_PHOTOS,
            )

            def __init__(self, type: GoalType, location_x: float, location_y: float):
                self.type = type
                self.location_x = location_x
                self.location_y = location_y

        def __init__(self, goals: list[Goal]):
            super().__init__()
            self.goals = goals

            self.current_goal = -1
            self.next_photo_label = 1

    def __init__(self):
        super().__init__("missions")

    def activate(self) -> None:
        self.__mission: Missions.Mission | None = None
        self.__mission_goal_handle: ServerGoalHandle | None = None

        self.__queued_mission: Missions.Mission | None = None
        self.__queued_mission_goal_handle: ServerGoalHandle | None = None

        # Callbacks read __mission and read/write __queued_mission.
        # Tick reads/writes __mission and __mission_goal_handle.
        self.__mission_condition = Condition()

        # Create a separate reentrant callback group for action server callbacks.
        # This will allow them to run on the separate thread and block until the mission is complete.
        self.__callback_group = ReentrantCallbackGroup()

        self.__rviz_tf_goal_sub = self.supervisor.create_subscription(
            PoseStamped, "missions/rviz_tf_goal", self.__rviz_tf_goal_cb, 10
        )
        self.__tf_goal_server = ActionServer(
            self.supervisor,
            SupervisorTfGoal,
            "missions/tf_goal",
            self.__tf_goal_cb,
            callback_group=self.__callback_group,
            cancel_callback=self.__action_cancel_cb,
        )
        self.__gps_goal_server = ActionServer(
            self.supervisor,
            SupervisorGpsGoal,
            "missions/gps_goal",
            self.__gps_goal_cb,
            callback_group=self.__callback_group,
            cancel_callback=self.__action_cancel_cb,
        )
        self.__gps_aruco_search_server = ActionServer(
            self.supervisor,
            SupervisorGpsArUcoSearch,
            "missions/gps_aruco_search",
            self.__gps_aruco_search_cb,
            callback_group=self.__callback_group,
            cancel_callback=self.__action_cancel_cb,
        )
        self.__gps_yolo_search_server = ActionServer(
            self.supervisor,
            SupervisorGpsYoloSearch,
            "missions/gps_yolo_search",
            self.__gps_yolo_search_cb,
            callback_group=self.__callback_group,
            cancel_callback=self.__action_cancel_cb,
        )
        self.mapping_server = ActionServer(
            self.supervisor,
            SupervisorMappingGoals,
            "missions/mapping_goals",
            self.__mapping_goals_cb,
            callback_group=self.__callback_group,
            cancel_callback=self.__action_cancel_cb,
        )

    def tick(self) -> None:
        # Global lock is the safest option.
        with self.__mission_condition:
            # If there's no ongoing mission and there's a queued mission,
            # start the queued mission.
            if self.__mission is None and self.__queued_mission is not None:
                self.supervisor.get_logger().info(
                    f"[Missions] Starting mission #{self.__queued_mission.id}."
                )

                # Overwrite the current mission with the queued one.
                self.__mission = self.__queued_mission
                self.__mission_goal_handle = self.__queued_mission_goal_handle
                self.__queued_mission = None
                self.__queued_mission_goal_handle = None

                # Notify actions waiting for the current mission to end.
                self.__mission_condition.notify_all()

            # If there's ongoing mission and a queued mission, abort the running mission.
            # NOTE: This is done after starting the mission to have one tick
            # without a mission when a new one overwrites the current one.
            if self.__mission is not None and self.__queued_mission is not None:
                self.supervisor.get_logger().info(
                    f"[Missions] Aborting mission #{self.__mission.id} in favor of #{self.__queued_mission.id}."
                )

                # Abort current mission's action.
                if self.__mission_goal_handle is not None:
                    self.__mission_goal_handle.abort()

                # Clear current mission.
                self.__mission = None
                self.__mission_goal_handle = None

                # Notify actions waiting for the current mission to end.
                self.__mission_condition.notify_all()

            # If there's an ongoing mission and action server connection is open,
            # send feedback to the client or cancel the mission if requested.
            if self.__mission is not None and self.__mission_goal_handle is not None:
                # If the action server wants us to cancel the mission, do so.
                if self.__mission_goal_handle.is_cancel_requested:
                    self.supervisor.get_logger().info(
                        f"Mission #{self.__mission.id} was canceled by the client."
                    )

                    # Cancel current mission's action.
                    self.__mission_goal_handle.canceled()

                    # Clear current mission.
                    self.__mission = None
                    self.__mission_goal_handle = None

                    # Notify actions waiting for the current mission to end.
                    self.__mission_condition.notify_all()
                else:
                    # Different missions require different feedbacks.
                    if isinstance(self.__mission, Missions.TfGoal):
                        feedback = SupervisorTfGoal.Feedback()
                        feedback.state = self.supervisor.state
                        self.__mission_goal_handle.publish_feedback(feedback)
                    elif isinstance(self.__mission, Missions.GpsGoal):
                        feedback = SupervisorGpsGoal.Feedback()
                        feedback.state = self.supervisor.state
                        self.__mission_goal_handle.publish_feedback(feedback)
                    elif isinstance(self.__mission, Missions.GpsArUcoSearch):
                        feedback = SupervisorGpsArUcoSearch.Feedback()
                        feedback.state = self.supervisor.state
                        feedback.marker_found = self.__mission.marker_found
                        feedback.marker_location.latitude = self.__mission.marker_lat
                        feedback.marker_location.longitude = self.__mission.marker_lon
                        self.__mission_goal_handle.publish_feedback(feedback)
                    elif isinstance(self.__mission, Missions.GpsYoloSearch):
                        feedback = SupervisorGpsYoloSearch.Feedback()
                        feedback.state = self.supervisor.state
                        feedback.object_found = self.__mission.obj_found
                        feedback.object_location.latitude = self.__mission.obj_lat
                        feedback.object_location.longitude = self.__mission.obj_lon
                        self.__mission_goal_handle.publish_feedback(feedback)

    def deactivate(self) -> None:
        self.__gps_yolo_search_server.destroy()
        self.__gps_aruco_search_server.destroy()
        self.__gps_goal_server.destroy()
        self.__tf_goal_server.destroy()
        self.supervisor.destroy_subscription(self.__rviz_tf_goal_sub)

    def __queue_up_mission(
        self, mission: Mission, goal_handle: ServerGoalHandle | None
    ) -> None:
        self.__queued_mission = mission
        self.__queued_mission_goal_handle = goal_handle

        if goal_handle is not None:
            self.__mission_condition.wait_for(
                lambda: (
                    self.__queued_mission is None
                    or self.__queued_mission.id != mission.id
                )
                and (self.__mission is None or self.__mission.id != mission.id)
            )

    def __rviz_tf_goal_cb(self, msg: PoseStamped) -> None:
        mission = Missions.TfGoal(
            msg.pose.position.x, msg.pose.position.y, msg.header.frame_id
        )
        with self.__mission_condition:
            self.__queue_up_mission(mission, None)

    def __tf_goal_cb(self, goal_handle: ServerGoalHandle) -> SupervisorTfGoal.Result:
        # Check if the frame exists.
        if not self.supervisor.tf.can_transform(
            goal_handle.request.location.header.frame_id,
            self.supervisor.tf.world_frame(),
        ):
            self.supervisor.get_logger().warn(
                f"[Missions] Frame '{goal_handle.request.location.header.frame_id}' does not exist. Mission aborted."
            )
            goal_handle.abort()
            return SupervisorTfGoal.Result()

        mission = Missions.TfGoal(
            goal_handle.request.location.point.x,
            goal_handle.request.location.point.y,
            goal_handle.request.location.header.frame_id,
        )
        with self.__mission_condition:
            self.__queue_up_mission(mission, goal_handle)
        return SupervisorTfGoal.Result()

    def __gps_goal_cb(self, goal_handle: ServerGoalHandle) -> SupervisorGpsGoal.Result:
        mission = Missions.GpsGoal(
            goal_handle.request.location.latitude,
            goal_handle.request.location.longitude,
        )
        with self.__mission_condition:
            self.__queue_up_mission(mission, goal_handle)
        return SupervisorGpsGoal.Result()

    def __gps_aruco_search_cb(
        self, goal_handle: ServerGoalHandle
    ) -> SupervisorGpsArUcoSearch.Result:
        mission = Missions.GpsArUcoSearch(
            goal_handle.request.initial_location.latitude,
            goal_handle.request.initial_location.longitude,
            goal_handle.request.marker_id,
        )
        with self.__mission_condition:
            self.__queue_up_mission(mission, goal_handle)
        return SupervisorGpsArUcoSearch.Result()

    def __gps_yolo_search_cb(
        self, goal_handle: ServerGoalHandle
    ) -> SupervisorGpsYoloSearch.Result:
        mission = Missions.GpsYoloSearch(
            goal_handle.request.initial_location.latitude,
            goal_handle.request.initial_location.longitude,
            goal_handle.request.object_class,
        )
        with self.__mission_condition:
            self.__queue_up_mission(mission, goal_handle)
        return SupervisorGpsYoloSearch.Result()

    def __mapping_goals_cb(
        self, goal_handle: ServerGoalHandle
    ) -> SupervisorMappingGoals.Result:
        goals: list[Missions.MappingGoals.Goal] = []
        # Map goals from msg to Supervisor's own goals
        goal: SupervisorMappingGoal
        for goal in goal_handle.request.goals:
            type = Missions.MappingGoals.Goal.types[goal.type]
            goals.append(
                Missions.MappingGoals.Goal(type, goal.location.x, goal.location.y)
            )
        # Insert precise navigation before loop closure and photos if they have a location and it is far away from last goal
        last_location = None
        i = 0
        while i < len(goals):
            goal = goals[i]
            if goal.type in (
                Missions.MappingGoals.Goal.NAVIGATE_TO_PRECISE_LOCATION,
                Missions.MappingGoals.Goal.NAVIGATE_TO_ROUGH_LOCATION,
            ):
                last_location = np.array([goal.location_x, goal.location_y])
            elif goal.type in (
                Missions.MappingGoals.Goal.ATTEMPT_LOOP_CLOSURE,
                Missions.MappingGoals.Goal.TAKE_PHOTOS,
            ):
                # If the loop/photo goal has a location
                if abs(goal.location_x) > 1e-3 or abs(goal.location_y) > 1e-3:
                    # If there was no last precise location or the last one was too far away:
                    if (
                        last_location is None
                        or np.linalg.norm(
                            np.array([goal.location_x, goal.location_y]) - last_location
                        )
                        > 0.5
                    ):
                        last_location = np.array([goal.location_x, goal.location_y])
                        goals.insert(
                            i,
                            Missions.MappingGoals.Goal(
                                Missions.MappingGoals.Goal.NAVIGATE_TO_PRECISE_LOCATION,
                                goal.location_x,
                                goal.location_y,
                            ),
                        )
            i += 1
        mission = Missions.MappingGoals(goals)
        with self.__mission_condition:
            self.__queue_up_mission(mission, goal_handle)
        return SupervisorMappingGoals.Result()

    def __action_cancel_cb(self, _) -> CancelResponse:
        return CancelResponse.ACCEPT

    def has_mission(self) -> bool:
        with self.__mission_condition:
            return self.__mission is not None

    def get_mission(self) -> Mission | None:
        with self.__mission_condition:
            return self.__mission

    def succeed_mission(self) -> None:
        with self.__mission_condition:
            if self.__mission is None:
                return

            self.supervisor.get_logger().info(
                f"[Missions] Mission #{self.__mission.id} completed successfully."
            )

            # Succeed current mission's action.
            if self.__mission_goal_handle is not None:
                self.__mission_goal_handle.succeed()

            # Clear the current mission.
            self.__mission = None
            self.__mission_goal_handle = None

            # Notify actions waiting for the current mission to end.
            self.__mission_condition.notify_all()
