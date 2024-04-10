import utm
import numpy as np

from rclpy import Future
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from nav2_msgs.action import NavigateToPose
from enum import Enum

from kalman_supervisor.modules.map import Map
from kalman_supervisor.module import Module

# Since this module is extremely complex it has been split into multiple classes that are overlayed on top of each other:
# BasicNav - Sends valid goal or no goal at all. Will fix bad goals that intersect with obstacles or are outside of the map. Does not recover from failed goals. Does not queue up goals, which means that has_goal() must return true before you call send_goal() again. Does not handle long goals (but, as mentioned, will send one partial goal to the furthest free position).
# NavWithRecovery - Extends BasicNav. Will attempt to recover from failed goals by sending the robot to the nearest or previously visited free position. Will disable obstacle layers before sending the robot to the free position and re-enable them after the robot has reached the free position. Will not attempt recovery if the goal is unreachable or if it can't find a free position for recovery.
# NavWithQueuedGoals - Extends NavWithRecovery. Allows to send goals without waiting for them to be canceled. If a goal is already in progress, the new goal will be queued up and sent once the current goal is finished. This is useful for sending goals in quick succession.
# NavWithLongGoals - Extends NavWithQueuedGoals. Allows to send (like BasicNav) long goals that are outside of the map and follows them to the end.


class BasicNav:
    def __init__(self, module: Module):
        self.supervisor = module.supervisor

    def activate(self) -> None:
        self.__client = ActionClient(
            self.supervisor, NavigateToPose, "nav/navigate_to_pose"
        )
        self.__nav2_req_future: Future | None = None
        self.__nav2_res_future: Future | None = None
        self.__nav2_goal_handle: ClientGoalHandle | None = None
        self.__cancel_requested_goal = False
        self.__last_goal_status = GoalStatus.STATUS_SUCCEEDED
        self.__last_corrected_goal: tuple[np.ndarray, str] | None = None

    def tick(self) -> None:
        # Check if a goal request went through and get the goal handle.
        # Request vars are cleared.
        if self.__nav2_req_future is not None:
            if self.__nav2_req_future.done():
                goal_handle: ClientGoalHandle = self.__nav2_req_future.result()
                if goal_handle.accepted:
                    # self.supervisor.get_logger().info("[Nav2] Goal accepted.")

                    # If the goal was accepted, start waiting for the result.
                    self.__nav2_res_future = goal_handle.get_result_async()
                    self.__nav2_goal_handle = goal_handle

                    # Immediately cancel the goal by its handle if requested earlier.
                    if self.__cancel_requested_goal:
                        goal_handle.cancel_goal_async()
                        self.__cancel_requested_goal = False
                else:
                    self.supervisor.get_logger().error(
                        "[Nav2] Goal rejected. This should never happen. Awaiting further goals."
                    )
                    self.__last_goal_status = GoalStatus.STATUS_SUCCEEDED
                    # A success because we are unable to handle a rejection.
                self.__nav2_req_future = None

        # Check if a running goal has finished and clear the state.
        if self.__nav2_res_future is not None:
            if self.__nav2_res_future.done():
                result = self.__nav2_res_future.result()
                # result.result is a NavigateToPose.Result

                if result.status == GoalStatus.STATUS_SUCCEEDED:
                    # self.supervisor.get_logger().info("[Nav2] Goal reached.")
                    self.__last_goal_status = GoalStatus.STATUS_SUCCEEDED
                elif result.status == GoalStatus.STATUS_CANCELED:
                    # self.supervisor.get_logger().info("[Nav2] Goal canceled.")
                    self.__last_goal_status = GoalStatus.STATUS_CANCELED
                else:
                    self.supervisor.get_logger().info("[Nav2] Goal failed.")
                    self.__last_goal_status = GoalStatus.STATUS_ABORTED

                self.__nav2_goal_handle = None
                self.__nav2_res_future = None

    def deactivate(self) -> None:
        self.__client.destroy()

    def has_goal(self) -> bool:
        return self.__nav2_goal_handle is not None or self.__nav2_req_future is not None

    def last_goal_status(self) -> GoalStatus:
        return self.__last_goal_status

    # Request to cancel the goal. Goal will not be canceled immediately. Wait for has_goal() to return False before sending another goal.
    def cancel_goal(self) -> None:
        # If goal was requested but it has not been yet accepted, cancel the goal request.
        if self.__nav2_req_future is not None:
            self.__cancel_requested_goal = True

        # If the goal is already in progress, use the handle to cancel it.
        # Also cancel the response future.
        if self.__nav2_goal_handle is not None:
            self.__nav2_goal_handle.cancel_goal_async()

    # NOTE: No other goal should be in progress when calling this function.
    def send_goal(self, pos: np.ndarray, frame: str) -> None:
        pos = pos.copy()

        # Wait for Nav2.
        if not self.__client.server_is_ready():
            self.supervisor.get_logger().info("Waiting for nav/navigate_to_pose...")
            self.__client.wait_for_server()

        # If the goal is in a frame other than the map, transform it to the map frame.
        pos = self.supervisor.tf.transform_numpy(
            pos, self.supervisor.map.frame(), frame
        )
        frame = self.supervisor.map.frame()

        # Correct the goal if it is too far away.
        if self.supervisor.map.occupancy(pos, frame) == Map.Occupancy.OUT_OF_BOUNDS:
            new_pos = self.supervisor.map.closest_in_bounds_towards_center(pos, frame)

        # Correct the goal if it is inside an obstacle.
        if self.supervisor.map.occupancy(pos, frame) != Map.Occupancy.FREE:
            new_pos = self.supervisor.map.closest_free(pos, frame)
            if new_pos is not None:
                pos = new_pos
            else:
                self.supervisor.get_logger().error(
                    "Goal is inside an obstacle, but there is no free space to travel to. Goal will not be sent."
                )
                # We cannot handle this. Treat this situation as a successful goal.
                self.__last_goal_status = GoalStatus.STATUS_SUCCEEDED
                return

        self.__last_corrected_goal = pos, frame

        # Send the goal.
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = frame
        goal.pose.pose.position.x = pos[0]
        goal.pose.pose.position.y = pos[1]
        if pos.shape[0] > 2:
            goal.pose.pose.position.z = pos[2]
        self.__nav2_req_future = self.__client.send_goal_async(goal)
        self.__cancel_requested_goal = False

    def last_corrected_goal(self) -> tuple[np.ndarray, str] | None:
        return self.__last_corrected_goal


# class Nav(Module):
#     def __init__(self):
#         super().__init__("nav")

#     def configure(self) -> None:
#         self.basic_nav = BasicNav(self)

#     def activate(self) -> None:
#         self.basic_nav.activate()

#     def tick(self) -> None:
#         self.basic_nav.tick()

#     def deactivate(self) -> None:
#         self.basic_nav.deactivate()

#     def has_goal(self) -> bool:
#         return self.basic_nav.has_goal()

#     def cancel_goal(self) -> None:
#         self.basic_nav.cancel_goal()

#     def send_goal(self, pos: np.ndarray, frame: str) -> None:
#         self.basic_nav.send_goal(pos, frame)


# Handles all failed goals, returns only when the goal is finished (reached or unreachable) or canceled.
class NavWithRecovery:
    class RecoveryState(Enum):
        DISENGAGED = 0
        REQUESTED_TO_DISABLE_OBSTACLE_LAYERS = 1
        DRIVING_TO_FREE_POS = 2
        REQUESTED_TO_REENABLE_OBSTACLE_LAYERS = 3
        ABORTING_RECOVERY_WAIT_FOR_LAYERS_AND_GOAL = 4

    def __init__(self, module: Module):
        self.supervisor = module.supervisor
        self.basic_nav = BasicNav(module)

    def activate(self) -> None:
        self.basic_nav.activate()

        self.__goal: tuple[np.ndarray, str] | None = None
        self.__recovery_state = NavWithRecovery.RecoveryState.DISENGAGED
        self.__recovery_free_pos: tuple[np.ndarray, str] | None = None
        self.__last_goal_status = GoalStatus.STATUS_SUCCEEDED

    def tick(self) -> None:
        self.basic_nav.tick()

        # If there should be an ongoing goal, but basic nav has no goal, check what is the status of it.
        if (
            self.__goal is not None
            and not self.basic_nav.has_goal()
            and self.__recovery_state == NavWithRecovery.RecoveryState.DISENGAGED
        ):
            status = self.basic_nav.last_goal_status()
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.__goal = None
                self.__last_goal_status = GoalStatus.STATUS_SUCCEEDED
            elif status == GoalStatus.STATUS_CANCELED:
                self.__goal = None
                self.__last_goal_status = GoalStatus.STATUS_CANCELED
            elif status == GoalStatus.STATUS_ABORTED:
                self.supervisor.get_logger().info(
                    "Goal has failed. Attempting recovery..."
                )

                # Find the nearest free position to send the robot there.
                robot_pos = self.supervisor.tf.robot_pos(self.__goal[1])
                self.__recovery_free_pos = (
                    self.supervisor.position_history.latest_free()
                )
                if self.__recovery_free_pos is None:
                    self.__recovery_free_pos = (
                        self.supervisor.map.closest_free(robot_pos, self.__goal[1]),
                        self.__goal[1],
                    )
                    if self.__recovery_free_pos is None:
                        self.supervisor.get_logger().error(
                            "There's no free positions in history and no free cells around the robot. Recovery is impossible."
                        )
                        self.__recovery_state = (
                            NavWithRecovery.RecoveryState.DISENGAGED
                        )
                        self.__goal = None
                        # Treat this as a finished goal. We are unable to handle this situation.
                        self.__last_goal_status = GoalStatus.STATUS_SUCCEEDED
                        return
                # NOTE: We prefer to navigate to the nearest free position in order to avoid going back and forth.

                # Offset the free position one meter away from the robot to have a good margin between the free goal and costmap edge.
                robot_to_free = self.__recovery_free_pos[0] - robot_pos
                robot_to_free /= np.linalg.norm(
                    robot_to_free[:2]
                )  # The positions should be in map's frame and thus Z can be zeroed.
                self.__recovery_free_pos = (
                    self.__recovery_free_pos[0] + robot_to_free,
                    self.__recovery_free_pos[1],
                )

                # Before sending the robot to the free position, disable obstacle layers.
                self.supervisor.map.disable_obstacle_layers()
                self.__recovery_state = (
                    NavWithRecovery.RecoveryState.REQUESTED_TO_DISABLE_OBSTACLE_LAYERS
                )

        if (
            self.__recovery_state
            == NavWithRecovery.RecoveryState.REQUESTED_TO_DISABLE_OBSTACLE_LAYERS
        ):
            if not self.supervisor.map.obstacle_layers_enabled():
                # Once the obstacle layers are disabled, send the robot to the free position.
                self.supervisor.get_logger().info(
                    "Sending the robot to an unobstructed position."
                )
                self.basic_nav.send_goal(*self.__recovery_free_pos)
                self.__recovery_state = (
                    NavWithRecovery.RecoveryState.DRIVING_TO_FREE_POS
                )

        if self.__recovery_state == NavWithRecovery.RecoveryState.DRIVING_TO_FREE_POS:
            if not self.basic_nav.has_goal():
                self.supervisor.get_logger().info(
                    "Robot is in position. Re-enabling obstacle layers..."
                )
                self.supervisor.map.enable_obstacle_layers()
                self.__recovery_state = (
                    NavWithRecovery.RecoveryState.REQUESTED_TO_REENABLE_OBSTACLE_LAYERS
                )

        if (
            self.__recovery_state
            == NavWithRecovery.RecoveryState.REQUESTED_TO_REENABLE_OBSTACLE_LAYERS
        ):
            if self.supervisor.map.obstacle_layers_enabled():
                # Recovery is complete - send the original goal, rinse and repeat.
                self.supervisor.get_logger().info(
                    "Recovery is complete. Sending the original goal again."
                )
                self.__recovery_state = NavWithRecovery.RecoveryState.DISENGAGED
                # No other goals should be running at the time of recovery.
                self.send_goal(*self.__goal)

        if (
            self.__recovery_state
            == NavWithRecovery.RecoveryState.ABORTING_RECOVERY_WAIT_FOR_LAYERS_AND_GOAL
        ):
            if (
                self.supervisor.map.obstacle_layers_enabled()
                and not self.basic_nav.has_goal()
            ):
                self.__recovery_state = NavWithRecovery.RecoveryState.DISENGAGED
                self.__goal = None
                self.__last_goal_status = GoalStatus.STATUS_CANCELED
                self.supervisor.get_logger().info(
                    "Recovery has been aborted. Goal has been canceled."
                )

    def deactivate(self) -> None:
        self.basic_nav.deactivate()

    def has_goal(self) -> bool:
        return (
            self.__goal is not None
            or self.__recovery_state != NavWithRecovery.RecoveryState.DISENGAGED
        )

    def last_goal_status(self) -> GoalStatus:
        return self.__last_goal_status

    def cancel_goal(self) -> None:
        if self.__recovery_state != NavWithRecovery.RecoveryState.DISENGAGED:
            # Cancel the recovery process.
            self.basic_nav.cancel_goal()
            self.supervisor.map.enable_obstacle_layers()
            self.__recovery_state = (
                NavWithRecovery.RecoveryState.ABORTING_RECOVERY_WAIT_FOR_LAYERS_AND_GOAL
            )
            # tick() will finish the cleanup.
        else:
            self.basic_nav.cancel_goal()
            self.__goal = None

    def send_goal(self, pos: np.ndarray, frame: str) -> None:
        pos = pos.copy()
        self.basic_nav.send_goal(pos, frame)
        self.__goal = (pos, frame)


# class Nav(Module):
#     def __init__(self):
#         super().__init__("nav")

#     def configure(self) -> None:
#         self.nav_w_recovery = NavWithRecovery(self)

#     def activate(self) -> None:
#         self.nav_w_recovery.activate()

#     def tick(self) -> None:
#         self.nav_w_recovery.tick()

#     def deactivate(self) -> None:
#         self.nav_w_recovery.deactivate()

#     def has_goal(self) -> bool:
#         return self.nav_w_recovery.has_goal()

#     def cancel_goal(self) -> None:
#         self.nav_w_recovery.cancel_goal()

#     def send_goal(self, pos: np.ndarray, frame: str) -> None:
#         self.nav_w_recovery.send_goal(pos, frame)


# Allows to send goals without waiting for them to be canceled.
class NavWithQueuedGoals:
    def __init__(self, module: Module):
        self.supervisor = module.supervisor
        self.nav_w_recovery = NavWithRecovery(module)

    def activate(self) -> None:
        self.nav_w_recovery.activate()

        self.__queued_goal: tuple[np.ndarray, str] | None = None
        self.__last_goal_status = GoalStatus.STATUS_SUCCEEDED

    def tick(self) -> None:
        self.nav_w_recovery.tick()

        if not self.nav_w_recovery.has_goal():
            # If there is a queued goal and the current goal is finished, send the queued goal.
            if self.__queued_goal is not None:
                self.nav_w_recovery.send_goal(*self.__queued_goal)
                self.__queued_goal = None
            else:
                # This will fire every tick after the goal is finished. Does not matter.
                self.__last_goal_status = self.nav_w_recovery.last_goal_status()

    def deactivate(self) -> None:
        self.nav_w_recovery.deactivate()

    def has_goal(self) -> bool:
        return self.nav_w_recovery.has_goal() or self.__queued_goal is not None

    def last_goal_status(self) -> GoalStatus:
        return self.__last_goal_status

    def cancel_goal(self) -> None:
        self.nav_w_recovery.cancel_goal()
        self.__queued_goal = None

    def send_goal(self, pos: np.ndarray, frame: str) -> None:
        pos = pos.copy()
        if self.nav_w_recovery.has_goal():
            self.nav_w_recovery.cancel_goal()
            self.__queued_goal = pos, frame
        else:
            self.nav_w_recovery.send_goal(pos, frame)


# class Nav(Module):
#     def __init__(self):
#         super().__init__("nav")

#     def configure(self) -> None:
#         self.nav_w_queue = NavWithQueuedGoals(self)

#     def activate(self) -> None:
#         self.nav_w_queue.activate()

#     def tick(self) -> None:
#         self.nav_w_queue.tick()

#     def deactivate(self) -> None:
#         self.nav_w_queue.deactivate()

#     def has_goal(self) -> bool:
#         return self.nav_w_queue.has_goal()

#     def cancel_goal(self) -> None:
#         self.nav_w_queue.cancel_goal()

#     def send_goal(self, pos: np.ndarray, frame: str) -> None:
#         self.nav_w_queue.send_goal(pos, frame)


class NavWithLongGoals:
    def __init__(self, module: Module):
        self.supervisor = module.supervisor
        self.nav_w_queue = NavWithQueuedGoals(module)

    def activate(self) -> None:
        self.nav_w_queue.activate()

        self.__goal: tuple[np.ndarray, str] | None = None
        self.__goal_is_long = False
        self.__last_goal_start_pos: np.ndarray | None = None
        self.__last_goal_status = GoalStatus.STATUS_SUCCEEDED

    def tick(self) -> None:
        self.nav_w_queue.tick()

        # Monitor whether the final goal has appeared in map and remove the long goal flag immediately.
        if (
            self.__goal is not None
            and self.__goal_is_long
            and self.nav_w_queue.has_goal()
        ):
            if (
                self.supervisor.map.occupancy(self.__goal[0], self.__goal[1])
                != Map.Occupancy.OUT_OF_BOUNDS
            ):
                self.supervisor.get_logger().info(
                    "Long goal just entered the map. Navigating directly there."
                )
                # NavWithRecovery supports send_goal without cancel_goal via NavWithQueuedGoals.
                self.nav_w_queue.send_goal(*self.__goal)
                self.__goal_is_long = False

        # Send another goal once the robot is sufficiently close to the partial goal.
        # Do not send the goal if the robot has not made meaningful progress since the last goal start position.
        if (
            self.__goal is not None
            and self.__goal_is_long
            and self.nav_w_queue.has_goal()
        ):
            last_corrected_goal = (
                self.nav_w_queue.nav_w_recovery.basic_nav.last_corrected_goal()
            )
            if last_corrected_goal is not None:
                robot_pos = self.supervisor.tf.robot_pos(last_corrected_goal[1])[:2]
                goal_pos = last_corrected_goal[0][:2]
                if np.linalg.norm(robot_pos - goal_pos) < 3.0 and (
                    self.__last_goal_start_pos is None
                    or np.linalg.norm(robot_pos - self.__last_goal_start_pos) > 2.0
                ):
                    self.supervisor.get_logger().info(
                        "Almost at the partial goal. Navigating further to the final goal."
                    )
                    self.nav_w_queue.send_goal(*self.__goal)
                    self.__last_goal_start_pos = robot_pos

        # If a goal has finished, check the result.
        if self.__goal is not None and not self.nav_w_queue.has_goal():
            status = self.nav_w_queue.last_goal_status()
            self.__last_goal_status = status

            if status == GoalStatus.STATUS_SUCCEEDED:
                if self.__goal_is_long:
                    # Check if we have meaningfully advanced towards the goal since last success.
                    robot_pos = self.supervisor.tf.robot_pos()[:2]
                    if (
                        self.__last_goal_start_pos is None
                        or np.linalg.norm(robot_pos - self.__last_goal_start_pos) > 2.0
                    ):
                        self.supervisor.get_logger().info(
                            "Partial goal reached. Navigating further to the final goal."
                        )
                        self.nav_w_queue.send_goal(*self.__goal)
                        self.__last_goal_start_pos = robot_pos
                        return
                    else:
                        self.supervisor.get_logger().info(
                            "Failed to advance towards a long goal. Nevertheless, goal is considered successful."
                        )

            self.__goal = None
            self.__goal_is_long = False
            self.__last_goal_start_pos = None

    def deactivate(self) -> None:
        self.nav_w_queue.deactivate()

    def has_goal(self) -> bool:
        return self.__goal is not None

    def last_goal_status(self) -> GoalStatus:
        return self.__last_goal_status

    def cancel_goal(self) -> None:
        self.nav_w_queue.cancel_goal()

    def send_goal(self, pos: np.ndarray, frame: str) -> None:
        pos = pos.copy()

        # Detect long goals.
        if self.supervisor.map.occupancy(pos, frame) == Map.Occupancy.OUT_OF_BOUNDS:
            self.__goal_is_long = True

        self.__goal = (pos, frame)
        self.nav_w_queue.send_goal(pos, frame)

    def distance_to_goal(self) -> float:
        if self.__goal is not None:
            robot_pos = self.supervisor.tf.robot_pos()[:2]
            goal_pos = self.supervisor.tf.transform_numpy(
                self.__goal[0], self.supervisor.tf.world_frame(), self.__goal[1]
            )[:2]
            return np.linalg.norm(robot_pos - goal_pos)
        return np.inf


class Nav(Module):
    def __init__(self):
        super().__init__("nav")

    def configure(self) -> None:
        self.long_nav = NavWithLongGoals(self)

    def activate(self) -> None:
        self.long_nav.activate()

    def tick(self) -> None:
        self.long_nav.tick()

    def deactivate(self) -> None:
        self.long_nav.deactivate()

    def has_goal(self) -> bool:
        return self.long_nav.has_goal()

    def cancel_goal(self) -> None:
        self.long_nav.cancel_goal()

    def send_goal(self, pos: np.ndarray, frame: str = "") -> None:
        # If frame is not provided, use the world frame.
        if frame == "":
            frame = self.supervisor.tf.world_frame()

        self.long_nav.send_goal(pos, frame)

    def send_gps_goal(self, lat: float, lon: float) -> None:
        x, y, _, _ = utm.from_latlon(lat, lon)
        self.send_goal(np.array([x, y, 0]), "utm")

    def distance_to_goal(self) -> float:
        return self.long_nav.distance_to_goal()
