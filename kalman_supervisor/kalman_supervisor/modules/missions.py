import copy

from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle

from kalman_supervisor.module import Module
from kalman_interfaces.action import SupervisorTfGoal

class Missions(Module):
    class Mission:
        pass

    class TfGoal(Mission):
        def __init__(self, x: float, y: float, frame: str):
            self.x = x
            self.y = y
            self.frame = frame

    class GpsYoloSearch(Mission):
        class LongTermMemory:
            def __init__(self):
                self.obj_lat: float | None = None
                self.obj_lon: float | None = None

        def __init__(self, init_lat: float, init_lon: float, obj_class: str):
            self.init_lat = init_lat
            self.init_lon = init_lon
            self.obj_class = obj_class
            self.memory = Missions.GpsYoloSearch.LongTermMemory()

    def __init__(self):
        super().__init__("missions")

    def activate(self) -> None:
        self.__mission: Missions.Mission | None = None
        self.__server_goal_handle: ServerGoalHandle | None = None
        self.__next_tick_cbs = []

        self.__rviz_tf_goal_sub = self.supervisor.create_subscription(PoseStamped, 'missions/rviz_tf_goal', self.__rviz_tf_goal_cb, 10)
        self.__tf_goal_server = ActionServer(self.supervisor, SupervisorTfGoal, 'missions/tf_goal', self.__tf_goal_callback)

    def tick(self) -> None:
        # Fire any queued up tick callbacks.
        cbs = copy.copy(self.__next_tick_cbs)
        self.__next_tick_cbs = []
        for cb in cbs:
            cb()

        # If there's an ongoing mission and action server connection is open,
        # send feedback to the client or cancel the mission if requested.
        if self.__mission is not None and self.__server_goal_handle is not None:
            # If the action server wants us to cancel the mission, do so.
            if self.__server_goal_handle.is_cancel_requested:
                self.__server_goal_handle.canceled()
                self.__server_goal_handle = None
                self.__mission = None
            else:
                # Different missions require different feedbacks.
                if isinstance(self.__mission, Missions.TfGoal):
                    feedback = SupervisorTfGoal.Feedback()
                    feedback.state = self.supervisor.state
                    self.__server_goal_handle.publish_feedback(feedback)
                elif isinstance(self.__mission, Missions.GpsYoloSearch):
                    feedback = SupervisorTfGoal.Feedback()
                    feedback.state = self.supervisor.state
                    feedback.object_location.latitude = self.__mission.memory.obj_lat
                    feedback.object_location.longitude = self.__mission.memory.obj_lon
                    self.__server_goal_handle.publish_feedback(feedback)

        # If there's no mission and action server connection is open,
        # send result to the client.
        if self.__mission is None and self.__server_goal_handle is not None:
            result = SupervisorTfGoal.Result()
            self.__server_goal_handle.succeed(result)
            self.__server_goal_handle = None

    def deactivate(self) -> None:
        self.__tf_goal_server.destroy()
        self.supervisor.destroy_subscription(self.__rviz_tf_goal_sub)

    def __queue_up_mission(self, mission: Mission, goal_handle: ServerGoalHandle | None) -> None:
        # If there's an ongoing mission, abort it.
        if self.__mission is not None:
            if self.__server_goal_handle is not None:
                self.__server_goal_handle.abort()
                self.__server_goal_handle = None
            self.__mission = None
        
        # Queue up the new mission to be set after one tick.
        def cb() -> None:
            # __queue_up_mission is always called from a rclpy callback.
            # After rclpy reaches the global tick() timer callback, it will execute Missions.tick().
            # But we want all the tick-able modules and states to know about the mission being cancelled before the new mission is set.
            # That's why on the next tick we set another callback to be sure that mission cancellation is received by
            # all tick-able objects on the next tick and only then we set the new mission on the second next tick.
            def cb2() -> None:
                self.__mission = mission
                self.__server_goal_handle = goal_handle
            self.__next_tick_cbs.append(cb2)
        self.__next_tick_cbs.append(cb)

    def __rviz_tf_goal_cb(self, msg: PoseStamped) -> None:
        mission = Missions.TfGoal(msg.pose.position.x, msg.pose.position.y, msg.header.frame_id)
        self.__queue_up_mission(mission, None)

    def __tf_goal_callback(self, goal_handle: ServerGoalHandle) -> None:
        mission = Missions.TfGoal(goal_handle.request.goal.x, goal_handle.request.goal.y, goal_handle.request.goal.frame)
        self.__queue_up_mission(mission, goal_handle)

    def has_mission(self) -> bool:
        return self.__mission is not None

    def get_mission(self) -> Mission | None:
        return self.__mission

    def end_mission(self) -> None:
        self.__mission = None
