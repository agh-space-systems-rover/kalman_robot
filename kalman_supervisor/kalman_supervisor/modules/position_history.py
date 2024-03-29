import numpy as np

from kalman_supervisor.modules.map import Map
from kalman_supervisor.module import Module

class PositionHistory(Module):
    def __init__(self):
        super().__init__("position_history")

    def configure(self) -> None:
        self.supervisor.declare_parameter("position_history.world_frame", "map")
        self.supervisor.declare_parameter("position_history.robot_frame", "base_link")
        self.supervisor.declare_parameter("position_history.waypoint_spacing", 0.5)
        self.supervisor.declare_parameter("position_history.max_waypoints", 1000)

    def activate(self) -> None:
        self.world_frame = self.supervisor.get_parameter("position_history.world_frame").value
        self.robot_frame = self.supervisor.get_parameter("position_history.robot_frame").value
        self.waypoint_spacing = self.supervisor.get_parameter("position_history.waypoint_spacing").value
        self.max_waypoints = self.supervisor.get_parameter("position_history.max_waypoints").value

        self.waypoints: list[np.ndarray] = []

    def tick(self) -> None:
        # If the transform is not available, skip this tick.
        if not self.supervisor.tf.can_transform(self.world_frame, self.robot_frame):
            return

        pos = self.supervisor.tf.transform_numpy(np.zeros(3), self.world_frame, self.robot_frame)
        last_pos = self.waypoints[-1] if self.waypoints else None

        if last_pos is None or np.linalg.norm(last_pos - pos) >= self.waypoint_spacing:
            self.waypoints.append(pos)
            if len(self.waypoints) > self.max_waypoints:
                self.waypoints.pop(0)

    def deactivate(self) -> None:
        self.waypoints.clear()

    def latest_at_distance(self, distance: float) -> tuple[np.ndarray, str] | None:
        for i in range(len(self.waypoints) - 1, -1, -1):
            if i == 0:
                return None

            if np.linalg.norm(self.waypoints[i] - self.waypoints[i - 1]) >= distance:
                return self.waypoints[i], self.world_frame

        return None

    def latest_free(self) -> tuple[np.ndarray, str] | None:
        for i in range(len(self.waypoints) - 1, -1, -1):
            if i == 0:
                return None

            waypoint = self.waypoints[i]
            occupancy = self.supervisor.map.occupancy(waypoint[:2], self.world_frame)
            if occupancy == Map.Occupancy.FREE:
                return waypoint, self.world_frame
        
        return None
    