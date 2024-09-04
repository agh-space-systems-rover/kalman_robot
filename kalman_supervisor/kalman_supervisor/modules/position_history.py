import numpy as np

from kalman_supervisor.modules.map import Map
from kalman_supervisor.module import Module


class PositionHistory(Module):
    def __init__(self):
        super().__init__("position_history")

    def configure(self) -> None:
        self.supervisor.declare_parameter("position_history.waypoint_spacing", 0.5)
        self.supervisor.declare_parameter("position_history.max_waypoints", 1000)

    def activate(self) -> None:
        self.waypoints: list[np.ndarray] = []

    def tick(self) -> None:
        waypoint_spacing = self.supervisor.get_parameter(
            "position_history.waypoint_spacing"
        ).value
        max_waypoints = self.supervisor.get_parameter(
            "position_history.max_waypoints"
        ).value

        # If the transform is not available, skip this tick.
        if not self.supervisor.tf.can_transform(
            self.supervisor.tf.world_frame(), self.supervisor.tf.robot_frame()
        ):
            return

        pos = self.supervisor.tf.robot_pos()
        last_pos = self.waypoints[-1] if self.waypoints else None

        if last_pos is None or np.linalg.norm(last_pos - pos) >= waypoint_spacing:
            self.waypoints.append(pos)
            if len(self.waypoints) > max_waypoints:
                self.waypoints.pop(0)

    def deactivate(self) -> None:
        self.waypoints.clear()

    def latest_at_distance(self, distance: float) -> tuple[np.ndarray, str] | None:
        for i in range(len(self.waypoints) - 1, -1, -1):
            if i == 0:
                return None

            if np.linalg.norm(self.waypoints[i] - self.waypoints[i - 1]) >= distance:
                return self.waypoints[i], self.supervisor.tf.world_frame()

        return None

    def latest_free(self) -> tuple[np.ndarray, str] | None:
        for i in range(len(self.waypoints) - 1, -1, -1):
            if i == 0:
                return None

            waypoint = self.waypoints[i]
            occupancy = self.supervisor.map.occupancy(
                waypoint[:2], self.supervisor.tf.world_frame()
            )
            if occupancy == Map.Occupancy.FREE or occupancy == Map.Occupancy.PARTIALLY_OCCUPIED:
                return waypoint, self.supervisor.tf.world_frame()

        return None
