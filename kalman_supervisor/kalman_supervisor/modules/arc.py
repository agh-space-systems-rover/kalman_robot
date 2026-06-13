from kalman_supervisor.module import Module
from std_msgs.msg import Float32

class Arc(Module):
    def __init__(self):
        super().__init__("arc")
        self.distance_sub = None
        self.laser_sub = None
        self._distance_traveled = 0.0
        self._laser_distance = float('inf')

    def activate(self) -> None:
        self.distance_sub = self.supervisor.create_subscription(
            Float32,
            "/arc/distance_traveled",
            self._distance_callback,
            10
        )
        self.laser_sub = self.supervisor.create_subscription(
            Float32,
            "/arc/laser_distance",
            self._laser_callback,
            10
        )

    def _distance_callback(self, msg: Float32) -> None:
        self._distance_traveled = msg.data

    def _laser_callback(self, msg: Float32) -> None:
        self._laser_distance = msg.data

    def deactivate(self) -> None:
        if self.distance_sub:
            self.supervisor.destroy_subscription(self.distance_sub)
        if self.laser_sub:
            self.supervisor.destroy_subscription(self.laser_sub)

    def get_distance_traveled(self) -> float:
        return self._distance_traveled

    def sees_ceiling(self) -> bool:
        return self._laser_distance < 3.0
