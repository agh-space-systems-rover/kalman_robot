from kalman_supervisor.module import Module
from std_msgs.msg import Float32
from rclpy.node import Publisher
from example_interfaces.msg import Empty


class Arc(Module):
    def __init__(self):
        super().__init__("arc")
        self.distance_sub = None
        self.laser_sub = None
        self.drop_off_antenna_pub = None
        self._distance_traveled = 0.0
        self._laser_distance = float('inf')
        

    def activate(self) -> None:
        self.distance_sub = self.supervisor.create_subscription(
            Float32, "/arc/distance_traveled", self._distance_callback, 10
        )
        self.drop_off_antenna_pub = self.supervisor.create_publisher(
            Empty, "/science/front_sand_storage/drop_of_antenna", 10
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
        if self.drop_off_antenna_pub:
            self.supervisor.destroy_publisher(self.drop_off_antenna_pub)

    def get_distance_traveled(self) -> float:
        return self._distance_traveled

    def sees_ceiling(self) -> bool:
        return self._laser_distance < 3.0

    def drop_off_antenna(self, msg=Empty()) -> None:
        self.drop_off_antenna_pub.publish(msg)
