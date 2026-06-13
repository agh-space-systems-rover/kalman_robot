from kalman_supervisor.module import Module
from std_msgs.msg import Float32
from rclpy.node import Publisher
from example_interfaces.msg import Empty


class Arc(Module):
    def __init__(self):
        super().__init__("arch")
        self.distance_sub = None
        self.drop_off_antenna: Publisher = None

    def activate(self) -> None:
        self.distance_sub = self.supervisor.create_subscription(
            Float32, "/arc/distance_traveled", self._distance_callback, 10
        )
        self.drop_off_antenna = self.supervisor.create_publisher(
            Empty, "/science/front_sand_storage/drop_of_antenna", 10
        )

    def _distance_callback(self, msg: Float32) -> None:
        self._distance_traveled = msg.data

    def deactivate(self) -> None:
        if self.distance_sub:
            self.supervisor.destroy_subscription(self.distance_sub)

    def get_distance_traveled(self) -> float:
        return self._distance_traveled

    def drop_off_antenna(self, msg=Empty()) -> None:
        self.drop_off_antenna.publish(msg)
