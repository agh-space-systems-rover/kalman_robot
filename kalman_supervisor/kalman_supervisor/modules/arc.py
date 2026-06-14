import numpy as np

from kalman_supervisor.module import Module
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty


class Arc(Module):
    def __init__(self):
        super().__init__("arc")
        self._peak_position: PoseStamped = None
        self._boulder_position: PoseStamped = None
        self.distance_sub = None
        self.laser_sub = None
        self.drop_off_antenna_pub = None
        self._distance_traveled = 0.0
        self._laser_distance = float('inf')
        

    def activate(self) -> None:
        self.distance_sub = self.supervisor.create_subscription(
            Float32, "/arc/distance_traveled", self._distance_callback, 10
        )
        self.peak_sub = self.supervisor.create_subscription(
            PoseStamped, "/peak_position", self._peak_pos_callback, 10
        )
        self.antenna_drop_pub = self.supervisor.create_publisher(
            Empty, "/science/front_sand_storage/drop_of_antenna", 10
        )
        self.boulder_pos_sub = self.supervisor.create_subscription(
            PoseStamped, "/boulder_position", self._boulder_pos_callback, 10
        )
        self.laser_sub = self.supervisor.create_subscription(
            Float32,
            "/arc/laser_distance",
            self._laser_callback,
            10
        )

    def _distance_callback(self, msg: Float32) -> None:
        self._distance_traveled = msg.data

    def _peak_pos_callback(self, msg: PoseStamped) -> None:
        self._peak_position = msg

    def _boulder_pos_callback(self, msg: PoseStamped) -> None:
        self._boulder_position = msg
    
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

    def get_boulder_position(self, frame: str = "utm") -> np.ndarray | None:
        if self._boulder_position is None:
            return None

        pos = np.array(
            [
                self._boulder_position.pose.position.x,
                self._boulder_position.pose.position.y,
                self._boulder_position.pose.position.z,
            ]
        )

        if frame != self._boulder_position.header.frame_id:
            pos = self.supervisor.tf.transform_numpy(
                pos, frame, self._peak_position.header.frame_id
            )

        return pos

    def get_peak_position(self, frame: str = "utm") -> np.ndarray | None:
        if self._peak_position is None:
            return None

        pos = np.array(
            [
                self._peak_position.pose.position.x,
                self._peak_position.pose.position.y,
                self._peak_position.pose.position.z,
            ]
        )

        if frame != self._peak_position.header.frame_id:
            pos = self.supervisor.tf.transform_numpy(
                pos, frame, self._peak_position.header.frame_id
            )

        return pos

    def drop_antenna(self, msg: Empty) -> None:
        self.antenna_drop_pub.publish(msg)
