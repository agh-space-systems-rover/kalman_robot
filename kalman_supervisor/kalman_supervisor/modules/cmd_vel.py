import time
from geometry_msgs.msg import Twist
from rclpy.duration import Duration

from kalman_supervisor.module import Module


class CmdVel(Module):
    def __init__(self):
        super().__init__("cmd_vel")
        self.__rotating = False
        self.__rotation_start_time = None
        self.__rotation_duration = None
        self.__target_angular_z = None
        self.__ramp_up_time = None

    def activate(self) -> None:
        self.__publisher = self.supervisor.create_publisher(Twist, "cmd_vel", 10)

    def tick(self) -> None:
        if self.__rotating:
            current_time = time.monotonic()
            elapsed = current_time - self.__rotation_start_time

            if elapsed >= self.__rotation_duration:
                self.__rotating = False
                self.send_cmd_vel(0.0, 0.0, 0.0001)
                return

            # Calculate ramped velocity using acceleration
            ramp_time = self.__ramp_up_time
            ramp_up = min(elapsed, ramp_time)
            ramp_down = min(self.__rotation_duration - elapsed, ramp_time)

            # Apply acceleration/deceleration profile
            if elapsed < ramp_time:  # Ramping up
                velocity_scale = ramp_up / ramp_time
            elif elapsed > (self.__rotation_duration - ramp_time):  # Ramping down
                velocity_scale = ramp_down / ramp_time
            else:  # Full speed
                velocity_scale = 1.0

            angular_z = self.__target_angular_z * velocity_scale
            self.send_cmd_vel(0.0, 0.0, angular_z)

    def deactivate(self) -> None:
        self.supervisor.destroy_publisher(self.__publisher)

    def send_cmd_vel(self, linear_x: float, linear_y: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.linear.y = float(linear_y)
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(angular_z)
        self.__publisher.publish(msg)

    def rotate_in_place(
        self, angular_z: float, duration: float, ramp_up_time: float = 1.0
    ) -> None:
        self.__rotating = True
        self.__rotation_start_time = time.monotonic()
        self.__rotation_duration = duration
        self.__target_angular_z = angular_z
        self.__ramp_up_time = ramp_up_time

    def is_rotating_in_place(self) -> bool:
        return self.__rotating

    def cancel_rotation_in_place(self) -> None:
        """Cancel any ongoing rotation and stop the robot."""
        if self.__rotating:
            self.__rotating = False
            self.send_cmd_vel(0.0, 0.0, 0.0)
