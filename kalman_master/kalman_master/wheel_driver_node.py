import rclpy
import numpy as np
import time
from rclpy.node import Node
from struct import pack

from kalman_interfaces.msg import (
    MasterMessage,
    WheelStates,
    WheelState,
    WheelTemperatures,
    WheelTemperature,
)


METRIC_VELOCITY_TO_MOTOR_VALUE_FACTOR = 100
MAX_RATE = 10
MIN_DT = 1 / MAX_RATE
WARN_RATE = 100


def decode_u2(data):
    return int.from_bytes(data, "big", signed=True)


class WheelDriver(Node):
    def __init__(self):
        super().__init__("wheel_driver")
        self.master_pub = self.create_publisher(
            MasterMessage, "master_com/ros_to_master", 10
        )

        self.create_subscription(WheelStates, "wheel_states", self.states_received, 1)
        self.create_subscription(
            MasterMessage,
            f"master_com/master_to_ros/{hex(MasterMessage.MOTOR_GET_WHEELS)[1:]}",
            self.return_received,
            10,
        )
        self.create_subscription(
            MasterMessage,
            f"master_com/master_to_ros/{hex(MasterMessage.MOTOR_GET_TEMPERATURE)[1:]}",
            self.temps_received,
            10,
        )

        self.return_pub = self.create_publisher(WheelStates, "wheel_states/return", 10)
        self.temp_pub = self.create_publisher(WheelTemperatures, "wheel_temps", 10)
        self.last_publish_time = 0
        self.rate_warn_msgs_since_last_check = 0
        self.rate_warn_last_check_time = 0

    def states_received(self, msg: WheelStates):
        # Monitor wheel states rate.
        current_time = time.time()
        if current_time - self.rate_warn_last_check_time > 1:
            if self.rate_warn_msgs_since_last_check > WARN_RATE:
                self.get_logger().warn(
                    f"Wheel states rate seems way too high: {self.rate_warn_msgs_since_last_check} Hz"
                )
            self.rate_warn_msgs_since_last_check = 0
            self.rate_warn_last_check_time = current_time

        # Throttle publishing rate.
        time_since_last_publish = current_time - self.last_publish_time
        if time_since_last_publish < MIN_DT:
            # Skip publishing if the rate is too high.
            return
        self.last_publish_time = current_time

        data = [
            msg.front_right.velocity * METRIC_VELOCITY_TO_MOTOR_VALUE_FACTOR,
            msg.back_right.velocity * METRIC_VELOCITY_TO_MOTOR_VALUE_FACTOR,
            msg.back_left.velocity * METRIC_VELOCITY_TO_MOTOR_VALUE_FACTOR,
            msg.front_left.velocity * METRIC_VELOCITY_TO_MOTOR_VALUE_FACTOR,
            -np.clip(np.rad2deg(msg.front_right.angle), -90, 90),
            np.clip(np.rad2deg(msg.back_right.angle), -90, 90),
            np.clip(np.rad2deg(msg.back_left.angle), -90, 90),
            -np.clip(np.rad2deg(msg.front_left.angle), -90, 90),
        ]
        data = [int(x) for x in data]
        data = list(pack("b" * len(data), *data))
        self.master_pub.publish(
            MasterMessage(cmd=MasterMessage.MOTOR_SET_WHEELS, data=data)
        )

    def return_received(self, msg: MasterMessage):
        data = list(msg.data)
        FRAME_RATE = 5
        # NOTE: This framerate adheres to docs, but it is wrong. The frames actually arrive at 2 Hz.
        velocities = [
            decode_u2(data[0:2]) * FRAME_RATE / 10000 / 30,
            decode_u2(data[2:4]) * FRAME_RATE / 10000 / 30,
            decode_u2(data[4:6]) * FRAME_RATE / 10000 / 30,
            decode_u2(data[6:8]) * FRAME_RATE / 10000 / 30,
        ]  # NOTE: I am not sure why the values are 30 times bigger. This is not compliant with Motor Controller's docs.
        angles = [
            -np.deg2rad(decode_u2([data[8]])),
            np.deg2rad(decode_u2([data[9]])),
            np.deg2rad(decode_u2([data[10]])),
            -np.deg2rad(decode_u2([data[11]])),
        ]
        angles = [np.arctan2(np.sin(x), np.cos(x)) for x in angles]
        self.return_pub.publish(
            WheelStates(
                front_right=WheelState(velocity=velocities[0], angle=angles[0]),
                back_right=WheelState(velocity=velocities[1], angle=angles[1]),
                back_left=WheelState(velocity=velocities[2], angle=angles[2]),
                front_left=WheelState(velocity=velocities[3], angle=angles[3]),
            )
        )

    def temps_received(self, msg: MasterMessage):
        data = list(msg.data)
        motor_temps = [float(decode_u2([data[i]])) + 273.15 for i in range(4)]
        turn_temps = [float(decode_u2([data[i]])) + 273.15 for i in range(4, 8)]
        self.temp_pub.publish(
            WheelTemperatures(
                front_right=WheelTemperature(
                    motor=motor_temps[0], swivel=turn_temps[0]
                ),
                back_right=WheelTemperature(motor=motor_temps[1], swivel=turn_temps[1]),
                back_left=WheelTemperature(motor=motor_temps[2], swivel=turn_temps[2]),
                front_left=WheelTemperature(motor=motor_temps[3], swivel=turn_temps[3]),
            )
        )


def main():
    try:
        rclpy.init()
        node = WheelDriver()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
