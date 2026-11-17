import rclpy
import random
import time
from rclpy.node import Node
from kalman_interfaces.msg import MasterMessage


class MasterLoopback(Node):
    def __init__(self):
        super().__init__("master_loopback")

        self.loss_rate = self.declare_parameter("loss_rate", 0.5).value
        self.min_delay = self.declare_parameter("min_delay", 0.0).value
        self.max_delay = self.declare_parameter("max_delay", 0.1).value

        self.sub = self.create_subscription(
            MasterMessage, "master_com/ros_to_master", self.ros_to_master, 10
        )
        self.pubs = {}

    def ros_to_master(self, msg: MasterMessage) -> None:
        if not msg.cmd in self.pubs:
            self.pubs[msg.cmd] = self.create_publisher(
                MasterMessage, "master_com/master_to_ros/" + hex(msg.cmd)[1:], 10
            )

        time.sleep(random.uniform(self.min_delay, self.max_delay))

        if random.random() > self.loss_rate:
            self.pubs[msg.cmd].publish(msg)


def main():
    try:
        rclpy.init()
        node = MasterLoopback()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
