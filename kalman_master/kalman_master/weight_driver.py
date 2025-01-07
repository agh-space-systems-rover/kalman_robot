import rclpy
from rclpy.node import Node
from kalman_interfaces.srv import Science
from kalman_interfaces.msg import MasterMessage
from kalman_interfaces.msg import ScienceWeight
import struct

NUMBER_OF_RETRIES_PER_CALL = 1
PUBLISH_RATE = 30  # NOTE: only published when there are messages to send
MAX_MESSAGES_IN_QUEUE = 5

class WeightDriver(Node):
    def __init__(self):
        super().__init__("weight_driver")


        self.weights_pub = self.create_publisher(ScienceWeight, "science/weights", 10)

        self.weights_sub = self.create_subscription(  # rock and sample
            MasterMessage, "master_com/master_to_ros/x5b", self.receive_weights, 10
        )

        self.weight_drill_sub = self.create_subscription(  # drill
            MasterMessage, "master_com/master_to_ros/xd4", self.receive_weight_drill, 10
        )

        self.first_rock_weight = 0
        self.second_rock_weight = 0
        self.first_rock_received = False
        self.second_rock_received = False



    def receive_weights(self, msg):
        if msg.data[0] == 0:  # rock
            last_four_bytes = msg.data[-4:]
            int32_value = struct.unpack("<i", bytearray(last_four_bytes))[0]
            if msg.data[1] == 0:
                self.first_rock_weight = int32_value
                self.first_rock_received = True
            elif msg.data[1] == 1:
                self.second_rock_weight = int32_value
                self.second_rock_received = True
            if self.first_rock_received and self.second_rock_received:
                weight_sum = self.first_rock_weight + self.second_rock_weight
                self.weights_pub.publish(
                    ScienceWeight(which_weight=1, weight=float(weight_sum))
                )
                self.first_rock_received = False
                self.second_rock_received = False

        elif msg.data[0] == 1:  # sample
            last_four_bytes = msg.data[-4:]
            int32_value = struct.unpack("<i", bytearray(last_four_bytes))[0]
            self.weights_pub.publish(
                ScienceWeight(which_weight=2, weight=float(int32_value))
            )

    def receive_weight_drill(self, msg):
        float_value = struct.unpack("<f", bytearray(msg.data))[0]
        self.weights_pub.publish(ScienceWeight(which_weight=0, weight=float_value))


def main():
    try:
        rclpy.init()
        node = WeightDriver()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass