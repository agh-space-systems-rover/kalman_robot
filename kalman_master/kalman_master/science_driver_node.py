import rclpy
from rclpy.node import Node
from kalman_interfaces.srv import Science
from kalman_interfaces.msg import MasterMessage
from kalman_interfaces.msg import ScienceWeight
import struct

NUMBER_OF_RETRIES_PER_CALL = 1
PUBLISH_RATE = 30  # NOTE: only published when there are messages to send
MAX_MESSAGES_IN_QUEUE = 5

CLOSED_FIRST_CONTAINER = 0
OPENED_FIRST_CONTAINER = 90
CLOSED_SECOND_CONTAINER = 0
OPENED_SECOND_CONTAINER = 90


class ScienceDriver(Node):
    def __init__(self):
        super().__init__("science_driver")

        self.master_pub = self.create_publisher(
            MasterMessage, "master_com/ros_to_master", 10
        )
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

        self.create_service(Science, "request_science", self.request_science)
        self.msgs_to_send = []
        self.create_timer(1 / PUBLISH_RATE, self.tick)

    def request_science(self, req: Science.Request, res: Science.Response):
        # todo sanity check
        cmd = 0
        data = []
        msgs = []

        command_map = {
            # science rocks
            Science.Request.SCIENCE_TARE_ROCKS: [
                MasterMessage(cmd=MasterMessage.SCIENCE_TARE, data=[0x00, 0x00]),
                MasterMessage(cmd=MasterMessage.SCIENCE_TARE, data=[0x00, 0x01]),
            ],
            Science.Request.SCIENCE_REQUEST_ROCKS: [
                MasterMessage(
                    cmd=MasterMessage.SCIENCE_REQUEST_WEIGHT, data=[0x00, 0x00]
                ),
                MasterMessage(
                    cmd=MasterMessage.SCIENCE_REQUEST_WEIGHT, data=[0x00, 0x01]
                ),
            ],
            # science sample
            Science.Request.SCIENCE_TARE_SAMPLE: [
                MasterMessage(cmd=MasterMessage.SCIENCE_TARE, data=[0x01, 0x00])
            ],
            Science.Request.SCIENCE_REQUEST_SAMPLE: [
                MasterMessage(
                    cmd=MasterMessage.SCIENCE_REQUEST_WEIGHT, data=[0x01, 0x00]
                )
            ],
            # tare
            Science.Request.SCIENCE_TARE_DRILL: [
                MasterMessage(cmd=MasterMessage.SCIENCE_DRILL, data=[0x00])
            ],
            Science.Request.SCIENCE_REQUEST_DRILL: [
                MasterMessage(cmd=MasterMessage.SCIENCE_DRILL, data=[0x01])
            ],
            # autonomy
            Science.Request.SCIENCE_AUTONOMY_RESET: [
                MasterMessage(cmd=MasterMessage.SCIENCE_DRILL_AUTONOMY, data=[0x00])
            ],
            Science.Request.SCIENCE_AUTONOMY_START: [
                MasterMessage(cmd=MasterMessage.SCIENCE_DRILL_AUTONOMY, data=[0x01])
            ],
            Science.Request.SCIENCE_AUTONOMY_PAUSE: [
                MasterMessage(cmd=MasterMessage.SCIENCE_DRILL_AUTONOMY, data=[0x02])
            ],
            # containers
            Science.Request.CONTAINER1_OPEN: [
                MasterMessage(
                    cmd=MasterMessage.SCIENCE_CONTAINER,
                    data=[0x00, 0x00, OPENED_FIRST_CONTAINER],
                )
            ],
            Science.Request.CONTAINER1_CLOSE: [
                MasterMessage(
                    cmd=MasterMessage.SCIENCE_CONTAINER,
                    data=[0x00, 0x00, CLOSED_FIRST_CONTAINER],
                )
            ],
            Science.Request.CONTAINER2_OPEN: [
                MasterMessage(
                    cmd=MasterMessage.SCIENCE_CONTAINER,
                    data=[0x00, 0x01, OPENED_SECOND_CONTAINER],
                )
            ],
            Science.Request.CONTAINER2_CLOSE: [
                MasterMessage(
                    cmd=MasterMessage.SCIENCE_CONTAINER,
                    data=[0x00, 0x01, CLOSED_SECOND_CONTAINER],
                )
            ],
        }

        msgs.extend(command_map.get(req.cmd, []))
        self.msgs_to_send.extend(msgs * NUMBER_OF_RETRIES_PER_CALL)
        self.msgs_to_send = self.msgs_to_send[-MAX_MESSAGES_IN_QUEUE:]

        return res

    def tick(self):
        if len(self.msgs_to_send) > 0:
            self.master_pub.publish(self.msgs_to_send.pop(0))

    def receive_weights(self, msg):
        if msg.data[0] == 0:  # rock
            last_four_bytes = msg.data[2:6]
            uint32_value = struct.unpack("<I", bytearray(last_four_bytes))[0]
            if msg.data[1] == 0:
                self.first_rock_weight = uint32_value
                self.first_rock_received = True
            elif msg.data[1] == 1:
                self.second_rock_weight = uint32_value
                self.second_rock_received = True
            if self.first_rock_received and self.second_rock_received:
                weight_sum = self.first_rock_weight + self.second_rock_weight
                self.weights_pub.publish(
                    ScienceWeight(which_weight=1, weight=float(weight_sum))
                )
                self.first_rock_received = False
                self.second_rock_received = False

        elif msg.data[0] == 1:  # sample
            last_four_bytes = msg.data[2:6]
            uint32_value = struct.unpack("<I", bytearray(last_four_bytes))[0]
            self.weights_pub.publish(
                ScienceWeight(which_weight=2, weight=float(uint32_value))
            )

    def receive_weight_drill(self, msg):
        float_value = struct.unpack("<f", bytearray(msg.data))[0]
        self.weights_pub.publish(ScienceWeight(which_weight=0, weight=float_value))


def main():
    try:
        rclpy.init()
        node = ScienceDriver()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
