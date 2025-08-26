import rclpy
import struct
import random
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

BOARD_ID = 0
CHANNEL_ID = 0

class MagnetometerMockNode(Node):
    def __init__(self):
        super().__init__("magnetometer_mock_node")
        self.req_sub = self.create_subscription(
            UInt8MultiArray, "magneto/request", self.cb_req, 10
        )
        self.res_pub = self.create_publisher(
            UInt8MultiArray, "magneto/data", 10
        )

    def cb_req(self, msg: UInt8MultiArray):
        if len(msg.data) < 2:
            return
        board_id, channel_id = msg.data[:2]
        if board_id != BOARD_ID or channel_id != CHANNEL_ID:
            return
        # Generate random float values for x, y, z
        x = random.uniform(-10.0, 10.0)
        y = random.uniform(-10.0, 10.0)
        z = random.uniform(-10.0, 10.0)
        res_data = list(struct.pack("<fff", x, y, z))
        # Optionally, append board/channel id for protocol completeness
        # res_data += [board_id, channel_id]
        res_msg = UInt8MultiArray()
        res_msg.data = res_data
        self.res_pub.publish(res_msg)

def main():
    try:
        rclpy.init()
        node = MagnetometerMockNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
