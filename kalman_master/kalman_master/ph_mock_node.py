import rclpy
import struct
import random
from rclpy.node import Node
from kalman_interfaces.msg import MasterMessage

BOARD_ID = 0
CHANNEL_ID = 0

class PHMockNode(Node):
    def __init__(self):
        super().__init__("ph_mock_node")
        self.req_sub = self.create_subscription(
            MasterMessage, "master_com/ros_to_master", self.cb_req, 10
        )
        self.res_pub = self.create_publisher(
            MasterMessage,
            f"master_com/master_to_ros/{hex(MasterMessage.PH_RES)[1:]}",
            10,
        )

    def cb_req(self, msg: MasterMessage):
        if msg.cmd != MasterMessage.PH_REQ:
            return
        # Unpack board/channel id
        if len(msg.data) < 2:
            return
        board_id, channel_id = struct.unpack("BB", msg.data[:2])
        if board_id != BOARD_ID or channel_id != CHANNEL_ID:
            return
        # Generate random value
        ph_value = random.randint(0, 1000)
        res_msg = MasterMessage()
        res_msg.cmd = MasterMessage.PH_RES
        res_msg.data = struct.pack("BBH", board_id, channel_id, ph_value)
        self.res_pub.publish(res_msg)

def main():
    rclpy.init()
    node = PHMockNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
