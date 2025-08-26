import rclpy
import struct
import random
from rclpy.node import Node
from kalman_interfaces.msg import MasterMessage

SAND_BOARD_ID = 0
SAND_CHANNEL_ID = 0
ROCK_BOARD_ID = 0
ROCK_CHANNEL_ID = 1

class StorageMockNode(Node):
    def __init__(self):
        super().__init__("storage_mock_node")
        self.req_sub = self.create_subscription(
            MasterMessage, "master_com/ros_to_master", self.cb_req, 10
        )
        self.scale_res_pub = self.create_publisher(
            MasterMessage,
            f"master_com/master_to_ros/{hex(MasterMessage.SCALE_RES)[1:]}",
            10,
        )

    def cb_req(self, msg: MasterMessage):
        if msg.cmd == MasterMessage.SCALE_REQ:
            self.handle_scale_request(msg)
        elif msg.cmd == MasterMessage.SERVO_SET:
            self.handle_servo_request(msg)

    def handle_scale_request(self, msg: MasterMessage):
        if len(msg.data) < 2:
            return
        board_id, channel_id = struct.unpack("BB", msg.data[:2])
        
        # Check if it's sand or rock container
        if (board_id == SAND_BOARD_ID and channel_id == SAND_CHANNEL_ID) or \
           (board_id == ROCK_BOARD_ID and channel_id == ROCK_CHANNEL_ID):
            container_name = "sand" if channel_id == SAND_CHANNEL_ID else "rock"
            # Generate random weight between 0 and 10000 (representing 0.000-10.000 kg in raw units)
            weight_value = random.randint(0, 10000)
            
            self.get_logger().info(f"Sending scale reading for {container_name} container: {weight_value} raw units")
            
            res_msg = MasterMessage()
            res_msg.cmd = MasterMessage.SCALE_RES
            res_msg.data = struct.pack("BBi", board_id, channel_id, weight_value)
            self.scale_res_pub.publish(res_msg)

    def handle_servo_request(self, msg: MasterMessage):
        if len(msg.data) < 3:
            return
        board_id, channel_id, angle = struct.unpack("BBB", msg.data[:3])
        
        if (board_id == SAND_BOARD_ID and channel_id == SAND_CHANNEL_ID) or \
           (board_id == ROCK_BOARD_ID and channel_id == ROCK_CHANNEL_ID):
            container_name = "sand" if channel_id == SAND_CHANNEL_ID else "rock"
            self.get_logger().info(f"SERVO event received for {container_name} container (angle={angle})")

def main():
    rclpy.init()
    node = StorageMockNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
