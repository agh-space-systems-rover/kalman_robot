import rclpy
import struct
import random
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

class MagnetometerMock(Node):
    def __init__(self):
        super().__init__("magnetometer_mock")
        self.req_sub = self.create_subscription(
            UInt8MultiArray, "magneto/request", self.cb_req, 10
        )
        self.res_pub = self.create_publisher(
            UInt8MultiArray, "magneto/data", 10
        )

    def cb_req(self, msg: UInt8MultiArray):
        # Generate random float values for x, y, z
        x = random.uniform(-10.0, 10.0)
        y = random.uniform(-10.0, 10.0)
        z = random.uniform(-10.0, 10.0)
        res_data = list(struct.pack("<fff", x, y, z))
        
        self.get_logger().info(f"Publishing magnetometer data: x={x:.2f}, y={y:.2f}, z={z:.2f}")
        
        res_msg = UInt8MultiArray()
        res_msg.data = res_data
        self.res_pub.publish(res_msg)

def main():
    try:
        rclpy.init()
        node = MagnetometerMock()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
