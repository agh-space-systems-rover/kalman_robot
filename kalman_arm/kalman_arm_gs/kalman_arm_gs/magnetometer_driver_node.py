import struct
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from std_srvs.srv import Trigger
from sensor_msgs.msg import MagneticField

BOARD_ID = 0
DEVICE_ID = 0



class MagnetometerDriver(Node):
    def __init__(self):
        super().__init__("magnetometer_driver")

        # Publisher for sensor_msgs/MagneticField
        self.magfield_pub = self.create_publisher(MagneticField, "science/magnetic_field/value", 10)

        # Service for requesting magnetometer data
        self.data_req_srv = self.create_service(
            Trigger,
            "science/magnetic_field/value/req",
            self.cb_data_req,
        )

        # Master comms (similarly to PHDriver, but using UInt8MultiArray for demo)
        self.master_pub = self.create_publisher(
            UInt8MultiArray, "magneto/request", 10
        )
        self.master_sub = self.create_subscription(
            UInt8MultiArray,
            "magneto/data",
            self.cb_master_res,
            10,
        )

    def cb_data_req(self, request, response):
        # Create the request message
        req_msg = UInt8MultiArray()
        req_msg.data = [BOARD_ID, DEVICE_ID]
        self.master_pub.publish(req_msg)
        response.success = True
        response.message = "magnetometer reading requested"
        return response

    def cb_master_res(self, msg: UInt8MultiArray):
        if len(msg.data) < 12:
            self.get_logger().warn("Received invalid magnetometer response")
            return
        
        # Unpack the response
        x, y, z = struct.unpack("<fff", bytearray(msg.data[:12]))

        # Publish sensor_msgs/MagneticField
        mag_msg = MagneticField()
        mag_msg.header.stamp = self.get_clock().now().to_msg()
        mag_msg.magnetic_field.x = x
        mag_msg.magnetic_field.y = y
        mag_msg.magnetic_field.z = z
        self.magfield_pub.publish(mag_msg)

def main():
    try:
        rclpy.init()
        node = MagnetometerDriver()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
