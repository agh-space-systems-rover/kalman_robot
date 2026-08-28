import struct

import rclpy
from kalman_interfaces.msg import DrillTelemetry, MasterMessage
from rclpy.node import Node
from std_msgs.msg import Float32, Int8, UInt8, UInt16MultiArray


class DrillDriver(Node):
    def __init__(self):
        super().__init__("drill_driver")

        self.master_pub = self.create_publisher(
            MasterMessage, "master_com/ros_to_master", 10
        )
        self.depth_sub = self.create_subscription(
            MasterMessage,
            f"master_com/master_to_ros/{hex(MasterMessage.DRILL_TELEMETRY)[1:]}",
            self.depth_response_cb,
            10,
        )
        self.weight_response_sub = self.create_subscription(
            MasterMessage,
            f"master_com/master_to_ros/{hex(MasterMessage.DRILL_WEIGHT_RESPONSE)[1:]}",
            self.weight_response_cb,
            10,
        )

        self.telemetry_pub = self.create_publisher(
            DrillTelemetry, "science/drill/telemetry", 10
        )
        self.weight_pub = self.create_publisher(
            Float32, "science/drill/weight", 10
        )

        self.rack_sub = self.create_subscription(
            Int8, "science/drill/rack", self.rack_cb, 10
        )
        self.drill_sub = self.create_subscription(
            Int8, "science/drill/drill", self.drill_cb, 10
        )
        self.servo_sub = self.create_subscription(
            UInt16MultiArray, "science/drill/servo", self.servo_cb, 10
        )
        self.weight_cmd_sub = self.create_subscription(
            UInt8, "science/drill/weight/cmd", self.weight_cmd_cb, 10
        )
        self.autonomy_sub = self.create_subscription(
            UInt8, "science/drill/autonomy", self.autonomy_cb, 10
        )

    def publish(self, command: int, data: list[int]):
        self.master_pub.publish(MasterMessage(cmd=command, data=data))

    def rack_cb(self, msg: Int8):
        value = max(-20, min(int(msg.data), 20))
        self.publish(
            MasterMessage.DRILL_RACK_SET,
            [value + (1 << 8) if value < 0 else value],
        )

    def drill_cb(self, msg: Int8):
        value = max(-100, min(int(msg.data), 100))
        self.publish(
            MasterMessage.DRILL_SET,
            [value + (1 << 8) if value < 0 else value],
        )

    def servo_cb(self, msg: UInt16MultiArray):
        if len(msg.data) != 2:
            return

        channel, angle = msg.data
        self.publish(
            MasterMessage.DRILL_SERVO_SET,
            [channel, (angle >> 8) & 0xFF, angle & 0xFF],
        )

    def weight_cmd_cb(self, msg: UInt8):
        self.publish(MasterMessage.DRILL_WEIGHT_CMD, [msg.data])

    def autonomy_cb(self, msg: UInt8):
        self.publish(MasterMessage.DRILL_AUTONOMY, [msg.data])

    def depth_response_cb(self, msg: MasterMessage):
        if len(msg.data) < 2:
            return

        depth_raw = struct.unpack(">h", bytes(msg.data[:2]))[0]
        self.telemetry_pub.publish(DrillTelemetry(depth_mm=depth_raw / 10))

    def weight_response_cb(self, msg: MasterMessage):
        if len(msg.data) != 2:
            return

        weight_raw = struct.unpack(">h", bytes(msg.data))[0]
        self.weight_pub.publish(Float32(data=weight_raw / 10))


def main():
    try:
        rclpy.init()
        node = DrillDriver()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
