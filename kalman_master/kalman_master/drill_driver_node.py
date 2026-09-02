import struct

import rclpy
from kalman_interfaces.msg import DrillTelemetry, MasterMessage
from rclpy.node import Node
from std_msgs.msg import Float32, Int8, UInt8, UInt16MultiArray

DEPTH_SCALE = 10.0
VELOCITY_SCALE = 100.0
WEIGHT_SCALE = 10.0
CURRENT_SCALE = 1000.0


class DrillDriver(Node):
    def __init__(self):
        super().__init__("drill_driver")

        self.master_pub = self.create_publisher(
            MasterMessage, "master_com/ros_to_master", 10
        )
        self.telemetry_sub = self.create_subscription(
            MasterMessage,
            f"master_com/master_to_ros/{hex(MasterMessage.DRILL_TELEMETRY)[1:]}",
            self.telemetry_cb,
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

    def telemetry_cb(self, msg: MasterMessage):
        if len(msg.data) < 12:
            self.get_logger().warn(
                f"Received drill telemetry of invalid length: {len(msg.data)}"
            )
            return

        depth_raw, velocity_raw = struct.unpack(">hh", bytes(msg.data[0:4]))
        flags = msg.data[4]
        # Byte 5 is reserved.
        weight_raw = struct.unpack(">h", bytes(msg.data[6:8]))[0]
        rack_current_raw, drill_current_raw = struct.unpack(
            ">hh", bytes(msg.data[8:12])
        )

        weight_g = weight_raw / WEIGHT_SCALE
        self.telemetry_pub.publish(
            DrillTelemetry(
                depth_mm=depth_raw / DEPTH_SCALE,
                rack_velocity_mmps=velocity_raw / VELOCITY_SCALE,
                weight_g=weight_g,
                rack_current_a=rack_current_raw / CURRENT_SCALE,
                drill_current_a=drill_current_raw / CURRENT_SCALE,
                flags=flags,
                upper_limit_pressed=bool(flags & 0x01),
                autonomy_active=bool(flags & 0x02),
                based=bool(flags & 0x04),
                autonomy_state=(flags >> 3) & 0x0F,
            )
        )
        self.weight_pub.publish(Float32(data=weight_g))

    def weight_response_cb(self, msg: MasterMessage):
        if len(msg.data) != 2:
            return

        weight_raw = struct.unpack(">h", bytes(msg.data))[0]
        self.weight_pub.publish(Float32(data=weight_raw / WEIGHT_SCALE))


def main():
    try:
        rclpy.init()
        node = DrillDriver()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
