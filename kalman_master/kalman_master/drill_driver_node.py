import struct
import rclpy
from rclpy.node import Node
from kalman_interfaces.msg import DrillTelemetry, MasterMessage
from std_msgs.msg import Int8, UInt8

MIN_BRIDGE_VALUE = -100
MAX_BRIDGE_VALUE = 100


class DrillDriver(Node):
    def __init__(self):
        super().__init__("drill_driver")

        self.master_pub = self.create_publisher(
            MasterMessage, "master_com/ros_to_master", 10
        )
        self.master_sub = self.create_subscription(
            MasterMessage,
            f"master_com/master_to_ros/{hex(MasterMessage.DRILL_TELEMETRY)[1:]}",
            self.master_res_cb,
            10,
        )

        self.telemetry_pub = self.create_publisher(
            DrillTelemetry, "science/drill/telemetry", 10
        )

        self.drill_b_sub = self.create_subscription(
            Int8, "science/drill/b", self.drill_b_cb, 10
        )
        self.drill_c_sub = self.create_subscription(
            Int8, "science/drill/c", self.drill_c_cb, 10
        )
        self.autonomy_sub = self.create_subscription(
            UInt8, "science/drill/autonomy", self.autonomy_cb, 10
        )

    def drill_b_cb(self, msg: Int8):
        value = max(MIN_BRIDGE_VALUE, min(int(msg.data), MAX_BRIDGE_VALUE))
        self.publish_bridge_command(MasterMessage.DRILL_RACK, value)

    def drill_c_cb(self, msg: Int8):
        value = max(MIN_BRIDGE_VALUE, min(int(msg.data), MAX_BRIDGE_VALUE))
        self.publish_bridge_command(MasterMessage.DRILL_DRILL, value)

    def autonomy_cb(self, msg: UInt8):
        # Autonomy values:
        #   0 = emergency stop
        #   1 = drilling
        #   2 = homing
        value = max(0, min(int(msg.data), 2))
        self.master_pub.publish(
            MasterMessage(
                cmd=MasterMessage.DRILL_AUTONOMY,
                data=[value],
            )
        )

    def publish_bridge_command(self, cmd: int, value: int):
        direction = 1 if value < 0 else 0
        velocity = abs(value)
        self.master_pub.publish(
            MasterMessage(
                cmd=cmd,
                data=[direction, velocity],
            )
        )

    def master_res_cb(self, msg: MasterMessage):
        if len(msg.data) != 8:
            self.get_logger().warn("Received invalid drill telemetry length")
            return

        payload = msg.data

        depth_raw = struct.unpack("<h", bytes(payload[0:2]))[0]
        rack_current_raw = payload[2]
        drill_current_raw = payload[3]
        flags = payload[4]
        autonomy_state = payload[5]

        self.telemetry_pub.publish(
            DrillTelemetry(
                depth_mm=depth_raw / 10.0,
                rack_current=rack_current_raw / 20.0,
                drill_current=drill_current_raw / 20.0,
                flags=flags,
                upper_limit_pressed=bool(flags & 0x01),
                lower_limit_pressed=bool(flags & 0x02),
                autonomy_active=bool(flags & 0x04),
                homed=bool(flags & 0x08),
                autonomy_state=autonomy_state,
            )
        )


def main():
    try:
        rclpy.init()
        node = DrillDriver()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
