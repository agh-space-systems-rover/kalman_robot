import struct
import rclpy
from rclpy.node import Node
from kalman_interfaces.msg import DrillTelemetry, MasterMessage
from std_msgs.msg import Int8, UInt8

MIN_BRIDGE_VALUE = -100
MAX_BRIDGE_VALUE = 100
MIN_STATE_VALUE = 0
MAX_STATE_VALUE = 9
DEPTH_SCALE = 10.0
CURRENT_MA_PER_LSB = 20.0
MA_PER_A = 1000.0


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
        self.state_sub = self.create_subscription(
            UInt8, "science/drill/state", self.state_cb, 10
        )

    def drill_b_cb(self, msg: Int8):
        value = max(MIN_BRIDGE_VALUE, min(int(msg.data), MAX_BRIDGE_VALUE))
        self.publish_bridge_command(MasterMessage.DRILL_B_BRIDGE_SET, value)

    def drill_c_cb(self, msg: Int8):
        value = max(MIN_BRIDGE_VALUE, min(int(msg.data), MAX_BRIDGE_VALUE))
        self.publish_bridge_command(MasterMessage.DRILL_C_BRIDGE_SET, value)

    def state_cb(self, msg: UInt8):
        # Drill state values:
        #   0 = stop
        #   1 = drill site 1
        #   2 = drill site 2
        #   3 = home
        #   4 = close tubes at site 1
        #   5 = close tubes at site 2
        #   6 = clean drill
        #   7 = open tubes at site 1
        #   8 = open tubes at site 2
        #   9 = open tubes at both sites
        value = int(msg.data)
        if not MIN_STATE_VALUE <= value <= MAX_STATE_VALUE:
            self.get_logger().warn(f"Received invalid drill state: {value}")
            return

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

        depth_raw = struct.unpack(">h", bytes(payload[0:2]))[0]
        rack_current_raw = payload[2]
        drill_current_raw = payload[3]
        flags = payload[4]
        autonomy_state = payload[5]

        self.telemetry_pub.publish(
            DrillTelemetry(
                depth_mm=depth_raw / DEPTH_SCALE,
                rack_current=rack_current_raw * CURRENT_MA_PER_LSB / MA_PER_A,
                drill_current=drill_current_raw * CURRENT_MA_PER_LSB / MA_PER_A,
                flags=flags,
                upper_limit_pressed=bool(flags & 0x01),
                lower_limit_pressed=bool(flags & 0x02),
                autonomy_active=bool(flags & 0x04),
                based=bool(flags & 0x08),
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
