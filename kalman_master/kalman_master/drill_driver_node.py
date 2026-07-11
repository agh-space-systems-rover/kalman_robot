import struct
import rclpy
from rclpy.node import Node
from kalman_interfaces.msg import DrillTelemetry, MasterMessage
from std_msgs.msg import Float32, Int8, UInt8

MIN_BRIDGE_VALUE = -100
MAX_BRIDGE_VALUE = 100
UNIVERSAL_ID = 0
DRILL_UNIVERSAL_ID = 0
UNIVERSAL_CAROUSEL_CHANNEL = 0
UNIVERSAL_CHANNEL_MIN = 0
UNIVERSAL_CAROUSEL_MAX = 100
UNIVERSAL_PROBE_MAX = 180


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
        self.weight_master_sub = self.create_subscription(
            MasterMessage,
            f"master_com/master_to_ros/{hex(MasterMessage.TO_UNIVERSAL)[1:]}",
            self.weight_master_res_cb,
            10,
        )

        self.telemetry_pub = self.create_publisher(
            DrillTelemetry, "science/drill/telemetry", 10
        )
        self.weight_pub = self.create_publisher(Float32, "science/drill/weight", 10)

        self.drill_b_sub = self.create_subscription(
            Int8, "science/drill/b", self.drill_b_cb, 10
        )
        self.drill_c_sub = self.create_subscription(
            Int8, "science/drill/c", self.drill_c_cb, 10
        )
        self.autonomy_sub = self.create_subscription(
            UInt8, "science/drill/autonomy", self.autonomy_cb, 10
        )
        self.gear_sub = self.create_subscription(
            UInt8, "science/drill/gear", self.gear_cb, 10
        )
        self.weight_req_sub = self.create_subscription(
            UInt8, "science/drill/weight/request", self.weight_req_cb, 10
        )
        self.universal_sub = self.create_subscription(
            Float32,
            "science/drill/universal",
            lambda msg: self.universal_channel_cb(msg, 0),
            10,
        )
        self.channel1_sub = self.create_subscription(
            Float32,
            "science/drill/channel1",
            lambda msg: self.universal_channel_cb(msg, 1),
            10,
        )
        self.channel2_sub = self.create_subscription(
            Float32,
            "science/drill/channel2",
            lambda msg: self.universal_channel_cb(msg, 2),
            10,
        )
        self.channel3_sub = self.create_subscription(
            Float32,
            "science/drill/channel3",
            lambda msg: self.universal_channel_cb(msg, 3),
            10,
        )
        self.channel4_sub = self.create_subscription(
            Float32,
            "science/drill/channel4",
            lambda msg: self.universal_channel_cb(msg, 4),
            10,
        )

    def drill_b_cb(self, msg: Int8):
        value = max(MIN_BRIDGE_VALUE, min(int(msg.data), MAX_BRIDGE_VALUE))
        self.publish_bridge_command(MasterMessage.DRILL_B_BRIDGE_SET, value)

    def drill_c_cb(self, msg: Int8):
        value = max(MIN_BRIDGE_VALUE, min(int(msg.data), MAX_BRIDGE_VALUE))
        self.publish_bridge_command(MasterMessage.DRILL_C_BRIDGE_SET, value)

    def autonomy_cb(self, msg: UInt8):
        # Autonomy values:
        #   0 = emergency stop
        #   1 = drilling
        #   2 = homing
        #   3 = open sarko
        #   4 = close sarko
        #   5 = clean drill
        value = max(0, min(int(msg.data), 5))
        self.master_pub.publish(
            MasterMessage(
                cmd=MasterMessage.DRILL_AUTONOMY,
                data=[value],
            )
        )

    def gear_cb(self, msg: UInt8):
        value = max(1, min(int(msg.data), 2))
        self.master_pub.publish(
            MasterMessage(
                cmd=MasterMessage.DRILL_SET_DRILL_GEAR,
                data=[value],
            )
        )

    def weight_req_cb(self, msg: UInt8):
        # Weight request values:
        #   0 = tare
        #   1 = request measurement
        value = max(0, min(int(msg.data), 1))
        self.master_pub.publish(
            MasterMessage(
                cmd=MasterMessage.TO_UNIVERSAL,
                data=[UNIVERSAL_ID, value],
            )
        )

    def universal_channel_cb(self, msg: Float32, channel: int):
        max_value = (
            UNIVERSAL_CAROUSEL_MAX
            if channel == UNIVERSAL_CAROUSEL_CHANNEL
            else UNIVERSAL_PROBE_MAX
        )
        value = max(UNIVERSAL_CHANNEL_MIN, min(round(float(msg.data)), max_value))
        self.master_pub.publish(
            MasterMessage(
                cmd=MasterMessage.SERVO_SET,
                data=[DRILL_UNIVERSAL_ID, channel, value],
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
                based=bool(flags & 0x08),
                autonomy_state=autonomy_state,
            )
        )

    def get_weight_payload(self, msg: MasterMessage):
        payload = list(msg.data)

        if len(payload) == 5:
            if payload[0] != UNIVERSAL_ID:
                return None
            payload = payload[1:]
        elif len(payload) == 6:
            if payload[0] != UNIVERSAL_ID or payload[1] != MasterMessage.SCALE_RES:
                return None
            payload = payload[2:]

        if len(payload) != 4:
            return None

        return payload

    def weight_master_res_cb(self, msg: MasterMessage):
        payload = self.get_weight_payload(msg)
        if payload is None:
            return

        weight = struct.unpack("<f", bytes(payload))[0]
        self.weight_pub.publish(Float32(data=float(weight)))


def main():
    try:
        rclpy.init()
        node = DrillDriver()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
