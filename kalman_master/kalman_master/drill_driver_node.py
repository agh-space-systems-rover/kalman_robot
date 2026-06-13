import rclpy
from rclpy.node import Node
from kalman_interfaces.msg import MasterMessage
from std_msgs.msg import Int8, UInt8

DRILL_B_BRIDGE_SET = 0xD1
DRILL_C_BRIDGE_SET = 0xD2
DRILL_AUTONOMY = 0xD5

MIN_BRIDGE_VALUE = -100
MAX_BRIDGE_VALUE = 100


class DrillDriver(Node):
    def __init__(self):
        super().__init__("drill_driver")

        self.master_pub = self.create_publisher(
            MasterMessage, "master_com/ros_to_master", 10
        )

        self.drill_a_sub = self.create_subscription(
            Int8, "science/drill/b", self.drill_b_cb, 10
        )
        self.drill_b_sub = self.create_subscription(
            Int8, "science/drill/c", self.drill_c_cb, 10
        )
        self.autonomy_sub = self.create_subscription(
            UInt8, "science/drill/autonomy", self.autonomy_cb, 10
        )

    def drill_b_cb(self, msg: Int8):
        value = max(MIN_BRIDGE_VALUE, min(int(msg.data), MAX_BRIDGE_VALUE))
        self.publish_bridge_command(DRILL_B_BRIDGE_SET, value)

    def drill_c_cb(self, msg: Int8):
        value = max(MIN_BRIDGE_VALUE, min(int(msg.data), MAX_BRIDGE_VALUE))
        self.publish_bridge_command(DRILL_C_BRIDGE_SET, value)

    def autonomy_cb(self, msg: UInt8):
        # Autonomy values:
        #   0 = emergency stop
        #   1 = drilling
        #   2 = homing
        value = max(0, min(int(msg.data), 2))
        self.master_pub.publish(
            MasterMessage(
                cmd=MasterMessage.DRILL_DRILL,
                data=[DRILL_AUTONOMY, 1, value],
            )
        )

    def publish_bridge_command(self, command_id: int, value: int):
        direction = 1 if value < 0 else 0
        velocity = abs(value)
        self.master_pub.publish(
            MasterMessage(
                cmd=MasterMessage.DRILL_DRILL,
                data=[command_id, 2, direction, velocity],
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
