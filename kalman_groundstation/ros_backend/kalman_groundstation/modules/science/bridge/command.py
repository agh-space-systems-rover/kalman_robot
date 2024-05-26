from kalman_interfaces.msg import DrillCommand, MasterMessage
from std_msgs.msg import UInt8MultiArray
from rclpy.node import Node
import rclpy


class CommandBridge:
    def __init__(self, parent_node: Node):
        self.parent_node = parent_node
        self.sub_drill = parent_node.create_subscription(
            DrillCommand, "/station/science/drill", self.handler_drill, qos_profile=10
        )

        self.ros2uart_pub = parent_node.create_publisher(
            MasterMessage, "/master_com/ros_to_master", qos_profile=10
        )

    def handler_drill(self, message: DrillCommand):
        drill_frame = UInt8MultiArray(
            data=[
                0x45,
                0x02,
                1 if message.drill > 0 else 0,
                int(abs(message.drill * 100)),
            ]
        )
        height_frame = UInt8MultiArray(
            data=[
                0x46,
                0x02,
                1 if message.height > 0 else 0,
                int(abs(message.height * 100)),
            ]
        )

        self.ros2uart_pub.publish(drill_frame)
        # rospy.sleep(0.05)
        delay = rclpy.duration.Duration(seconds=0, nanoseconds=50_000_000)
        self.parent_node.get_clock().sleep_for(delay)

        self.ros2uart_pub.publish(height_frame)
