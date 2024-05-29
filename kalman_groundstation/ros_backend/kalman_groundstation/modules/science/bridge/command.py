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

        self.msg_to_send = 0

    def handler_drill(self, message: DrillCommand):
        drill_frame = MasterMessage(
            cmd = 
                0x47,
            data=[
                1 if message.drill > 0 else 0,
                int(abs(message.drill * 100)),
            ]
        )
        height_frame = MasterMessage(
            cmd = 
                0x45,
            data=[
                1 if message.height > 0 else 0,
                int(abs(message.height * 100)),
            ]
        )

        height2_frame = MasterMessage(
            cmd = 
                0x46,
            data=[
                1 if message.height2 > 0 else 0,
                int(abs(message.height2 * 100)),
            ]
        )
        target = self.msg_to_send

        self.msg_to_send += 1
        if self.msg_to_send > 3:
            self.msg_to_send = 0

        if( target == 0 ):
            self.ros2uart_pub.publish(drill_frame)
        elif (target == 1 ):
            self.ros2uart_pub.publish(height_frame)
        elif (target == 2 ):
            self.ros2uart_pub.publish(height2_frame)
