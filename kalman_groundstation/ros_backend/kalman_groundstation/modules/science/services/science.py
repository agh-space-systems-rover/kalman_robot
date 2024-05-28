# from std_msgs.msg import UInt8MultiArray
# from kalman_groundstation.msg import ScienceState as ScienceStateMsg
from kalman_interfaces.msg import MasterMessage, ScienceState as ScienceStateMsg

from ..model.science import ScienceState
from struct import unpack
from rclpy.node import Node


class ScienceService:
    def __init__(self, parent_node: Node):
        # self.uart2ros_sub = parent_node.create_subscription(
        #     MasterMessage, "/master_com/master_to_ros/xc5", self.update_temperature, qos_profile=10
        # )

        self.state_publisher = parent_node.create_publisher(
            ScienceStateMsg, "/station/science/state", qos_profile=10
        )

        self.state = ScienceState().dict()

    def update_temperature(self, msg: MasterMessage):
        slot = f"temp_slot_{msg.data[2] + 1}"

        temp = unpack("f", msg.data[3:])[0]

        self.state[slot] = 1.2 * temp - 4

        self.broadcast()

    def broadcast(self):
        msg = ScienceState(**self.state).to_ros_msg()
        self.state_publisher.publish(msg)
