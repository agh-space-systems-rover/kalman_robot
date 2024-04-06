import rclpy
import pyudev
import time
from rclpy.node import Node
from kalman_interfaces.msg import MasterMessage
from kalman_master.serial_driver import SerialDriver, SerialMsg
from typing import List

"""
The idea of this node is to provide the ROS <--> Master bridge.

Every frame received from master is published as a ROS message (MasterMessage) on a dynamically 
created topic master_com/master_to_ros/{x(lowercase hexadecimal frame ID)} [msg command, argv_0, ... , argv_n].
Frame data interpretation should be handled by client.

Every ROS message sent on topic master_com/ros_to_master should have MasterMessage format.
Message is then encoded as a binary frame and sent out using the serial driver.
"""

# Returns first ttyXXXN that includes model string in its ID_MODEL property
def find_tty_by_model(model: str):
    context = pyudev.Context()
    for device in context.list_devices(subsystem='tty'):
        try:
            id_model = device.properties['ID_MODEL'].strip()
            if model in id_model:
                return device.device_node
        except KeyError:
            pass
    return None

class MasterCom(Node):
    def __init__(self) -> None:
        super().__init__("master_com")

        # Attempt to find the Master UART port indefinitely.
        while True:
            port_name = find_tty_by_model("Master_Autonomy_UART")
            if port_name is None:
                self.get_logger().error("Master UART port not found. Will retry in 5 seconds.")
                time.sleep(5)
            else:
                self.get_logger().info(f"Master is connected to {port_name}.")
                break

        self.driver = SerialDriver(
            self,
            port_name=port_name,
            start_byte="<",
            stop_byte=">",
            baud_rate=115200,
            ascii_mode=False,
        )

        self.create_subscription(
            MasterMessage, "master_com/ros_to_master", self.ros_to_master, 10
        )
        self.pubs = {}

        self.create_timer(1.0 / 400.0, self.master_to_ros)

    def master_to_ros(self) -> None:
        msgs: List[SerialMsg] = self.driver.read_all_msgs()
        for msg in msgs:
            if not msg.cmd in self.pubs:
                self.pubs[msg.cmd] = self.create_publisher(
                    MasterMessage, "master_com/master_to_ros/" + hex(msg.cmd)[1:], 10
                )

            ros_msg = MasterMessage()

            ros_msg.cmd = msg.cmd
            ros_msg.data = msg.argv

            self.pubs[msg.cmd].publish(ros_msg)

    def ros_to_master(self, ros_msg: MasterMessage) -> None:
        serial_msg = SerialMsg(ros_msg.cmd, len(ros_msg.data), ros_msg.data)

        self.driver.write_msg(serial_msg)
        self.driver.tick()


def main(args=None):
    rclpy.init(args=args)

    node = MasterCom()
    rclpy.spin(node)

    rclpy.shutdown()
