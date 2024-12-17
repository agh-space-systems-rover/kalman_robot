from serial import SerialException
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
def find_tty_by_model(*id_model_substrings: str) -> str:
    context = pyudev.Context()
    for device in context.list_devices(subsystem="tty"):
        try:
            id_model = device.properties["ID_MODEL"].strip()
            for substr in id_model_substrings:
                if substr in id_model:
                    return device.device_node
        except KeyError:
            pass
    return None


class MasterCom(Node):
    def __init__(self) -> None:
        super().__init__("master_com")

        # Declare params
        self.port = self.declare_parameter("port", "").value
        self.baud_rate = self.declare_parameter("baud_rate", 115200).value

        self.driver = None
        self.reconnect()

        self.sub = self.create_subscription(
            MasterMessage, "master_com/ros_to_master", self.ros_to_master, 10
        )
        self.pubs = {}

        self.timer = self.create_timer(1.0 / 10.0, self.master_to_ros)

    def destroy_node(self) -> None:
        self.timer.destroy()
        for pub in self.pubs.values():
            pub.destroy()
        self.sub.destroy()
        if self.driver:
            self.driver.destroy()

        super().destroy_node()

    def reconnect(self) -> None:
        if self.driver:
            self.driver.destroy()

        while not self.driver:
            if not self.port:
                # Attempt to find the Master UART port indefinitely.
                while True:
                    self.port = find_tty_by_model(
                        "Master_Autonomy_UART", "USB-RS422_Converter", "Kalman_GS_Converter"
                    )
                    if self.port is None:
                        self.get_logger().error(
                            "Master was not found. Will retry in 5 seconds."
                        )
                        time.sleep(5)
                    else:
                        break

            try:
                self.driver = SerialDriver(
                    self,
                    port_name=self.port,
                    start_byte="<",
                    stop_byte=">",
                    baud_rate=self.baud_rate,
                    ascii_mode=False,
                )
                self.get_logger().info(f"Master is connected via {self.port}.")
            except (SerialException, OSError):
                self.get_logger().error(
                    f"Master is not available via {self.port}. Will retry in 5 seconds."
                )
                time.sleep(5)

    def master_to_ros(self) -> None:
        try:
            self.driver.tick()
            msgs: List[SerialMsg] = self.driver.read_all_msgs()
        except (SerialException, OSError):
            self.get_logger().error(
                "Serial port has lost connection. Attempting to reconnect."
            )
            self.reconnect()
            return

        for msg in msgs:
            if not msg.cmd in self.pubs:
                topic_name = "master_com/master_to_ros/" + hex(msg.cmd)[1:]
                self.get_logger().info(f"Publishing frames to {topic_name}")
                self.pubs[msg.cmd] = self.create_publisher(
                    MasterMessage, topic_name, 10
                )

            ros_msg = MasterMessage()

            ros_msg.cmd = msg.cmd
            ros_msg.data = msg.argv

            self.pubs[msg.cmd].publish(ros_msg)

    def ros_to_master(self, ros_msg: MasterMessage) -> None:
        serial_msg = SerialMsg(ros_msg.cmd, len(ros_msg.data), ros_msg.data)

        try:
            self.driver.write_msg(serial_msg)
            self.driver.tick()
        except (SerialException, OSError):
            self.get_logger().error(
                "Serial port has lost connection. Attempting to reconnect."
            )
            self.reconnect()
            return


def main():
    try:
        rclpy.init()
        node = MasterCom()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
