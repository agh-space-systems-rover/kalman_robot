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
        port = self.declare_parameter("port", "").value
        rf_baud = self.declare_parameter("rf_baud", False).value

        if not port:
            # Attempt to find the Master UART port indefinitely.
            while True:
                port = find_tty_by_model("Master_Autonomy_UART", "USB-RS422_Converter")
                if port is None:
                    self.get_logger().error(
                        "Master UART port not found. Will retry in 5 seconds."
                    )
                    time.sleep(5)
                else:
                    self.get_logger().info(f"Master is connected to {port}.")
                    break

        self.driver = SerialDriver(
            self,
            port_name=port,
            start_byte="<",
            stop_byte=">",
            baud_rate=(38400 if rf_baud else 115200),
            ascii_mode=False,
        )

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
        self.driver.destroy()

        super().destroy_node()

    def master_to_ros(self) -> None:
        self.driver.tick()
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


def main():
    while True:
        try:
            rclpy.init()
            node = MasterCom()
            rclpy.spin(node)
            node.destroy_node()
            rclpy.shutdown()
        except KeyboardInterrupt:
            exit()
        except (SerialException, OSError):
            node.get_logger().error(
                "Serial port has lost connection. Restarting the node..."
            )
            node.destroy_node()
            rclpy.shutdown()
            time.sleep(1)
