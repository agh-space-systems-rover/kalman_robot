import rclpy
import os
import select
import sys
import tty
import termios
import traceback

from rclpy.node import Node
from kalman_interfaces.msg import MasterMessage

SHELL_OVER_MASTER_ID = 100
SHELL_INPUT_CHUNK_SIZE = 254
ESCAPE_SEQUENCE = "qwerty"

class ShellOverMasterClient(Node):
    def __init__(self) -> None:
        super().__init__("shell_over_rf_client")

        self.escape_code_window = "x" * len(ESCAPE_SEQUENCE)

        # Remember the old tty settings and set raw mode.
        self.old_tty = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())

        self.ros_to_master = self.create_publisher(MasterMessage, "master_com/ros_to_master", 10)
        self.master_to_ros = self.create_subscription(
            MasterMessage, "master_com/master_to_ros/" +
            hex(MasterMessage.FORWARD_TO_RF)[1:],
            self.master_to_ros_callback, 10
        )

        self.create_timer(0.5, self.timer_callback)

    def master_to_ros_callback(self, msg: MasterMessage) -> None:
        if msg.data[0] == SHELL_OVER_MASTER_ID:
            os.write(sys.stdout.fileno(), bytes(msg.data[1:]))

    def timer_callback(self) -> None:
        try:
            r, w, e = select.select([sys.stdin], [], [])

            if sys.stdin in r:
                # If received input from stdin, forward it to master.
                i = os.read(sys.stdin.fileno(), SHELL_INPUT_CHUNK_SIZE)
                
                # Check if the escape code was entered and raise KeyboardInterrupt if so.
                self.escape_code_window += i.decode()
                if ESCAPE_SEQUENCE in self.escape_code_window:
                    raise KeyboardInterrupt
                self.escape_code_window = self.escape_code_window[-len(ESCAPE_SEQUENCE):]
                
                data = [int(x) for x in list(i)]
                self.ros_to_master.publish(MasterMessage(
                    cmd=MasterMessage.FORWARD_TO_PC,
                    data=[SHELL_OVER_MASTER_ID] + data
                ))
        except Exception as e:
            # Restore tty settings back.
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_tty)
            raise e


def main():
    try:
        rclpy.init()
        node = ShellOverMasterClient()
        rclpy.spin(node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
