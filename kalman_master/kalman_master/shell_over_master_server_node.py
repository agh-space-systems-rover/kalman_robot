import rclpy
import os
import select
import pty
import traceback

from rclpy.node import Node
from subprocess import Popen
from kalman_interfaces.msg import MasterMessage

# Multiple modules may use the FORWARD frames, so we need to differentiate them.
# Let 100 be the communication ID for this node.
SHELL_OVER_MASTER_ID = 100
SHELL_CMD = "bash"
SHELL_INPUT_CHUNK_SIZE = 254
SHELL_OUTPUT_CHUNK_SIZE = 24

class ShellOverMasterServer(Node):
    def __init__(self) -> None:
        super().__init__("shell_over_rf_server")

        # written to in master_to_ros_callback; read from in timer_callback -> data forwarded in one big chunk to shell PTY
        self.input_buffer_master, self.input_buffer_slave = pty.openpty()
        # write to shell stdin, read from shell stdout
        self.shell_pty_master, self.shell_pty_slave = pty.openpty()

        self.ros_to_master = self.create_publisher(MasterMessage, "master_com/ros_to_master", 10)
        self.master_to_ros = self.create_subscription(
            MasterMessage, "master_com/master_to_ros/" +
            hex(MasterMessage.FORWARD_TO_PC)[1:],
            self.master_to_ros_callback, 10
        )

        # Use os.setsid() make it run in a new process group, or bash job control will not be enabled.
        self.p = Popen(SHELL_CMD,
            preexec_fn=os.setsid,
            stdin=self.shell_pty_slave,
            stdout=self.shell_pty_slave,
            stderr=self.shell_pty_slave,
            universal_newlines=True
        )
        self.create_timer(0.05, self.timer_callback)
    
    def master_to_ros_callback(self, msg: MasterMessage) -> None:
        if msg.data[0] == SHELL_OVER_MASTER_ID:
            os.write(self.input_buffer_slave, bytes(msg.data[1:]))

    def timer_callback(self) -> None:
        try:
            r, w, e = select.select([self.input_buffer_master, self.shell_pty_master], [], [])

            if self.input_buffer_master in r:
                # If received buffered input from master, write it to stdin.
                d = os.read(self.input_buffer_master, SHELL_INPUT_CHUNK_SIZE)
                os.write(self.shell_pty_master, d)
            if self.shell_pty_master in r:
                # If received from shell, forward to master.
                o = os.read(self.shell_pty_master, SHELL_OUTPUT_CHUNK_SIZE)
                if o:
                    data = [int(x) for x in list(o)]
                    self.ros_to_master.publish(MasterMessage(
                        cmd=MasterMessage.FORWARD_TO_RF,
                        data=[SHELL_OVER_MASTER_ID] + data
                    ))
        except Exception as e:
            # Kill the shell, log and exit.
            try:
                self.p.kill()
            except:
                pass
            raise e


def main():
    try:
        rclpy.init()
        node = ShellOverMasterServer()
        rclpy.spin(node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
