import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

from kalman_interfaces.msg import MasterMessage

UNIQUE_TUNNEL_FORWARD_ID = 73
REBOOT_PC_ID = 46


class TunnelClient(Node):
    def __init__(self):
        super().__init__("tunnel_client")
        self.pub = self.create_publisher(MasterMessage, "master_com/ros_to_master", 10)

        self.create_service(Empty, "tunnel/reboot_pc", self.reboot_pc)

        self.reboot_frames_to_send = 0
        self.create_timer(0.1, self.send_frames)

    def reboot_pc(self, req: Empty.Request, res: Empty.Response):
        self.reboot_frames_to_send = 10
        self.get_logger().info("Received a request to reboot the PC. Sending 10 frames over the course of 1 second...")
        return res

    def send_frames(self):
        if self.reboot_frames_to_send > 0:
            self.reboot_frames_to_send -= 1
            self.pub.publish(
                MasterMessage(
                    cmd=MasterMessage.GS_TO_PC,
                    data=[UNIQUE_TUNNEL_FORWARD_ID, REBOOT_PC_ID],
                )
            )


def main():
    try:
        rclpy.init()
        node = TunnelClient()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
