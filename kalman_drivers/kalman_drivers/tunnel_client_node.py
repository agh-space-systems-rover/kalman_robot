import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

from kalman_interfaces.msg import MasterMessage

UNIQUE_TUNNEL_FORWARD_ID = 73
REBOOT_PC_ID = 46
ARCH_LAUNCH_STACK_ID = 47
ARCH_SEND_GOAL_ID = 48


class TunnelClient(Node):
    def __init__(self):
        super().__init__("tunnel_client")
        self.pub = self.create_publisher(MasterMessage, "master_com/ros_to_master", 10)

        self.create_service(Empty, "tunnel/reboot_pc", self.reboot_pc)
        self.create_service(Empty, "tunnel/arch_launch_stack", self.arch_launch_stack)
        self.create_service(Empty, "tunnel/arch_send_goal", self.arch_send_goal)

        self.frames_to_send = []
        self.create_timer(0.1, self.send_frames)

    def reboot_pc(self, req: Empty.Request, res: Empty.Response):
        self.frames_to_send += [REBOOT_PC_ID for _ in range(10)]
        self.get_logger().info(
            "Received a request to reboot the PC. Sending 10 frames over the course of 1 second..."
        )
        return res

    def arch_launch_stack(self, req: Empty.Request, res: Empty.Response):
        self.frames_to_send += [ARCH_LAUNCH_STACK_ID]
        self.get_logger().info(
            "Received a request to launch the ARCh stack. Sending 1 frame..."
        )
        return res

    def arch_send_goal(self, req: Empty.Request, res: Empty.Response):
        self.frames_to_send += [ARCH_SEND_GOAL_ID]
        self.get_logger().info(
            "Received a request to send the ARCh autonomy goal. Sending 1 frame..."
        )
        return res

    def send_frames(self):
        if len(self.frames_to_send) > 0:
            code = self.frames_to_send.pop(0)
            self.pub.publish(
                MasterMessage(
                    cmd=MasterMessage.GS_TO_PC,
                    data=[UNIQUE_TUNNEL_FORWARD_ID, code],
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
