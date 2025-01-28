
# restart_joints_client.py
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class RestartJointsClient(Node):
    def __init__(self):
        super().__init__('restart_joints_client')
        self.client = self.create_client(Trigger, 'restart_joints')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.send_request()

    def send_request(self):
        request = Trigger.Request()
        future = self.client.call_async(request)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        response = future.result()
        if response.success:
            self.get_logger().info('Joints restarted successfully')
        else:
            self.get_logger().error('Failed to restart joints')

def main(args=None):
    rclpy.init(args=args)
    client = RestartJointsClient()
    rclpy.spin(client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
