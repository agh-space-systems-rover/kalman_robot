
#  restart_joints_service.py
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class RestartJointsService(Node):
    def __init__(self):
        super().__init__('restart_joints_service')
        self.srv = self.create_service(Trigger, 'restart_joints', self.restart_joints_callback)

    def restart_joints_callback(self, request, response):
        # Logic to restart joints
        self.get_logger().info('Restarting joints...')
        # You can add your logic here to actually restart the joints, such as:
        # - Sending commands to hardware
        # - Resetting parameters, etc.
        
        # Assuming success for this example
        response.success = True
        response.message = "Joints restarted successfully"
        return response
        print("Joints restarted successfully")

def main(args=None):
    rclpy.init(args=args)
    node = RestartJointsService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
