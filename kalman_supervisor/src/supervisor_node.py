import rclpy
import rclpy.service
import rclpy.node
from rclpy.client import Client
from rclpy.timer import Rate
from main import Supervisor
from std_srvs.srv import SetBool

timer = Rate()
client = Client()
def main():
    rclpy.init()
    node = rclpy.create_node('supervisor')
    # wait for ueuos
    node.get_logger().info("Supervisor will start in 7.5 seconds.")
    timer.sleep(7.5)

    if node.declare_parameter('~simulated_robot') == False:
        client.wait_for_service("autonomy")
        autonomy_on_off = node.create_client("/autonomy_on_off")
        # autonomy_on_off(True)


    supervisor = Supervisor()
    while not rclpy.ok():
        supervisor.run()
        timer.sleep(0.3)
        
if __name__ == "__main__":
    main()
        
        