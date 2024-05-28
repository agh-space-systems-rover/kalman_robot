# import rospy
# from fastapi import APIRouter
# from std_msgs.msg import UInt8MultiArray

# from kalman_interfaces.msg import MasterMessage

# autoclick_router = APIRouter(prefix="/autoclick", tags=["autoclick"])

# ros2uart_pub = rospy.Publisher(
#     "/kalman_rover/ros2uart", UInt8MultiArray, queue_size=10
# )

# @autoclick_router.put("/autoclick", name="Autoclick")
# async def put(value: int):
#     if value < 0 or value > 255:
#         return False
#     frame = UInt8MultiArray(data=[0xE3, 0x01, value])

#     ros2uart_pub.publish(frame)
#     return True



from rclpy.node import Node
from fastapi import APIRouter

from kalman_interfaces.msg import MasterMessage


class AutoclickRouter(APIRouter):
    def __init__(self, parent_node: Node):
        self.parent_node = parent_node
        self.ros2uart_publisher = parent_node.create_publisher(
            MasterMessage, "/master_com/ros_to_master", qos_profile=10
        )

        # Web stuffs
        super().__init__(prefix="/autoclick", tags=["autoclick"])

        self.add_api_route(
            "/autoclick",
            self.handle_autoclick, # Callable
            name="Controls autoclick",
            response_model=bool,
            methods=["PUT"],
        )
        
        
        self.add_api_route(
            "/screwdriver",
            self.handle_screwdriver, # Callable
            name="Controls screwdriver",
            response_model=bool,
            methods=["PUT"],
        )


    def handle_autoclick(self,value: int):
        if value < 0 or value > 180:
            return False
        frame = MasterMessage(cmd=0x50, data=[value])

        self.ros2uart_publisher.publish(frame)
        return True
    
    def handle_screwdriver(self,value: int):
        if value < -100 or value > 100:
            return False
        frame = MasterMessage(cmd=0x53, data=[0,1,abs(value),(0 if value < 0 else 1)])

        self.ros2uart_publisher.publish(frame)
        return True