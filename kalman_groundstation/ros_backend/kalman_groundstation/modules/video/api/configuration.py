import rclpy
from rclpy.node import Node
from fastapi import APIRouter
from kalman_interfaces.msg import MasterMessage
from struct import pack


class VideoConfigurationRouter(APIRouter):
    def __init__(self, parent_node: Node):
        self.parent_node = parent_node
        self.ros2uart_publisher = parent_node.create_publisher(
            MasterMessage, "/master_com/ros_to_master", qos_profile=10
        )

        # Web stuffs
        super().__init__(prefix="/configuration", tags=["configuration"])

        self.add_api_route(
            "/camera",
            self.set_camera, # Callable
            name="Sets current camera",
            response_model=bool,
            methods=["PUT"],
        )

        self.add_api_route(
            "/channel",
            self.set_channel, # Callable
            name="Sets current camera",
            response_model=bool,
            methods=["PUT"],
        )

        self.add_api_route(
            "/power",
            self.set_power, # Callable
            name="Sets current power? TODO: fix",
            response_model=bool,
            methods=["PUT"],
        )

        

    def set_camera(self, feed: int, camera: int):
        if feed > 2 or feed < 1 or camera < 1 or camera > 8:
            return False
        # transform = [1, 3, 5, 7, 2, 4, 6, 8]
        transform = [1, 2, 3, 4, 5, 6, 7, 8]
        # data = [int(x) for x in [0xB1, 0x02, feed, transform[camera - 1]]]
        # data = list(pack("B" * 2 + "B" * (len(data) - 2), *data))
        self.parent_node.get_logger().info(f"Set camera: {[feed, transform[camera - 1]]}")
        self.ros2uart_publisher.publish(MasterMessage(cmd=0xB1,data=[feed, transform[camera - 1]]))
        return True


    def set_channel(self, dupa: bool, feed: int, channel: int):
        if not dupa:
            frame_id = 0xB2
        else:
            frame_id = 0xB4

        if feed > 2 or feed < 1 or channel < 1 or channel > 40:
            return False
        # data = [int(x) for x in [frame_id, 0x02, feed, channel]]
        # data = list(pack("B" * 2 + "B" * (len(data) - 2), *data))
        self.parent_node.get_logger().info(f"Set channel: {[feed, channel]}")
        self.ros2uart_publisher.publish(MasterMessage(cmd=frame_id,data=[feed, channel]))
        return True


    def set_power(self, feed: int, power: int):
        if feed > 2 or feed < 1 or power < 0 or power > 4:
            return False
        # data = [int(x) for x in [0xB3, 0x02, feed, power]]
        # data = list(pack("B" * 2 + "B" * (len(data) - 2), *data))
        self.parent_node.get_logger().info(f"Set power: {[feed, power]}")
        self.ros2uart_publisher.publish(MasterMessage(cmd=0xB3,data=[feed, power]))
        return True



# configuration_router = APIRouter(prefix="/configuration", tags=["configuration"])

# ros2uart_publisher = rospy.Publisher(
#     "/kalman_rover/ros2uart", UInt8MultiArray, queue_size=10
# )


# @configuration_router.put("/camera", name="Sets current camera", response_model=bool)
# async def put(feed: int, camera: int):
#     if feed > 2 or feed < 1 or camera < 1 or camera > 8:
#         return False
#     # transform = [1, 3, 5, 7, 2, 4, 6, 8]
#     transform = [1, 2, 3, 4, 5, 6, 7, 8]
#     data = [int(x) for x in [0xB1, 0x02, feed, transform[camera - 1]]]
#     data = list(pack("B" * 2 + "B" * (len(data) - 2), *data))
#     rospy.loginfo(data)
#     ros2uart_publisher.publish(UInt8MultiArray(data=data))
#     return True


# @configuration_router.put("/channel", name="Sets current camera", response_model=bool)
# async def put(dupa: bool, feed: int, channel: int):
#     if not dupa:
#         frame_id = 0xB2
#     else:
#         frame_id = 0xB4

#     if feed > 2 or feed < 1 or channel < 1 or channel > 40:
#         return False
#     data = [int(x) for x in [frame_id, 0x02, feed, channel]]
#     data = list(pack("B" * 2 + "B" * (len(data) - 2), *data))
#     rospy.loginfo(data)
#     ros2uart_publisher.publish(UInt8MultiArray(data=data))
#     return True


# @configuration_router.put("/power", name="Sets current camera", response_model=bool)
# async def put(feed: int, power: int):
#     if feed > 2 or feed < 1 or power < 0 or power > 4:
#         return False
#     data = [int(x) for x in [0xB3, 0x02, feed, power]]
#     data = list(pack("B" * 2 + "B" * (len(data) - 2), *data))
#     rospy.loginfo(data)
#     ros2uart_publisher.publish(UInt8MultiArray(data=data))
#     return True
