import rospy
from fastapi import APIRouter
from std_msgs.msg import UInt8MultiArray

autoclick_router = APIRouter(prefix="/autoclick", tags=["autoclick"])

ros2uart_pub = rospy.Publisher(
    "/kalman_rover/ros2uart", UInt8MultiArray, queue_size=10
)


@autoclick_router.put("/autoclick", name="Autoclick")
async def put(value: int):
    if value < 0 or value > 255:
        return False
    frame = UInt8MultiArray(data=[0xE3, 0x01, value])

    ros2uart_pub.publish(frame)
    return True
