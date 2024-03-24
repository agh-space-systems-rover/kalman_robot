import rospy
from kalman_groundstation.msg import DrillCommand
from std_msgs.msg import UInt8MultiArray


class CommandBridge:
    def __init__(self):
        self.sub_drill = rospy.Subscriber(
            "/station/science/drill", DrillCommand, self.handler_drill
        )

        self.ros2uart_pub = rospy.Publisher(
            "/kalman_rover/ros2uart", UInt8MultiArray, queue_size=10
        )

    def handler_drill(self, message: DrillCommand):
        drill_frame = UInt8MultiArray(
            data=[0x45, 0x02, 1 if message.drill > 0 else 0, int(abs(message.drill*100))])
        height_frame = UInt8MultiArray(
            data=[0x46, 0x02, 1 if message.height > 0 else 0, int(abs(message.height*100))])

        self.ros2uart_pub.publish(drill_frame)
        rospy.sleep(0.05)
        self.ros2uart_pub.publish(height_frame)
