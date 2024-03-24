import rospy
from kalman_groundstation.msg import WheelsCommand
from geometry_msgs.msg import Twist, TwistStamped
from kalman_rover_uart.msg import MovementControl, GSMovementControl

from enum import Enum


class DrivingMode(Enum):
    NORMAL = 0
    IN_PLACE = 1
    SIDEWAYS = 2


class DrivingBridge:
    def __init__(self) -> None:
        self.sub_ws = rospy.Subscriber(
            "/station/wheels/command", WheelsCommand, self.handler
        )

        self.pub_normal = rospy.Publisher(
            "/kalman/navigation/gs_cmd_vel", GSMovementControl, queue_size=10
        )

        self.pub_in_place = rospy.Publisher(
            "/kalman/navigation/gs_cmd_vel_in_place", Twist, queue_size=10
        )

        self.pub_sideways = rospy.Publisher(
            "/kalman/navigation/gs_cmd_vel_sideways", GSMovementControl, queue_size=10
        )

    def handler(self, message: WheelsCommand):
        mode = message.mode

        if mode == DrivingMode.NORMAL.value:
            msg = GSMovementControl()
            msg.velocity = message.x
            msg.i_radius = message.y
            msg.translate = message.z
            self.pub_normal.publish(msg)

        elif mode == DrivingMode.IN_PLACE.value:
            msg = Twist()
            msg.linear.x = 0
            msg.angular.z = message.z
            self.pub_in_place.publish(msg)

        elif mode == DrivingMode.SIDEWAYS.value:
            msg = GSMovementControl()
            msg.velocity = message.x
            msg.translate = message.y
            msg.i_radius = message.z
            self.pub_sideways.publish(msg)

        else:
            rospy.logerr("Invalid driving mode received")
