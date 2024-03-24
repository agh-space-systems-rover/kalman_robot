import rospy

from std_msgs.msg import UInt8MultiArray
from kalman_groundstation.msg import PlatformState as PlatformStateMsg
from ..model.motors import PlatformState
from struct import unpack

# rosparam paths
STATE_PATH = "/station/system/rover/platform/state/"
MOTORS_STATE_PATH = STATE_PATH + "targetMotors/"
TARGET_MOTORS_STATE_PATH = STATE_PATH + "motors/"


class MotorsBridge:
    def __init__(self) -> None:
        self.ros2uart_subscriber = rospy.Subscriber(
            "/kalman_rover/ros2uart", UInt8MultiArray, self.update_motors
        )

        self.uart2ros_subscriber = rospy.Subscriber(
            "/kalman_rover/uart2ros/79", UInt8MultiArray, self.update_motors
        )

        self.state_publisher = rospy.Publisher(
            "/station/wheels/state", PlatformStateMsg, queue_size=10
        )

        self.state = PlatformState().dict()

    def update_motors(self, msg):
        if msg.data[0] == 0x40:
            rvel, brvel, blvel, lvel, fr, br, bl, fl = unpack("bbbbbbbb", msg.data[2:])
            data = {
                "targetMotors": {
                    "bl": {"angle": -bl, "velocity": blvel},
                    "br": {"angle": -br, "velocity": brvel},
                    "fl": {"angle": fl, "velocity": lvel},
                    "fr": {"angle": fr, "velocity": rvel},
                }
            }
            self.state.update(data)
            # rospy.set_param(
            #     TARGET_MOTORS_STATE_PATH,
            #     data,
            # )
            self.broadcast()
        elif msg.data[0] == 0x4F:
            fr, br, bl, fl = unpack("bbbb", msg.data[-4:])
            data = {
                "motors": {
                    "bl": {"angle": -bl, "velocity": 0},
                    "br": {"angle": -br, "velocity": 0},
                    "fl": {"angle": fl, "velocity": 0},
                    "fr": {"angle": fr, "velocity": 0},
                }
            }
            self.state.update(data)
            # rospy.set_param(MOTORS_STATE_PATH, data)
            self.broadcast()

    def broadcast(self):
        msg = PlatformState(**self.state).to_ros_msg()
        self.state_publisher.publish(msg)
