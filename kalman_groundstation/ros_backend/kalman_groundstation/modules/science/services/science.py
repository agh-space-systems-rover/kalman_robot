import rospy
from std_msgs.msg import UInt8MultiArray
from kalman_groundstation.msg import ScienceState as ScienceStateMsg
from ..model.science import ScienceState
from struct import unpack


class ScienceService:
    def __init__(self):
        self.uart2ros_sub = rospy.Subscriber(
            "/kalman_rover/uart2ros/197", UInt8MultiArray, self.update_temperature
        )

        self.state_publisher = rospy.Publisher(
            "/station/science/state", ScienceStateMsg,
            queue_size=10
        )

        self.state = ScienceState().dict()

    def update_temperature(self, msg):
        slot = f"temp_slot_{msg.data[2] + 1}"

        temp = unpack("f", msg.data[3:])[0]

        self.state[slot] = 1.2 * temp - 4

        self.broadcast()

    def broadcast(self):
        msg = ScienceState(**self.state).to_ros_msg()
        self.state_publisher.publish(msg)
