import yaml
import rospkg
import os
import rospy
from std_msgs.msg import UInt8MultiArray, Float32
from kalman_groundstation.msg import ScienceState as ScienceStateMsg
from ..model.science import ScienceState


class WeightService:
    def __init__(self):
        self.uart2ros_sub = rospy.Subscriber(
            "/kalman_rover/uart2ros/143", UInt8MultiArray, self.update_weight
        )

        self.weight_publisher = rospy.Publisher(
            "/station/science/weight", Float32,
            queue_size=10
        )

    def update_weight(self, msg):
        arr = msg.data
        raw_weight = arr[-1] + arr[-2] * 2**8 + \
            arr[-3] * 2**16 + arr[-4] * 2**24
        rospy.logerr(raw_weight)
        # load raw_zero from file
        r = rospkg.RosPack()
        pkg_path = r.get_path('kalman_groundstation')
        current_cfg_path: str = os.path.join(
            pkg_path, f'cfg/science/current_weight.yaml')
        tared_cfg_path: str = os.path.join(
            pkg_path, f'cfg/science/tared_weight.yaml')
        raw_zero = -1

        with open(current_cfg_path, 'w') as stream:
            yaml.safe_dump({'weight': raw_weight}, stream)

        with open(tared_cfg_path, 'r') as stream:
            raw_zero = yaml.safe_load(stream)['weight']

        raw_per_gram = 780
        grams = (raw_weight - raw_zero) // raw_per_gram
        self.weight_publisher.publish(grams)
