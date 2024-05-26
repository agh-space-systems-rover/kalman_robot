from kalman_interfaces.msg import ScienceState as ScienceStateMsg
from kalman_groundstation.core.models.ros_model import RosModel
from std_msgs.msg import Float32


class ScienceState(RosModel):
    temp_slot_1: float = 0.0
    temp_slot_2: float = 0.0
    temp_slot_3: float = 0.0

    def to_ros_msg(self) -> ScienceStateMsg:
        msg = ScienceStateMsg()
        msg.temp_slot_1 = Float32(data=self.temp_slot_1)
        msg.temp_slot_2 = Float32(data=self.temp_slot_2)
        msg.temp_slot_3 = Float32(data=self.temp_slot_3)
        return msg

    @classmethod
    def from_ros_msg(cls, msg: ScienceStateMsg):
        return ScienceState(
            temp_slot_1=msg.temp_slot_1,
            temp_slot_2=msg.temp_slot_2,
            temp_slot_3=msg.temp_slot_3,
        )
