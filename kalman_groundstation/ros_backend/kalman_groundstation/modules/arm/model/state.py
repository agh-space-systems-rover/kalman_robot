from enum import Enum
from kalman_groundstation.core.models.ros_model import RosModel
from kalman_interfaces.msg import ArmState as ArmStateMsg


class ArmState(RosModel):
    servo_status: str = "COULD_NOT_FETCH_SERVO_STATUS"
    joint_1: float = 2137
    joint_2: float = 2137
    joint_3: float = 2137
    joint_4: float = 2137
    joint_5: float = 2137
    joint_6: float = 2137
    gripper_moving_joint: float = 2137
    collision_velocity_factor: float = 0.0

    def to_ros_msg(self):
        return ArmStateMsg(**self.dict())


class ServoStatus(Enum):
    INVALID: -1
    OK = 0
    DECELERATE_FOR_SINGULARITY = 1
    HALT_FOR_SINGULARITY = 2
    DECELERATE_FOR_COLLISION = 3
    HALT_FOR_COLLISION = 4
    JOINT_LIMIT_REACHED = 5
