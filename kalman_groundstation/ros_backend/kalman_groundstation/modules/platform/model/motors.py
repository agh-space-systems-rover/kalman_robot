from kalman_interfaces.msg import (
    MotorsState as MotorsStateMsg,
    PlatformState as PlatformStateMsg,
    MotorState as MotorStateMsg,
)
# from core.models.ros_model import RosModel
from kalman_groundstation.core.models.ros_model import RosModel


class MotorState(RosModel):
    velocity: float = 0
    angle: float = 0

    def to_ros_msg(self) -> MotorStateMsg:
        msg = MotorStateMsg()
        msg.angle = self.angle
        msg.velocity = self.velocity
        return msg

    @classmethod
    def from_ros_msg(cls, msg: MotorStateMsg):
        return MotorState(velocity=msg.velocity, angle=msg.angle)


class MotorsState(RosModel):
    fl: MotorState = MotorState()
    fr: MotorState = MotorState()
    bl: MotorState = MotorState()
    br: MotorState = MotorState()

    def to_ros_msg(self) -> MotorsStateMsg:
        msg = MotorsStateMsg()
        msg.fl = self.fl
        msg.fr = self.fr
        msg.br = self.br
        msg.bl = self.bl
        return msg

    @classmethod
    def from_ros_msg(cls, msg: MotorsStateMsg):
        return MotorsState(fl=msg.fl, fr=msg.fr, bl=msg.bl, br=msg.br)

class PlatformState(RosModel):
    targetMotors: MotorsState = MotorsState()
    motors: MotorsState = MotorsState()

    def to_ros_msg(self):
        msg = PlatformStateMsg()
        msg.motors = self.motors
        msg.targetMotors = self.targetMotors
        return msg
