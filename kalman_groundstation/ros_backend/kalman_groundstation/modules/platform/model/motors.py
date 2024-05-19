from kalman_interfaces.msg import (
    MotorsState as MotorsStateMsg,
    PlatformState as PlatformStateMsg,
    MotorState as MotorStateMsg,
    WheelStates as WheelStatesMsg,
    WheelState as WheelStateMsg,
)

# from core.models.ros_model import RosModel
from kalman_groundstation.core.models.ros_model import RosModel


# class MotorState(RosModel):
#     velocity: float = 0
#     angle: float = 0

#     def to_ros_msg(self) -> MotorStateMsg:
#         msg = MotorStateMsg()
#         msg.angle = self.angle
#         msg.velocity = self.velocity
#         return msg

#     @classmethod
#     def from_ros_msg(cls, msg: MotorStateMsg):
#         return MotorState(velocity=msg.velocity, angle=msg.angle)

#     @classmethod
#     def from_wheel_msg(cls, msg: WheelStateMsg):
#         return MotorState(velocity=msg.velocity, angle=msg.angle)


# class MotorsState(RosModel):
#     fl: MotorState = MotorState()
#     fr: MotorState = MotorState()
#     bl: MotorState = MotorState()
#     br: MotorState = MotorState()

#     def to_ros_msg(self) -> MotorsStateMsg:
#         msg = MotorsStateMsg()
#         msg.fl = self.fl.to_ros_msg()
#         msg.fr = self.fr.to_ros_msg()
#         msg.br = self.br.to_ros_msg()
#         msg.bl = self.bl.to_ros_msg()
#         return msg

#     @classmethod
#     def from_ros_msg(cls, msg: MotorsStateMsg):
#         return MotorsState(fl=msg.fl, fr=msg.fr, bl=msg.bl, br=msg.br)

#     @classmethod
#     def from_wheels_msg(cls, msg: WheelStatesMsg):
#         return MotorsState(
#             fl=MotorState.from_wheel_msg(msg.front_left),
#             fr=MotorState.from_wheel_msg(msg.front_right),
#             bl=MotorState.from_wheel_msg(msg.back_left),
#             br=MotorState.from_wheel_msg(msg.back_right),
#         )


class WheelState(RosModel):
    velocity: float = 0.0
    angle: float = 0.0

    def to_ros_msg(self) -> WheelStateMsg:
        msg = WheelStateMsg()
        msg.velocity = self.velocity
        msg.angle = self.angle
        return msg

    @classmethod
    def from_ros_msg(cls, msg: WheelStateMsg):
        return WheelState(velocity=msg.velocity, angle=msg.angle)


class WheelStates(RosModel):
    front_left: WheelState = WheelState()
    front_right: WheelState = WheelState()
    back_left: WheelState = WheelState()
    back_right: WheelState = WheelState()

    def to_ros_msg(self) -> WheelStatesMsg:
        msg = WheelStatesMsg()
        msg.front_left = self.front_left.to_ros_msg()
        msg.front_right = self.front_right.to_ros_msg()
        msg.back_right = self.back_right.to_ros_msg()
        msg.back_left = self.back_left.to_ros_msg()
        return msg

    @classmethod
    def from_ros_msg(cls, msg: WheelStatesMsg):
        return WheelStates(
            front_left=WheelState.from_ros_msg(msg.front_left),
            front_right=WheelState.from_ros_msg(msg.front_right),
            back_left=WheelState.from_ros_msg(msg.back_left),
            back_right=WheelState.from_ros_msg(msg.back_right),
        )


class PlatformState(RosModel):
    target_motors: WheelStates = WheelStates()
    motors: WheelStates = WheelStates()

    def to_ros_msg(self) -> PlatformStateMsg:
        msg = PlatformStateMsg()
        msg.motors = self.motors.to_ros_msg()
        msg.target_motors = self.target_motors.to_ros_msg()
        return msg
