import rospy

from kalman_groundstation.msg import MotorsTemperature, MotorTemperature
from std_msgs.msg import UInt8MultiArray


class TemperatureBridge:
    def __init__(self) -> None:
        self.uart = rospy.Subscriber(
            "/kalman_rover/uart2ros/66", UInt8MultiArray, self._update
        )
        self.ws = rospy.Publisher(
            "/station/wheels/temperatures", MotorsTemperature, queue_size=10
        )

        self.MAX_TURN_TEMP = 60
        self.MAX_MOTOR_TEMP = 60

    def _update(self, msg: UInt8MultiArray):
        data = MotorsTemperature()
        temps = msg.data[2:]

        (
            data.fr.motor,
            data.br.motor,
            data.bl.motor,
            data.fl.motor,
            data.fr.turn,
            data.br.turn,
            data.bl.turn,
            data.fl.turn,
        ) = temps

        overheating = (
            lambda i: temps[i] > self.MAX_MOTOR_TEMP
            or temps[i + 4] > self.MAX_TURN_TEMP
        )

        data.fr.overheating = overheating(0)
        data.br.overheating = overheating(1)
        data.bl.overheating = overheating(2)
        data.fl.overheating = overheating(3)

        self.ws.publish(data)
