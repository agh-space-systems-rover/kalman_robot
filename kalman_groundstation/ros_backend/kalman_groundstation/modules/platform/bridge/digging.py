from struct import pack
import rospy
from std_msgs.msg import Float32MultiArray, UInt8MultiArray
from dataclasses import dataclass
import numpy as np

class WheelCommand:
    def __init__(self, angle, speed):
        """
        Args:
            angle - in degrees [-90, 90]
            speed - abstract velocity units [-100, 100]
        """
        self._angle = np.clip(int(angle), -90, 90)
        self._speed = np.clip(int(speed), -100, 100)

    @property
    def angle(self) -> int:
        return self._angle

    @property
    def speed(self) -> int:
        return self._speed


@dataclass
class PlatformCommand:
    fl: WheelCommand
    fr: WheelCommand
    bl: WheelCommand
    br: WheelCommand

    def to_uart_message(self) -> UInt8MultiArray:
        data = [
            0x40, # wheels command frame id
            0x8,  # number of args
            self.fr.speed,
            self.br.speed,
            self.bl.speed,
            self.fl.speed,
            self.fr.angle,  # no idea why
            -self.br.angle,
            -self.bl.angle,
            self.fl.angle,  # no idea why
        ]
        data = list(pack('b'*len(data), *data))
        return UInt8MultiArray(data=data)


class DiggingBridge:
    def __init__(self) -> None:
        self.width = float(
            rospy.get_param("/configurations/dimensions/suspension_width")
        )
        self.length = float(
            rospy.get_param("/configurations/dimensions/suspension_length")
        )

        self.sub_command = rospy.Subscriber(
            "/station/digging/command", Float32MultiArray, self.command_handler
        )

        self.uart = rospy.Publisher(
            "/kalman_rover/ros2uart", UInt8MultiArray, queue_size=10
        )

    def command_handler(self, msg: Float32MultiArray):
        # digging wheel speed [-1, 1] as in wheelsMapping
        x = msg.data[0] * 100
        y = msg.data[1] * 100

        angle = -np.rad2deg(np.arctan(self.width / self.length)) + 90
        diagonal = (self.width**2 + self.length**2)**0.5


        command = PlatformCommand(
            fl=WheelCommand(0, x),
            fr=WheelCommand(0, y * self.width / diagonal),
            bl=WheelCommand(90, y * self.length / diagonal),
            br=WheelCommand(angle, y),
        )
        self.uart.publish(command.to_uart_message())
