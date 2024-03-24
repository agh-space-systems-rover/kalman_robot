import numpy as np
import rospy
from std_msgs.msg import UInt8MultiArray
from kalman_groundstation.msg import SmartProbe


class SmartProbeService:
    def __init__(self):
        self.uart2ros_sub = rospy.Subscriber(
            "/kalman_rover/uart2ros/252", UInt8MultiArray, self.update_measurements
        )

        self.smart_probe_publisher = rospy.Publisher(
            "/station/science/smart_probe", SmartProbe,
            queue_size=10
        )

        self.HUM_MAX = 3162
        self.HUM_MIN = 1626

    def update_measurements(self, data):
        arr = data.data[2:]

        raw_temperature = arr[1] << 8 | arr[0]
        raw_humidity = arr[3] << 8 | arr[2]

        msg = SmartProbe()
        msg.temperature = self.calculate_temp(raw_temperature)
        msg.humidity = self.calculate_humidity(raw_humidity)
        rospy.logerr(arr)
        rospy.logerr(msg)
        self.smart_probe_publisher.publish(msg)

    def calculate_humidity(self, raw_humidity: int) -> float:
        if raw_humidity < self.HUM_MIN:
            raw_humidity = self.HUM_MIN

        if raw_humidity > self.HUM_MAX:
            raw_humidity = self.HUM_MAX

        return self.lin_map(raw_humidity, self.HUM_MIN, self.HUM_MAX, 100, 0)

    def calculate_temp(self, raw_temperature: int) -> float:
        """
        https://en.wikipedia.org/wiki/Thermistor#B_or_%CE%B2_parameter_equation
        https://www.makeralot.com/download/Reprap-Hotend-Thermistor-NTC-3950-100K.pdf

        raw_temperature is a V_out on voltage divisor. Ask Mily from ele if you want to understand more 
        about this code lmao
        """

        Vout = 3.3 * raw_temperature / 4095
        Rt = 10**5 * Vout / (3.3 - Vout)
        B = 3950
        T0 = 298
        R0 = 10**5

        r_inf = R0 * np.exp(-B / T0)

        T = B / np.log(Rt / r_inf)

        return T - 273

    def lin_map(self, x, s1, e1, s2, e2):
        return s2 + ((e2 - s2) / (e1 - s1)) * (x - s1)
