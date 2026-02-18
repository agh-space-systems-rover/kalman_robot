import struct
import rclpy
from rclpy.node import Node
import numpy as np
from struct import pack
from kalman_interfaces.msg import Drill, MasterMessage
from std_srvs.srv import Trigger
from std_msgs.msg import Float32
from dataclasses import dataclass

ARM_SPEED = 50
RACK_SPEED = 50
DRILL_SPEED = 100
MAX_ZEROFRAMES_SPAM = 5

SCALE_DEVICES = [
    # (board, channel), ...
    (1, 2), (1, 3)
]
WEIGHT_SCALES = [1.0, 1.0]
WEIGHT_BIAS = 0.0

class DrillDriver(Node):
    def __init__(self):
        super().__init__("drill_driver")

        # mechanism controls
        self.vel_sub = self.create_subscription(Drill, "science/drill/cmd", self.drill_vel_cb, 1)
        self.master_pub = self.create_publisher(
            MasterMessage, "master_com/ros_to_master", 10
        )
        self.zero_arm = 0
        self.zero_rack = 0
        self.zero_drill = 0

        # autonomy controls
        self.auto_start_srv = self.create_service(Trigger, "science/drill/auto/start", self.start_autonomy_cb)
        self.auto_stop_srv = self.create_service(Trigger, "science/drill/auto/stop", self.stop_autonomy_cb)

        # scale controls
        self.weight_req_srv = self.create_service(
            Trigger, "science/drill/weight/req", self.scale_req_cb
        )
        self.weight_reading_sub = self.create_subscription(
            MasterMessage,
            f"master_com/master_to_ros/{hex(MasterMessage.SCALE_RES)[1:]}",
            self.scale_res_cb,
            10,
        )
        self.weight_pub = self.create_publisher(Float32, "science/drill/weight", 10)
        self.last_weight_readings = [0.0] * len(SCALE_DEVICES)

    def drill_vel_cb(self, msg: Drill):
        msg.arm = np.clip(msg.arm, -1, 1)
        msg.rack = np.clip(msg.rack, -1, 1)
        msg.drill = np.clip(msg.drill, -1, 1)

        data_arm = [1 if msg.arm < 0 else 0, round(abs(msg.arm * ARM_SPEED))]
        data_rack = [1 if msg.rack < 0 else 0, round(abs(msg.rack * RACK_SPEED))]
        data_drill = [1 if msg.drill < 0 else 0, round(abs(msg.drill * DRILL_SPEED))]

        self.zero_arm = self.zero_arm + 1 if self.is_zero_frame(data_arm) else 0
        self.zero_rack = self.zero_rack + 1 if self.is_zero_frame(data_rack) else 0
        self.zero_drill = self.zero_drill + 1 if self.is_zero_frame(data_drill) else 0

        if self.zero_arm <= MAX_ZEROFRAMES_SPAM:
            self.master_pub.publish(
                MasterMessage(cmd=MasterMessage.DRILL_ARM, data=data_arm)
            )

        if self.zero_rack <= MAX_ZEROFRAMES_SPAM:
            self.master_pub.publish(
                MasterMessage(cmd=MasterMessage.DRILL_RACK, data=data_rack)
            )

        if self.zero_drill <= MAX_ZEROFRAMES_SPAM:
            self.master_pub.publish(
                MasterMessage(cmd=MasterMessage.DRILL_DRILL, data=data_drill)
            )

    def is_zero_frame(self, data):
        return True if data[0] == 0 and data[1] == 0 else False
    
    def start_autonomy_cb(self, req: Trigger.Request, res: Trigger.Response):
        msg = MasterMessage()
        msg.cmd = MasterMessage.DRILL_AUTONOMY
        msg.data = [1]
        self.master_pub.publish(msg)
        res.success = True
        res.message = "Autonomy started"
        return res

    def stop_autonomy_cb(self, req: Trigger.Request, res: Trigger.Response):
        msg = MasterMessage()
        msg.cmd = MasterMessage.DRILL_AUTONOMY
        msg.data = [0]
        self.master_pub.publish(msg)
        res.success = True
        res.message = "Autonomy stopped"
        return res

    def scale_req_cb(self, req: Trigger.Request, res: Trigger.Response):
        for board_id, channel_id in SCALE_DEVICES:
            # Create the request message
            req_msg = MasterMessage()
            req_msg.cmd = MasterMessage.SCALE_REQ
            req_msg.data = pack("BB", board_id, channel_id)

            # Publish the request to the master
            self.master_pub.publish(req_msg)

        res.success = True
        res.message = "Requested weight reading"
        return res
    
    def scale_res_cb(self, msg: MasterMessage):
        if msg.cmd == MasterMessage.SCALE_RES:
            board_id, channel_id, value = struct.unpack("<BBi", bytes(msg.data[:6]))

            scale_idx = SCALE_DEVICES.index((board_id, channel_id))
            self.last_weight_readings[scale_idx] = value

            self.get_logger().info(f"{self.last_weight_readings}")

            total_weight = sum(reading * scale for reading, scale in zip(self.last_weight_readings, WEIGHT_SCALES)) + WEIGHT_BIAS
            self.weight_pub.publish(Float32(data=total_weight))

def main():
    try:
        rclpy.init()
        node = DrillDriver()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
