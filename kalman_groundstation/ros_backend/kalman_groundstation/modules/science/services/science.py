# from std_msgs.msg import UInt8MultiArray
# from kalman_groundstation.msg import ScienceState as ScienceStateMsg
from kalman_interfaces.msg import MasterMessage, NewScienceResp, ScienceState as ScienceStateMsg

from ..model.science import ScienceState
from std_msgs.msg import Float32, Bool
from struct import unpack
from rclpy.node import Node

from kalman_groundstation.modules.science.universal_module import (
    CAN_CMD_SET_RESPONSE,
    SetResponseFrame,
    CAN_CMD_SET_DIGITAL_OUTPUT,
    CAN_CMD_SET_PWM_OUTPUT,
    CAN_CMD_SET_LED_DRIVER,
    CAN_CMD_SET_HBRIDGE,
    CAN_CMD_AUTOMATION_SEQUENCE_STATE_RESPONSE,
    SequenceManagerStateFrame,
)


class ScienceService:
    def __init__(self, parent_node: Node):
        # self.uart2ros_sub = parent_node.create_subscription(
        #     MasterMessage, "/master_com/master_to_ros/xc5", self.update_temperature, qos_profile=10
        # )

        self.set_response_sub = parent_node.create_subscription(
            MasterMessage, f"/master_com/master_to_ros/{hex(CAN_CMD_SET_RESPONSE)[1:]}", self.update_set_response, qos_profile=10
        )

        self.seq_response_sub =  parent_node.create_subscription(
            MasterMessage, f"/master_com/master_to_ros/{hex(CAN_CMD_AUTOMATION_SEQUENCE_STATE_RESPONSE)[1:]}", self.update_seq_response, qos_profile=10
        )

        self.state_publisher = parent_node.create_publisher(
            ScienceStateMsg, "/station/science/state", qos_profile=10
        )

        self.response_pub = parent_node.create_publisher(
            NewScienceResp, "/station/science/resp", qos_profile=10
        )

        self.state = ScienceState().dict()
        self.new_state = NewScienceResp()

    def update_temperature(self, msg: MasterMessage):
        slot = f"temp_slot_{msg.data[2] + 1}"

        temp = unpack("f", msg.data[3:])[0]

        self.state[slot] = 1.2 * temp - 4

        self.broadcast()

    def broadcast(self):
        msg = ScienceState(**self.state).to_ros_msg()
        self.state_publisher.publish(msg)

    def update_set_response(self, msg: MasterMessage):
        frame = SetResponseFrame.parse(msg.data)
        if frame.command_id == CAN_CMD_SET_PWM_OUTPUT:
            if frame.channel_id == 1:
                self.new_state.washer.data = float(frame.state.byte_state)
        elif frame.command == CAN_CMD_SET_LED_DRIVER:
            value = float(frame.state.byte_state)
            if frame.channel_id == 0:
                self.new_state.led1.data = value
            elif frame.channel_id == 1:
                self.new_state.led2.data = value
            elif frame.channel_id == 2:
                self.new_state.led3.data = value
        elif frame.command == CAN_CMD_SET_HBRIDGE:
            value = float(frame.state.speed)
            if frame.channel_id == 0:
                self.new_state.heating_down.data = value
            elif frame.channel_id == 1:
                self.new_state.heating_up.data = value

        self.response_pub.publish(self.new_state)

    def update_seq_response(self, msg: MasterMessage):
        frame = SequenceManagerStateFrame().parse(msg.data)
        state = frame.sequence_state
        stage = frame.current_stage
        if frame.sequence_id == 0:
            self.new_state.seq_state1 = state
            self.new_state.seq_stage1 = stage
        elif frame.sequence_id == 1:
            self.new_state.seq_state2 = state
            self.new_state.seq_stage2 = stage
        elif frame.sequence_id == 2:
            self.new_state.seq_state3 = state
            self.new_state.seq_stage3 = stage

        self.response_pub.publish(self.new_state)
