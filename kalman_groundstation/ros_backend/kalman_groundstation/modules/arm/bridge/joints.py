from fastapi import APIRouter
from ..model.state import ArmState
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8, Float64
from std_srvs.srv import Empty
from ..model.state import ServoStatus
from kalman_interfaces.msg import ArmState as ArmStateMsg
from rclpy.node import Node


ARM_STATE_PATH = "/station/system/rover/arm/state/"


class JointStateBridge:
    def __init__(self, parent_node: Node) -> None:
        self.parent_node = parent_node
        self.joint_states_subscriber = parent_node.create_subscription(
            JointState,
            "/arm_controllers/joint_states",
            callback=self.update_joint_states,
            qos_profile=10,
        )
        self.servo_status_subscriber = parent_node.create_subscription(
            Int8,
            "/servo_server/status",
            callback=self.update_servo_status,
            qos_profile=10,
        )
        self.collision_velocity_factor_subscriber = parent_node.create_subscription(
            Float64,
            "/servo_server/internal/collision_velocity_scale",
            callback=self.update_collision_velocity_factor,
            qos_profile=10,
        )

        # self.reset_servo_status = rospy.ServiceProxy(
        #     "/servo_server/reset_servo_status", Empty
        # )
        self.joint_states = ArmState().dict()

        self.state_publisher = parent_node.create_publisher(
            ArmStateMsg, "/station/arm/state", qos_profile=10
        )

    def update_joint_states(self, msg: JointState):
        joint_states = self.joint_states
        for name, position in zip(msg.name, msg.position):
            joint_states[name] = position
        # rospy.set_param(ARM_STATE_PATH, joint_states)
        self.broadcast()

    def update_servo_status(self, msg: Int8):
        joint_states = self.joint_states
        joint_states["servo_status"] = str(ServoStatus(msg.data))
        # rospy.set_param(ARM_STATE_PATH, joint_states)

    def update_collision_velocity_factor(self, msg: Float64):
        joint_states = self.joint_states
        joint_states["collision_velocity_factor"] = msg.data
        # rospy.set_param(ARM_STATE_PATH, joint_states)

        # servo server sam debil nie resetuje jak wyjdzie z kolizji albo z singularity
        if (
            joint_states["collision_velocity_factor"] == 1
            and joint_states["servo_status"]
            == str(ServoStatus.DECELERATE_FOR_COLLISION)
        ) or joint_states["servo_status"] == str(
            ServoStatus.DECELERATE_FOR_SINGULARITY
        ):
            self.reset_servo_status()

    def broadcast(self):
        # rospy.logerr(ArmState(**self.joint_states).to_ros_msg())
        self.state_publisher.publish(ArmState(**self.joint_states).to_ros_msg())
