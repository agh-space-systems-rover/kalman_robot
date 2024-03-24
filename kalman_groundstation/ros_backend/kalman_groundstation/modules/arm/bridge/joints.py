import rospy
from fastapi import APIRouter
from ..model.state import ArmState
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8, Float64
from std_srvs.srv import Empty
from ..model.state import ServoStatus
from kalman_groundstation.msg import ArmState as ArmStateMsg


ARM_STATE_PATH = "/station/system/rover/arm/state/"


class JointStateBridge:
    def __init__(self) -> None:
        self.joint_states_subscriber = rospy.Subscriber(
            "/arm_controllers/joint_states",
            JointState,
            callback=self.update_joint_states,
        )
        self.servo_status_subscriber = rospy.Subscriber(
            "/servo_server/status",
            Int8,
            callback=self.update_servo_status,
        )
        self.collision_velocity_factor_subscriber = rospy.Subscriber(
            "/servo_server/internal/collision_velocity_scale",
            Float64,
            callback=self.update_collision_velocity_factor,
        )

        self.reset_servo_status = rospy.ServiceProxy(
            "/servo_server/reset_servo_status", Empty
        )

        self.joint_states = ArmState().dict()

        self.state_publisher = rospy.Publisher(
            "/station/arm/state", ArmStateMsg, queue_size=10
        )

    def update_joint_states(self, msg: JointState):
        joint_states = self.joint_states
        for name, position in zip(msg.name, msg.position):
            joint_states[name] = position
        rospy.set_param(ARM_STATE_PATH, joint_states)
        self.broadcast()

    def update_servo_status(self, msg: Int8):
        joint_states = self.joint_states
        joint_states["servo_status"] = str(ServoStatus(msg.data))
        rospy.set_param(ARM_STATE_PATH, joint_states)

    def update_collision_velocity_factor(self, msg: Float64):
        joint_states = self.joint_states
        joint_states["collision_velocity_factor"] = msg.data
        rospy.set_param(ARM_STATE_PATH, joint_states)

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
