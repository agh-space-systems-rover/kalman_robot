import rclpy
from rclpy.node import Node
from fastapi import APIRouter
from ..model.state import ArmState
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8, Float64
from std_srvs.srv import Empty
from ..model.state import ServoStatus


class ArmStatePublisher(APIRouter):
    def __init__(self, parent_node: Node) -> None:
        super().__init__(prefix="/state", tags=["state"])
        self.parent_node = parent_node

        self.joint_states = ArmState().dict()

        # ROS subscribtions
        self.parent_node.create_subscription(
            JointState,
            "/arm_controllers/joint_states",
            callback=self.update_joint_states,
            qos_profile=10,
        )
        self.parent_node.create_subscription(
            Int8,
            "/servo_server/status",
            callback=self.update_servo_status,
            qos_profile=10,
        )
        self.parent_node.create_subscription(
            Float64,
            "/servo_server/internal/collision_velocity_scale",
            callback=self.update_collision_velocity_factor,
            qos_profile=10,
        )

        # Webserver stuff
        self.add_api_route(
            "/",
            self.get, # Callable
            name="Get arm state",
            response_model=ArmState,
            response_description="Returns current state of arm",
            methods=["GET"],
        )

    def update_joint_states(self, msg: JointState):
        # global joint_states
        joint_states = dict()
        for name, position in zip(msg.name, msg.position):
            joint_states[name] = position
        self.joint_states = joint_states
        # rospy.set_param(ARM_STATE_PATH, joint_states)

    def update_servo_status(self, msg: Int8):
        # global joint_states
        joint_states = dict()
        joint_states["servo_status"] = str(ServoStatus(msg.data))
        # rospy.set_param(ARM_STATE_PATH, joint_states)
        self.joint_states = joint_states

    def update_collision_velocity_factor(self, msg: Float64):
        joint_states = dict()
        joint_states["collision_velocity_factor"] = msg.data
        # rospy.set_param(ARM_STATE_PATH, joint_states)
        self.joint_states = joint_states

        # servo server sam debil nie resetuje jak wyjdzie z kolizji albo z singularity
        if (
            joint_states["collision_velocity_factor"] == 1
            and joint_states["servo_status"]
            == str(ServoStatus.DECELERATE_FOR_COLLISION)
        ) or joint_states["servo_status"] == str(
            ServoStatus.DECELERATE_FOR_SINGULARITY
        ):
            # reset_servo_status = rospy.ServiceProxy("/servo_server/reset_servo_status", Empty)
            # reset_servo_status()
            pass  # TODO: reimplement

    def get(self):
        return self.joint_states


# @state_router.get(
#     "/",
#     name="Get arm state",
#     response_model=ArmState,
#     response_description="Returns current state of arm",
# )
# async def get():
#     return rospy.get_param(ARM_STATE_PATH)


# joint_states_subscriber = rospy.Subscriber(
#     "/arm_controllers/joint_states", JointState, callback=update_joint_states
# )
# servo_status_subscriber = rospy.Subscriber(
#     "/servo_server/status", Int8, callback=update_servo_status
# )
# collision_velocity_factor_subscriber = rospy.Subscriber(
#     "/servo_server/internal/collision_velocity_scale",
#     Float64,
#     callback=update_collision_velocity_factor,
# )
