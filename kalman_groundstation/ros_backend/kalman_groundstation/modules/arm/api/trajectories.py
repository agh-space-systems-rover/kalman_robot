from fastapi import APIRouter
from std_msgs.msg import String
import rospy
from arm_trajectories.msg import TrajectoryAction, TrajectoryGoal
import actionlib

trajectory_router = APIRouter(prefix="/trajectories", tags=["trajectories"])

# trajectory action client
client = actionlib.ActionClient("/arm_controllers/trajectory", TrajectoryAction)

new_trajectory_publisher = rospy.Publisher(
    "/arm_controllers/trajectory/new_trajectory", String, queue_size=10
)

add_point_publisher = rospy.Publisher(
    "/arm_controllers/trajectory/add_point", String, queue_size=10
)


@trajectory_router.get(
    "/",
    name="Handles new trajectory system that doesn't use Move Group",
    response_description="Returns saved 6dof trajectories in {id, name} format",
)
async def get():
    #TODO add response model to request below
    trajectories = rospy.get_param("/arm/trajectories", [])
    rospy.logerr(trajectories)
    return trajectories


@trajectory_router.put(
    "/execute", name="Sends request to execute trajectory with specified id"
)
async def put(name: str):
    client.send_goal(TrajectoryGoal(goal_name=name))


@trajectory_router.put("/abort", name="Sends request to abort all queued trajectories")
async def put():
    client.cancel_all_goals()


@trajectory_router.put("/new_trajectory", name="Creates new, empty trajectory")
async def put(name: str):
    new_trajectory_publisher.publish(name)


@trajectory_router.put(
    "/add_point", name="Adds current arm position to specified trajectory"
)
async def put(name: str):
    add_point_publisher.publish(name)
