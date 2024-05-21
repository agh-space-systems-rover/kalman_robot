from fastapi import APIRouter
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
# from arm_trajectories.msg import TrajectoryAction, TrajectoryGoal
# import actionlib

class TrajectoriesAPI(APIRouter):
    def __init__(self, parent_node: Node):
        super().__init__(prefix="/trajectories", tags=["trajectories"])
        self.parent_node = parent_node

        self.new_trajectory_publisher =  self.parent_node.create_publisher(
            String, "/arm_controllers/trajectory/new_trajectory", qos_profile=10
        )


        self.add_point_publisher = self.parent_node.create_publisher(
            String, "/arm_controllers/trajectory/add_point", qos_profile=10
        )

        # Webserver stuff
        self.add_api_route(
            "/",
            self.get, # Callable
            name="Handles new trajectory system that doesn't use Move Group",
            # response_model=ArmState,
            response_description="Returns saved 6dof trajectories in {id, name} format",
            methods=["GET"],
        )

        self.add_api_route(
            "/execute",
            self.execute, # Callable
            name="Sends request to execute trajectory with specified id",
            methods=["PUT"],
        )

        self.add_api_route(
            "/abort",
            self.abort, # Callable
            name="Sends request to abort all queued trajectories",
            methods=["PUT"],
        )

        self.add_api_route(
            "/new_trajectory",
            self.new_trajectory, # Callable
            name="Creates new, empty trajectory",
            methods=["PUT"],
        )

        self.add_api_route(
            "/add_point",
            self.add_point, # Callable
            name="Adds current arm position to specified trajectory",
            methods=["PUT"],
        )


    def get(self):
        #TODO add response model to request below
        # trajectories = rclpy.get_param("/arm/trajectories", [])
        # rospy.logerr(trajectories)
        # return trajectories
        return []

    def execute(self, name: str):
        # client.send_goal(TrajectoryGoal(goal_name=name))
        return

    def abort(self):
        # client.cancel_all_goals()
        return

    def new_trajectory(self, name: str):
        # new_trajectory_publisher.publish(name)
        return

    def add_point(self, name: str):
        # add_point_publisher.publish(name)
        return




# trajectory_router = APIRouter(prefix="/trajectories", tags=["trajectories"])

# trajectory action client
# client = actionlib.ActionClient("/arm_controllers/trajectory", TrajectoryAction)

# new_trajectory_publisher = rospy.Publisher(
#     "/arm_controllers/trajectory/new_trajectory", String, queue_size=10
# )

# add_point_publisher = rospy.Publisher(
#     "/arm_controllers/trajectory/add_point", String, queue_size=10
# )


# @trajectory_router.get(
#     "/",
#     name="Handles new trajectory system that doesn't use Move Group",
#     response_description="Returns saved 6dof trajectories in {id, name} format",
# )
# async def get():
#     #TODO add response model to request below
#     trajectories = rospy.get_param("/arm/trajectories", [])
#     rospy.logerr(trajectories)
#     return trajectories


# @trajectory_router.put(
#     "/execute", name="Sends request to execute trajectory with specified id"
# )
# async def put(name: str):
#     client.send_goal(TrajectoryGoal(goal_name=name))


# @trajectory_router.put("/abort", name="Sends request to abort all queued trajectories")
# async def put():
#     client.cancel_all_goals()


# @trajectory_router.put("/new_trajectory", name="Creates new, empty trajectory")
# async def put(name: str):
#     new_trajectory_publisher.publish(name)


# @trajectory_router.put(
#     "/add_point", name="Adds current arm position to specified trajectory"
# )
# async def put(name: str):
#     add_point_publisher.publish(name)
