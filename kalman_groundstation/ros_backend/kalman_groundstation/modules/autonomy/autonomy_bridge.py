# from utils.logger import GpsLogger
from sensor_msgs.msg import Imu as ImuMsg, NavSatFix
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import Vector3, Pose
from threading import Lock
from std_msgs.msg import Int8, Float32MultiArray, String
# from supervisor.msg import SupervisorStatus
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatusArray
from rclpy.duration import Duration
# from united_supervising_protocol.msg import USPConfig
# from big_autonomy_translator.msg import GpsTagArray
import json
from rclpy.node import Node

initial_state = {
    "imu": {
        "alfa": 0,
        "beta": 0,
        "gamma": 0,
    },
    "imuRaw": {
        "alfa": 0,
        "beta": 0,
        "gamma": 0,
    },
    "ueuos": "unknown",
    "gps": {
        "position": {
            "latitude": None,
            "longitude": None,
            "altitude": None
        },
    },
    "odom": {
        "position": {
            "x": None,
            "y": None,
            "z": None,
        },
    },
    "map": {
        "position": {
            "x": None,
            "y": None,
            "z": None,
        },
    },
    "supervisorState": "DEAD",
    "tags": [],
    "usp": None,
    "moveBaseStatus": None,
}


class AutonomyBridge:
    def __init__(self, parent_node: Node):
        self.parent_node = parent_node
        self.state = initial_state
        self.timestamp = parent_node.get_clock().now()
        self.lock = Lock()

        # self.gps_logger = GpsLogger('gps_path')

        self.imu_subscriber = parent_node.create_subscription(ImuMsg, "/imu/data", self._update_imu, qos_profile=10)

        self.usuos_subscriber = parent_node.create_subscription(
            Int8, "/ueuos/state", self._update_ueuos, qos_profile=10
        )

        self.gps_subscriber = parent_node.create_subscription(
            NavSatFix, "/gps/fix", self._update_gps, qos_profile=10
        )

        # self.supervisor_subscriber = parent_node.create_subscription(
        #     "/supervisor/status", SupervisorStatus, self._update_supervisor
        # )

        self.odometry_subscriber = parent_node.create_subscription(
            Odometry, "/odometry/filtered", self._update_odometry, qos_profile=10
        )

        # self.rover_map_pose_subscriber = parent_node.create_subscription(
        #     Pose, "/gs_rover_map_pose", self._update_rover_map_pose, qos_profile=10
        # )

        # self.supervisor_subscriber = parent_node.create_subscription(
        #     Float32MultiArray, "/supervisor/waypoints", self._update_supervisor, qos_profile=10
        # )

        # self.usp_subscriber = parent_node.create_subscription(
        #     "/usp/get/response", USPConfig, self._update_usp
        # )

        self.move_base_subscriber = parent_node.create_subscription(
            GoalStatusArray, "/move_base/status", self._update_move_base, qos_profile=10
        )

        self.ws_publisher = parent_node.create_publisher(
            String, "/station/autonomy/state", qos_profile=10
        )

    def _publish(self):
        self.lock.acquire()
        current_time = self.parent_node.get_clock().now()

        if current_time - self.timestamp > Duration(seconds=1):
            self.timestamp = current_time
            msg = String(data=json.dumps(self.state))
            self.ws_publisher.publish(msg)

        self.lock.release()

    def _update_imu(self, msg: ImuMsg):
        o = msg.orientation
        alpha, beta, gamma = euler_from_quaternion((o.x, o.y, o.z, o.w))
        imu = {
            "alpha": alpha,
            "beta": beta,
            "gamma": gamma,
        }
        self.state["imu"] = imu
        self.state["imuRaw"] = imu
        self._publish()

    def _update_ueuos(self, msg: Int8):
        mapping = {0: "OFF", 1: "AUTO", 2: "MANUAL", 3: "COMPLETED"}

        self.state["ueuos"] = mapping[msg.data]
        self._publish()

    def _update_gps(self, msg: Vector3):
        self.state["gps"] = {
            "position": {
                "latitude": msg.latitude,
                "longitude": msg.longitude,
                "altitude": msg.altitude,
            },
        }
        # self.gps_logger.log(msg)
        self._publish()

    # def _update_supervisor(self, msg: SupervisorStatus):
    #     mapping = {
    #         0: "MANUAL",
    #         1: "IDLE",
    #         2: "TRAVEL",
    #         3: "APPROACH",
    #         4: "EXPLORE",
    #         5: "LOCATE",
    #         6: "CROSS",
    #         7: "COMPLETED",
    #     }

    #     try:
    #         self.state["supervisorState"] = mapping[msg.status]
    #     except KeyError:
    #         # rospy.logerr("Received invalid supervisor state")
    #         self.parent_node.get_logger().error("Received invalid supervisor state")
    #     self._publish()

    def _update_odometry(self, msg: Odometry):
        pos = msg.pose.pose.position
        self.state["odom"] = {"position": {"x": pos.x, "y": pos.y, "z": pos.z}}
        self._publish()

    def _update_rover_map_pose(self, msg: Pose):
        pos = msg.position
        self.state["map"] = {"position": {"x": pos.x, "y": pos.y, "z": pos.z}}
        self._publish()

    # def _update_tags(self, msg: GpsTagArray):
    #     tags = []
    #     for tag in msg.tags:
    #         tags.append({"id": tag.tag_id, "lat": tag.tag_lat, "lon": tag.tag_lon})

    #     self.state["tags"] = tags
    #     self._publish()

    # def _update_usp(self, msg: USPConfig):
    #     usp = {
    #         "x": msg.x,
    #         "y": msg.y,
    #         "autonomousDriving": msg.autonomous_driving,
    #         "goalSet": msg.goal_set,
    #         "multipleWaypoints": msg.multiple_waypoints,
    #         "frame": msg.frame,
    #         "mode": msg.mode,
    #         "tag1": msg.tag1,
    #         "tag2": msg.tag2,
    #         "args": msg.args,
    #     }
    #     self.state["usp"] = usp
    #     self._publish()

    def _update_move_base(self, msg: GoalStatusArray):
        if msg.status_list:
            self.state["moveBaseStatus"] = msg.status_list[-1].status
            self._publish()
