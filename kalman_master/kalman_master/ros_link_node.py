import yaml
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from threading import Lock

from kalman_interfaces.msg import MasterMessage

from kalman_master.ros_link_topics import RosLinkTopics
from kalman_master.ros_link_services import RosLinkServices
from kalman_master.ros_link_actions import RosLinkActions

# Unique ID that allows to identify ROS link messages in FORWARD frames.
FORWARD_ID = 47

from example_interfaces.action import Fibonacci
from sensor_msgs.msg import Imu
from vision_msgs.msg import Detection2DArray

a = Fibonacci.Result()
a.sequence
b = Imu()
b.orientation_covariance
c = Detection2DArray()
c.detections


class RosLink(Node):
    def __init__(self):
        super().__init__("ros_link")

        # Declare parameters.
        config_path = self.declare_parameter(
            "config_path",
            None,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Path to the config file.",
            ),
        ).value
        self.side = self.declare_parameter(
            "side",
            'rover',
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Side of communication. Must be 'rover' or 'station'.",
            ),
        ).value
        self.rover_endpoint = self.declare_parameter(
            "rover_endpoint",
            "pc",
            ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description="Use Arm<->GS or PC<->GS, etc. Must be either 'pc' or 'arm'.",
            ),
        ).value
        self.loopback_mangling = self.declare_parameter(
            "loopback_mangling",
            False,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description="Enable name mangling for loopback connections where both the publisher and subscriber are in the same ROS network. Names are mangled on the opposite side by appending a /loopback suffix. E.g. /imu/data published by the PC will be republished as /imu/data/loopback on the GS.",
            ),
        ).value
        self.debug_info = self.declare_parameter(
            "debug_info",
            False,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description="Log additional debug information on INFO level.",
            ),
        ).value

        # Load config.
        with open(config_path, "r") as file:
            self.config = yaml.safe_load(file)

        self.normalize_config()
        self.order_names()
        self.import_interfaces()

        # Create a reentrant callback group to enable parallel execution of callbacks.
        self.callback_group = ReentrantCallbackGroup()

        # Create master publisher.
        self.master_pub = self.create_publisher(
            MasterMessage, "master_com/ros_to_master", 10
        )
        self.master_pub_lock = Lock()

        # Create master subscriber.
        self.create_subscription(
            MasterMessage,
            f"master_com/master_to_ros/{hex(self.get_forward_to_this_side_frame_id())[1:]}",
            self.master_callback,
            10,
            callback_group=self.callback_group,
        )

        # Initialize modules.
        self.topics_module = RosLinkTopics(self)
        self.services_module = RosLinkServices(self)
        self.actions_module = RosLinkActions(self)

    # Normalize the config to make sure it has all required fields that future code depends on.
    # I.e. Insert default values into the config whenever a field is missing and more.
    def normalize_config(self) -> None:
        # Replace missing sides with empty objects.
        if "rover" not in self.config:
            self.config["rover"] = {}
        if "station" not in self.config:
            self.config["station"] = {}

        # Replace missing topics/services/actions with empty lists.
        for side in ["rover", "station"]:
            if "topics" not in self.config[side]:
                self.config[side]["topics"] = []
            if "services" not in self.config[side]:
                self.config[side]["services"] = []
            if "actions" not in self.config[side]:
                self.config[side]["actions"] = []

        # For each topic/service/action config...
        for side in ["rover", "station"]:
            for config in self.config[side]["topics"]:
                # Add a default empty field prefix if no fields are provided.
                # Empty prefix will match any field path.
                if len(config.get("fields", [])) == 0:
                    config["fields"] = [""]
                # Add missing max_rate.
                if "max_rate" not in config:
                    config["max_rate"] = 1
            for config in self.config[side]["services"]:
                if len(config.get("request_fields", [])) == 0:
                    config["request_fields"] = [""]
                if len(config.get("response_fields", [])) == 0:
                    config["response_fields"] = [""]
                # Add missing retry_rate.
                if "retry_rate" not in config:
                    config["retry_rate"] = 1
            for config in self.config[side]["actions"]:
                # Add a default empty field prefix if no fields are provided.
                # Empty prefix will match any field path.
                if len(config.get("goal_fields", [])) == 0:
                    config["goal_fields"] = [""]
                if len(config.get("feedback_fields", [])) == 0:
                    config["feedback_fields"] = [""]
                if len(config.get("result_fields", [])) == 0:
                    config["result_fields"] = [""]
                # Add missing rates.
                if "retry_rate" not in config:
                    config["retry_rate"] = 1
                if "max_feedback_rate" not in config:
                    config["max_feedback_rate"] = 1

        for side in ["rover", "station"]:
            for config in (
                self.config[side]["topics"]
                + self.config[side]["services"]
                + self.config[side]["actions"]
            ):
                for fields_key in [
                    "fields",
                    "request_fields",
                    "response_fields",
                    "goal_fields",
                    "feedback_fields",
                    "result_fields",
                ]:
                    if fields_key in config:
                        # Convert all string paths to objects.
                        for i, field in enumerate(config[fields_key]):
                            if isinstance(field, str):
                                config[fields_key][i] = {"path": field}

                        # If a common range or enum is present, insert it into objects with missing ranges.
                        for quant_key in ["int_range", "float_range", "string_enum"]:
                            if quant_key in config:
                                for field in config[fields_key]:
                                    if quant_key not in field:
                                        field[quant_key] = config[quant_key]

                        # For each field that contains string_enum dict (not list), add fallback=unknown if no fallback is specified.
                        for field in config[fields_key]:
                            if "string_enum" in field:
                                # If string_enum is a list, convert it to a dict with the list as values.
                                if type(field["string_enum"]) is list:
                                    field["string_enum"] = {"values": field["string_enum"]}
                                # If fallback is missing, add it.
                                if "fallback" not in field["string_enum"]:
                                    field["string_enum"]["fallback"] = "unknown"

                # Remove common ranges and string enums from the config.
                for quant_key in ["int_range", "float_range", "string_enum"]:
                    if quant_key in config:
                        del config[quant_key]

    # Get the config for topics published on this side and services provided by this side.
    # Equivalently the config for topics subscribed on the opposite side and services used by the opposite side.
    def get_this_side_config(self) -> dict:
        return self.config[self.side]

    # Get the config for topics subscribed on this side and services used by this side.
    # Equivalently the config for topics published on the opposite side and services provided by the opposite side.
    def get_opposite_side_config(self) -> dict:
        return self.config["rover" if self.side == "station" else "station"]

    # Get the ID of frames forwarded to this side.
    def get_forward_to_this_side_frame_id(self) -> int:
        if self.rover_endpoint == "arm":
            return (
                MasterMessage.ARM_TO_GS
                if self.side == "station"
                else MasterMessage.GS_TO_ARM
            )
        else:
            return (
                MasterMessage.PC_TO_GS
                if self.side == "station"
                else MasterMessage.GS_TO_PC
            )

    # Get the ID of frames forwarded to the opposite side.
    def get_forward_to_opposite_side_frame_id(self) -> int:
        if self.rover_endpoint == "arm":
            return (
                MasterMessage.GS_TO_ARM
                if self.side == "station"
                else MasterMessage.ARM_TO_GS
            )
        else:
            return (
                MasterMessage.GS_TO_PC
                if self.side == "station"
                else MasterMessage.PC_TO_GS
            )

    # Sort all names lexically so that each one has a forward ID - the index in the list.
    # Store the sorted names in self.ordered_names.
    def order_names(self) -> None:
        self.ordered_names = []
        for side_config in [
            self.get_this_side_config(),
            self.get_opposite_side_config(),
        ]:
            for topic_config in side_config["topics"]:
                if topic_config["name"] in self.ordered_names:
                    raise ValueError(
                        f"Duplicate topic name: {topic_config['name']}\nPlease make sure that each topic is configured only on one side."
                    )
                self.ordered_names.append(topic_config["name"])
            for service_config in side_config["services"]:
                if service_config["name"] in self.ordered_names:
                    raise ValueError(
                        f"Duplicate service name: {service_config['name']}\nPlease make sure that each service is configured only on one side."
                    )
                self.ordered_names.append(service_config["name"])
            for action_config in side_config["actions"]:
                if action_config["name"] in self.ordered_names:
                    raise ValueError(
                        f"Duplicate action name: {action_config['name']}\nPlease make sure that each action is configured only on one side."
                    )
                self.ordered_names.append(action_config["name"])
        self.ordered_names.sort()

    # Import all interfaces specified in the config.
    # Store the imported interfaces in self.imported_interfaces.
    def import_interfaces(self) -> None:
        self.imported_interfaces: dict[str, type] = {}
        for side_config in [
            self.get_this_side_config(),
            self.get_opposite_side_config(),
        ]:
            for config in (
                side_config["topics"] + side_config["services"] + side_config["actions"]
            ):
                type_path = config["type"]

                # example for type=std_msgs/msg/String:
                # from std_msgs.msg import String
                # return String

                # Split the name into package and type.
                path_items = type_path.split("/")
                package = ".".join(path_items[:-1])
                class_name = path_items[-1]

                # Import the interface and save it in the cache.
                module = __import__(package, fromlist=[class_name])
                self.imported_interfaces[type_path] = getattr(module, class_name)

    # Determine the type of a name (topic/service/action).
    def determine_name_type(self, name: str) -> str:
        for side_config in [
            self.get_this_side_config(),
            self.get_opposite_side_config(),
        ]:
            for config in side_config["topics"]:
                if config["name"] == name:
                    return "topic"
            for config in side_config["services"]:
                if config["name"] == name:
                    return "service"
            for config in side_config["actions"]:
                if config["name"] == name:
                    return "action"
        return ""

    # Called whenever a FORWARD frame is received from the master.
    # The frame might contain a topic message to publish or a service request/reqACK/response/resACK to handle.
    def master_callback(self, msg: MasterMessage) -> None:
        # Only consider frames that are longer than 2 bytes.
        if len(msg.data) < 2:
            return

        # Only consider frames with the correct forward ID.
        if msg.data[0] != FORWARD_ID:
            return

        # Read the ID.
        id = msg.data[1]

        # If the topic ID is out of bounds, ignore the message.
        if id < 0 or id >= len(self.ordered_names):
            self.get_logger().error(
                f"Master delivered a message with invalid topic/service ID: {id}"
            )
            return

        # Get the name of the channel and determine if it is a topic, a service or an action.
        name = self.ordered_names[id]
        name_type = self.determine_name_type(name)
        data = msg.data[2:]  # Skip forward ID and name ID.
        if name_type == "topic":
            self.topics_module.handle_topic_frame(name, data)
        elif name_type == "service":
            self.services_module.handle_service_frame(name, data)
        elif name_type == "action":
            self.actions_module.handle_action_frame(name, data)
        else:
            self.get_logger().error(
                f"Internal error! Name ID #{id} has no associated type."
            )

    # Lock the master publisher and send a message to the opposite side.
    def send_to_opposite_side(self, data: list[int]) -> None:
        with self.master_pub_lock:
            self.master_pub.publish(
                MasterMessage(
                    cmd=self.get_forward_to_opposite_side_frame_id(),
                    data=[FORWARD_ID, *data],
                )
            )


def main():
    try:
        rclpy.init()
        executor = MultiThreadedExecutor(num_threads=100)
        node = RosLink()
        executor.add_node(node)
        executor.spin()
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
