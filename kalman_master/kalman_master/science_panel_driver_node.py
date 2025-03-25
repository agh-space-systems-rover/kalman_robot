from enum import Enum
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Empty

# from example_interfaces.msg import
from kalman_interfaces.msg import MasterMessage, ScienceElement, ScienceButton
from kalman_interfaces.srv import GetScienceElements


class ScienceElementType(Enum):
    DISPLAY = 0
    CONTAINER = 1
    PLAYER = 2
    STATUS = 3


class SciencePanelDriver(Node):
    def __init__(self):
        super().__init__("science_panel_driver")
        filename = self.declare_parameter(
            "config_path", "config/science_panel_config.yaml"
        ).value

        with open(filename, "r") as f:
            self.science_config = yaml.safe_load(f)

        # Init master publisher.
        self.science_pub = self.create_publisher(
            MasterMessage, "master_com/ros_to_master", 10
        )

        # Create service to serve configuration
        self.create_service(
            GetScienceElements, "science_panel/get_config", self.get_science_elements
        )

        # Create topic arrays
        display_list = []
        container_list = []
        player_list = []
        status_list = []
        button_list = []
        for element in self.science_config.get("elements", []):
            parent_id = element.get("id")
            for button in element.get("buttons", []):
                button_list.append(
                    {
                        "topic": f"{parent_id}_{button.get('id')}",
                        "message": eval(button.get("message")),
                        "data": button.get("data", None),
                    }
                )

            if eval(element.get("type")).value is ScienceElementType.DISPLAY.value:
                if element.get("type_data", None) is None:
                    continue
                display_list.append(
                    {
                        "master_topic": element.get("type_data").get("topic", None),
                        "topic": f"{parent_id}_display",
                        "data_prefix": element.get("type_data").get("data_prefix"),
                        "parser": eval(element.get("type_data").get("parser")),
                    }
                )

            if eval(element.get("type")).value is ScienceElementType.CONTAINER.value:
                container_list.append(
                    {
                        "topic": f"{parent_id}_open_container",
                        "message": eval(element.get("type_data").get("open_message")),
                        "data": element.get("type_data").get("open_data", None),
                    }
                )

                container_list.append(
                    {
                        "topic": f"{parent_id}_close_container",
                        "message": eval(element.get("type_data").get("close_message")),
                        "data": element.get("type_data").get("close_data", None),
                    }
                )

            if eval(element.get("type")).value is ScienceElementType.PLAYER.value:
                player_list.append(
                    {
                        "topic": f"{parent_id}_player_play",
                        "message": eval(element.get("type_data").get("play_message")),
                        "data": element.get("type_data").get("play_data", None),
                    }
                )

                player_list.append(
                    {
                        "topic": f"{parent_id}_player_pause",
                        "message": eval(element.get("type_data").get("pause_message")),
                        "data": element.get("type_data").get("pause_data", None),
                    }
                )

                player_list.append(
                    {
                        "topic": f"{parent_id}_player_stop",
                        "message": eval(element.get("type_data").get("stop_message")),
                        "data": element.get("type_data").get("stop_data", None),
                    }
                )

            if eval(element.get("type")).value is ScienceElementType.STATUS.value:
                status_list.append(
                    {
                        "master_topic": element.get("type_data").get("topic", None),
                        "topic": f"{parent_id}_status",
                        "data_prefix": element.get("type_data").get("data_prefix"),
                        "parser": eval(element.get("type_data").get("parser")),
                    }
                )

        # Create topics sending data to frontend
        data_list = display_list + status_list

        for topic_data in data_list:
            if topic_data.get("master_topic") is None:
                continue
            self.create_subscription(
                MasterMessage,
                topic_data.get("master_topic"),
                self.handle_data_topic(topic_data),
                10,
            )

        # Create topics with empty data
        empty_topics = button_list + container_list + player_list
        for topic_data in empty_topics:
            self.create_subscription(
                Empty,
                f"science_panel/{topic_data.get('topic')}",
                self.handle_empty_topic(topic_data),
                10,
            )

    def get_science_elements(
        self, request: GetScienceElements.Request, response: GetScienceElements.Response
    ):
        elements = []

        for element in self.science_config.get("elements", []):
            science_element_buttons = []
            for button in element.get("buttons", []):
                science_element_buttons.append(
                    ScienceButton(
                        id=button.get("id"),
                        name=button.get("name"),
                        color=button.get("color", ""),
                    )
                )

            elements.append(
                ScienceElement(
                    id=element.get("id"),
                    type=eval(element.get("type")).value,
                    display_name=element.get("display_name", ""),
                    buttons=science_element_buttons,
                    color=element.get("type_data", {}).get("color", ""),
                )
            )

        response.enable = self.science_config.get("enable", False)
        response.science_elements = elements

        return response

    def handle_data_topic(self, topic_data: dict):
        def callback(msg: MasterMessage, topic_data=topic_data):
            data_prefix = topic_data.get("data_prefix")
            if data_prefix == msg.data[: len(data_prefix)].tolist():
                data = msg.data[len(topic_data.get("data_prefix")) :]
                topic_data.get("parser")(data, topic_data, self)

        return callback

    def handle_empty_topic(self, topic_data: dict):
        def callback(msg: Empty):
            data_to_send = []
            for byte in topic_data.get("data"):
                if isinstance(byte, int):
                    data_to_send.append(byte)
                elif isinstance(byte, str) and len(byte) == 1:
                    data_to_send.append(ord(byte))
                else:
                    self.get_logger().error(f"Unsupported data type: {type(byte)}")
            message = MasterMessage(
                cmd=topic_data.get("message"),
                data=data_to_send,
            )
            self.science_pub.publish(message)

        return callback


def main():
    try:
        rclpy.init()
        node = SciencePanelDriver()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass


### PARSER FUNCTIONS
def send_weight(data, topic_data, node):
    publisher = node.create_publisher(
        String, f"science_panel/{topic_data.get('topic')}", 10
    )
    msg = String()
    msg.data = "".join([str(x) for x in data])
    publisher.publish(msg)


def send_status(data, topic_data, node):
    publisher = node.create_publisher(
        Bool, f"science_panel/{topic_data.get('topic')}", 10
    )
    msg = Bool()
    msg.data = bool(data)
    publisher.publish(msg)


def parse_stacjolab_status(data, topic_data, node):
    publisher = node.create_publisher(
        String, f"science_panel/{topic_data.get('topic')}", 10
    )
    msg = String()
    msg.data = "".join([str(x) for x in data])
    publisher.publish(msg)
