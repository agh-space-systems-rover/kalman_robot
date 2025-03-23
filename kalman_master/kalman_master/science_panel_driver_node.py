from enum import Enum
import yaml
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Empty
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
        filename = self.declare_parameter("config_path", "").value

        with open(filename, "r") as f:
            self.science_config = yaml.safe_load(f)

        # Init master publisher.
        self.science_pub = self.create_publisher(
            MasterMessage, "master_com/ros_to_master", 10
        )

        # Create service to serve configuration
        self.create_service(GetScienceElements, "science_panel/get_config", self.get_science_elements)

        # Create topic arrays
        button_list = []
        container_list = []
        player_list = []
        for element in self.science_config.get("elements", []):
            parent_id = element.get("id")
            for button in element.get("buttons", []):
                button_list.append({
                    "topic": f"{parent_id}_{button.get('id')}",
                    "message": eval(button.get("message")),
                    "data": button.get("data", None)
                })

            if eval(element.get("type")).value is ScienceElementType.CONTAINER:
                container_list.append({
                    "topic": f"{parent_id}_open_container",
                    "message": eval(element.get("type_data").get("open_message")),
                    "data": element.get("type_data").get("open_data", None)
                })

                container_list.append({
                    "topic": f"{parent_id}_close_container",
                    "message": eval(element.get("type_data").get("close_message")),
                    "data": element.get("type_data").get("close_data", None)
                })

            if eval(element.get("type")).value is ScienceElementType.PLAYER:
                player_list.append({
                    "topic": f"{parent_id}_player_play",
                    "message": eval(element.get("type_data").get("play_message")),
                    "data": element.get("type_data").get("play_data", None)
                })

                player_list.append({
                    "topic": f"{parent_id}_player_pause",
                    "message": eval(element.get("type_data").get("pause_message")),
                    "data": element.get("type_data").get("pause_data", None)
                })

                player_list.append({
                    "topic": f"{parent_id}_player_stop",
                    "message": eval(element.get("type_data").get("stop_message")),
                    "data": element.get("type_data").get("stop_data", None)
                })

        # Create topics with empty data
        all_topics = button_list + container_list + player_list
        for topic_data in all_topics:
            self.create_subscription(
                Empty, f"science_panel/{topic_data.get('topic')}", self.handle_element_topic(topic_data), 10
            )

    def get_science_elements(self, request: GetScienceElements.Request, response: GetScienceElements.Response):
        elements = []

        for element in self.science_config.get("elements", []):
            science_element_buttons = []
            for button in self.science_config.get("buttons", []):
                science_element_buttons.append(ScienceButton(
                    id=button.get("id"),
                    name=button.get("name"),
                    color=button.get("color", ""),
                ))

            elements.append(ScienceElement(
                id=element.get("id"),
                type=eval(element.get("type")).value,
                display_name=element.get("display_name", ""),
                buttons=science_element_buttons,
                color=element.get("type_data").get("color", ""),
            ))

        response.enable = self.science_config.get("enable", False)
        response.science_elements = elements

        return response

    def handle_element_topic(self, topic_data: dict):
        def callback(msg: Empty):
            self.get_logger().info(f"Received on topic: {topic_data['topic']} with data: {topic_data['data']}")

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
