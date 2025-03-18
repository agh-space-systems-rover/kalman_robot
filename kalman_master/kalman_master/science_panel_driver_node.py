from enum import Enum
import yaml
import rclpy
from rclpy.node import Node
from kalman_interfaces.msg import MasterMessage, ScienceElement, ScienceButton
from kalman_interfaces.srv import GetScienceElements


class ScienceElementType(Enum):
    DISPLAY = 0
    CONTAINER = 1
    PLAYER = 2


class SciencePanelDriver(Node):
    def __init__(self):
        super().__init__("science_panel_driver")
        filename = self.declare_parameter("config_path", "").value

        with open(filename, "r") as f:
            self.science_config = yaml.safe_load(f)

        # Init master publisher.
        self.ueuos_pub = self.create_publisher(
            MasterMessage, "master_com/ros_to_master", 10
        )

        # Create service to serve configuration
        self.create_service(GetScienceElements, "science_panel/get_config", self.get_science_elements)

        # Create buttons array
        button_list = []
        for element in self.science_config.get("elements", []):
            parent_id = element.get("id")
            for button in element.get("buttons", []):
                button_list.append({
                    "topic": f"{parent_id}-{button.get('id')}",
                    "message": str(button.get("message", "")),
                    "data": button.get("data", None)
                })

        # Create containers array

        # Create players array

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


def main():
    try:
        rclpy.init()
        node = SciencePanelDriver()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
