from enum import Enum
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Empty
import struct

# from example_interfaces.msg import
from kalman_interfaces.msg import MasterMessage, ScienceElement, ScienceButton
from kalman_interfaces.srv import GetScienceElements, RequestMagnetoTare


class ScienceElementType(Enum):
    DISPLAY = 0
    CONTAINER = 1
    PLAYER = 2
    STATUS = 3
    NONE = 4


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
                        "message": eval(button.get("message", "''")),
                        "data": button.get("data", None),
                        "custom": button.get("custom", False),
                        "custom_sender": eval(button.get("custom_sender", "''")),
                    }
                )

            if eval(element.get("type")).value is ScienceElementType.DISPLAY.value:
                if element.get("type_data", None) is None:
                    continue
                display_list.append(
                    {
                        "type": element.get("type_data").get("type", None),
                        "master_topic": element.get("type_data").get("topic", None),
                        "topic": f"{parent_id}_display",
                        "data_prefix": element.get("type_data").get("data_prefix", []),
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
                        "data_prefix": element.get("type_data").get("data_prefix", []),
                        "parser": eval(element.get("type_data").get("parser")),
                    }
                )

        # Create topics sending data to frontend
        data_list = display_list + status_list

        for topic_data in data_list:
            if topic_data.get("master_topic") is None:
                continue
            if topic_data.get("type", None) in [None, "MasterMessage"]:
                self.create_subscription(
                    MasterMessage,
                    topic_data.get("master_topic"),
                    self.handle_data_topic(topic_data),
                    10,
                )
            else:
                self._create_custom_subscriber(topic_data)

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
            data_prefix = self._convert_data_to_integers(topic_data.get("data_prefix"))

            if data_prefix == msg.data[: len(data_prefix)].tolist():
                data = msg.data[len(topic_data.get("data_prefix")) :]
                topic_data.get("parser")(data, topic_data, self)

        return callback

    def handle_empty_topic(self, topic_data: dict):
        def callback(msg: Empty):
            if topic_data.get("custom", False):
                topic_data.get("custom_sender")(topic_data, self)

            else:
                data_to_send = self._convert_data_to_integers(
                    topic_data.get("data", [])
                )
                message = MasterMessage(
                    cmd=topic_data.get("message"),
                    data=data_to_send,
                )
                self.science_pub.publish(message)

        return callback

    def handle_custom_topic(self, topic_data: dict):
        def callback(msg):
            topic_data.get("parser")(msg, topic_data, self)

        return callback

    def _convert_data_to_integers(self, data):
        data_int = []
        for byte in data:
            if isinstance(byte, int):
                data_int.append(byte)
            elif isinstance(byte, str) and len(byte) == 1:
                data_int.append(ord(byte))
            else:
                self.get_logger().error(f"Unsupported data type: {type(byte)}")

        return data_int

    def _create_custom_subscriber(self, topic_data):
        msg_type = self._get_msg_type(topic_data.get("type"))
        self.create_subscription(
            msg_type,
            topic_data.get("master_topic"),
            self.handle_custom_topic(topic_data),
            10,
        )

    def _get_msg_type(self, type_path):
        path_items = type_path.split("/")
        package = ".".join(path_items[:-1])
        class_name = path_items[-1]

        # Import the interface and save it in the cache.
        module = __import__(package, fromlist=[class_name])
        return getattr(module, class_name)


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


def send_magneto(data, topic_data, node):
    publisher = node.create_publisher(
        String, f"science_panel/{topic_data.get('topic')}", 10
    )
    msg = String()
    msg.data = f"{data.length:.2f} / {data.percentage:.2f}"
    publisher.publish(msg)


def magneto_tare_new(topic_data, node: Node):
    publisher = node.create_client(RequestMagnetoTare, "magneto/request_tare")
    request = RequestMagnetoTare.Request()
    request.type = RequestMagnetoTare.Request.NEW
    publisher.call_async(request)


def magneto_tare_reset(topic_data, node: Node):
    publisher = node.create_client(RequestMagnetoTare, "magneto/request_tare")
    request = RequestMagnetoTare.Request()
    request.type = RequestMagnetoTare.Request.RESET
    publisher.call_async(request)


def get_pump_status(data, topic_data, node):
    publisher = node.create_publisher(
        String, f"science_panel/{topic_data.get('topic')}", 10
    )
    pump_desired, pump_current = f"{data[0]:08b}"[6:8]

    msg = String()
    msg.data = str(pump_desired) + "/" + str(pump_current)
    publisher.publish(msg)


def get_heater_status(data, topic_data, node):
    publisher = node.create_publisher(
        String, f"science_panel/{topic_data.get('topic')}", 10
    )
    heater_desired, heater_current = f"{data[0]:08b}"[2:4]
    heater_temp = struct.unpack("<h", data[1:3])[0]

    msg = String()
    msg.data = (
        str(heater_desired)
        + "/"
        + str(heater_current)
        + " : "
        + str(heater_temp)
        + "°C"
    )
    publisher.publish(msg)


def get_peltier_status(data, topic_data, node):
    publisher = node.create_publisher(
        String, f"science_panel/{topic_data.get('topic')}", 10
    )
    peltier_desired, peltier_current = f"{data[0]:08b}"[4:6]
    peltier_temp_cold = struct.unpack("<h", data[5:7])[0]
    peltier_temp_hot = struct.unpack("<h", data[7:9])[0]

    msg = String()
    msg.data = (
        str(peltier_desired)
        + "/"
        + str(peltier_current)
        + ": cold/hot "
        + str(peltier_temp_cold)
        + "/"
        + str(peltier_temp_hot)
        + "°C"
    )
    publisher.publish(msg)


def get_stacjolab_status(data, topic_data, node):
    publisher = node.create_publisher(
        String, f"science_panel/{topic_data.get('topic')}", 10
    )
    motor_desired, motor_current = f"{data[0]:08b}"[6:8]
    chamber_temp = struct.unpack("<h", data[3:5])[0]
    battery_voltage = float(struct.unpack("<h", data[9:11])[0]) / 100.0
    weight = struct.unpack("<i", data[11:15])[0]  # in grams

    msg = String()
    msg.data = f"{motor_desired}/{motor_current} : {chamber_temp}°C : {battery_voltage:.2f}V : {weight}g"
    publisher.publish(msg)
