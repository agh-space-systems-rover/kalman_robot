import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import threading
import time
import json
import websocket
import importlib

from rosbridge_library.internal.message_conversion import extract_values


def resolve_ros_type(type_param):
    """
    Convert a ROS type string like 'std_msgs/msg/Header' to a Python import path and class name.
    Returns (module_path, class_name)
    """
    parts = type_param.split("/")
    if len(parts) == 3:
        pkg, subdir, msg = parts
        module_path = f"{pkg}.{subdir.lower()}"
        class_name = msg
        return module_path, class_name
    elif len(parts) == 2:
        pkg, msg = parts
        module_path = f"{pkg}.msg"
        class_name = msg
        return module_path, class_name
    else:
        raise ValueError(f"Invalid ROS type string: {type_param}")


def import_ros_message_type(type_param):
    module_path, class_name = resolve_ros_type(type_param)
    module = importlib.import_module(module_path)
    msg_class = getattr(module, class_name)
    return msg_class


class RosbridgeWsBridge(Node):
    def __init__(self):
        super().__init__("rosbridge_ws_bridge")
        self.declare_parameter("ws_address", "localhost:9090")
        self.declare_parameter("topic", "/chatter")
        self.declare_parameter("mode", "send")  # 'send' or 'recv'
        self.declare_parameter(
            "type", "std_msgs/msg/String"
        )  # Now used for dynamic import

        self.ws_address = (
            self.get_parameter("ws_address").get_parameter_value().string_value
        )
        self.topic = self.get_parameter("topic").get_parameter_value().string_value
        self.mode = (
            self.get_parameter("mode").get_parameter_value().string_value.lower()
        )
        self.type_param = self.get_parameter("type").get_parameter_value().string_value

        try:
            self.msg_class = import_ros_message_type(self.type_param)
        except Exception as e:
            self.get_logger().error(
                f"Could not import message type '{self.type_param}': {e}"
            )
            raise

        self.ws_url = f"ws://{self.ws_address}"
        self.ws = None
        self.connected_event = threading.Event()

        if self.mode == "send":
            self.subscription = self.create_subscription(
                self.msg_class, self.topic, self.ros_callback, 10
            )
            self.ws_thread = threading.Thread(target=self.ws_send_loop)
        elif self.mode == "recv":
            self.publisher = self.create_publisher(self.msg_class, self.topic, 10)
            self.ws_thread = threading.Thread(target=self.ws_recv_loop)
        else:
            self.get_logger().error(f"Unknown mode: {self.mode}")
            return

        self.ws_thread.start()

    def ros_callback(self, msg):
        if self.ws and self.ws.connected:
            # Convert ROS message to dict for JSON serialization
            msg_dict = extract_values(msg)
            pub_msg = {"op": "publish", "topic": self.topic, "msg": msg_dict}
            self.ws.send(json.dumps(pub_msg))

    def ws_send_loop(self):
        while rclpy.ok():
            try:
                self.ws = websocket.WebSocket()
                self.ws.connect(self.ws_url)

                self.connected_event.set()
                self.get_logger().info("Connected to WebSocket")

                # Advertise topic on rosbridge
                adv_msg = {
                    "op": "advertise",
                    "topic": self.topic,
                    "type": self.type_param,
                }
                self.ws.send(json.dumps(adv_msg))

                # Keep connection alive until ROS shuts down or socket errors
                while rclpy.ok():
                    if not self.ws.connected:
                        raise Exception("WebSocket disconnected")
                    time.sleep(0.1)  # avoid busy loop

            except Exception as e:
                self.get_logger().error(f"WebSocket error: {e}")
                self.connected_event.clear()
                self.get_logger().warn(
                    "WebSocket disconnected, will reconnect in 10 seconds..."
                )
                for _ in range(10):
                    if not rclpy.ok():
                        break
                    time.sleep(0.1)

    def ws_recv_loop(self):
        def on_message(ws, message):
            msg_obj = json.loads(message)
            if (
                msg_obj.get("op") == "publish"
                and msg_obj.get("topic") == self.topic
            ):
                ros_msg = self.dict_to_ros_msg(msg_obj["msg"], self.msg_class)
                self.publisher.publish(ros_msg)

        def on_open(ws):
            self.connected_event.set()
            self.get_logger().info("Connected to WebSocket")
            sub_msg = {"op": "subscribe", "topic": self.topic, "type": self.type_param}
            ws.send(json.dumps(sub_msg))

        def on_error(ws, error):
            if rclpy.ok():
                self.get_logger().error(f"WebSocket error: {error}")
            ws.close()

        def monitor_sigint():
            while rclpy.ok():
                time.sleep(0.1)
            if self.ws:
                self.ws.close()
        self.recv_sigint_monitor_thread = threading.Thread(target=monitor_sigint)
        self.recv_sigint_monitor_thread.start()

        while rclpy.ok():
            self.ws = websocket.WebSocketApp(
                self.ws_url,
                on_open=on_open,
                on_message=on_message,
                on_error=on_error,
            )
            self.ws.run_forever()

            if rclpy.ok():
                self.get_logger().warn(
                    "WebSocket disconnected, will reconnect in 10 seconds..."
                )
                for _ in range(10):
                    if not rclpy.ok():
                        break
                    time.sleep(0.1)

    def dict_to_ros_msg(self, data, msg_class):
        # Construct ROS msg from dict recursively
        msg = msg_class()
        for field in msg_class.get_fields_and_field_types():
            if field in data:
                value = data[field]
                field_type = msg_class.get_fields_and_field_types()[field]
                # Handle nested messages
                if "/" in field_type and isinstance(value, dict):
                    nested_class = import_ros_message_type(field_type)
                    setattr(msg, field, self.dict_to_ros_msg(value, nested_class))
                else:
                    setattr(msg, field, self.set_nan_where_null(value))
        return msg

    def set_nan_where_null(self, data):
        if data == None:
            return float("nan")
        elif isinstance(data, list):
            return [self.set_nan_where_null(v) for v in data]
        elif isinstance(data, dict):
            return {k: self.set_nan_where_null(v) for k, v in data.items()}
        else:
            return data

def main():
    try:
        rclpy.init()
        node = RosbridgeWsBridge()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
