import rclpy
from rclpy.node import Node
import json
import websockets
import asyncio
from dataclasses import asdict

from .subscriber import Subscriber
from .publisher import Publisher
from .message import Message
from .server_node import ServerNode


HOST = "localhost"
PORT = 8001


class Server:
    """
    This class is a websocket server. It is responsible for communication between ROS2 and
    websocket server. It creates a websocket server which receives messages from websocket
    client and sends them to ROS2. It also creates a ROS2 publisher which sends messages
    to websocket server.

    Example:
    ```python
    node = ServerNode()
    node.publish_to_websocket("/topic", String) # creates a ROS2 subscriber which sends messages to websocket server
    node.publish_from_websocket("/topic", String) # creates a ROS2 publisher which receives messages from websocket server

    ros_node = Node()
    server = Server(ros_node)
    server.include(node)

    server.run()
    ```
    """
    def __init__(self, node: Node) -> None:
        self.connections = set()
        self.subscribers = set()
        self.publishers = {}
        self.loop = None
        self.node = node

    def subscribe(self, topic, type):
        subscriber = Subscriber(self.node, topic, type, self._send)
        self.subscribers.add(subscriber)

    def publish(self, topic, type):
        publisher = Publisher(self.node, topic, type)
        self.publishers[topic] = publisher

    def include(self, service_node: ServerNode):
        for topic, type in service_node.get_publishers():
            self.publish(topic, type)

        for topic, type in service_node.get_subscribers():
            self.subscribe(topic, type)

    def _send(self, msg):
        if not self.loop:
            return 

        async def send_async(message):
            for connection in self.connections:
                await connection.send(json.dumps(asdict(message)))

        asyncio.run_coroutine_threadsafe(send_async(msg), self.loop)

    def run(self):
        async def _connection_handler(websocket):
            self.connections.add(websocket)
            try:
                while True:
                    raw = await websocket.recv()
                    message = Message(**json.loads(raw))
                    loop = asyncio.get_event_loop()
                    try:
                        loop.run_in_executor(
                            None, self.publishers[message.topic].publish, message
                        )
                    except KeyError as e:
                        self.node.get_logger().warn(
                            f"[Websocket] Received topic without assigned publisher: \n{e}"
                        )
            except Exception as e:
                self.node.get_logger().warn(f"[Websocket] Connection terminated: \n{e}")
            finally:
                self.connections.remove(websocket)

        async def _async_run():
            self.loop = asyncio.get_running_loop()

            async with websockets.serve(_connection_handler, HOST, PORT):
                while rclpy.ok():
                    await asyncio.sleep(1)

        asyncio.run(_async_run())
