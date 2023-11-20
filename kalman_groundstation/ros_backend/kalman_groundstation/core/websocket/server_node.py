class ServerNode:
    """
    This class makes easy communication between ROS2 and websocket server. It is a nice
    interface for creating publishers and subscribers. 

    Example:
    ```python
    node = ServerNode()
    node.publish_to_websocket("/topic", String) # creates a ROS2 subscriber which sends messages to websocket server
    node.publish_from_websocket("/topic", String) # creates a ROS2 publisher which receives messages from websocket server
    ```
    """
    def __init__(self):
        self.subscribers = []
        self.publishers = []

    def publish_to_websocket(self, topic, type):
        self.subscribers.append((topic, type))

    def publish_from_websocket(self, topic, type):
        self.publishers.append((topic, type))

    def include(self, service_node):
        for topic, type in service_node.get_publishers():
            self.publishers.append((topic, type))

        for topic, type in service_node.get_subscribers():
            self.subscribers.append(topic, type)

    def get_publishers(self):
        return self.publishers

    def get_subscribers(self):
        return self.subscribers
