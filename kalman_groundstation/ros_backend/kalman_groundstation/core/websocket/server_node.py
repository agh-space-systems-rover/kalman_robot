class ServerNode:
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
