from core import ServerNode
from std_msgs.msg import String

PREFIX = "/access_point"

server_node = ServerNode()
server_node.publish_from_websocket(PREFIX + "/connect", String)
server_node.publish_from_websocket(PREFIX + "/transfer", String)
server_node.publish_to_websocket(PREFIX + "/webpage/joined", String)
server_node.publish_to_websocket(PREFIX + "/status", String)