from core import ServerNode
from std_msgs.msg import String

server_node = ServerNode()
server_node.publish_to_websocket("/station/autonomy/state", String)
