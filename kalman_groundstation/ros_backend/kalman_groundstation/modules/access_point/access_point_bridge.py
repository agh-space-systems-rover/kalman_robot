import rospy
from std_msgs.msg import String

PREFIX = "/access_point"


class AccessPointBridge:
    def __init__(self) -> None:
        self.content_sub = rospy.Subscriber(
            PREFIX + "/webpage", String, self._content_cb
        )

        self.joined_content_pub = rospy.Publisher(
            PREFIX + "/webpage/joined", String, queue_size=10
        )

        self.connect_sub = rospy.Subscriber(
            PREFIX + "/connect", String, self._connect_sub
        )

        self.content = set()

    def _content_cb(self, msg):
        self.content.add(msg.data)
        self.upload_content()

    def _connect_sub(self, msg):
        self.content.clear()
        self.upload_content()

    def upload_content(self):
        msg = String()
        msg.data = "\n".join(list(self.content))
        self.joined_content_pub.publish(msg)
