import math, rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class LevelFrame(Node):
    def __init__(self):
        super().__init__("level_frame")
        self.buf = Buffer(); TransformListener(self.buf, self)
        self.br = TransformBroadcaster(self)
        self.create_timer(1/50, self.tick)

    def tick(self):
        try:
            t = self.buf.lookup_transform("odom", "base_link", rclpy.time.Time())
        except Exception:
            return
        q = t.transform.rotation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        out = TransformStamped()
        out.header.stamp = t.header.stamp
        out.header.frame_id = "odom"
        out.child_frame_id = "base_link_level"
        out.transform.translation = t.transform.translation
        out.transform.rotation.z = math.sin(yaw/2)
        out.transform.rotation.w = math.cos(yaw/2)
        self.br.sendTransform(out)


def main():
    try:
        rclpy.init()
        node = LevelFrame()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
