import rclpy
from rclpy.lifecycle import TransitionCallbackReturn

from yolo_ros.yolo_detect_node import YOLODetect


def main():
    try:
        rclpy.init()

        node = YOLODetect()
        result = node.trigger_activate()
        if result != TransitionCallbackReturn.SUCCESS:
            node.get_logger().error(f"Failed to auto-activate.")

        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
