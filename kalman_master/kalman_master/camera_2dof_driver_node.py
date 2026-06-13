import rclpy
from rclpy.node import Node
from kalman_interfaces.msg import Camera2Dof, MasterMessage


def angle_to_uint8(angle: float, angle_min: float, angle_max: float) -> int:
    """Map angle linearly onto [0, 255].

    angle_min → 0, angle_max → 255. Firmware mirrors this mapping
    to recover the physical angle without precision loss.
    """
    angle = max(angle_min, min(angle_max, angle))
    return round((angle - angle_min) / (angle_max - angle_min) * 255)


class Camera2DofDriver(Node):
    def __init__(self):
        super().__init__("camera_2dof_driver")

        # Servo IDs sent in the CAN frame for each axis.
        # Override via ROS params if the firmware uses different numbering.
        self.declare_parameter("yaw_servo_id", 0)
        self.declare_parameter("pitch_servo_id", 1)

        # Physical angle range for each axis [degrees].
        # 0 maps to *_min, 255 maps to *_max — firmware uses the same range.
        self.declare_parameter("yaw_min", 0.0)
        self.declare_parameter("yaw_max", 180.0)
        self.declare_parameter("pitch_min", 0.0)
        self.declare_parameter("pitch_max", 180.0)

        self.yaw_servo_id = self.get_parameter("yaw_servo_id").value
        self.pitch_servo_id = self.get_parameter("pitch_servo_id").value
        self.yaw_min = self.get_parameter("yaw_min").value
        self.yaw_max = self.get_parameter("yaw_max").value
        self.pitch_min = self.get_parameter("pitch_min").value
        self.pitch_max = self.get_parameter("pitch_max").value

        self.master_pub = self.create_publisher(
            MasterMessage, "master_com/ros_to_master", 10
        )
        self.cmd_sub = self.create_subscription(
            Camera2Dof, "camera_2dof/cmd", self.camera_cmd_cb, 1
        )

    def camera_cmd_cb(self, msg: Camera2Dof):
        yaw_val = angle_to_uint8(msg.yaw, self.yaw_min, self.yaw_max)
        pitch_val = angle_to_uint8(msg.pitch, self.pitch_min, self.pitch_max)

        self.get_logger().debug(
            f"cam={msg.camera_id} yaw={msg.yaw:.1f}°→{yaw_val} pitch={msg.pitch:.1f}°→{pitch_val}"
        )

        self.master_pub.publish(
            MasterMessage(
                cmd=MasterMessage.CAM_2DOF_SET,
                data=[self.yaw_servo_id, yaw_val],
            )
        )
        self.master_pub.publish(
            MasterMessage(
                cmd=MasterMessage.CAM_2DOF_SET,
                data=[self.pitch_servo_id, pitch_val],
            )
        )


def main():
    try:
        rclpy.init()
        node = Camera2DofDriver()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
