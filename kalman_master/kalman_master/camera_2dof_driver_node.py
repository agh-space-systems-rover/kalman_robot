import rclpy
from rclpy.node import Node
from kalman_interfaces.msg import Camera2Dof, MasterMessage

# Angle range for a standard 180° servo mapped to uint8 (0–255).
SERVO_ANGLE_MAX = 180.0


def angle_to_uint8(angle: float) -> int:
    return round(max(0.0, min(SERVO_ANGLE_MAX, angle)) / SERVO_ANGLE_MAX * 255)


class Camera2DofDriver(Node):
    def __init__(self):
        super().__init__("camera_2dof_driver")

        # Servo IDs sent in the CAN frame for each axis.
        # Override via ROS params if the firmware uses different numbering.
        self.declare_parameter("yaw_servo_id", 0)
        self.declare_parameter("pitch_servo_id", 1)

        # Physical angle limits in degrees (clamped before encoding).
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
        yaw = max(self.yaw_min, min(self.yaw_max, msg.yaw))
        pitch = max(self.pitch_min, min(self.pitch_max, msg.pitch))

        yaw_val = angle_to_uint8(yaw)
        pitch_val = angle_to_uint8(pitch)

        self.get_logger().debug(
            f"cam={msg.camera_id} yaw={yaw:.1f}°→{yaw_val} pitch={pitch:.1f}°→{pitch_val}"
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
