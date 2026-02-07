import rclpy
from rclpy.node import Node
from kalman_interfaces.srv import ReadPkgConfigFile
from ament_index_python.packages import get_package_share_directory
import os


class PkgConfigReaderNode(Node):
    def __init__(self):
        super().__init__("pkg_config_reader")

        self.srv = self.create_service(
            ReadPkgConfigFile,
            "read_pkg_config_file",
            self.read_pkg_config_file_callback,
        )

        self.get_logger().info("Package config reader service is ready.")

    def read_pkg_config_file_callback(self, request, response):
        try:
            # Get the package share directory using ament_index
            pkg_share_dir = get_package_share_directory(request.pkg)

            # Concatenate with the requested path
            full_path = os.path.join(pkg_share_dir, request.path)

            # Read the file
            with open(full_path, "r") as f:
                response.content = f.read()

            response.success = True
            self.get_logger().info(f"Successfully read file: {full_path}")

        except Exception as e:
            response.content = ""
            response.success = False
            self.get_logger().error(f"Failed to read file: {str(e)}")

        return response


def main(args=None):
    rclpy.init(args=args)
    node = PkgConfigReaderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
