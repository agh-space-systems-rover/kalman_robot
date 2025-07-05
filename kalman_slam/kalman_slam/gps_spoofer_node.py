import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu

from kalman_interfaces.srv import SpoofGps


def spherical_normal(lambda_, phi):
    """
    Returns the normal vector to the sphere at the given latitude and longitude.
    Parameters:
    lambda_: Longitude in radians
    phi: Latitude in radians
    Returns:
    Normal vector in frame: x = east, y = north, z = up
    """

    x = np.cos(phi) * np.cos(lambda_)
    y = np.cos(phi) * np.sin(lambda_)
    z = np.sin(phi)
    return np.array([x, y, z])


def great_circle_bearing(latlon1, latlon2):
    """
    Returns the bearing from latlon1 to latlon2.
    Parameters:
    latlon1: Tuple of latitude and longitude in degrees
    latlon2: Tuple of latitude and longitude in degrees
    Returns:
    Bearing in degrees: 0 is north, 90 is east, 180 is south, 270 is west
    """

    phi1 = np.radians(latlon1[0])
    lambda1 = np.radians(latlon1[1])
    phi2 = np.radians(latlon2[0])
    lambda2 = np.radians(latlon2[1])

    n1 = spherical_normal(lambda1, phi1)
    n2 = spherical_normal(lambda2, phi2)

    up = np.array([0, 0, 1])  # z = up
    east1 = np.cross(up, n1)
    north1 = np.cross(n1, east1)

    n_diff2 = np.dot(n2, n1) * n1  # projection of n2 onto n1
    azim_vec2 = (n2 - n_diff2) / np.linalg.norm(n2 - n_diff2)
    # = surface-parallel vector from latlon1 towards latlon2

    cos_alpha = np.dot(north1, azim_vec2)
    sin_alpha = np.dot(east1, azim_vec2)
    alpha = np.arctan2(sin_alpha, cos_alpha)
    return np.degrees(alpha)


# This node creates a service that can be used to send a fake GPS fix.
# In practice, this service is forwarded over RF to be used on the ground station.
class GpsSpoofer(Node):
    def __init__(self) -> None:
        super().__init__("gps_spoofer")

        self.frame_id = self.declare_parameter("frame_id", "base_link").value
        self.covariance = self.declare_parameter("covariance", 0.0).value

        self.srv = self.create_service(SpoofGps, "spoof_gps", self.spoof_callback)
        self.look_at_srv = self.create_service(
            SpoofGps, "spoof_gps/look_at", self.look_at_callback
        )

        self.fix_pub = self.create_publisher(NavSatFix, "fix/out", 10)
        self.imu_pub = self.create_publisher(Imu, "imu", 10)
        self.fix_sub = self.create_subscription(
            NavSatFix, "fix/in", self.fix_callback, 10
        )

        self.last_lat = 0
        self.last_lon = 0

    def fix_callback(self, msg: NavSatFix):
        self.last_lat = msg.latitude
        self.last_lon = msg.longitude

    def spoof_callback(self, req: SpoofGps.Request, res: SpoofGps.Response):
        fix = NavSatFix()
        fix.header.frame_id = self.frame_id
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.latitude = req.location.latitude
        fix.longitude = req.location.longitude
        fix.altitude = req.location.altitude
        fix.status.status = 0
        fix.status.service = 1
        fix.position_covariance_type = 0
        fix.position_covariance = [self.covariance] * 9
        self.fix_pub.publish(fix)
        return res

    def look_at_callback(self, req: SpoofGps.Request, res: SpoofGps.Response):
        lat1 = self.last_lat
        lon1 = self.last_lon
        lat2 = req.location.latitude
        lon2 = req.location.longitude

        # Calculate the bearing from the current location to the target location
        bearing = great_circle_bearing((lat1, lon1), (lat2, lon2))

        # Convert to vehicle body rotation in ENU
        east_centric = bearing - 90
        ccw = -east_centric
        yaw = np.radians(ccw)

        # Convert to quaternion
        qw = np.cos(yaw * 0.5)
        qz = np.sin(yaw * 0.5)

        # Publish as IMU message
        imu = Imu()
        imu.header.frame_id = self.frame_id
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.orientation.x = 0.0
        imu.orientation.y = 0.0
        imu.orientation.z = qz
        imu.orientation.w = qw
        imu.orientation_covariance[8] = self.covariance
        imu.angular_velocity_covariance[0] = -1
        imu.linear_acceleration_covariance[0] = -1
        self.imu_pub.publish(imu)
        return res


def main():
    try:
        rclpy.init()
        node = GpsSpoofer()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
