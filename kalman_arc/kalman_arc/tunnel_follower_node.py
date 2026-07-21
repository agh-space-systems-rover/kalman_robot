import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import SetBool
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs_py.point_cloud2 import read_points_numpy, create_cloud_xyz32
from tf2_ros import Buffer, TransformListener
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

class TunnelFollower(Node):
	def __init__(self):
		super().__init__("tunnel_follower")
		
		self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
		self.viz_pub = self.create_publisher(MarkerArray, "/arc/tunnel_debug_markers", 10)
		self.cloud_pub = self.create_publisher(PointCloud2, "/arc/tunnel_debug_cloud", 10)
		
		self.tf_buffer = Buffer()
		self.tf_listener = TransformListener(self.tf_buffer, self)

		self.pc_sub = self.create_subscription(
			PointCloud2, 
			"/d455_front/point_cloud", 
			self.cloud_cb, 
			1
		)

		self.srv = self.create_service(
			SetBool, 
			"/arc/follow_tunnel", 
			self.toggle_cb
		)

		self.enabled = False
		self.latest_twist = Twist()
		
		# wall detection params
		self.z_min = -0.2
		self.z_max = 0.2
		self.x_max = 2.5
		self.y_gap = 0.2
		
		# control params
		self.target_v = 0.5
		self.k_lat = 1.0 # CHANGE K PARAM
		self.offset_dist = 1.5

		self.create_timer(0.1, self.control_tick)

	def toggle_cb(self, request, response):
		self.enabled = request.data
		if not self.enabled:
			self.stop_robot()
		
		response.success = True
		response.message = f"Tunnel following: {self.enabled}"
		return response

	def stop_robot(self):
		self.latest_twist = Twist()
		self.cmd_pub.publish(self.latest_twist)

	def create_marker(self, pos, id, color, frame_id):
		marker = Marker()
		marker.header.frame_id = frame_id
		marker.header.stamp = self.get_clock().now().to_msg()
		marker.ns = "clusters"
		marker.id = id
		marker.type = Marker.SPHERE
		marker.action = Marker.ADD
		marker.pose.position.x = float(pos[0])
		marker.pose.position.y = float(pos[1])
		marker.pose.position.z = 0.0
		marker.scale.x = 0.15
		marker.scale.y = 0.15
		marker.scale.z = 0.15
		marker.color.a = 1.0
		marker.color.r = float(color[0])
		marker.color.g = float(color[1])
		marker.color.b = float(color[2])
		return marker

	def cloud_cb(self, msg):
		try:
			pts_raw = read_points_numpy(msg, field_names=("x", "y", "z"))
			clean_msg = create_cloud_xyz32(msg.header, pts_raw)

			trans = self.tf_buffer.lookup_transform(
				"base_link",
				clean_msg.header.frame_id,
				clean_msg.header.stamp,
				rclpy.duration.Duration(seconds=0.1)
			)
			msg_transformed = do_transform_cloud(clean_msg, trans)
		except Exception as e:
			self.get_logger().warn(f"Could not transform cloud: {e}")
			return

		pts_all = read_points_numpy(msg_transformed, field_names=("x", "y", "z"))
		
		if pts_all.shape[0] == 0:
			self.latest_twist = Twist()
			self.latest_twist.linear.x = 0.2
			return

		mask = (pts_all[:, 2] > self.z_min) & (pts_all[:, 2] < self.z_max) & \
		       (pts_all[:, 0] > 0.1) & (pts_all[:, 0] < self.x_max) & \
		       (np.abs(pts_all[:, 1]) > self.y_gap)
		pts = pts_all[mask]

		debug_cloud = create_cloud_xyz32(msg_transformed.header, pts)
		self.cloud_pub.publish(debug_cloud)

		left_pts = pts[pts[:, 1] > 0]
		right_pts = pts[pts[:, 1] < 0]
		
		l_mean = np.mean(left_pts[:, :2], axis=0) if left_pts.shape[0] > 5 else None
		r_mean = np.mean(right_pts[:, :2], axis=0) if right_pts.shape[0] > 5 else None

		target = None
		markers = MarkerArray()
		frame = msg_transformed.header.frame_id

		if l_mean is not None and r_mean is not None:
			target = (l_mean + r_mean) / 2.0
			markers.markers.append(self.create_marker(l_mean, 0, (0, 1, 0), frame))
			markers.markers.append(self.create_marker(r_mean, 1, (0, 1, 0), frame))
		elif l_mean is not None:
			if np.abs(l_mean[1]) < 1.5:
				target = l_mean.copy()
				target[1] -= self.offset_dist
				markers.markers.append(self.create_marker(l_mean, 0, (0, 1, 0), frame))
		elif r_mean is not None:
			if np.abs(r_mean[1]) < 1.5:
				target = r_mean.copy()
				target[1] += self.offset_dist
				markers.markers.append(self.create_marker(r_mean, 1, (0, 1, 0), frame))

		t = Twist()
		if target is not None:
			markers.markers.append(self.create_marker(target, 2, (1, 0, 0), frame))
			L2 = target[0]**2 + target[1]**2
			t.angular.z = (2.0 * target[1] / L2) * self.k_lat
			t.linear.x = self.target_v
		else:
			t.linear.x = 0.2

		self.viz_pub.publish(markers)
		self.latest_twist = t

	def control_tick(self):
		if self.enabled:
			self.cmd_pub.publish(self.latest_twist)

def main():
	try:
		rclpy.init()
		node = TunnelFollower()
		rclpy.spin(node)
		node.destroy_node()
		rclpy.shutdown()
	except KeyboardInterrupt:
		pass
