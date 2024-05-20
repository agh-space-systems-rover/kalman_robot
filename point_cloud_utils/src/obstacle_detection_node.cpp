#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

constexpr int         default_queue_size                    = 10;
constexpr const char *default_world_frame                   = "odom";
constexpr float       default_normal_estimation_radius      = 0.2;
constexpr float       default_max_ground_angle              = 25 * M_PI / 180;
constexpr float       default_outlier_removal_radius        = 0.2;
constexpr int         default_outlier_removal_min_neighbors = 10;
constexpr const char *default_robot_frame                   = "base_link";
constexpr float       default_min_obstacle_height           = -1.0;
constexpr float       default_max_obstacle_height           = 1.0;

namespace point_cloud_utils {

class ObstacleDetection : public rclcpp::Node {
  public:
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub;
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub;
	tf2_ros::Buffer::SharedPtr                                     tf_buffer;
	std::shared_ptr<tf2_ros::TransformListener>                    tf_listener;

	ObstacleDetection(const rclcpp::NodeOptions &options)
	    : Node("obstacle_detection", options) {
		// Declare parameters.
		declare_parameter("queue_size", default_queue_size);
		declare_parameter("world_frame", default_world_frame);
		declare_parameter(
		    "normal_estimation_radius", default_normal_estimation_radius
		);
		declare_parameter("max_ground_angle", default_max_ground_angle);
		declare_parameter(
		    "outlier_removal_radius", default_outlier_removal_radius
		);
		declare_parameter(
		    "outlier_removal_min_neighbors",
		    default_outlier_removal_min_neighbors
		);
		declare_parameter("robot_frame", default_robot_frame);
		declare_parameter("min_obstacle_height", default_min_obstacle_height);
		declare_parameter("max_obstacle_height", default_max_obstacle_height);

		// Read static parameters.
		int queue_size = default_queue_size;
		get_parameter("queue_size", queue_size);

		// Create a TF buffer.
		tf_buffer   = std::make_shared<tf2_ros::Buffer>(get_clock());
		tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

		// Create a publisher.
		pub = create_publisher<sensor_msgs::msg::PointCloud2>(
		    "output", queue_size
		);

		// Create a subscriber.
		sub = create_subscription<sensor_msgs::msg::PointCloud2>(
		    "input",
		    queue_size,
		    std::bind(&ObstacleDetection::callback, this, std::placeholders::_1)
		);
	}

	void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
		// Read dynamic parameters.
		std::string world_frame              = default_world_frame;
		float       normal_estimation_radius = default_normal_estimation_radius;
		float       max_ground_angle         = default_max_ground_angle;
		float       outlier_removal_radius   = default_outlier_removal_radius;
		int         outlier_removal_min_neighbors =
		    default_outlier_removal_min_neighbors;
		std::string robot_frame         = default_robot_frame;
		float       min_obstacle_height = default_min_obstacle_height;
		float       max_obstacle_height = default_max_obstacle_height;
		get_parameter("world_frame", world_frame);
		get_parameter("normal_estimation_radius", normal_estimation_radius);
		get_parameter("max_ground_angle", max_ground_angle);
		get_parameter("outlier_removal_radius", outlier_removal_radius);
		get_parameter(
		    "outlier_removal_min_neighbors", outlier_removal_min_neighbors
		);
		get_parameter("robot_frame", robot_frame);
		get_parameter("min_obstacle_height", min_obstacle_height);
		get_parameter("max_obstacle_height", max_obstacle_height);

		// Convert the message to a PCL point cloud.
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr
		    cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::fromROSMsg(*msg, *cloud);

		// Get the transform from the frame of the point cloud to the world
		// frame.
		geometry_msgs::msg::TransformStamped pc_to_world_tf;
		try {
			pc_to_world_tf = tf_buffer->lookupTransform(
			    world_frame, msg->header.frame_id, tf2::TimePointZero
			);
		} catch (tf2::TransformException &ex) {
			// RCLCPP_ERROR(get_logger(), "%s", ex.what());
			return;
		}
		Eigen::Isometry3d pc_to_world = tf2::transformToEigen(pc_to_world_tf);

		// Estimate normals if there are any points in the cloud.
		if (cloud->size() > 0) {
			pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
			ne.setInputCloud(cloud);
			pcl::search::KdTree<pcl::PointXYZRGB>::Ptr
			    tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
			ne.setSearchMethod(tree);
			pcl::PointCloud<pcl::Normal>::Ptr
			    normals(new pcl::PointCloud<pcl::Normal>);
			ne.setRadiusSearch(normal_estimation_radius);
			ne.compute(*normals);

			// DEBUG: Visualize normals using the color of the points.
			// for (size_t i = 0; i < normals->size(); i++) {
			// 	// Transform the normal to world_frame
			// 	Eigen::Vector3d normal(normals->points[i].normal_x,
			// normals->points[i].normal_y, normals->points[i].normal_z);
			// normal = pc_to_world.linear() * normal;

			// 	cloud->points[i].r = (normal.x() * 0.5F + 0.5F) * 255;
			// 	cloud->points[i].g = (normal.y() * 0.5F + 0.5F) * 255;
			// 	cloud->points[i].b = (normal.z() * 0.5F + 0.5F) * 255;
			// }

			// Filter points based on the angle between the normal and the
			// z-axis.
			pcl::PointIndices::Ptr obstacles(new pcl::PointIndices);
			for (size_t i = 0; i < normals->size(); i++) {
				// Transform the normal to world_frame
				Eigen::Vector3d normal(
				    normals->points[i].normal_x,
				    normals->points[i].normal_y,
				    normals->points[i].normal_z
				);
				normal = pc_to_world.linear() * normal;

				if (abs(normal.z()) < cos(max_ground_angle)) {
					obstacles->indices.push_back(i);
				}
			}
			pcl::ExtractIndices<pcl::PointXYZRGB> extract;
			extract.setInputCloud(cloud);
			extract.setIndices(obstacles);
			extract.setNegative(false);
			extract.filter(*cloud);
		}

		// Remove outliers if there are any points in the obstacle cloud.
		if (cloud->size() > 0) {
			pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> ror;
			ror.setInputCloud(cloud);
			ror.setRadiusSearch(outlier_removal_radius);
			ror.setMinNeighborsInRadius(outlier_removal_min_neighbors);
			ror.filter(*cloud);
		}

		// Apply min/max obstacle height.
		if (cloud->size() > 0) {
			// Find out the Z position of the robot.
			geometry_msgs::msg::TransformStamped robot_to_world_tf;
			try {
				robot_to_world_tf = tf_buffer->lookupTransform(
				    world_frame, robot_frame, tf2::TimePointZero
				);
			} catch (tf2::TransformException &ex) {
				// RCLCPP_ERROR(get_logger(), "%s", ex.what());
				return;
			}
			Eigen::Isometry3d robot_to_world =
			    tf2::transformToEigen(robot_to_world_tf);
			double robot_z = robot_to_world.translation().z();

			// Filter points based on the min/max obstacle height.
			pcl::PointIndices::Ptr slice(new pcl::PointIndices);
			for (size_t i = 0; i < cloud->size(); i++) {
				// Transform the position to world_frame
				Eigen::Vector3d position(
				    cloud->points[i].x, cloud->points[i].y, cloud->points[i].z
				);
				position = pc_to_world * position;

				if (position.z() > robot_z + min_obstacle_height &&
				    position.z() < robot_z + max_obstacle_height) {
					slice->indices.push_back(i);
				}
			}
			pcl::ExtractIndices<pcl::PointXYZRGB> extract;
			extract.setInputCloud(cloud);
			extract.setIndices(slice);
			extract.setNegative(false);
			extract.filter(*cloud);
		}

		// Convert the point cloud back to a message.
		std_msgs::msg::Header og_header = msg->header;
		pcl::toROSMsg(*cloud, *msg);
		msg->header = og_header;

		// Publish the message.
		pub->publish(*msg);
	}
};

} // namespace point_cloud_utils

RCLCPP_COMPONENTS_REGISTER_NODE(point_cloud_utils::ObstacleDetection)
