#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <mutex>

namespace rock_finder {

constexpr int   default_queue_size      = 10;
constexpr float default_voxel_leaf      = 0.3f;
constexpr int   default_outlier_mean_k  = 10;
constexpr float default_outlier_std_mul = 1.0f;
constexpr int   default_knn_blur_k      = 5;
constexpr float default_publish_rate    = 1.0f;

class DarkestRockFinder : public rclcpp::Node {
  public:
	using Detection2DArray = vision_msgs::msg::Detection2DArray;
	using PoseStamped      = geometry_msgs::msg::PoseStamped;
	using PointCloud2      = sensor_msgs::msg::PointCloud2;
	using Trigger          = std_srvs::srv::Trigger;
	using CloudI           = pcl::PointCloud<pcl::PointXYZI>;

	rclcpp::Subscription<Detection2DArray>::SharedPtr sub_;
	rclcpp::Publisher<PoseStamped>::SharedPtr         pub_;
	rclcpp::Publisher<PointCloud2>::SharedPtr         debug_pub_;
	rclcpp::Service<Trigger>::SharedPtr               clear_srv_;
	rclcpp::TimerBase::SharedPtr                      timer_;

	std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

	CloudI::Ptr cloud_;
	std::mutex  cloud_mutex_;

	DarkestRockFinder(const rclcpp::NodeOptions &options)
	    : Node("darkest_rock_finder", options) {
		declare_parameter("map_frame",           std::string("map"));
		declare_parameter("queue_size",          default_queue_size);
		declare_parameter("voxel_leaf",          default_voxel_leaf);
		declare_parameter("outlier_mean_k",      default_outlier_mean_k);
		declare_parameter("outlier_std_mul",     default_outlier_std_mul);
		declare_parameter("knn_blur_k",          default_knn_blur_k);
		declare_parameter("publish_rate",        default_publish_rate);
		declare_parameter("publish_debug_cloud", true);

		int queue_size = get_parameter("queue_size").as_int();

		tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
		tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

		cloud_ = std::make_shared<CloudI>();

		sub_ = create_subscription<Detection2DArray>(
		    "/yolo_detections",
		    queue_size,
		    std::bind(&DarkestRockFinder::detections_cb, this, std::placeholders::_1)
		);

		pub_       = create_publisher<PoseStamped>("/boulder_position", queue_size);
		debug_pub_ = create_publisher<PointCloud2>("/debug_cloud", queue_size);

		clear_srv_ = create_service<Trigger>(
		    "/boulder_position_clear",
		    std::bind(
		        &DarkestRockFinder::clear_cb, this,
		        std::placeholders::_1, std::placeholders::_2
		    )
		);

		double rate = get_parameter("publish_rate").as_double();
		auto period = std::chrono::duration<double>(1.0 / rate);
		timer_ = create_wall_timer(
		    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
		    std::bind(&DarkestRockFinder::timer_cb, this)
		);
	}

	void detections_cb(const Detection2DArray::SharedPtr msg) {
		std::string map_frame = get_parameter("map_frame").as_string();
		float       leaf      = get_parameter("voxel_leaf").as_double();

		CloudI::Ptr new_points(new CloudI);

		for (auto &det : msg->detections) {
			if (det.results.empty()) continue;

			// parse luminosity from id - lower means darker
			float luminosity = 255.0f;
			try {
				luminosity = std::stof(det.id);
			} catch (...) {
				RCLCPP_WARN_ONCE(
				    get_logger(),
				    "Could not parse luminosity from detection id '%s', using 255.",
				    det.id.c_str()
				);
				continue;
			}

			// transform pose to map frame
			geometry_msgs::msg::PoseStamped pose_in, pose_out;
			pose_in.header        = msg->header;
			pose_in.pose          = det.results[0].pose.pose;

			try {
				tf_buffer_->transform(pose_in, pose_out, map_frame);
			} catch (const tf2::TransformException &ex) {
				RCLCPP_WARN(get_logger(), "TF transform failed: %s", ex.what());
				continue;
			}

			pcl::PointXYZI pt;
			pt.x         = pose_out.pose.position.x;
			pt.y         = pose_out.pose.position.y;
			pt.z         = pose_out.pose.position.z;
			pt.intensity = luminosity;
			new_points->push_back(pt);
		}

		if (new_points->empty()) return;

		{
			std::lock_guard<std::mutex> lock(cloud_mutex_);

			*cloud_ += *new_points;

			// keep cloud compact after each batch
			if (cloud_->size() > 500) {
				pcl::VoxelGrid<pcl::PointXYZI> vg;
				vg.setInputCloud(cloud_);
				vg.setLeafSize(leaf, leaf, leaf);
				vg.filter(*cloud_);
			}
		}
	}

	void timer_cb() {
		CloudI::Ptr cloud_copy;
		{
			std::lock_guard<std::mutex> lock(cloud_mutex_);
			if (cloud_->empty()) return;
			cloud_copy = std::make_shared<CloudI>(*cloud_);
		}

		int         mean_k    = get_parameter("outlier_mean_k").as_int();
		float       std_mul   = get_parameter("outlier_std_mul").as_double();
		int         knn_k     = get_parameter("knn_blur_k").as_int();
		std::string map_frame = get_parameter("map_frame").as_string();
		bool        dbg_cloud = get_parameter("publish_debug_cloud").as_bool();

		if (dbg_cloud) {
			PointCloud2 dbg_msg;
			pcl::toROSMsg(*cloud_copy, dbg_msg);
			dbg_msg.header.stamp    = now();
			dbg_msg.header.frame_id = map_frame;
			debug_pub_->publish(dbg_msg);
		}

		if ((int)cloud_copy->size() > mean_k) {
			pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
			sor.setInputCloud(cloud_copy);
			sor.setMeanK(mean_k);
			sor.setStddevMulThresh(std_mul);
			sor.filter(*cloud_copy);
		}

		if (cloud_copy->empty()) return;

		// knn intensity blur - smooth out per-point luminosity noise
		CloudI::Ptr blurred(new CloudI(*cloud_copy));
		if ((int)cloud_copy->size() > knn_k) {
			pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
			kdtree.setInputCloud(cloud_copy);

			std::vector<int>   indices(knn_k);
			std::vector<float> dists(knn_k);

			for (size_t i = 0; i < cloud_copy->size(); ++i) {
				int found = kdtree.nearestKSearch(
				    cloud_copy->points[i], knn_k, indices, dists
				);
				if (found < 1) continue;

				float sum = 0.0f;
				for (int idx : indices) sum += cloud_copy->points[idx].intensity;
				blurred->points[i].intensity = sum / found;
			}
		}

		// find darkest (lowest intensity) point
		auto darkest = std::min_element(
		    blurred->points.begin(), blurred->points.end(),
		    [](const pcl::PointXYZI &a, const pcl::PointXYZI &b) {
			    return a.intensity < b.intensity;
		    }
		);

		PoseStamped out;
		out.header.stamp    = now();
		out.header.frame_id = map_frame;
		out.pose.position.x = darkest->x;
		out.pose.position.y = darkest->y;
		out.pose.position.z = darkest->z;
		out.pose.orientation.w = 1.0;

		pub_->publish(out);
	}

	void clear_cb(
	    const Trigger::Request::SharedPtr,
	    Trigger::Response::SharedPtr resp
	) {
		std::lock_guard<std::mutex> lock(cloud_mutex_);
		cloud_->clear();
		resp->success = true;
		resp->message = "State cleared.";
		RCLCPP_INFO(get_logger(), "State cleared.");
	}
};

} // namespace rock_finder

RCLCPP_COMPONENTS_REGISTER_NODE(rock_finder::DarkestRockFinder)