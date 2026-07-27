#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <grid_map_core/iterators/CircleIterator.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_core/iterators/SpiralIterator.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace kalman_arc {

class PeakFinder : public rclcpp::Node {
public:
	explicit PeakFinder(const rclcpp::NodeOptions &options)
	    : Node("peak_finder", options), map_({"elevation"}) {

		map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
		robot_frame_ = this->declare_parameter<std::string>("robot_frame", "base_link");
		search_radius_ = this->declare_parameter<double>("search_radius", 10.0);
		double resolution = this->declare_parameter<double>("map_resolution", 0.3);
		double map_size = this->declare_parameter<double>("map_size", 20.0);

		peak_support_radius_  = this->declare_parameter<double>("peak_support_radius", 1.2);
		peak_min_support_     = this->declare_parameter<int>("peak_min_support", 12);
		wall_ratio_threshold_ = this->declare_parameter<double>("wall_ratio_threshold", 3.0);
		// A neighbor cell counts as "support" for a candidate peak if it is
		// at most this many meters below the candidate.
		support_height_drop_  = this->declare_parameter<double>("support_height_drop", 0.6);

		// Accepted Z band for incoming cloud points, relative to robot height.
		z_min_offset_         = this->declare_parameter<double>("z_min_offset", -3.0);
		z_max_offset_         = this->declare_parameter<double>("z_max_offset", 5.0);
		// Median filter window radius used to suppress spikes before search.
		median_filter_radius_ = this->declare_parameter<double>("median_filter_radius", 0.45);
		// Candidates whose surroundings descend steeper than this are rejected.
		slope_check_radius_   = this->declare_parameter<double>("slope_check_radius", 1.0);
		max_slope_deg_        = this->declare_parameter<double>("max_slope_deg", 45.0);

		map_.setFrameId(map_frame_);
		map_.setGeometry(grid_map::Length(map_size, map_size), resolution);
		map_.setBasicLayers({"elevation"});
		map_.add("elevation_filtered");
		map_.clearAll();

		tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
		tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

		camera_ids_ = this->declare_parameter<std::vector<std::string>>(
		    "camera_ids", {"d455_front", "d455_left", "d455_right", "d455_back"}
		);
		pc_subs_.reserve(camera_ids_.size());
		for (const auto &camera_id : camera_ids_) {
			const std::string topic = "/" + camera_id + "/point_cloud";
			pc_subs_.push_back(
			    this->create_subscription<sensor_msgs::msg::PointCloud2>(
			        topic,
			        rclcpp::SensorDataQoS(),
			        std::bind(&PeakFinder::pointcloud_callback, this, std::placeholders::_1)
			    )
			);
		}

		auto map_qos = rclcpp::QoS(1).transient_local().reliable();
		map_pub_     = this->create_publisher<grid_map_msgs::msg::GridMap>("out/elevation_map", map_qos);
		peak_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/peak_position", 10);

		peak_timer_ = this->create_wall_timer(
		    std::chrono::seconds(2), std::bind(&PeakFinder::find_peak, this)
		);

		clear_elevation_map_srv_ = this->create_service<std_srvs::srv::Trigger>(
		    "~/clear_elevation_map",
		    std::bind(&PeakFinder::clear_elevation_map_callback, this,
		              std::placeholders::_1, std::placeholders::_2)
		);

		RCLCPP_INFO(this->get_logger(), "PeakFinder initialized (service: %s).",
		            clear_elevation_map_srv_->get_service_name());
	}

private:
	void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
		geometry_msgs::msg::TransformStamped sensor_to_map_tf;
		geometry_msgs::msg::TransformStamped robot_to_map_tf;

		try {
			sensor_to_map_tf = tf_buffer_->lookupTransform(map_frame_, msg->header.frame_id, tf2::TimePointZero);
			robot_to_map_tf  = tf_buffer_->lookupTransform(map_frame_, robot_frame_, tf2::TimePointZero);
		} catch (const tf2::TransformException &ex) {
			RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
			                     "TF error in pointcloud_callback: %s", ex.what());
			return;
		}

		map_.move(grid_map::Position(
		    robot_to_map_tf.transform.translation.x,
		    robot_to_map_tf.transform.translation.y));

		sensor_msgs::msg::PointCloud2 cloud_map;
		try {
			tf2::doTransform(*msg, cloud_map, sensor_to_map_tf);
		} catch (const std::exception &ex) {
			RCLCPP_WARN(this->get_logger(), "Cloud transform failed: %s", ex.what());
			return;
		}

		if (cloud_map.data.empty() || cloud_map.point_step == 0u) return;

		// Reject implausible heights relative to the robot so stray returns
		// (dust, reflections, distant structures) never enter the map.
		const double robot_z = robot_to_map_tf.transform.translation.z;
		const float  z_lo    = static_cast<float>(robot_z + z_min_offset_);
		const float  z_hi    = static_cast<float>(robot_z + z_max_offset_);

		sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_map, "x");
		sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_map, "y");
		sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_map, "z");

		for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
			if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) continue;
			if (*iter_z < z_lo || *iter_z > z_hi) continue;
			const grid_map::Position pos(*iter_x, *iter_y);
			if (!map_.isInside(pos)) continue;
			float &cell = map_.atPosition("elevation", pos);
			if (std::isnan(cell) || *iter_z > cell) cell = *iter_z;
		}

		map_.setTimestamp(rclcpp::Time(msg->header.stamp).nanoseconds());
		map_pub_->publish(std::move(grid_map::GridMapRosConverter::toMessage(map_)));
	}

	// Median-filter the elevation layer into "elevation_filtered" to remove
	// single-cell spikes and sensor noise before peak search. Unknown (NaN)
	// cells stay unknown so the filter never invents terrain.
	void build_filtered_layer() {
		map_["elevation_filtered"].setConstant(NAN);

		std::vector<float> window;
		for (grid_map::GridMapIterator it(map_); !it.isPastEnd(); ++it) {
			if (std::isnan(map_.at("elevation", *it))) continue;

			grid_map::Position center;
			map_.getPosition(*it, center);

			window.clear();
			for (grid_map::CircleIterator cit(map_, center, median_filter_radius_);
			     !cit.isPastEnd(); ++cit) {
				const float v = map_.at("elevation", *cit);
				if (!std::isnan(v)) window.push_back(v);
			}
			if (window.empty()) continue;

			auto mid = window.begin() + window.size() / 2;
			std::nth_element(window.begin(), mid, window.end());
			map_.at("elevation_filtered", *it) = *mid;
		}
	}

	// A real hill top is surrounded by terrain close to its own height. A
	// leftover spike or a mapped person has few such cells (or they form a
	// thin wall), so it gets rejected here.
	bool is_valid_peak(const grid_map::Position &center, float candidate_z) const {
		const float min_support_z = candidate_z - static_cast<float>(support_height_drop_);

		int   count = 0;
		float x_min =  std::numeric_limits<float>::max();
		float x_max = -std::numeric_limits<float>::max();
		float y_min =  std::numeric_limits<float>::max();
		float y_max = -std::numeric_limits<float>::max();

		for (grid_map::CircleIterator it(map_, center, peak_support_radius_); !it.isPastEnd(); ++it) {
			const float z = map_.at("elevation_filtered", *it);
			if (std::isnan(z) || z < min_support_z) continue;

			grid_map::Position p;
			map_.getPosition(*it, p);
			++count;
			x_min = std::min(x_min, static_cast<float>(p.x()));
			x_max = std::max(x_max, static_cast<float>(p.x()));
			y_min = std::min(y_min, static_cast<float>(p.y()));
			y_max = std::max(y_max, static_cast<float>(p.y()));
		}

		if (count < peak_min_support_) return false;

		const float dx    = (x_max - x_min) + static_cast<float>(map_.getResolution());
		const float dy    = (y_max - y_min) + static_cast<float>(map_.getResolution());
		const float ratio = std::max(dx, dy) / std::min(dx, dy);

		return ratio <= static_cast<float>(wall_ratio_threshold_);
	}

	// Reject candidates whose surroundings drop off too steeply. Hills have
	// gentle slopes; people or poles produce near-vertical walls (~70-90 deg)
	// even if they survive the median filter.
	bool passes_slope_check(const grid_map::Position &center, float candidate_z) const {
		const double max_slope_rad = max_slope_deg_ * M_PI / 180.0;
		const double resolution    = map_.getResolution();

		for (grid_map::CircleIterator it(map_, center, slope_check_radius_);
		     !it.isPastEnd(); ++it) {
			const float z = map_.at("elevation_filtered", *it);
			if (std::isnan(z)) continue;

			grid_map::Position p;
			map_.getPosition(*it, p);
			const double dist = (p - center).norm();
			if (dist < 0.5 * resolution) continue; // skip the candidate cell

			const double slope = std::atan2(candidate_z - z, dist);
			if (slope > max_slope_rad) return false;
		}
		return true;
	}

	void find_peak() {
		if (!search_center_set_) return;

		build_filtered_layer();

		double             max_elevation = -std::numeric_limits<double>::infinity();
		grid_map::Position peak_pos;
		bool               peak_found = false;

		for (grid_map::SpiralIterator it(map_, search_center_, search_radius_); !it.isPastEnd(); ++it) {
			const float z = map_.at("elevation_filtered", *it);
			if (std::isnan(z) || z <= max_elevation) continue;

			grid_map::Position candidate;
			map_.getPosition(*it, candidate);
			if (!is_valid_peak(candidate, z)) continue;
			if (!passes_slope_check(candidate, z)) continue;

			max_elevation = z;
			peak_pos      = candidate;
			peak_found    = true;
		}

		if (peak_found) {
			geometry_msgs::msg::PoseStamped peak_pose;
			peak_pose.header.stamp       = this->now();
			peak_pose.header.frame_id    = map_frame_;
			peak_pose.pose.position.x    = peak_pos.x();
			peak_pose.pose.position.y    = peak_pos.y();
			peak_pose.pose.position.z    = max_elevation;
			peak_pose.pose.orientation.w = 1.0;
			peak_pos_pub_->publish(peak_pose);
		}
	}

	void clear_elevation_map_callback(
	    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
	    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
	{
		try {
			auto tf = tf_buffer_->lookupTransform(map_frame_, robot_frame_, tf2::TimePointZero);
			search_center_     = grid_map::Position(tf.transform.translation.x, tf.transform.translation.y);
			search_center_set_ = true;
			RCLCPP_INFO(this->get_logger(), "Search center fixed at X: %.2f  Y: %.2f",
			            search_center_.x(), search_center_.y());
		} catch (const tf2::TransformException &ex) {
			response->success = false;
			response->message = std::string("Failed to get robot pose: ") + ex.what();
			RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
			return;
		}

		map_.clearAll();
		map_.setTimestamp(this->now().nanoseconds());
		map_pub_->publish(std::move(grid_map::GridMapRosConverter::toMessage(map_)));

		response->success = true;
		response->message = "Elevation map cleared and search center set.";
	}

	std::string       map_frame_;
	std::string       robot_frame_;
	double            search_radius_;
	double            peak_support_radius_;
	int               peak_min_support_;
	double            wall_ratio_threshold_;
	double            support_height_drop_;
	double            z_min_offset_;
	double            z_max_offset_;
	double            median_filter_radius_;
	double            slope_check_radius_;
	double            max_slope_deg_;
	grid_map::GridMap map_;

	grid_map::Position search_center_{0.0, 0.0};
	bool               search_center_set_{false};

	std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
	std::vector<std::string>                    camera_ids_;
	std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> pc_subs_;
	rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr     map_pub_;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr peak_pos_pub_;
	rclcpp::TimerBase::SharedPtr                                  peak_timer_;
	rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr            clear_elevation_map_srv_;
};

} // namespace kalman_arc

RCLCPP_COMPONENTS_REGISTER_NODE(kalman_arc::PeakFinder)