#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace kalman_arc {

class PeakFinder : public rclcpp::Node {
public:
	explicit PeakFinder(const rclcpp::NodeOptions &options)
	    : Node("peak_finder", options), map_({"elevation"}) {
		map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
		robot_frame_ =
		    this->declare_parameter<std::string>("robot_frame", "base_link");
		search_radius_ = this->declare_parameter<double>("search_radius", 10.0);
		double resolution = this->declare_parameter<double>("map_resolution", 0.3);
		double map_size   = this->declare_parameter<double>("map_size", 20.0);

		// Configure map
		map_.setFrameId(map_frame_);
		map_.setGeometry(grid_map::Length(map_size, map_size), resolution);
		map_.setBasicLayers({"elevation"});
		// CRITICAL: explicitly fill all cells with NaN so toMessage serialises
		// a well-defined layer; Eigen does NOT guarantee NaN for uninitialized
		// floats.
		map_.clearAll();

		tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
		tf_listener_ =
		    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

		pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
		    "/d455_right/point_cloud",
		    rclcpp::SensorDataQoS(),
		    std::bind(
		        &PeakFinder::pointcloud_callback, this, std::placeholders::_1
		    )
		);

		// Use transient_local so late subscribers (e.g. RViz) receive the last
		// map immediately on connection instead of seeing nothing until the
		// next publish.
		auto map_qos = rclcpp::QoS(1).transient_local().reliable();
		map_pub_     = this->create_publisher<grid_map_msgs::msg::GridMap>(
            "out/elevation_map", map_qos
        );
		peak_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
		    "/peak_position", 10
		);

		peak_timer_ = this->create_wall_timer(
		    std::chrono::seconds(2), std::bind(&PeakFinder::find_peak, this)
		);

		clear_elevation_map_srv_ = this->create_service<std_srvs::srv::Trigger>(
		    "~/clear_elevation_map",
		    std::bind(
		        &PeakFinder::clear_elevation_map_callback,
		        this,
		        std::placeholders::_1,
		        std::placeholders::_2
		    )
		);

		RCLCPP_INFO(
		    this->get_logger(),
		    "PeakFinder node initialized (service: %s).",
		    clear_elevation_map_srv_->get_service_name()
		);
	}

private:
	// ── Pointcloud callback ──────────────────────────────────────────────────
	void
	pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
		geometry_msgs::msg::TransformStamped sensor_to_map_tf;
		geometry_msgs::msg::TransformStamped robot_to_map_tf;

		try {
			sensor_to_map_tf = tf_buffer_->lookupTransform(
			    map_frame_, msg->header.frame_id, tf2::TimePointZero
			);
			robot_to_map_tf = tf_buffer_->lookupTransform(
			    map_frame_, robot_frame_, tf2::TimePointZero
			);
		} catch (const tf2::TransformException &ex) {
			RCLCPP_WARN_THROTTLE(
			    this->get_logger(),
			    *this->get_clock(),
			    2000,
			    "TF error in pointcloud_callback: %s",
			    ex.what()
			);
			return;
		}

		// Move the sliding window map to follow the robot.
		// Newly exposed cells are cleared to NaN automatically for basic
		// layers.
		grid_map::Position robot_pos(
		    robot_to_map_tf.transform.translation.x,
		    robot_to_map_tf.transform.translation.y
		);
		map_.move(robot_pos);

		// Transform cloud into map frame
		sensor_msgs::msg::PointCloud2 cloud_map;
		tf2::doTransform(*msg, cloud_map, sensor_to_map_tf);

		sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_map, "x");
		sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_map, "y");
		sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_map, "z");

		for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
			if (std::isnan(*iter_x) || std::isnan(*iter_y) ||
			    std::isnan(*iter_z)) {
				continue;
			}
			const grid_map::Position pos(*iter_x, *iter_y);
			if (!map_.isInside(pos)) {
				continue;
			}
			float &cell = map_.atPosition("elevation", pos);
			// Keep the maximum observed height per cell (max-height fusion).
			if (std::isnan(cell) || *iter_z > cell) {
				cell = *iter_z;
			}
		}

		// Always update the timestamp; publish unconditionally (transient_local
		// means the last retained message is useful even for future
		// subscribers).
		map_.setTimestamp(rclcpp::Time(msg->header.stamp).nanoseconds());
		auto map_msg = grid_map::GridMapRosConverter::toMessage(map_);
		map_pub_->publish(std::move(map_msg));
	}

	// ── Peak search timer ────────────────────────────────────────────────────
	void find_peak() {
		geometry_msgs::msg::TransformStamped robot_to_map_tf;
		try {
			robot_to_map_tf = tf_buffer_->lookupTransform(
			    map_frame_, robot_frame_, tf2::TimePointZero
			);
		} catch (const tf2::TransformException &ex) {
			RCLCPP_WARN_THROTTLE(
			    this->get_logger(),
			    *this->get_clock(),
			    2000,
			    "TF error in find_peak: %s",
			    ex.what()
			);
			return;
		}

		const grid_map::Position search_center(
		    robot_to_map_tf.transform.translation.x,
		    robot_to_map_tf.transform.translation.y
		);

		double max_elevation = -std::numeric_limits<double>::infinity();
		grid_map::Position peak_pos;
		bool               peak_found = false;

		for (grid_map::SpiralIterator it(map_, search_center, search_radius_);
		     !it.isPastEnd();
		     ++it) {
			const float z = map_.at("elevation", *it);
			if (!std::isnan(z) && z > max_elevation) {
				max_elevation = z;
				map_.getPosition(*it, peak_pos);
				peak_found = true;
			}
		}

		if (peak_found) {
			geometry_msgs::msg::PoseStamped peak_pose;
			peak_pose.header.stamp    = this->now();
			peak_pose.header.frame_id = map_frame_;
			peak_pose.pose.position.x = peak_pos.x();
			peak_pose.pose.position.y = peak_pos.y();
			peak_pose.pose.position.z = max_elevation;
			peak_pose.pose.orientation.w = 1.0;
			peak_pos_pub_->publish(peak_pose);

			RCLCPP_INFO(
			    this->get_logger(),
			    "Peak found -> X: %.2f  Y: %.2f  Z: %.2f",
			    peak_pos.x(),
			    peak_pos.y(),
			    max_elevation
			);
		} else {
			RCLCPP_DEBUG(
			    this->get_logger(),
			    "No valid elevation data within search radius."
			);
		}
	}

	void clear_elevation_map_callback(
	    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
	    std::shared_ptr<std_srvs::srv::Trigger::Response> response
	) {
		map_.clearAll();
		map_.setTimestamp(this->now().nanoseconds());
		auto map_msg = grid_map::GridMapRosConverter::toMessage(map_);
		map_pub_->publish(std::move(map_msg));

		response->success = true;
		response->message = "Elevation map cleared.";
		RCLCPP_INFO(this->get_logger(), "Elevation map cleared via service.");
	}

	// ── Members ──────────────────────────────────────────────────────────────
	std::string       map_frame_;
	std::string       robot_frame_;
	double            search_radius_;
	grid_map::GridMap map_;

	std::unique_ptr<tf2_ros::Buffer>                               tf_buffer_;
	std::shared_ptr<tf2_ros::TransformListener>                    tf_listener_;
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
	rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr      map_pub_;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr peak_pos_pub_;
	rclcpp::TimerBase::SharedPtr                                   peak_timer_;
	rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_elevation_map_srv_;
};

} // namespace kalman_arc

RCLCPP_COMPONENTS_REGISTER_NODE(kalman_arc::PeakFinder)