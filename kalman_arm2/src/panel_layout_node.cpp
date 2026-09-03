#include <geometry_msgs/msg/detail/quaternion__struct.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <yaml-cpp/yaml.h>

#include <cmath>

namespace kalman_arm2 {

struct MarkerInfo {
		double u{0.0};
		double v{0.0};
		double yaw_rad{0.0};
	};

class PanelLayout : public rclcpp::Node {
public:
	PanelLayout(const rclcpp::NodeOptions &options)
	    : Node("panel_layout", options) {

		yaml_path_ =
		    declare_parameter<std::string>("layout_yaml", "panel_layout.yaml");

		static_broadcaster_ =
		    std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

		loadLayout();
		publishLayout();
	}

private:
	void loadLayout() {
		try {
			YAML::Node config = YAML::LoadFile(yaml_path_);
			marker_size_      = config["marker_size"].as<double>();
			board_w_          = config["board_width"].as<double>();
			board_h_          = config["board_height"].as<double>();
			YAML::Node m      = config["markers"];
			markers_.clear();
			for (const auto &entry : m) {
				const int id = entry.second["id"].as<int>();
				MarkerInfo mi;
				mi.u         = entry.second["u"].as<double>();
				mi.v         = entry.second["v"].as<double>();
				mi.yaw_rad   = entry.second["yaw_deg"].as<double>(0.0) *
				               M_PI / 180.0;
				markers_[id] = mi;
			}
			RCLCPP_INFO(
			    get_logger(),
			    "Loaded %zu markers from %s",
			    markers_.size(),
			    yaml_path_.c_str()
			);
		} catch (const std::exception &e) {
			RCLCPP_FATAL(
			    get_logger(),
			    "Failed to load YAML '%s': %s",
			    yaml_path_.c_str(),
			    e.what()
			);
			throw;
		}
	}

	void publishLayout() {
		for (const auto &[id, marker_info] : markers_) {
			const std::string base_frame = "aruco_board";
			const std::string marker_frame =
			    "model_marker_" + std::to_string(id);
			geometry_msgs::msg::TransformStamped st;
			st.header.stamp            = now();
			st.header.frame_id         = base_frame;
			st.child_frame_id          = marker_frame;
			st.transform.translation.x = marker_info.u - board_w_ * 0.5;
			st.transform.translation.y = -(marker_info.v - board_h_ * 0.5);
			tf2::Quaternion rotation;
			rotation.setRPY(0.0, 0.0, marker_info.yaw_rad);
			st.transform.rotation = tf2::toMsg(rotation);
			static_broadcaster_->sendTransform(st);
			RCLCPP_INFO(
			    get_logger(),
			    "Published static TF %s -> %s",
			    st.header.frame_id.c_str(),
			    st.child_frame_id.c_str()
			);
		}
	}

	std::string yaml_path_, base_frame_, board_frame_, marker_prefix_;
	double      marker_size_, board_w_, board_h_;
	std::map<int, MarkerInfo>                            markers_;
	std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
};

} // namespace kalman_arm2

RCLCPP_COMPONENTS_REGISTER_NODE(kalman_arm2::PanelLayout)
