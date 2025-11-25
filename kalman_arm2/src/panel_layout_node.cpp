#include <geometry_msgs/msg/detail/quaternion__struct.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <yaml-cpp/yaml.h>

namespace kalman_arm2 {

struct MarkerInfo {
	double u{0.0};
	double v{0.0};
};

class PanelLayout : public rclcpp::Node {
  public:
	PanelLayout(const rclcpp::NodeOptions &options)
	    : Node("panel_layout", options) {

		yaml_path_ = declare_parameter<std::string>(
		    "layout_yaml", "panel_layout.yaml"
		);

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
			for (auto it : m) {
				int        id = it.first.as<int>();
				MarkerInfo mi;
				mi.u         = it.second["u"].as<double>();
				mi.v         = it.second["v"].as<double>();
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
