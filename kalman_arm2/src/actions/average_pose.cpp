#include "actions/average_pose.hpp"
#include "mission_state.hpp"
// #include "transform_ema_filter.hpp"
#include <behaviortree_cpp_v3/basic_types.h>
#include <cstdint>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/logging.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

AveragePose::AveragePose(
    const std::string           &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node                *parent
)
    : BT::StatefulActionNode(name, config), parent_{parent} {

	aruco_sub_ =
	    parent_->create_subscription<aruco_opencv_msgs::msg::ArucoDetection>(
	        "/d455_arm/aruco_detections", // topic name
	        10,                           // queue size
	        std::bind(&AveragePose::aruco_callback, this, std::placeholders::_1)
	    );

	arm_pub_ = parent_->create_publisher<geometry_msgs::msg::TwistStamped>(
	    "target_twist", 10
	);

	RCLCPP_ERROR_STREAM(
	    parent_->get_logger(),
	    this->name() << " AveragePose constructor is being run..."
	);
	tf_buffer_ = std::make_unique<tf2_ros::Buffer>(parent_->get_clock());
	// tf_buffer_->setUsingDedicatedThread(true);
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
	static_broadcaster_ =
	    std::make_shared<tf2_ros::StaticTransformBroadcaster>(parent_);

	tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(parent_);
}

BT::PortsList AveragePose::providedPorts() {
	return {
	    BT::OutputPort<geometry_msgs::msg::PoseStamped>("aruco_position"),
	    BT::InputPort<std::shared_ptr<MissionHelper>>("mission_helper"),
	    BT::InputPort<uint16_t>("aruco_id")
	};
}

BT::NodeStatus AveragePose::onStart() {
	const auto aruco_id_opt = getInput<uint16_t>("aruco_id");
	if (!aruco_id_opt.has_value()) {
		RCLCPP_ERROR_STREAM(
		    parent_->get_logger(), name() << ": aruco_id is empty, failing"
		);
		return BT::NodeStatus::FAILURE;
	}
	tracked_marker = aruco_id_opt.value();

	filter = TransformEmaFilter{}; // reset filter for new marker

	return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AveragePose::onRunning() {
	const std::string base_frame = "base_link";
	const std::string frame      = "marker_" + std::to_string(tracked_marker);
	auto              ts         = tf_buffer_->lookupTransform(
        base_frame, frame, tf2::TimePointZero, tf2::durationFromSec(0.05)
    );

	tf2::Transform T_base_marker;
	tf2::fromMsg(ts.transform, T_base_marker);

	tf2::Transform filtered        = filter.filter(T_base_marker);
	bool           publish_static_ = true;
	if (publish_static_) {
		geometry_msgs::msg::TransformStamped st;
		st.header.stamp    = parent_->now();
		st.header.frame_id = base_frame;
		st.child_frame_id  = frame + std::string("_avg");
		st.transform       = tf2::toMsg(filtered);
		static_broadcaster_->sendTransform(st);
		RCLCPP_INFO(
		    parent_->get_logger(),
		    "Published static TF %s -> %s",
		    st.header.frame_id.c_str(),
		    st.child_frame_id.c_str()
		);
		update_board_from_marker(st, tracked_marker);
	}
	if (filter.get_sample_count() > 10) {
		RCLCPP_ERROR_STREAM(
		    parent_->get_logger(), name() << ": returning SUCCESS"
		);
    mark_done_in_mission_helper();
		return BT::NodeStatus::SUCCESS;
	}

	RCLCPP_INFO_STREAM(parent_->get_logger(), name() << ": returning RUNNING");
	return BT::NodeStatus::RUNNING;
}

void AveragePose::onHalted() {
	tracked_marker  = MARKER_INV;
	last_aruco_pose = {};
	// TODO: reset the filter
	filter = TransformEmaFilter{};
}

void AveragePose::aruco_callback(
    const aruco_opencv_msgs::msg::ArucoDetection::SharedPtr msg
) {
	// RCLCPP_INFO_STREAM(
	//     parent_->get_logger(), name() << " Got aruco marker msg"
	// );
	if (tracked_marker == MARKER_INV) {
		last_aruco_msg = *msg;
	} else {
		const auto &markers = msg->markers;

		const auto is_currently_selected_marker =
		    [&](const aruco_opencv_msgs::msg::MarkerPose &mp) -> bool {
			return mp.marker_id == tracked_marker;
		};
		const auto found_it = std::find_if(
		    std::begin(markers), std::end(markers), is_currently_selected_marker
		);
		if (found_it == std::end(markers)) {
			return;
		}

		const auto                      my_marker = *found_it;
		geometry_msgs::msg::PoseStamped pose;
		pose.pose       = my_marker.pose;
		pose.header     = msg->header;
		last_aruco_pose = pose;
	}
}

namespace {
static tf2::Transform toTf(const geometry_msgs::msg::Transform &tmsg) {
	tf2::Transform t;
	tf2::fromMsg(tmsg, t);
	return t;
}
static tf2::Transform toTf(const geometry_msgs::msg::Pose &pmsg) {
	tf2::Transform t;
	tf2::fromMsg(pmsg, t);
	return t;
}
static geometry_msgs::msg::Pose transformPose_withTf2Transform(
    const geometry_msgs::msg::Pose &in,
    const tf2::Transform           &T_parent_child
) // transform from parent->child
{
	geometry_msgs::msg::TransformStamped ts;
	ts.transform = tf2::toMsg(T_parent_child); // fill only the transform part

	geometry_msgs::msg::Pose out;
	tf2::doTransform(in, out, ts); // applies both rotation & translation
	return out;                    // pose now expressed in 'parent' frame
}
} // namespace

void AveragePose::update_board_from_marker(
    const geometry_msgs::msg::TransformStamped &pose_stamped, uint16_t marker_id
) {
	const std::string base_frame_   = "base_link";
	const std::string camera_frame_ = "base_link";
	const std::string board_frame_  = "aruco_board";
	const std::string marker_frame_ =
	    "model_marker_" + std::to_string(marker_id);
	const auto stamp = pose_stamped.header.stamp;
	try {
		// 1) base -> camera (from robot tree, at detection time)
		auto base_T_cam_st = tf_buffer_->lookupTransform(
		    base_frame_,
		    camera_frame_,
		    stamp,
		    rclcpp::Duration::from_seconds(0.1)
		);
		tf2::Transform T_base_cam = toTf(base_T_cam_st.transform);

		// 2) camera -> marker (from detector message)
		tf2::Transform T_cam_mark = toTf(pose_stamped.transform);

		// 3) board -> marker (from fixture tree, static)
		auto board_T_mark_st = tf_buffer_->lookupTransform(
		    board_frame_, marker_frame_, rclcpp::Time(0)
		);
		tf2::Transform T_board_mark = toTf(board_T_mark_st.transform);

		// 4) Compose: base -> board
		tf2::Transform T_base_board =
		    T_base_cam * T_cam_mark * T_board_mark.inverse();

		// 5) Publish as dynamic TF
		geometry_msgs::msg::TransformStamped out;
		out.header.stamp    = stamp;
		out.header.frame_id = base_frame_;
		out.child_frame_id  = board_frame_;
		out.transform       = tf2::toMsg(T_base_board);
		tf_broadcaster_->sendTransform(out);
	} catch (const tf2::TransformException &ex) {
		RCLCPP_WARN_THROTTLE(
		    parent_->get_logger(),
		    *parent_->get_clock(),
		    2000,
		    "TF problem: %s",
		    ex.what()
		);
	}
}
void AveragePose::mark_done_in_mission_helper() {
	auto mission_helper_opt = getInput<std::shared_ptr<MissionHelper>>("mission_helper");
  if (!mission_helper_opt.has_value()){
    RCLCPP_ERROR_STREAM(parent_->get_logger(), name() << ": mission helper not set");
    return;
  }
  auto mission_helper = mission_helper_opt.value();
  if (nullptr == mission_helper){
    RCLCPP_ERROR_STREAM(parent_->get_logger(), name() << ": mission helper is nullptr");
    return;
  }
  mission_helper->state.visited_map_[this->tracked_marker] = true;
}

