#include <behaviortree_cpp_v3/basic_types.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <aruco_opencv_msgs/msg/aruco_detection.hpp>

#include <cstdint>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <limits>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <sys/types.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <thread>
#include <iostream>

class TransformEmaFilter {
  public:
	explicit TransformEmaFilter(double alpha = 0.2)
	    : alpha_(clampAlpha(alpha)), initialized_(false) {}

	/// Set smoothing factor (0<alpha<=1). Higher alpha trusts new samples more.
	void setAlpha(double a) {
		alpha_ = clampAlpha(a);
	}
	double alpha() const {
		return alpha_;
	}

	/// Clear internal state; next call to filter() will output the input.
	void reset() {
		initialized_ = false;
	}

	/// Current filtered estimate (valid after first filter() call).
	const tf2::Transform &state() const {
		return state_;
	}

	size_t get_sample_count() const {
		return samples;
	}

	/// Feed one sample; returns the smoothed transform.
	tf2::Transform filter(const tf2::Transform &sample) {
		if (!initialized_) {
			state_       = sample;
			initialized_ = true;
			return state_;
		}
		samples++;

		// --- Blend translation (EMA) ---
		const tf2::Vector3 x_prev = state_.getOrigin();
		const tf2::Vector3 x_new  = sample.getOrigin();
		const tf2::Vector3 x_out  = (1.0 - alpha_) * x_prev + alpha_ * x_new;

		// --- Blend rotation (Quaternion SLERP with hemisphere fix) ---
		tf2::Quaternion q_prev = state_.getRotation();
		tf2::Quaternion q_new  = sample.getRotation();
		// Ensure shortest-path interpolation
		if (q_prev.dot(q_new) < 0.0) {
			q_new =
			    tf2::Quaternion(-q_new.x(), -q_new.y(), -q_new.z(), -q_new.w());
		}
		tf2::Quaternion q_out = q_prev.slerp(q_new, alpha_);
		q_out.normalize();

		state_.setOrigin(x_out);
		state_.setRotation(q_out);
		return state_;
	}

  private:
	static double clampAlpha(double a) {
		// constrain to sensible range (avoid 0 to keep filter “alive”)
		const double eps = 1e-9;
		return std::min(1.0, std::max(eps, a));
	}

	double         alpha_;
	bool           initialized_;
	size_t         samples{0};
	tf2::Transform state_;
};

class AveragePose : public BT::StatefulActionNode
{
public:
  AveragePose(
	  const std::string           &name,
	  const BT::NodeConfiguration &config,
	  rclcpp::Node                *parent
  );

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void
  aruco_callback(const aruco_opencv_msgs::msg::ArucoDetection::SharedPtr msg);

  void update_board_from_marker(
	  const geometry_msgs::msg::TransformStamped &pose_stamped,
	  uint16_t                                    marker_id
  );

  void mark_done_in_mission_helper();

  rclcpp::Node *parent_;
  rclcpp::Subscription<aruco_opencv_msgs::msg::ArucoDetection>::SharedPtr aruco_sub_;

  aruco_opencv_msgs::msg::ArucoDetection last_aruco_msg;
  constexpr static uint16_t MARKER_INV = std::numeric_limits<uint16_t>::max();
  uint16_t tracked_marker = MARKER_INV;
  std::optional<geometry_msgs::msg::PoseStamped> last_aruco_pose{};

	rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr arm_pub_;
	std::unique_ptr<tf2_ros::Buffer>                     tf_buffer_;
	std::shared_ptr<tf2_ros::TransformListener>          tf_listener_;
	std::shared_ptr<tf2_ros::TransformBroadcaster>       tf_broadcaster_;
	std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  
  TransformEmaFilter filter; // FIXME: should be reset with onHalted
};
