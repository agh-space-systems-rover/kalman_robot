#include "conditions/is_panel_pose_good.hpp"

#include <algorithm>
#include <cmath>

IsPanelPoseGood::IsPanelPoseGood(
    const std::string           &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node                *parent
)
    : BT::SimpleConditionNode(
          name, std::bind(&IsPanelPoseGood::tick, this), config
      ),
      parent_(parent) {
	panel_sub_ = parent_->create_subscription<
	    geometry_msgs::msg::PoseWithCovarianceStamped>(
	    "panel_pose",
	    10,
	    std::bind(
	        &IsPanelPoseGood::panel_pose_callback, this, std::placeholders::_1
	    )
	);
}

BT::PortsList IsPanelPoseGood::providedPorts() {
	return {
	    BT::InputPort<double>("max_age_ms"),
	    BT::InputPort<double>("max_position_error"),
	    BT::InputPort<double>("max_normal_error_deg"),
	};
}

void IsPanelPoseGood::panel_pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg
) {
	last_panel_pose_ = *msg;
}

bool IsPanelPoseGood::pose_is_finite(
    const geometry_msgs::msg::PoseWithCovarianceStamped &pose
) const {
	const auto &p = pose.pose.pose.position;
	const auto &q = pose.pose.pose.orientation;
	return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
	       std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) &&
	       std::isfinite(q.w) &&
	       std::all_of(
	           pose.pose.covariance.begin(),
	           pose.pose.covariance.end(),
	           [](double v) {
		           return std::isfinite(v);
	           }
	       );
}

BT::NodeStatus IsPanelPoseGood::tick() {
	if (!last_panel_pose_.has_value()) {
		RCLCPP_WARN_STREAM(
		    parent_->get_logger(), name() << " failed: no panel pose available"
		);
		return BT::NodeStatus::FAILURE;
	}

	const auto   max_age_ms_input = getInput<double>("max_age_ms");
	const double max_age_ms =
	    max_age_ms_input ? max_age_ms_input.value() : 300.0;
	const double age_ms =
	    (parent_->now() - rclcpp::Time(last_panel_pose_->header.stamp))
	        .seconds() *
	    1000.0;

	if (age_ms > max_age_ms || !pose_is_finite(*last_panel_pose_)) {
		if (age_ms > max_age_ms) {
			RCLCPP_WARN_STREAM(
			    parent_->get_logger(),
			    name() << " failed: panel pose too old (" << age_ms << " ms > "
			           << max_age_ms << " ms)"
			);
		} else {
			RCLCPP_WARN_STREAM(
			    parent_->get_logger(),
			    name() << " failed: panel pose contains non-finite values"
			);
		}
		return BT::NodeStatus::FAILURE;
	}

	const auto max_position_error_input =
	    getInput<double>("max_position_error");
	const auto max_normal_error_deg_input =
	    getInput<double>("max_normal_error_deg");
	const double max_position_error =
	    max_position_error_input ? max_position_error_input.value() : 0.03;
	const double max_normal_error_rad =
	    (max_normal_error_deg_input ? max_normal_error_deg_input.value() : 8.0
	    ) *
	    M_PI / 180.0;

	const auto  &cov = last_panel_pose_->pose.covariance;
	const double position_stddev =
	    std::sqrt(std::max({cov[0], cov[7], cov[14], 0.0}));
	const double orientation_stddev =
	    std::sqrt(std::max({cov[21], cov[28], cov[35], 0.0}));

	if (position_stddev > max_position_error ||
	    orientation_stddev > max_normal_error_rad) {
		RCLCPP_WARN_STREAM(
		    parent_->get_logger(),
		    name() << " failed: panel covariance too large (pos_stddev="
		           << position_stddev
		           << ", rot_stddev_rad=" << orientation_stddev << ")"
		);
		return BT::NodeStatus::FAILURE;
	}
	return BT::NodeStatus::SUCCESS;
}
