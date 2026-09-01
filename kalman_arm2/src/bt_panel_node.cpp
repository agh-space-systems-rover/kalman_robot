#include "actions/acquire_panel_pose.hpp"
#include "actions/arm_navigate_to_pose.hpp"
#include "actions/average_pose.hpp"
#include "actions/build_uv.hpp"
#include "actions/come_closer.hpp"
#include "actions/compute_panel_target.hpp"
#include "actions/do_something.hpp"
#include "actions/find_panel.hpp"
#include "actions/get_next_goal.hpp"
#include "actions/gripper_actions.hpp"
#include "actions/ik_navigate_to_pose.hpp"
#include "actions/pose_ik_navigate_to_pose.hpp"
#include "actions/rotate_ee_along_normal_joint.hpp"
#include "actions/rotate_ee_along_normal_twist.hpp"
#include "actions/set_mission_feedback.hpp"
#include "actions/say_something.hpp"
#include "actions/show_board.hpp"
#include "actions/transform_pose.hpp"
#include "actions/visual_refine_to_panel.hpp"
#include "actions/visual_refine_to_panel_twist.hpp"
#include "conditions/has_next_goal.hpp"
#include "conditions/is_panel_pose_available.hpp"
#include "conditions/is_panel_pose_fresh.hpp"
#include "conditions/is_panel_pose_good.hpp"
#include "conditions/is_recent_detection.hpp"
#include "mission_feedback.hpp"
#include "mission_state.hpp"
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h> // optional (Groot)
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <kalman_interfaces/action/move_to_panel_pose.hpp>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/create_server.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_action/server.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <rclcpp_action/types.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <unistd.h>

namespace kalman_arm2 {
namespace {
geometry_msgs::msg::Pose sanitizeRequestedPanelPose(
    const geometry_msgs::msg::Pose &input_pose
) {
	auto pose = input_pose;

	const auto &q = pose.orientation;
	const double norm_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
	if (norm_sq <= 1e-12) {
		pose.orientation.x = 0.0;
		pose.orientation.y = 0.0;
		pose.orientation.z = 0.0;
		pose.orientation.w = 1.0;
		return pose;
	}

	tf2::Quaternion normalized_q;
	tf2::fromMsg(pose.orientation, normalized_q);
	normalized_q.normalize();
	pose.orientation = tf2::toMsg(normalized_q);
	return pose;
}
} // namespace

class BTPanel : public rclcpp::Node, public MissionFeedbackHandle {
public:
	using MoveToPanelPose = kalman_interfaces::action::MoveToPanelPose;
	using GoalHandle = rclcpp_action::ServerGoalHandle<MoveToPanelPose>;

	BTPanel(const rclcpp::NodeOptions &options) : Node("bt_panel", options) {
		panel_layout_yaml_path_ = declare_parameter<std::string>("layout_yaml");
		declare_parameter<bool>("enable_zmq_publisher", false);

		declare_parameter<double>("tick_rate_hz", 20.0);
		declare_parameter<int>("visual_refinement_dof", 3);
		declare_parameter<int>("visual_refinement_max_corrections", 10);
		declare_parameter<double>("visual_refinement_max_measurement_age_s", 0.3);
		declare_parameter<double>("visual_refinement_settle_time_s", 0.4);
		declare_parameter<double>("visual_refinement_max_translation_step", 0.10);
		declare_parameter<double>("visual_refinement_max_rotation_step_deg", 5.0);
		declare_parameter<double>("visual_refinement_position_tolerance", 0.015);
		declare_parameter<double>("visual_refinement_orientation_tolerance_deg", 4.0);
		declare_parameter<double>("visual_refinement_nominal_position_tolerance", 0.01);
		declare_parameter<double>("visual_refinement_nominal_orientation_tolerance_deg", 5.0);
		declare_parameter<std::string>("visual_refinement_base_frame", "base_link");
		declare_parameter<std::string>("visual_refinement_ee_frame", "arm_link_gripper");
		declare_parameter<std::string>("visual_refinement_panel_frame", "aruco_board");
		declare_parameter<double>("visual_refinement_twist_linear_kp", 0.8);
		declare_parameter<double>("visual_refinement_twist_max_linear_speed", 0.12);
		declare_parameter<double>("visual_refinement_twist_min_linear_speed", 0.015);
		declare_parameter<double>("visual_refinement_twist_min_speed_activation_distance", 0.03);
		declare_parameter<double>("visual_refinement_twist_angular_kp", 2.0);
		declare_parameter<double>("visual_refinement_twist_max_angular_speed", 0.6);
		declare_parameter<double>("gripper_open_position", 1.57);
		declare_parameter<double>("gripper_closed_position", 0.0);
		declare_parameter<std::string>("rotate_ee_base_frame", "base_link");
		declare_parameter<std::string>("rotate_ee_frame", "arm_link_gripper");
		declare_parameter<double>("rotate_ee_angular_kp", 1.5);
		declare_parameter<double>("rotate_ee_max_angular_speed", 0.3);
		declare_parameter<double>("rotate_ee_min_angular_speed", 0.05);
		declare_parameter<double>("rotate_ee_joint_kp", 1.5);
		declare_parameter<double>("rotate_ee_joint_max_speed", 0.3);
		declare_parameter<double>("rotate_ee_joint_min_speed", 0.05);
		declare_parameter<double>("rotate_ee_tolerance_deg", 1.0);

		mission_helper_ = std::make_shared<MissionHelper>(
		    // FIXME: assumption that the panel layout was read successfully
		    MissionState{
		        PanelLayout::read_yaml(panel_layout_yaml_path_, get_logger())
		            .value()
		    }
		);

		// Register node types and preload all installed behavior tree XML files.
		configure();

		using namespace std::placeholders;
		this->actionServer = rclcpp_action::create_server<MoveToPanelPose>(
		    this,
		    "move_to_panel_pose",
		    std::bind(&BTPanel::handle_goal, this, _1, _2),
		    std::bind(&BTPanel::handle_cancel, this, _1),
		    std::bind(&BTPanel::handle_accepted, this, _1)
		);
	}

	void configure() {
		try {
			factory_ = std::make_unique<BT::BehaviorTreeFactory>();

			factory_->registerBuilder<AcquirePanelPose>(
			    "AcquirePanelPose", Builder<AcquirePanelPose>()
			);
			factory_->registerBuilder<DoSomething>(
			    "DoSomething", Builder<DoSomething>()
			);
			factory_->registerBuilder<ArmNavigateToPose>(
			    "ArmNavigateToPose", Builder<ArmNavigateToPose>()
			);
			factory_->registerBuilder<IsRecentDetection>(
			    "IsRecentDetection", Builder<IsRecentDetection>()
			);
			factory_->registerBuilder<ComeCloser>(
			    "ComeCloser", Builder<ComeCloser>()
			);
			factory_->registerBuilder<AveragePose>(
			    "AveragePose", Builder<AveragePose>()
			);
			factory_->registerBuilder<ComputePanelTarget>(
			    "ComputePanelTarget", Builder<ComputePanelTarget>()
			);
			factory_->registerBuilder<HasNextGoal>(
			    "HasNextGoal", Builder<HasNextGoal>()
			);
			factory_->registerBuilder<GetNextGoal>(
			    "GetNextGoal", Builder<GetNextGoal>()
			);
			factory_->registerBuilder<FindPanel>(
			    "FindPanel", Builder<FindPanel>()
			);
			factory_->registerBuilder<IKNavigateToPoseIterative>(
			    "IKNavigateToPoseIterative",
			    Builder<IKNavigateToPoseIterative>()
			);
			factory_->registerBuilder<PoseIKNavigateToPose>(
			    "IKNavigateToPose", Builder<PoseIKNavigateToPose>()
			);
			factory_->registerBuilder<IsPanelPoseAvailable>(
			    "IsPanelPoseAvailable", Builder<IsPanelPoseAvailable>()
			);
			factory_->registerBuilder<IsPanelPoseFresh>(
			    "IsPanelPoseFresh", Builder<IsPanelPoseFresh>()
			);
			factory_->registerBuilder<IsPanelPoseGood>(
			    "IsPanelPoseGood", Builder<IsPanelPoseGood>()
			);
			factory_->registerBuilder<SaySomething>(
			    "Say", Builder<SaySomething>()
			);
			factory_->registerBuilder<SetMissionFeedback>(
			    "SetMissionFeedback", Builder<SetMissionFeedback>()
			);
			factory_->registerBuilder<BuildUV>("BuildUV", Builder<BuildUV>());
			factory_->registerBuilder<ShowBoard>(
			    "ShowBoard", Builder<ShowBoard>()
			);
			factory_->registerBuilder<TransformPose>(
			    "TransformPose", Builder<TransformPose>()
			);
			factory_->registerBuilder<OpenGripper>(
			    "OpenGripper", Builder<OpenGripper>()
			);
			factory_->registerBuilder<CloseGripper>(
			    "CloseGripper", Builder<CloseGripper>()
			);
			factory_->registerBuilder<SetGripperAngle>(
			    "SetGripperAngle", Builder<SetGripperAngle>()
			);
			factory_->registerBuilder<RotateEEAlongNormalTwist>(
			    "RotateEEAlongNormalTwist",
			    Builder<RotateEEAlongNormalTwist>()
			);
			factory_->registerBuilder<RotateEEAlongNormalTwist>(
			    "RotateEEAlongNormal", Builder<RotateEEAlongNormalTwist>()
			);
			factory_->registerBuilder<RotateEEAlongNormalJoint>(
			    "RotateEEAlongNormalJoint",
			    Builder<RotateEEAlongNormalJoint>()
			);
			factory_->registerBuilder<RotateEEAlongNormalJoint>(
			    "RotateEEAloneNormalJoint",
			    Builder<RotateEEAlongNormalJoint>()
			);
			factory_->registerBuilder<VisualRefineToPanel>(
			    "VisualRefineToPanel", Builder<VisualRefineToPanel>()
			);
			factory_->registerBuilder<VisaulRefineToPanelTwist>(
			    "VisaulRefineToPanelTwist",
			    Builder<VisaulRefineToPanelTwist>()
			);

			const auto trees_directory = std::filesystem::path(
			    ament_index_cpp::get_package_share_directory("kalman_arm2")
			) / "trees";
			for (const auto &entry : std::filesystem::directory_iterator(trees_directory)) {
				if (!entry.is_regular_file() || entry.path().extension() != ".xml") {
					continue;
				}
				std::ifstream stream(entry.path());
				if (!stream) {
					throw std::runtime_error(
					    "Cannot read behavior tree " + entry.path().string()
					);
				}
				std::ostringstream contents;
				contents << stream.rdbuf();
				const auto name = entry.path().stem().string();
				tree_xml_by_name_[name] = contents.str();
				RCLCPP_INFO(get_logger(), "Loaded behavior tree '%s'", name.c_str());
			}
			if (tree_xml_by_name_.empty()) {
				throw std::runtime_error(
				    "No behavior tree XML files found in " + trees_directory.string()
				);
			}
		} catch (const std::exception &e) {
			RCLCPP_ERROR(get_logger(), "Failed to configure BT: %s", e.what());
			tree_.reset();
		}
		stop_requested_ = false;
	}

private:
	rclcpp_action::GoalResponse handle_goal(
	    const rclcpp_action::GoalUUID          &uuid,
	    std::shared_ptr<const MoveToPanelPose::Goal> goal
	) {
		(void)uuid;
		(void)goal;
		if (running_.load()) {
			RCLCPP_WARN(
			    get_logger(),
			    "Rejecting MoveToPanelPose goal because another mission is already running"
			);
			return rclcpp_action::GoalResponse::REJECT;
		}
		return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
	}

	rclcpp_action::CancelResponse
	handle_cancel(const std::shared_ptr<GoalHandle> goal_handle) {
		(void)goal_handle;
		return rclcpp_action::CancelResponse::ACCEPT;
	}

	void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
		auto feedback      = std::make_shared<MoveToPanelPose::Feedback>();
		feedback->progress = "checking";
		goal_handle->publish_feedback(feedback);

		const auto &tree_name = goal_handle->get_goal()->behavior_tree;
		const auto tree_xml = tree_xml_by_name_.find(tree_name);
		if (tree_xml == tree_xml_by_name_.end()) {
			auto result     = std::make_shared<MoveToPanelPose::Result>();
			result->result  = false;
			result->message = tree_name + " does not exist";
			RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
			goal_handle->abort(result);
			return;
		}

		try {
			zmq_publisher_.reset();
			tree_ = std::make_unique<BT::Tree>(
			    factory_->createTreeFromText(tree_xml->second)
			);
			tree_->rootBlackboard()->set("state", mission_helper_);
			tree_->rootBlackboard()->set(
			    "requested_panel_pose",
			    sanitizeRequestedPanelPose(goal_handle->get_goal()->target_pose)
			);
			if (get_parameter("enable_zmq_publisher").as_bool()) {
				zmq_publisher_ = std::make_unique<BT::PublisherZMQ>(*tree_);
			}
		} catch (const std::exception &e) {
			auto result     = std::make_shared<MoveToPanelPose::Result>();
			result->result  = false;
			result->message = "Failed to create " + tree_name + ": " + e.what();
			RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
			goal_handle->abort(result);
			return;
		}
		currentHandle = goal_handle;
		last_feedback_progress_.clear();
		startTicking();
	}

	void publish_progress(const std::string &progress) override {
		if (!currentHandle) {
			return;
		}
		if (progress == last_feedback_progress_) {
			return;
		}

		auto feedback      = std::make_shared<MoveToPanelPose::Feedback>();
		feedback->progress = progress;
		try {
			currentHandle->publish_feedback(feedback);
		} catch (const std::exception &e) {
			RCLCPP_WARN_STREAM(
			    get_logger(),
			    "Failed to publish mission feedback '" << progress
			                                           << "': " << e.what()
			);
			return;
		}
		last_feedback_progress_ = progress;
	}

	void startTicking() {
		if (!tree_) {
			RCLCPP_ERROR(
			    get_logger(),
			    "Cannot start BT execution: tree is not configured"
			);
			return;
		}
		if (worker_.joinable()) {
			worker_.join();
		}
		if (tree_) {
			tree_->rootNode()->halt();
		}
		double hz    = get_parameter("tick_rate_hz").as_double();
		tick_period_ = std::chrono::duration<double>(1.0 / std::max(1e-3, hz));
		stop_requested_ = false;
		running_        = true;
		worker_         = std::thread([this] {
            tickLoop();
        });
	}

	void tickLoop() {
		if (!tree_) {
			RCLCPP_ERROR(
			    get_logger(), "BT tick loop started without a valid tree"
			);
			return;
		}
		while (!stop_requested_.load()) {
			if (currentHandle && currentHandle->is_canceling()) {
				if (tree_) {
					tree_->rootNode()->halt();
				}
					auto result     = std::make_shared<MoveToPanelPose::Result>();
					result->result  = false;
					result->message = "Mission canceled";
					currentHandle->canceled(result);
					currentHandle.reset();
					last_feedback_progress_.clear();
					running_        = false;
					stop_requested_ = true;
				RCLCPP_INFO(
				    get_logger(), "Arm mission transitioned to canceled"
				);
				break;
			}

			auto status = tree_->rootNode()->executeTick();

			// Optionally react to terminal states (SUCCESS/FAILURE) by halting
			// or restarting:
			if (status != BT::NodeStatus::RUNNING) {
				RCLCPP_INFO(
				    get_logger(),
				    "Tree finished with status: %s",
				    toStr(status, true).c_str()
				);

				auto result     = std::make_shared<MoveToPanelPose::Result>();
				result->result  = (status == BT::NodeStatus::SUCCESS);
				result->message = result->result
				                      ? "Mission succeeded"
				                      : "Behavior tree returned failure";

				// Only transition if the goal is still active
				if (currentHandle && currentHandle->is_active()) {
					if (status == BT::NodeStatus::SUCCESS) {
						currentHandle->succeed(result);
					} else {
						currentHandle->abort(result);
					}
				}

				tree_->rootNode()->halt();
				currentHandle.reset();
				last_feedback_progress_.clear();
				running_        = false;
				stop_requested_ = true;
				break;
			}

			std::this_thread::sleep_for(tick_period_);

			// RCLCPP_ERROR(
			//     get_logger(),
			//     "Mission state: %s",
			//     mission_helper_->to_string().c_str()
			// );
		}
	}

	void requestStop() {
		if (!stop_requested_.exchange(true)) {
			if (worker_.joinable()) {
				worker_.join();
			}
			if (tree_) {
				tree_->rootNode()->halt();
			}
		}
		running_ = false;
	}

	template <typename T> const BT::NodeBuilder Builder() {
		return [this](
		           const std::string &name, const BT::NodeConfiguration &config
		       ) {
			return std::make_unique<T>(name, config, this);
		};
	}

	std::unique_ptr<BT::BehaviorTreeFactory> factory_;
	std::map<std::string, std::string>       tree_xml_by_name_;
	std::unique_ptr<BT::Tree>                tree_;
	std::unique_ptr<BT::PublisherZMQ>        zmq_publisher_; // optional
	std::string                              panel_layout_yaml_path_;

	std::thread                                  worker_;
	std::atomic<bool>                            stop_requested_{true};
	std::atomic<bool>                            running_{false};
	std::chrono::duration<double>                tick_period_{0.05};
	std::shared_ptr<MissionHelper>               mission_helper_;
	std::shared_ptr<GoalHandle>                  currentHandle;
	std::string                                  last_feedback_progress_;
	rclcpp_action::Server<MoveToPanelPose>::SharedPtr actionServer;
};
} // namespace kalman_arm2

RCLCPP_COMPONENTS_REGISTER_NODE(kalman_arm2::BTPanel)
