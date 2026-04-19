#include "actions/acquire_panel_pose.hpp"
#include "actions/arm_navigate_to_pose.hpp"
#include "actions/average_pose.hpp"
#include "actions/build_uv.hpp"
#include "actions/come_closer.hpp"
#include "actions/compute_panel_target.hpp"
#include "actions/do_something.hpp"
#include "actions/get_next_goal.hpp"
#include "actions/ik_navigate_to_pose.hpp"
#include "actions/pose_ik_navigate_to_pose.hpp"
#include "actions/set_mission_feedback.hpp"
#include "actions/say_something.hpp"
#include "actions/show_board.hpp"
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
#include <kalman_interfaces/action/move_to_panel_pose.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/create_server.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_action/server.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <rclcpp_action/types.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <unistd.h>

namespace kalman_arm2 {
class BTPanel : public rclcpp::Node, public MissionFeedbackHandle {
public:
	using MoveToPanelPose = kalman_interfaces::action::MoveToPanelPose;
	using GoalHandle = rclcpp_action::ServerGoalHandle<MoveToPanelPose>;

	BTPanel(const rclcpp::NodeOptions &options) : Node("bt_panel", options) {
		declare_parameter<std::string>("tree_xml");

		panel_layout_yaml_path_ = declare_parameter<std::string>("layout_yaml");
		declare_parameter<bool>("enable_zmq_publisher", false);

		declare_parameter<double>("tick_rate_hz", 20.0);
		declare_parameter<bool>("auto_start", true);

		mission_helper_ = std::make_shared<MissionHelper>(
		    // FIXME: assumption that the panel layout was read successfully
		    MissionState{
		        PanelLayout::read_yaml(panel_layout_yaml_path_, get_logger())
		            .value()
		    }
		);

		// Build the tree
		configure();

		if (tree_ && get_parameter("auto_start").as_bool()) {
			startTicking();
		}

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
		const auto tree_xml = get_parameter("tree_xml").as_string();
		// Load XML (supports package:// if you use BT factory helpers)

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

			// Build tree
			tree_ = std::make_unique<BT::Tree>(
			    factory_->createTreeFromFile(tree_xml)
			);

			const auto bboard = tree_->rootBlackboard();
			bboard->set("state", mission_helper_);

			// Optional: publish to Groot (set enable_zmq_publisher=true)
			if (get_parameter("enable_zmq_publisher").as_bool()) {
				zmq_publisher_ = std::make_unique<BT::PublisherZMQ>(*tree_);
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

		if (!tree_) {
			auto result    = std::make_shared<MoveToPanelPose::Result>();
			result->result = false;
			goal_handle->abort(result);
			return;
		}

		tree_->rootBlackboard()->set(
		    "requested_panel_pose", goal_handle->get_goal()->target_pose
		);
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
					auto result    = std::make_shared<MoveToPanelPose::Result>();
					result->result = false;
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

				auto result    = std::make_shared<MoveToPanelPose::Result>();
				result->result = (status == BT::NodeStatus::SUCCESS);

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
