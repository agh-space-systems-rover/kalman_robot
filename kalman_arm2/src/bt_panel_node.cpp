#include "actions/arm_navigate_to_pose.hpp"
#include "actions/set_jaw.hpp"
#include "actions/average_pose.hpp"
#include "actions/build_uv.hpp"
#include "actions/come_closer.hpp"
#include "actions/get_next_goal.hpp"
#include "actions/ik_navigate_to_pose.hpp"
#include "actions/say_something.hpp"
#include "actions/show_board.hpp"
#include "conditions/has_next_goal.hpp"
#include "conditions/is_recent_detection.hpp"
#include "mission_state.hpp"
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h> // optional (Groot)
#include <kalman_interfaces/action/arm_mission.hpp>
#include <memory>
#include <rclcpp/logging.hpp>
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
class BTPanel : public rclcpp::Node {
  public:
	using ArmMission = kalman_interfaces::action::ArmMission;
	using GoalHandle = rclcpp_action::ServerGoalHandle<ArmMission>;

	BTPanel(const rclcpp::NodeOptions &options) : Node("bt_panel", options) {
		declare_parameter<std::string>("tree_xml");

		panel_layout_yaml_path_ = declare_parameter<std::string>("layout_yaml");
		declare_parameter<bool>("enable_zmq_publisher", false);

		declare_parameter<double>("tick_rate_hz", 20.0);
		declare_parameter<bool>("auto_start", true);

		mission_helper_ = std::make_shared<MissionHelper>(
			// FIXME: assumption that the panel layout was read successfully
		    MissionState{PanelLayout::read_yaml(panel_layout_yaml_path_, get_logger()).value()}
		);

		// Build the tree
		configure();

		if (get_parameter("auto_start").as_bool()) {
			startTicking();
		}

		using namespace std::placeholders;
		this->actionServer = rclcpp_action::create_server<ArmMission>(
		    this,
		    "arm_mission",
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
			factory_->registerBuilder<HasNextGoal>(
			    "HasNextGoal", Builder<HasNextGoal>()
			);
			factory_->registerBuilder<GetNextGoal>(
			    "GetNextGoal", Builder<GetNextGoal>()
			);
			factory_->registerBuilder<IKNavigateToPose>(
			    "IKNavigateToPose", Builder<IKNavigateToPose>()
			);
			factory_->registerBuilder<SaySomething>(
			    "Say", Builder<SaySomething>()
			);
			factory_->registerBuilder<BuildUV>("BuildUV", Builder<BuildUV>());
			factory_->registerBuilder<ShowBoard>(
			    "ShowBoard", Builder<ShowBoard>()
			);
			factory_->registerBuilder<SetJaw>(
			    "SetJaw", Builder<SetJaw>()
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
	    std::shared_ptr<const ArmMission::Goal> goal
	) {
		return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
	}

	rclcpp_action::CancelResponse
	handle_cancel(const std::shared_ptr<GoalHandle> goal_handle) {
		requestStop();
		return rclcpp_action::CancelResponse::ACCEPT;
	}

	void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
		using namespace std::placeholders;

		auto feedback      = std::make_shared<ArmMission::Feedback>();
		feedback->progress = "checking";
		goal_handle->publish_feedback(feedback);

		uint8_t cmd_id = goal_handle->get_goal()->command_id;
		if (cmd_id == 1) { // FIXME: make theese enum comparisons
			currentHandle = goal_handle;
			startTicking();
		} else if (cmd_id == 2) {
			feedback->progress = "stopping";
			goal_handle->publish_feedback(feedback);
			// tree.haltTree();
		}
	}

	void startTicking() {
		double hz    = get_parameter("tick_rate_hz").as_double();
		tick_period_ = std::chrono::duration<double>(1.0 / std::max(1e-3, hz));
		stop_requested_ = false;
		running_        = true;
		worker_         = std::thread([this] {
            tickLoop();
        });
	}

	void tickLoop() {
		while (!stop_requested_.load()) {
			auto status = tree_->rootNode()->executeTick();

			// Optionally react to terminal states (SUCCESS/FAILURE) by halting
			// or restarting:
			if (status != BT::NodeStatus::RUNNING) {
				RCLCPP_INFO(
				    get_logger(),
				    "Tree finished with status: %s",
				    toStr(status, true).c_str()
				);

				auto result    = std::make_shared<ArmMission::Result>();
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
				stop_requested_ = true;
				break;
			}

			std::this_thread::sleep_for(tick_period_);

			RCLCPP_INFO_THROTTLE(
			    get_logger(),
				*get_clock(),
				2000,
			    "Mission state: %s",
			    mission_helper_->to_string().c_str()
			);
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
	std::string panel_layout_yaml_path_;

	std::thread                                  worker_;
	std::atomic<bool>                            stop_requested_{true};
	std::atomic<bool>                            running_{false};
	std::chrono::duration<double>                tick_period_{0.05};
	std::shared_ptr<MissionHelper>               mission_helper_;
	std::shared_ptr<GoalHandle>                  currentHandle;
	rclcpp_action::Server<ArmMission>::SharedPtr actionServer;
};
} // namespace kalman_arm2

RCLCPP_COMPONENTS_REGISTER_NODE(kalman_arm2::BTPanel)
