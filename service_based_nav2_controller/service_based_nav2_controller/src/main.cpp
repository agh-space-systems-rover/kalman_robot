#include <nav2_core/controller.hpp>
#include <nav2_util/geometry_utils.hpp>
#include <nav2_util/node_utils.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <service_based_nav2_controller_srvs/srv/compute_velocity_commands.hpp>

namespace service_based_nav2_controller {

class ServiceBasedNav2Controller : public nav2_core::Controller {
  public:
	ServiceBasedNav2Controller() = default;

	~ServiceBasedNav2Controller() override = default;

	void configure(
	    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
	    std::string                                     name,
	    std::shared_ptr<tf2_ros::Buffer>                tf,
	    std::shared_ptr<nav2_costmap_2d::Costmap2DROS>  costmap_ros
	) override;

	void cleanup() override;

	void activate() override;

	void deactivate() override;

	void setPlan(const nav_msgs::msg::Path &path) override;

	geometry_msgs::msg::TwistStamped computeVelocityCommands(
	    const geometry_msgs::msg::PoseStamped &pose,
	    const geometry_msgs::msg::Twist       &velocity,
	    nav2_core::GoalChecker                *goal_checker
	) override;

	virtual void
	setSpeedLimit(const double &speed_limit, const bool &percentage) override;

	rclcpp::Logger logger{rclcpp::get_logger("ServiceBasedNav2Controller")};
	rclcpp_lifecycle::LifecycleNode::SharedPtr node;
	nav_msgs::msg::Path                        path;
	rclcpp::Client<
	    service_based_nav2_controller_srvs::srv::ComputeVelocityCommands>::
	    SharedPtr cmd_vel_service_client;
};

void ServiceBasedNav2Controller::
    configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name, std::shared_ptr<tf2_ros::Buffer>, std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) {
	node = parent.lock();

	nav2_util::declare_parameter_if_not_declared(
	    node,
	    name + ".compute_velocity_commands_service",
	    rclcpp::ParameterValue("")
	);
	std::string cmd_vel_service_id =
	    node->get_parameter(name + ".compute_velocity_commands_service")
	        .as_string();

	// Create service client for cmd_vel service.
	cmd_vel_service_client = node->create_client<
	    service_based_nav2_controller_srvs::srv::ComputeVelocityCommands>(
	    cmd_vel_service_id
	);

	// Wait for cmd_vel service to become available.
	while (!cmd_vel_service_client->wait_for_service(std::chrono::seconds(1))) {
		if (!rclcpp::ok()) {
			RCLCPP_ERROR(
			    logger,
			    "Interrupted while waiting for \"%s\". Exiting.",
			    cmd_vel_service_id.c_str()
			);
			return;
		}
		RCLCPP_INFO(
		    logger,
		    "\"%s\" not available, waiting again...",
		    cmd_vel_service_id.c_str()
		);
	}
}

void ServiceBasedNav2Controller::cleanup() {
	// Destroy service client for cmd_vel service.
	cmd_vel_service_client.reset();
}

void ServiceBasedNav2Controller::activate() {}

void ServiceBasedNav2Controller::deactivate() {}

geometry_msgs::msg::TwistStamped ServiceBasedNav2Controller::
    computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose, const geometry_msgs::msg::Twist &velocity, nav2_core::GoalChecker *) {
	// Create request for cmd_vel service.
	auto request  = std::make_shared<service_based_nav2_controller_srvs::srv::
	                                     ComputeVelocityCommands::Request>();
	request->pose = pose;
	request->velocity = velocity;
	request->path     = path;

	// Call cmd_vel service.
	auto future = cmd_vel_service_client->async_send_request(request);

	// Wait for response.
	if (future.wait_for(std::chrono::seconds(1)) != std::future_status::ready) {
		RCLCPP_ERROR(
		    logger,
		    "Failed to call \"%s\".",
		    cmd_vel_service_client->get_service_name()
		);
		return geometry_msgs::msg::TwistStamped();
	}

	// Return the response.
	return future.get()->cmd_vel;
}

void ServiceBasedNav2Controller::setPlan(const nav_msgs::msg::Path &path) {
	this->path = path;
}

void ServiceBasedNav2Controller::setSpeedLimit(const double &, const bool &) {}

} // namespace service_based_nav2_controller

PLUGINLIB_EXPORT_CLASS(
    service_based_nav2_controller::ServiceBasedNav2Controller,
    nav2_core::Controller
)
