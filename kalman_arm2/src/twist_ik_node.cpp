#include <geometry_msgs/msg/twist_stamped.hpp>
#include <kalman_interfaces/msg/arm_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <std_msgs/msg/string.hpp>
#include <urdf/model.h>

#include <cmath>
#include <limits>
#include <sstream>
#include <vector>

namespace kalman_arm2 {

class TwistIK : public rclcpp::Node {
public:
	// ROS interfaces
	rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub;
	rclcpp::Subscription<kalman_interfaces::msg::ArmValues>::SharedPtr
	    joint_pos_sub;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
	    robot_description_sub;
	rclcpp::Publisher<kalman_interfaces::msg::ArmValues>::SharedPtr
	                             joint_vel_pub;
	rclcpp::TimerBase::SharedPtr compute_timer;

	// TF2
	std::shared_ptr<tf2_ros::TransformListener> tf_listener;
	std::shared_ptr<tf2_ros::Buffer>            tf_buffer;

	// Parameters
	std::string base_link;
	std::string end_effector_link;
	float       max_joint_vel;
	double      update_rate;
	double      control_timeout;
	double      base_damping;
	double      max_damping;
	double      singularity_sigma_threshold;
	double      joint_centering_gain;
	bool        enable_singularity_logging;
	double      singularity_log_period_ms;

	// Kinematics
	KDL::Chain                                arm_chain;
	std::unique_ptr<KDL::ChainJntToJacSolver> jacobian_solver;
	KDL::JntArray                             current_joint_positions;
	Eigen::VectorXd                           joint_lower_limits;
	Eigen::VectorXd                           joint_upper_limits;
	Eigen::VectorXd                           joint_preferred_positions;
	Eigen::VectorXd                           joint_motion_weights;
	Eigen::VectorXd                           singularity_avoidance_gains;
	Eigen::VectorXd                           singularity_avoidance_thresholds;
	Eigen::VectorXd                           singularity_preferred_positions;
	bool                                      joints_initialized;
	bool                                      kinematics_ready;

	// Latest twist message
	geometry_msgs::msg::TwistStamped::SharedPtr latest_twist;
	std::mutex                                  twist_mutex;
	rclcpp::Time                                last_twist_time;

	TwistIK(const rclcpp::NodeOptions &options)
	    : Node("twist_ik", options), joints_initialized(false),
	      kinematics_ready(false), last_twist_time(now()) {

		this->declare_parameter<std::string>("base_link", "base_link");
		this->declare_parameter<std::string>(
		    "end_effector_link", "arm_link_end"
		);
		this->declare_parameter<float>("max_joint_vel", 0.5);
		this->declare_parameter<double>("update_rate", 10.0);
		this->declare_parameter<double>("control_timeout", 0.5);
		this->declare_parameter<double>("base_damping", 0.03);
		this->declare_parameter<double>("max_damping", 0.35);
		this->declare_parameter<double>("singularity_sigma_threshold", 0.12);
		this->declare_parameter<double>("joint_centering_gain", 0.35);
		this->declare_parameter<bool>("enable_singularity_logging", true);
		this->declare_parameter<double>("singularity_log_period_ms", 1000.0);
		this->declare_parameter<std::vector<double>>(
		    "joint_motion_weights", std::vector<double>{}
		);
		this->declare_parameter<std::vector<double>>(
		    "singularity_avoidance_gains", std::vector<double>{}
		);
		this->declare_parameter<std::vector<double>>(
		    "singularity_avoidance_thresholds", std::vector<double>{}
		);
		this->declare_parameter<std::vector<double>>(
		    "singularity_preferred_positions", std::vector<double>{}
		);
		this->get_parameter("base_link", base_link);
		this->get_parameter("end_effector_link", end_effector_link);
		this->get_parameter("max_joint_vel", max_joint_vel);
		this->get_parameter("update_rate", update_rate);
		this->get_parameter("control_timeout", control_timeout);
		this->get_parameter("base_damping", base_damping);
		this->get_parameter("max_damping", max_damping);
		this->get_parameter(
		    "singularity_sigma_threshold", singularity_sigma_threshold
		);
		this->get_parameter("joint_centering_gain", joint_centering_gain);
		this->get_parameter(
		    "enable_singularity_logging", enable_singularity_logging
		);
		this->get_parameter(
		    "singularity_log_period_ms", singularity_log_period_ms
		);

		// Initialize TF2
		tf_buffer   = std::make_shared<tf2_ros::Buffer>(get_clock());
		tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

		// Publishers & subscribers
		joint_vel_pub = create_publisher<kalman_interfaces::msg::ArmValues>(
		    "target_vel", 10
		);

		joint_pos_sub = create_subscription<kalman_interfaces::msg::ArmValues>(
		    "current_pos",
		    10,
		    std::bind(&TwistIK::on_joint_positions, this, std::placeholders::_1)
		);

		twist_sub = create_subscription<geometry_msgs::msg::TwistStamped>(
		    "target_twist",
		    10,
		    std::bind(&TwistIK::on_target_twist, this, std::placeholders::_1)
		);

		// Create timer for periodic computation
		auto timer_period = std::chrono::duration<double>(1.0 / update_rate);
		compute_timer     = create_wall_timer(
		    timer_period, std::bind(&TwistIK::compute_joint_velocities, this)
		);

		// QoS profile for robot_description (latched topic)
		rclcpp::QoS robot_description_qos(1);
		robot_description_qos.transient_local();
		robot_description_qos.reliable();

		robot_description_sub = create_subscription<std_msgs::msg::String>(
		    "/robot_description",
		    robot_description_qos,
		    std::bind(
		        &TwistIK::on_robot_description, this, std::placeholders::_1
		    )
		);

		RCLCPP_INFO(
		    get_logger(),
		    "TwistIK node waiting for robot description on "
		    "/robot_description..."
		);
	}

	Eigen::VectorXd vector_from_parameter(
	    const std::string &name, size_t size, const Eigen::VectorXd &fallback
	) {
		auto values = this->get_parameter(name).as_double_array();
		if (values.empty()) {
			return fallback;
		}

		if (values.size() != size) {
			RCLCPP_WARN(
			    get_logger(),
			    "Parameter '%s' has %zu elements, expected %zu. Using "
			    "defaults.",
			    name.c_str(),
			    values.size(),
			    size
			);
			return fallback;
		}

		Eigen::VectorXd result(size);
		for (size_t i = 0; i < size; ++i) {
			result(i) = values[i];
		}
		return result;
	}

	std::string format_vector(const Eigen::VectorXd &values) const {
		std::ostringstream stream;
		stream << "[";
		for (Eigen::Index i = 0; i < values.size(); ++i) {
			if (i != 0) {
				stream << ", ";
			}
			stream << values(i);
		}
		stream << "]";
		return stream.str();
	}

	void on_robot_description(const std_msgs::msg::String::SharedPtr msg) {
		if (kinematics_ready) {
			return;
		}

		urdf::Model model;
		if (!model.initString(msg->data)) {
			RCLCPP_ERROR(
			    get_logger(), "Failed to parse URDF from /robot_description."
			);
			return;
		}

		KDL::Tree kdl_tree;
		if (!kdl_parser::treeFromUrdfModel(model, kdl_tree)) {
			RCLCPP_ERROR(get_logger(), "Failed to create KDL tree from URDF.");
			return;
		}

		if (!kdl_tree.getChain(base_link, end_effector_link, arm_chain)) {
			RCLCPP_ERROR(
			    get_logger(),
			    "Failed to extract KDL chain from '%s' to '%s'.",
			    base_link.c_str(),
			    end_effector_link.c_str()
			);
			return;
		}

		size_t nj       = arm_chain.getNrOfJoints();
		jacobian_solver = std::make_unique<KDL::ChainJntToJacSolver>(arm_chain);

		joint_lower_limits = Eigen::VectorXd::Constant(
		    nj, -std::numeric_limits<double>::infinity()
		);
		joint_upper_limits = Eigen::VectorXd::Constant(
		    nj, std::numeric_limits<double>::infinity()
		);
		joint_preferred_positions        = Eigen::VectorXd::Zero(nj);
		joint_motion_weights             = Eigen::VectorXd::Ones(nj);
		singularity_avoidance_gains      = Eigen::VectorXd::Zero(nj);
		singularity_avoidance_thresholds = Eigen::VectorXd::Zero(nj);
		singularity_preferred_positions  = Eigen::VectorXd::Zero(nj);

		size_t joint_index = 0;
		for (const auto &segment : arm_chain.segments) {
			const auto &joint = segment.getJoint();
			if (joint.getType() == KDL::Joint::None) {
				continue;
			}

			if (auto urdf_joint = model.getJoint(joint.getName())) {
				if (urdf_joint->limits) {
					joint_lower_limits(joint_index) = urdf_joint->limits->lower;
					joint_upper_limits(joint_index) = urdf_joint->limits->upper;
					joint_preferred_positions(joint_index) =
					    0.5 *
					    (urdf_joint->limits->lower + urdf_joint->limits->upper);
				}
			}

			++joint_index;
		}

		// Historical defaults. Keep them configurable from launch/CLI.
		if (nj > 3) {
			joint_motion_weights(3) = 10.0;
		}
		if (nj > 4) {
			joint_motion_weights(4) = 0.5;
		}
		if (nj > 5) {
			singularity_avoidance_gains(5)      = 1.2;
			singularity_avoidance_thresholds(5) = 0.35;
			singularity_preferred_positions(5)  = 1.4;
		}

		joint_motion_weights = vector_from_parameter(
		    "joint_motion_weights", nj, joint_motion_weights
		);
		singularity_avoidance_gains = vector_from_parameter(
		    "singularity_avoidance_gains", nj, singularity_avoidance_gains
		);
		singularity_avoidance_thresholds = vector_from_parameter(
		    "singularity_avoidance_thresholds",
		    nj,
		    singularity_avoidance_thresholds
		);
		singularity_preferred_positions = vector_from_parameter(
		    "singularity_preferred_positions",
		    nj,
		    singularity_preferred_positions
		);

		current_joint_positions.resize(arm_chain.getNrOfJoints());
		kinematics_ready = true;

		RCLCPP_INFO(
		    get_logger(),
		    "Kinematic chain initialized with %d joints. base_damping=%.3f "
		    "max_damping=%.3f sigma_threshold=%.3f center_gain=%.3f",
		    arm_chain.getNrOfJoints(),
		    base_damping,
		    max_damping,
		    singularity_sigma_threshold,
		    joint_centering_gain
		);
		RCLCPP_INFO(
		    get_logger(),
		    "IK config: motion_weights=%s avoidance_gains=%s "
		    "avoidance_thresholds=%s preferred_positions=%s",
		    format_vector(joint_motion_weights).c_str(),
		    format_vector(singularity_avoidance_gains).c_str(),
		    format_vector(singularity_avoidance_thresholds).c_str(),
		    format_vector(singularity_preferred_positions).c_str()
		);
	}

	void
	on_joint_positions(const kalman_interfaces::msg::ArmValues::SharedPtr msg) {
		if (!kinematics_ready) {
			return;
		}

		if (msg->joints.size() < current_joint_positions.rows()) {
			RCLCPP_WARN(
			    get_logger(),
			    "Received joint position size (%ld) < expected (%d).",
			    msg->joints.size(),
			    current_joint_positions.rows()
			);
			return;
		}

		for (size_t i = 0; i < current_joint_positions.rows(); ++i) {
			current_joint_positions(i) = msg->joints[i];
		}

		joints_initialized = true;
	}

	geometry_msgs::msg::Twist transform_twist_to_base_frame(
	    const geometry_msgs::msg::TwistStamped::SharedPtr &msg
	) {

		geometry_msgs::msg::Twist base_twist = msg->twist;

		// Transform twist to base frame if frame_id is provided and not empty
		if (!msg->header.frame_id.empty() &&
		    msg->header.frame_id != base_link) {
			try {
				// Get transform from source frame to base frame
				geometry_msgs::msg::TransformStamped transform =
				    tf_buffer->lookupTransform(
				        base_link,
				        msg->header.frame_id,
				        msg->header.stamp,
				        rclcpp::Duration::from_nanoseconds(100000000)
				    ); // 100ms timeout
				auto q = tf2::Quaternion(
				             transform.transform.rotation.x,
				             transform.transform.rotation.y,
				             transform.transform.rotation.z,
				             transform.transform.rotation.w
				)
				             .normalize();

				// Transform linear velocity
				tf2::Vector3 lin(
				    msg->twist.linear.x,
				    msg->twist.linear.y,
				    msg->twist.linear.z
				);
				tf2::Vector3 lin_base = tf2::quatRotate(q, lin);

				// Transform angular velocity
				tf2::Vector3 ang(
				    msg->twist.angular.x,
				    msg->twist.angular.y,
				    msg->twist.angular.z
				);
				tf2::Vector3 ang_base = tf2::quatRotate(q, ang);
				// ^ Note: Angular velocity is an rvec, it can be safely
				// transformed like this

				base_twist.linear.x  = lin_base.x();
				base_twist.linear.y  = lin_base.y();
				base_twist.linear.z  = lin_base.z();
				base_twist.angular.x = ang_base.x();
				base_twist.angular.y = ang_base.y();
				base_twist.angular.z = ang_base.z();

				RCLCPP_DEBUG(
				    get_logger(),
				    "Transformed twist from frame '%s' to '%s'",
				    msg->header.frame_id.c_str(),
				    base_link.c_str()
				);
			} catch (const tf2::TransformException &ex) {
				RCLCPP_WARN_THROTTLE(
				    get_logger(),
				    *get_clock(),
				    1000,
				    "Could not transform twist from '%s' to '%s': %s. Using "
				    "twist as-is.",
				    msg->header.frame_id.c_str(),
				    base_link.c_str(),
				    ex.what()
				);
			}
		}

		return base_twist;
	}

	void
	on_target_twist(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
		std::lock_guard<std::mutex> lock(twist_mutex);
		latest_twist    = msg;
		last_twist_time = now();
	}

	Eigen::VectorXd current_joint_positions_eigen() const {
		Eigen::VectorXd q(current_joint_positions.rows());
		for (size_t i = 0; i < current_joint_positions.rows(); ++i) {
			q(i) = current_joint_positions(i);
		}
		return q;
	}

	double compute_adaptive_damping(double min_sigma) const {
		if (min_sigma >= singularity_sigma_threshold) {
			return base_damping;
		}
		if (singularity_sigma_threshold <= 0.0) {
			return max_damping;
		}

		const double ratio = 1.0 - (min_sigma / singularity_sigma_threshold);
		return base_damping + (max_damping - base_damping) * ratio * ratio;
	}

	double compute_min_singular_value(const Eigen::MatrixXd &jacobian) const {
		Eigen::JacobiSVD<Eigen::MatrixXd> svd(
		    jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV
		);
		if (svd.singularValues().size() == 0) {
			return 0.0;
		}

		return svd.singularValues().minCoeff();
	}

	Eigen::VectorXd compute_posture_bias(const Eigen::VectorXd &q) const {
		Eigen::VectorXd preferred = joint_preferred_positions;
		Eigen::VectorXd bias      = joint_centering_gain * (preferred - q);

		for (Eigen::Index i = 0; i < q.size(); ++i) {
			const double gain      = singularity_avoidance_gains(i);
			const double threshold = singularity_avoidance_thresholds(i);
			if (gain <= 0.0 || threshold <= 0.0) {
				continue;
			}

			const double joint_angle = q(i);
			if (std::abs(joint_angle) < threshold) {
				const double preferred = singularity_preferred_positions(i);
				const double direction = joint_angle >= 0.0 ? 1.0 : -1.0;
				const double signed_preferred =
				    preferred == 0.0 ? 0.0 : direction * std::abs(preferred);
				const double ratio = 1.0 - (std::abs(joint_angle) / threshold);
				bias(i) += gain * ratio * (signed_preferred - joint_angle);
			}
		}

		return bias;
	}

	bool solve_joint_velocities(
	    const geometry_msgs::msg::Twist &base_twist,
	    Eigen::VectorXd                 &joint_velocities
	) {
		KDL::Jacobian kdl_jacobian(current_joint_positions.rows());
		const int     jacobian_result =
		    jacobian_solver->JntToJac(current_joint_positions, kdl_jacobian);
		if (jacobian_result < 0) {
			RCLCPP_WARN_THROTTLE(
			    get_logger(),
			    *get_clock(),
			    1000,
			    "Jacobian solver failed (code %d)",
			    jacobian_result
			);
			return false;
		}

		const Eigen::MatrixXd jacobian = kdl_jacobian.data;
		Eigen::MatrixXd       weight_inv =
		    joint_motion_weights.cwiseInverse().asDiagonal();

		Eigen::Matrix<double, 6, 1> twist_vector;
		twist_vector << base_twist.linear.x, base_twist.linear.y,
		    base_twist.linear.z, base_twist.angular.x, base_twist.angular.y,
		    base_twist.angular.z;

		const double min_sigma = compute_min_singular_value(jacobian);
		const double damping   = compute_adaptive_damping(min_sigma);
		const Eigen::Matrix<double, 6, 6> task_metric =
		    jacobian * weight_inv * jacobian.transpose() +
		    (damping * damping) * Eigen::Matrix<double, 6, 6>::Identity();

		const Eigen::MatrixXd weighted_pseudoinverse =
		    weight_inv * jacobian.transpose() *
		    task_metric.ldlt().solve(Eigen::Matrix<double, 6, 6>::Identity());

		const Eigen::VectorXd task_velocity =
		    weighted_pseudoinverse * twist_vector;

		const Eigen::VectorXd q            = current_joint_positions_eigen();
		const Eigen::VectorXd posture_bias = compute_posture_bias(q);
		const Eigen::MatrixXd nullspace_projector =
		    Eigen::MatrixXd::Identity(q.size(), q.size()) -
		    weighted_pseudoinverse * jacobian;

		joint_velocities = task_velocity + nullspace_projector * posture_bias;

		if (enable_singularity_logging) {
			RCLCPP_INFO_THROTTLE(
			    get_logger(),
			    *get_clock(),
			    static_cast<int64_t>(singularity_log_period_ms),
			    "IK diagnostics: sigma_min=%.4f damping=%.4f q=%s qdot=%s",
			    min_sigma,
			    damping,
			    format_vector(q).c_str(),
			    format_vector(joint_velocities).c_str()
			);
		}
		return true;
	}

	void compute_joint_velocities() {
		if (!kinematics_ready || !joints_initialized) {
			return;
		}

		geometry_msgs::msg::TwistStamped::SharedPtr twist_msg;
		{
			std::lock_guard<std::mutex> lock(twist_mutex);
			if (!latest_twist ||
			    (now() - last_twist_time).seconds() > control_timeout) {
				return;
			}
			twist_msg = latest_twist;
		}

		geometry_msgs::msg::Twist base_twist =
		    transform_twist_to_base_frame(twist_msg);

		Eigen::VectorXd joint_velocities;
		if (!solve_joint_velocities(base_twist, joint_velocities)) {
			return;
		}

		// Scale joint velocities to fit within max_joint_vel
		double max_computed_vel = joint_velocities.cwiseAbs().maxCoeff();
		double scale            = max_joint_vel / max_computed_vel;
		if (max_computed_vel > max_joint_vel) {
			joint_velocities *= scale;
			RCLCPP_DEBUG(
			    get_logger(),
			    "Scaling joint velocities by %.2f to fit within %.2f",
			    scale,
			    max_joint_vel
			);
		}

		// Translate to a joint velocity message
		auto vel_msg         = kalman_interfaces::msg::ArmValues();
		vel_msg.header.stamp = now();
		for (Eigen::Index i = 0; i < joint_velocities.size(); ++i) {
			vel_msg.joints[i] = joint_velocities(i);
		}

		joint_vel_pub->publish(vel_msg);
	}
};

} // namespace kalman_arm2

RCLCPP_COMPONENTS_REGISTER_NODE(kalman_arm2::TwistIK)
