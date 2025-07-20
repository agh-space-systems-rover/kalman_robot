#include <kalman_interfaces/action/detail/arm_joint_move__struct.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <kalman_interfaces/msg/arm_joint_values.hpp>
#include <kalman_interfaces/action/arm_joint_move.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <std_msgs/msg/string.hpp>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/jntarray.hpp>

namespace kalman_arm2 {

class TwistIK : public rclcpp::Node {
public:
    // ROS interfaces
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub;
    rclcpp::Subscription<kalman_interfaces::msg::ArmJointValues>::SharedPtr joint_pos_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub;
    rclcpp::Publisher<kalman_interfaces::msg::ArmJointValues>::SharedPtr joint_vel_pub;
    rclcpp::TimerBase::SharedPtr compute_timer;

    using ArmJointMove = kalman_interfaces::action::ArmJointMove;
    using GoalHandleJointMove = rclcpp_action::ServerGoalHandle<ArmJointMove>;

    // TF2
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;

    // Parameters
    std::string base_link;
    std::string end_effector_link;
    float max_joint_vel;
    double update_rate;

    // Kinematics
    KDL::Chain arm_chain;
    std::unique_ptr<KDL::ChainIkSolverVel_wdls> ik_solver;
    KDL::JntArray current_joint_positions;
    bool joints_initialized;
    bool kinematics_ready;

    // Latest twist message
    geometry_msgs::msg::TwistStamped::SharedPtr latest_twist;
    std::mutex twist_mutex;

    TwistIK(const rclcpp::NodeOptions &options)
        : Node("twist_ik", options), joints_initialized(false), kinematics_ready(false) {
        
        this->declare_parameter<std::string>("base_link", "base_link");
        this->declare_parameter<std::string>("end_effector_link", "arm_link_end");
        this->declare_parameter<float>("max_joint_vel", 0.5);
        this->declare_parameter<double>("update_rate", 10.0);
        this->get_parameter("base_link", base_link);
        this->get_parameter("end_effector_link", end_effector_link);
        this->get_parameter("max_joint_vel", max_joint_vel);
        this->get_parameter("update_rate", update_rate);

        // Initialize TF2
        tf_buffer = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        // Publishers & subscribers
        joint_vel_pub = create_publisher<kalman_interfaces::msg::ArmJointValues>("target_vel", 10);

        joint_pos_sub = create_subscription<kalman_interfaces::msg::ArmJointValues>(
            "current_pos", 10,
            std::bind(&TwistIK::on_joint_positions, this, std::placeholders::_1));

        twist_sub = create_subscription<geometry_msgs::msg::TwistStamped>(
            "target_twist", 10,
            std::bind(&TwistIK::on_target_twist, this, std::placeholders::_1));

        // Create timer for periodic computation
        auto timer_period = std::chrono::duration<double>(1.0 / update_rate);
        compute_timer = create_wall_timer(
            timer_period,
            std::bind(&TwistIK::compute_joint_velocities, this));

        // QoS profile for robot_description (latched topic)
        rclcpp::QoS robot_description_qos(1);
        robot_description_qos.transient_local();
        robot_description_qos.reliable();

        robot_description_sub = create_subscription<std_msgs::msg::String>(
            "/robot_description", robot_description_qos,
            std::bind(&TwistIK::on_robot_description, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "TwistIK node waiting for robot description on /robot_description...");
        // timer_ = this->create_wall_timer(
        //     std::chrono::milliseconds{500}, [this](){stupid_test();});

    }
    rclcpp::TimerBase::SharedPtr timer_;

    void on_robot_description(const std_msgs::msg::String::SharedPtr msg) {
        if (kinematics_ready) return;

        urdf::Model model;
        if (!model.initString(msg->data)) {
            RCLCPP_ERROR(get_logger(), "Failed to parse URDF from /robot_description.");
            return;
        }

        KDL::Tree kdl_tree;
        if (!kdl_parser::treeFromUrdfModel(model, kdl_tree)) {
            RCLCPP_ERROR(get_logger(), "Failed to create KDL tree from URDF.");
            return;
        }

        if (!kdl_tree.getChain(base_link, end_effector_link, arm_chain)) {
            RCLCPP_ERROR(get_logger(), "Failed to extract KDL chain from '%s' to '%s'.",
                         base_link.c_str(), end_effector_link.c_str());
            return;
        }

        // init joint weights
        size_t nj = arm_chain.getNrOfJoints();
        Eigen::MatrixXd joint_weights = Eigen::MatrixXd::Identity(nj, nj);
        // 4th joint often flips 180 degrees when 5th goes through zero, let's dampen it
        joint_weights(3, 3) = 10.0;
        joint_weights(4, 4) = 0.5;

        ik_solver = std::make_unique<KDL::ChainIkSolverVel_wdls>(arm_chain);
        ik_solver->setWeightJS(joint_weights);
        ik_solver->setLambda(0.5);

        current_joint_positions.resize(arm_chain.getNrOfJoints());
        kinematics_ready = true;

        RCLCPP_INFO(get_logger(), "Kinematic chain initialized with %d joints.", arm_chain.getNrOfJoints());
    }

    void on_joint_positions(const kalman_interfaces::msg::ArmJointValues::SharedPtr msg) {
        if (!kinematics_ready) return;

        if (msg->joints.size() < current_joint_positions.rows()) {
            RCLCPP_WARN(get_logger(), "Received joint position size (%ld) < expected (%d).",
                        msg->joints.size(), current_joint_positions.rows());
            return;
        }

        for (size_t i = 0; i < current_joint_positions.rows(); ++i) {
            current_joint_positions(i) = msg->joints[i];
        }

        joints_initialized = true;
    }

    geometry_msgs::msg::Twist transform_twist_to_base_frame(
        const geometry_msgs::msg::TwistStamped::SharedPtr& msg) {
        
        geometry_msgs::msg::Twist base_twist = msg->twist;
        
        // Transform twist to base frame if frame_id is provided and not empty
        if (!msg->header.frame_id.empty() && msg->header.frame_id != base_link) {
            try {
                // Get transform from source frame to base frame
                geometry_msgs::msg::TransformStamped transform = tf_buffer->lookupTransform(
                    base_link, msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_nanoseconds(100000000)); // 100ms timeout
                auto q = tf2::Quaternion(
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w
                ).normalize();

                // Transform linear velocity
                tf2::Vector3 lin(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
                tf2::Vector3 lin_base = tf2::quatRotate(q, lin);

                // Transform angular velocity
                tf2::Vector3 ang(msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z);
                tf2::Vector3 ang_base = tf2::quatRotate(q, ang);
                // ^ Note: Angular velocity is an rvec, it can be safely transformed like this

                base_twist.linear.x = lin_base.x();
                base_twist.linear.y = lin_base.y();
                base_twist.linear.z = lin_base.z();
                base_twist.angular.x = ang_base.x();
                base_twist.angular.y = ang_base.y();
                base_twist.angular.z = ang_base.z();
                
                RCLCPP_DEBUG(get_logger(), "Transformed twist from frame '%s' to '%s'", 
                           msg->header.frame_id.c_str(), base_link.c_str());
            } catch (const tf2::TransformException &ex) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                     "Could not transform twist from '%s' to '%s': %s. Using twist as-is.",
                                     msg->header.frame_id.c_str(), base_link.c_str(), ex.what());
            }
        }
        
        return base_twist;
    }

    void on_target_twist(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(twist_mutex);
        latest_twist = msg;
    }

    void compute_joint_velocities() {
        if (!kinematics_ready || !joints_initialized) {
            return;
        }

        geometry_msgs::msg::TwistStamped::SharedPtr twist_msg;
        {
            std::lock_guard<std::mutex> lock(twist_mutex);
            if (!latest_twist) {
                return;
            }
            twist_msg = latest_twist;
        }

        geometry_msgs::msg::Twist base_twist = transform_twist_to_base_frame(twist_msg);

        KDL::Twist target_twist;
        target_twist.vel.x(base_twist.linear.x);
        target_twist.vel.y(base_twist.linear.y);
        target_twist.vel.z(base_twist.linear.z);
        target_twist.rot.x(base_twist.angular.x);
        target_twist.rot.y(base_twist.angular.y);
        target_twist.rot.z(base_twist.angular.z);

        KDL::JntArray joint_velocities(current_joint_positions.rows());

        int result = ik_solver->CartToJnt(current_joint_positions, target_twist, joint_velocities);
        if (result < 0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "IK velocity solver failed (code %d)", result);
            return;
        }

        // Scale joint velocities to fit within max_joint_vel
        float max_computed_vel = joint_velocities.data.maxCoeff();
        float scale = max_joint_vel / max_computed_vel;
        if (max_computed_vel > max_joint_vel) {
            for (size_t i = 0; i < joint_velocities.rows(); ++i) {
                joint_velocities(i) *= scale;
            }
            RCLCPP_DEBUG(get_logger(), "Scaling joint velocities by %.2f to fit within %.2f", scale, max_joint_vel);
        }

        // Translate to a joint velocity message
        auto vel_msg = kalman_interfaces::msg::ArmJointValues();
        vel_msg.header.stamp = now();
        for (size_t i = 0; i < joint_velocities.rows(); ++i) {
            vel_msg.joints[i] = joint_velocities(i);
        }
        vel_msg.jaw = 0.0;

        joint_vel_pub->publish(vel_msg);
    }

    void stupid_test() {
        auto vel_msg = kalman_interfaces::msg::ArmJointValues();
        vel_msg.header.stamp = now();
        // for (size_t i = 0; i < joint_velocities.rows(); ++i) {
        //     vel_msg.joints[i] = joint_velocities(i);
        // }
        for (auto &v : vel_msg.joints) {
            v = 1.0;
        }
        vel_msg.jaw = 0.0;

        joint_vel_pub->publish(vel_msg);

        RCLCPP_INFO(get_logger(), "Sending some shit");
    }
};

} // namespace kalman_arm2

RCLCPP_COMPONENTS_REGISTER_NODE(kalman_arm2::TwistIK)
