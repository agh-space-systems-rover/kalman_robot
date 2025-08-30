#include <algorithm>
#include <cmath>
#include <chrono>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <kalman_interfaces/action/arm_go_to_point.hpp>  // Goal: PoseStamped target ; Feedback: string progress

using namespace std::chrono_literals;

namespace kalman_arm2
{

namespace
{
inline tf2::Transform toTf(const geometry_msgs::msg::Transform &tmsg)
{
  tf2::Transform t;
  tf2::fromMsg(tmsg, t);
  return t;
}

inline tf2::Transform toTf(const geometry_msgs::msg::Pose &pmsg)
{
  tf2::Transform t;
  tf2::fromMsg(pmsg, t);
  return t;
}

inline geometry_msgs::msg::Pose transformPose_withTf2Transform(
    const geometry_msgs::msg::Pose &in,
    const tf2::Transform &T_parent_child)  // transform from parent->child
{
  geometry_msgs::msg::TransformStamped ts;
  ts.transform = tf2::toMsg(T_parent_child);
  geometry_msgs::msg::Pose out;
  tf2::doTransform(in, out, ts);  // rotation & translation
  return out;
}

inline geometry_msgs::msg::Vector3 angularVelToTarget(
    const geometry_msgs::msg::Quaternion &q_current_msg,
    const geometry_msgs::msg::Quaternion &q_target_msg,
    double kp = 2.0, double max_w = 2.0)
{
  tf2::Quaternion q_c, q_t;
  tf2::fromMsg(q_current_msg, q_c);
  tf2::fromMsg(q_target_msg, q_t);
  q_c.normalize();
  q_t.normalize();

  tf2::Quaternion q_err = q_c.inverse() * q_t;
  q_err.normalize();

  if (q_err.getW() < 0.0) {
    q_err = tf2::Quaternion(-q_err.getX(), -q_err.getY(), -q_err.getZ(), -q_err.getW());
  }

  double w = std::clamp(static_cast<double>(q_err.getW()), -1.0, 1.0);
  double ang = 2.0 * std::acos(w);
  double s = std::sqrt(std::max(1e-16, 1.0 - w * w));

  tf2::Vector3 axis(1.0, 0.0, 0.0);
  if (s > 1e-8) {
    axis = tf2::Vector3(q_err.getX() / s, q_err.getY() / s, q_err.getZ() / s);
  }

  tf2::Vector3 rotvec = axis * ang;
  tf2::Vector3 w_cmd = kp * rotvec;

  double n = w_cmd.length();
  if (n > max_w) w_cmd *= (max_w / n);

  geometry_msgs::msg::Vector3 out;
  out.x = w_cmd.x();
  out.y = w_cmd.y();
  out.z = w_cmd.z();
  return out;
}
}  // namespace

class IKNavigateToPoseActionServer : public rclcpp::Node
{
public:
  using ArmGoToPoint        = kalman_interfaces::action::ArmGoToPoint;
  using GoalHandle          = rclcpp_action::ServerGoalHandle<ArmGoToPoint>;

  IKNavigateToPoseActionServer(const rclcpp::NodeOptions &options)
  : rclcpp::Node("ik_navigate_to_pose_action_server", options)
  {
    // Parameters (tunable)
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<std::string>("end_effector_frame", "arm_link_end");
    declare_parameter<double>("update_rate", 20.0);
    declare_parameter<double>("kp_linear", 0.5);          // m/s per meter error
    declare_parameter<double>("max_linear_speed", 0.5);   // m/s cap
    declare_parameter<double>("pos_tolerance", 1e-3);     // m (squared check in loop)
    declare_parameter<bool>("use_orientation", false);
    declare_parameter<double>("kp_angular", 0.2);
    declare_parameter<double>("max_angular_speed", 0.3);
    declare_parameter<double>("ang_tolerance", 0.2);      // rad threshold per axis (simple check)

    get_parameter("base_frame", base_frame_);
    get_parameter("end_effector_frame", end_effector_frame_);
    get_parameter("update_rate", update_rate_);
    get_parameter("kp_linear", kp_linear_);
    get_parameter("max_linear_speed", max_linear_speed_);
    get_parameter("pos_tolerance", pos_tolerance_);
    get_parameter("use_orientation", use_orientation_);
    get_parameter("kp_angular", kp_angular_);
    get_parameter("max_angular_speed", max_angular_speed_);
    get_parameter("ang_tolerance", ang_tolerance_);

    // Publishers
    twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("target_twist", 10);
    marker_pub_ =
        create_publisher<visualization_msgs::msg::MarkerArray>("debug_markers", 10);

    // TF
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // Timer
    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, update_rate_));
    timer_ = create_wall_timer(period, std::bind(&IKNavigateToPoseActionServer::on_timer, this));

    // Action server
    using std::placeholders::_1;
    using std::placeholders::_2;

    action_server_ = rclcpp_action::create_server<ArmGoToPoint>(
        this,
        "goto_point",
        std::bind(&IKNavigateToPoseActionServer::handle_goal, this, _1, _2),
        std::bind(&IKNavigateToPoseActionServer::handle_cancel, this, _1),
        std::bind(&IKNavigateToPoseActionServer::handle_accepted, this, _1));

    RCLCPP_INFO(get_logger(), "IKNavigateToPoseActionServer ready.");
  }

private:
  // ROS interfaces
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

  // Action
  rclcpp_action::Server<ArmGoToPoint>::SharedPtr action_server_;
  std::shared_ptr<GoalHandle> current_goal_;

  // Parameters / config
  std::string base_frame_;
  std::string end_effector_frame_;
  double update_rate_{20.0};
  double kp_linear_{0.5};
  double max_linear_speed_{0.5};
  double pos_tolerance_{1e-3};
  bool   use_orientation_{false};
  double kp_angular_{0.2};
  double max_angular_speed_{0.3};
  double ang_tolerance_{0.2};

  // State
  geometry_msgs::msg::Pose target_pose_base_;
  bool have_target_{false};

  // --- Action callbacks ---
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &,
      std::shared_ptr<const ArmGoToPoint::Goal> goal)
  {
    // Validate frame existence up-front; abort if it does not exist (per requirement).
    const auto &frame = goal->target.header.frame_id;
    if (frame.empty()) {
      RCLCPP_WARN(get_logger(), "Goal rejected: target.frame_id is empty.");
      return rclcpp_action::GoalResponse::REJECT;
    }

    // We cannot throw here (no handle yet). Just check transform availability.
    const auto query_time = goal->target.header.stamp;
    const rclcpp::Time when = query_time.nanosec == 0 ? rclcpp::Time(0) : rclcpp::Time(query_time);
    // const rclcpp::Time when = query_time;

    bool ok = tf_buffer_->canTransform(base_frame_, frame, when, 100ms);
    if (!ok) {
      RCLCPP_WARN(get_logger(), "Goal rejected: transform %s -> %s not available.",
                  frame.c_str(), base_frame_.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(get_logger(), "Received goal with target in frame '%s'.", frame.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> gh)
  {
    RCLCPP_INFO(get_logger(), "Received cancel request.");
    if (current_goal_ == gh) {
      current_goal_.reset();
      have_target_ = false;
      publish_zero_twist();
      RCLCPP_INFO(get_logger(), "Goal cancelled.");
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> gh)
  {
    current_goal_ = gh;
    have_target_  = false; // will set once transformed
    RCLCPP_INFO(get_logger(), "Goal accepted.");
    // Precompute (transform) the target into base frame. Abort if any TF error.
    try {
      const auto &goal = current_goal_->get_goal()->target;

      // const rclcpp::Time when = goal.header.stamp.nanosec == 0 ? rclcpp::Time(0) : goal.header.stamp;
      const rclcpp::Time when = goal.header.stamp.nanosec == 0 ? rclcpp::Time(0) : rclcpp::Time(goal.header.stamp);
      const auto T_base_goalFrame_st = tf_buffer_->lookupTransform(
          base_frame_, goal.header.frame_id, when, 200ms);

      const tf2::Transform T_base_goalFrame = toTf(T_base_goalFrame_st.transform);
      target_pose_base_ = transformPose_withTf2Transform(goal.pose, T_base_goalFrame);
      have_target_ = true;

      // Debug marker
      publish_target_marker();

      RCLCPP_INFO(get_logger(), "Transformed target into %s.", base_frame_.c_str());
    } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(get_logger(), "Transform error: %s. Aborting goal.", ex.what());
      auto result = std::make_shared<ArmGoToPoint::Result>();
      current_goal_->abort(result);
      current_goal_.reset();
      publish_zero_twist();
    }
  }

  // --- Control loop ---
  void on_timer()
  {
    if (!current_goal_ || !have_target_) {
      return;
    }

    try {
      // Current end-effector pose (expressed in base frame)
      const auto T_base_ee_st = tf_buffer_->lookupTransform(
          base_frame_, end_effector_frame_, now() - 100ms, 100ms);
      const tf2::Transform T_base_ee = toTf(T_base_ee_st.transform);
      const geometry_msgs::msg::Pose zero_pose;
      const geometry_msgs::msg::Pose current_pose =
          transformPose_withTf2Transform(zero_pose, T_base_ee);

      // Position control (P)
      geometry_msgs::msg::TwistStamped cmd;
      cmd.header.stamp = now();
      cmd.header.frame_id = base_frame_;

      const double ex = (current_pose.position.x - target_pose_base_.position.x);
      const double ey = (current_pose.position.y - target_pose_base_.position.y);
      const double ez = (current_pose.position.z - target_pose_base_.position.z);

      // negative sign = drive toward target
      cmd.twist.linear.x = clamp(-kp_linear_ * ex, -max_linear_speed_, max_linear_speed_);
      cmd.twist.linear.y = clamp(-kp_linear_ * ey, -max_linear_speed_, max_linear_speed_);
      cmd.twist.linear.z = clamp(-kp_linear_ * ez, -max_linear_speed_, max_linear_speed_);

      // Optional orientation alignment (disabled by default)
      if (use_orientation_) {
        cmd.twist.angular = angularVelToTarget(
            current_pose.orientation, target_pose_base_.orientation,
            kp_angular_, max_angular_speed_);
      } else {
        cmd.twist.angular = geometry_msgs::msg::Vector3();
      }

      twist_pub_->publish(cmd);

      // Termination check
      const double pos_err_sq = ex*ex + ey*ey + ez*ez;
      bool pos_reached = (pos_err_sq < (pos_tolerance_ * pos_tolerance_));

      bool ang_reached = true;
      if (use_orientation_) {
        const auto a = cmd.twist.angular;
        ang_reached = (std::abs(a.x) < ang_tolerance_) &&
                      (std::abs(a.y) < ang_tolerance_) &&
                      (std::abs(a.z) < ang_tolerance_);
      }

      // Feedback
      auto fb = std::make_shared<ArmGoToPoint::Feedback>();
      fb->progress = "distance=" + std::to_string(std::sqrt(pos_err_sq)) + " m";
      current_goal_->publish_feedback(fb);

      if (pos_reached && ang_reached) {
        publish_zero_twist();
        auto result = std::make_shared<ArmGoToPoint::Result>();
        current_goal_->succeed(result);
        RCLCPP_INFO(get_logger(), "Goal succeeded.");
        current_goal_.reset();
        have_target_ = false;
      }
    } catch (const tf2::TransformException &ex) {
      // If we cannot read the current EE pose, keep trying, but throttle warnings
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF problem: %s", ex.what());
    }
  }

  // --- Utils ---
  static double clamp(double v, double lo, double hi)
  {
    return std::max(lo, std::min(v, hi));
  }

  void publish_zero_twist()
  {
    geometry_msgs::msg::TwistStamped zero;
    zero.header.stamp = now();
    zero.header.frame_id = base_frame_;
    twist_pub_->publish(zero);
  }

  void publish_target_marker()
  {
    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker m;
    m.header.frame_id = base_frame_;
    m.header.stamp = now();
    m.ns = "ik_nav_debug";
    m.id = 999;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.type = visualization_msgs::msg::Marker::CUBE;
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.pose = target_pose_base_;
    m.color.a = 0.15;
    m.color.r = 0.9;
    m.color.g = 0.0;
    m.color.b = 0.0;
    arr.markers.push_back(m);
    marker_pub_->publish(arr);
  }
};

}  // namespace kalman_arm2

RCLCPP_COMPONENTS_REGISTER_NODE(kalman_arm2::IKNavigateToPoseActionServer)
