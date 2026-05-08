#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "unitree_go/msg/sport_mode_state.hpp"

namespace
{

double clamp(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(max_value, value));
}

double wrap_to_pi(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double yaw_from_quaternion(double x, double y, double z, double w)
{
  tf2::Quaternion q(x, y, z, w);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

double yaw_from_wxyz(const std::array<float, 4> & quat_wxyz)
{
  return yaw_from_quaternion(
    static_cast<double>(quat_wxyz[1]),
    static_cast<double>(quat_wxyz[2]),
    static_cast<double>(quat_wxyz[3]),
    static_cast<double>(quat_wxyz[0]));
}

struct BasePose
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

bool is_finite_value(double value)
{
  return std::isfinite(value);
}

bool finite_pose(const BasePose & pose)
{
  return is_finite_value(pose.x) && is_finite_value(pose.y) && is_finite_value(pose.yaw);
}

}  // namespace

class GoalToCmdVelNode : public rclcpp::Node
{
public:
  GoalToCmdVelNode()
  : Node("goal_to_cmd_vel"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    world_frame_ = declare_parameter<std::string>("world_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base");
    goal_pose_topic_ = declare_parameter<std::string>("goal_pose_topic", "/goal_pose");
    clicked_point_topic_ = declare_parameter<std::string>("clicked_point_topic", "/clicked_point");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    sport_mode_state_topic_ =
      declare_parameter<std::string>("sport_mode_state_topic", "/sportmodestate");

    use_tf_pose_ = declare_parameter<bool>("use_tf_pose", true);
    use_sport_mode_state_pose_ = declare_parameter<bool>("use_sport_mode_state_pose", true);
    prefer_sport_mode_state_pose_ =
      declare_parameter<bool>("prefer_sport_mode_state_pose", true);
    pose_timeout_sec_ = declare_parameter<double>("pose_timeout_sec", 0.5);
    goal_timeout_sec_ = declare_parameter<double>("goal_timeout_sec", 0.0);
    update_rate_ = declare_parameter<double>("update_rate", 50.0);

    velocity_control_stiffness_ =
      declare_parameter<double>("velocity_control_stiffness", 1.0);
    heading_control_stiffness_ =
      declare_parameter<double>("heading_control_stiffness", 1.5);
    only_positive_lin_vel_x_ =
      declare_parameter<bool>("only_positive_lin_vel_x", true);
    max_lin_vel_x_ = declare_parameter<double>("max_lin_vel_x", 0.8);
    max_lin_vel_y_ = declare_parameter<double>("max_lin_vel_y", 0.0);
    max_ang_vel_z_ = declare_parameter<double>("max_ang_vel_z", 1.0);
    target_dis_threshold_ = declare_parameter<double>("target_dis_threshold", 0.3);
    target_slowdown_distance_ = declare_parameter<double>("target_slowdown_distance", 0.6);
    enable_soft_target_slowdown_ =
      declare_parameter<bool>("enable_soft_target_slowdown", true);
    enable_heading_speed_gate_ =
      declare_parameter<bool>("enable_heading_speed_gate", true);
    heading_speed_gate_min_ =
      declare_parameter<double>("heading_speed_gate_min", 0.25);
    disallow_reverse_target_component_ =
      declare_parameter<bool>("disallow_reverse_target_component", true);
    lin_vel_threshold_ = declare_parameter<double>("lin_vel_threshold", 0.02);
    ang_vel_threshold_ = declare_parameter<double>("ang_vel_threshold", 0.02);
    max_linear_cmd_step_ = declare_parameter<double>("max_linear_cmd_step", 0.05);
    max_angular_cmd_step_ = declare_parameter<double>("max_angular_cmd_step", 0.08);
    command_smoothing_factor_ =
      declare_parameter<double>("command_smoothing_factor", 0.1);

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    local_target_pub_ =
      create_publisher<geometry_msgs::msg::PointStamped>("~/local_target_b", 10);

    goal_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_pose_topic_, 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        set_goal(msg->header.frame_id, msg->pose.position.x, msg->pose.position.y);
      });
    clicked_point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      clicked_point_topic_, 10,
      [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        set_goal(msg->header.frame_id, msg->point.x, msg->point.y);
      });

    if (use_sport_mode_state_pose_ && !sport_mode_state_topic_.empty()) {
      sport_mode_state_sub_ = create_subscription<unitree_go::msg::SportModeState>(
        sport_mode_state_topic_, rclcpp::SensorDataQoS(),
        [this](const unitree_go::msg::SportModeState::SharedPtr msg) {
          sport_pose_.x = static_cast<double>(msg->position[0]);
          sport_pose_.y = static_cast<double>(msg->position[1]);
          sport_pose_.yaw = yaw_from_wxyz(msg->imu_state.quaternion);
          last_sport_pose_time_ = now();
          has_sport_pose_ = true;
        });
    }

    const auto period =
      std::chrono::duration<double>(1.0 / std::max(update_rate_, 1.0));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&GoalToCmdVelNode::update, this));
    last_update_time_ = now();

    RCLCPP_INFO(
      get_logger(),
      "goal_to_cmd_vel ready: goal_pose=%s clicked_point=%s cmd_vel=%s world_frame=%s base_frame=%s",
      goal_pose_topic_.c_str(), clicked_point_topic_.c_str(), cmd_vel_topic_.c_str(),
      world_frame_.c_str(), base_frame_.c_str());
  }

private:
  bool recent(const rclcpp::Time & stamp, double timeout_sec) const
  {
    if (stamp.nanoseconds() == 0) {
      return false;
    }
    if (timeout_sec <= 0.0) {
      return true;
    }
    return (now() - stamp).seconds() <= timeout_sec;
  }

  bool get_base_pose(BasePose & pose)
  {
    if (prefer_sport_mode_state_pose_ && has_sport_pose_ &&
      recent(last_sport_pose_time_, pose_timeout_sec_))
    {
      if (finite_pose(sport_pose_)) {
        pose = sport_pose_;
        return true;
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring non-finite SportModeState pose.");
    }

    if (use_tf_pose_) {
      try {
        const auto tf = tf_buffer_.lookupTransform(
          world_frame_, base_frame_, tf2::TimePointZero);
        pose.x = tf.transform.translation.x;
        pose.y = tf.transform.translation.y;
        pose.yaw = yaw_from_quaternion(
          tf.transform.rotation.x,
          tf.transform.rotation.y,
          tf.transform.rotation.z,
          tf.transform.rotation.w);
        if (finite_pose(pose)) {
          return true;
        }
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Ignoring non-finite TF pose from '%s' to '%s'.",
          world_frame_.c_str(), base_frame_.c_str());
      } catch (const tf2::TransformException & ex) {
        RCLCPP_DEBUG(get_logger(), "TF pose lookup failed: %s", ex.what());
      }
    }

    if (!prefer_sport_mode_state_pose_ && has_sport_pose_ &&
      recent(last_sport_pose_time_, pose_timeout_sec_))
    {
      if (finite_pose(sport_pose_)) {
        pose = sport_pose_;
        return true;
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring non-finite SportModeState pose.");
    }

    return false;
  }

  bool transform_point_to_world(
    const std::string & frame_id,
    double x,
    double y,
    double & world_x,
    double & world_y)
  {
    if (frame_id.empty() || frame_id == world_frame_) {
      world_x = x;
      world_y = y;
      return true;
    }

    if (!use_tf_pose_) {
      return false;
    }

    try {
      const auto tf = tf_buffer_.lookupTransform(world_frame_, frame_id, tf2::TimePointZero);
      const double yaw = yaw_from_quaternion(
        tf.transform.rotation.x,
        tf.transform.rotation.y,
        tf.transform.rotation.z,
        tf.transform.rotation.w);
      const double c = std::cos(yaw);
      const double s = std::sin(yaw);
      world_x = tf.transform.translation.x + c * x - s * y;
      world_y = tf.transform.translation.y + s * x + c * y;
      return is_finite_value(world_x) && is_finite_value(world_y);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Cannot transform goal from '%s' to '%s': %s",
        frame_id.c_str(), world_frame_.c_str(), ex.what());
    }

    return false;
  }

  void set_goal(const std::string & frame_id_in, double x, double y)
  {
    if (!is_finite_value(x) || !is_finite_value(y)) {
      RCLCPP_WARN(get_logger(), "Rejected non-finite goal [%.3f %.3f].", x, y);
      return;
    }

    const std::string frame_id = frame_id_in.empty() ? world_frame_ : frame_id_in;

    if (frame_id == base_frame_) {
      local_goal_x_b_ = x;
      local_goal_y_b_ = y;
      goal_is_world_ = false;
      goal_active_ = true;
      RCLCPP_INFO(
        get_logger(),
        "Accepted base-frame local goal [%.3f %.3f].",
        x, y);
    } else {
      double world_x = 0.0;
      double world_y = 0.0;
      if (transform_point_to_world(frame_id, x, y, world_x, world_y)) {
        goal_x_w_ = world_x;
        goal_y_w_ = world_y;
        goal_is_world_ = true;
        goal_active_ = true;
        RCLCPP_INFO(
          get_logger(),
          "Accepted world goal [%.3f %.3f] in frame '%s'.",
          goal_x_w_, goal_y_w_, world_frame_.c_str());
      } else {
        RCLCPP_WARN(
          get_logger(),
          "Rejected goal in frame '%s' because it cannot be transformed to '%s'.",
          frame_id.c_str(), world_frame_.c_str());
        return;
      }
    }

    goal_stamp_ = now();
  }

  void update()
  {
    const auto current_time = now();
    double dt = (current_time - last_update_time_).seconds();
    if (dt <= 0.0 || !std::isfinite(dt)) {
      dt = 1.0 / std::max(update_rate_, 1.0);
    }
    last_update_time_ = current_time;

    if (!goal_active_ ||
      (goal_timeout_sec_ > 0.0 && !recent(goal_stamp_, goal_timeout_sec_)))
    {
      publish_zero();
      return;
    }

    double target_x_b = 0.0;
    double target_y_b = 0.0;

    if (goal_is_world_) {
      BasePose base_pose;
      if (!get_base_pose(base_pose)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "No valid base pose from SportModeState or TF; publishing zero cmd_vel.");
        publish_zero();
        return;
      }
      const double dx = goal_x_w_ - base_pose.x;
      const double dy = goal_y_w_ - base_pose.y;
      const double c = std::cos(base_pose.yaw);
      const double s = std::sin(base_pose.yaw);
      target_x_b = c * dx + s * dy;
      target_y_b = -s * dx + c * dy;
    } else {
      target_x_b = local_goal_x_b_;
      target_y_b = local_goal_y_b_;
    }

    publish_local_target(target_x_b, target_y_b);

    const double target_dist = std::hypot(target_x_b, target_y_b);
    if (!is_finite_value(target_x_b) || !is_finite_value(target_y_b) ||
      !is_finite_value(target_dist))
    {
      goal_active_ = false;
      filtered_vx_ = 0.0;
      filtered_vy_ = 0.0;
      filtered_wz_ = 0.0;
      publish_zero();
      RCLCPP_ERROR(get_logger(), "Rejected non-finite local target; publishing zero cmd_vel.");
      return;
    }

    if (target_dist <= target_dis_threshold_) {
      goal_active_ = false;
      filtered_vx_ = 0.0;
      filtered_vy_ = 0.0;
      filtered_wz_ = 0.0;
      publish_zero();
      RCLCPP_INFO(get_logger(), "Goal reached.");
      return;
    }

    auto cmd = compute_command(target_x_b, target_y_b, target_dist);
    apply_rate_limits_and_smoothing(cmd[0], cmd[1], cmd[2]);

    if (!is_finite_value(filtered_vx_) || !is_finite_value(filtered_vy_) ||
      !is_finite_value(filtered_wz_))
    {
      filtered_vx_ = 0.0;
      filtered_vy_ = 0.0;
      filtered_wz_ = 0.0;
      goal_active_ = false;
      RCLCPP_ERROR(get_logger(), "Computed non-finite cmd_vel; publishing zero cmd_vel.");
    }

    geometry_msgs::msg::Twist msg;
    msg.linear.x = filtered_vx_;
    msg.linear.y = filtered_vy_;
    msg.angular.z = filtered_wz_;
    cmd_vel_pub_->publish(msg);

    if (!goal_is_world_) {
      const double old_x_b = local_goal_x_b_;
      const double old_y_b = local_goal_y_b_;
      local_goal_x_b_ += (-filtered_vx_ + filtered_wz_ * old_y_b) * dt;
      local_goal_y_b_ += (-filtered_vy_ - filtered_wz_ * old_x_b) * dt;
    }
  }

  std::array<double, 3> compute_command(
    double target_x_b, double target_y_b, double target_dist) const
  {
    double vx = velocity_control_stiffness_ * target_x_b;
    double vy = velocity_control_stiffness_ * target_y_b;
    const double heading_error = wrap_to_pi(std::atan2(target_y_b, target_x_b));
    double wz = heading_control_stiffness_ * heading_error;
    wz *= clamp((target_dist - target_dis_threshold_) / target_dis_threshold_, 0.0, 1.0);

    if (only_positive_lin_vel_x_) {
      vx = clamp(vx, 0.0, max_lin_vel_x_);
      vy = clamp(vy, -max_lin_vel_y_, max_lin_vel_y_);
    } else {
      vx = clamp(vx, -max_lin_vel_x_, max_lin_vel_x_);
      vy = clamp(vy, -max_lin_vel_y_, max_lin_vel_y_);
    }
    wz = clamp(wz, -max_ang_vel_z_, max_ang_vel_z_);

    if (enable_soft_target_slowdown_) {
      const double slowdown_distance =
        std::max(target_slowdown_distance_, target_dis_threshold_ + 1.0e-3);
      const double stop_to_slow = slowdown_distance - target_dis_threshold_;
      const double slow_scale =
        clamp((target_dist - target_dis_threshold_) / (stop_to_slow + 1.0e-6), 0.0, 1.0);
      vx *= slow_scale;
      vy *= slow_scale;
      wz *= slow_scale;
    }

    if (enable_heading_speed_gate_) {
      const double heading_scale =
        clamp(std::cos(heading_error), heading_speed_gate_min_, 1.0);
      vx *= heading_scale;
      vy *= heading_scale;
    }

    const double lin_norm = std::hypot(vx, vy);
    if (lin_norm <= lin_vel_threshold_) {
      vx = 0.0;
      vy = 0.0;
    }
    if (std::abs(wz) <= ang_vel_threshold_) {
      wz = 0.0;
    }

    if (disallow_reverse_target_component_ && lin_norm > 1.0e-6 && target_dist > 1.0e-6) {
      const double unit_x = target_x_b / target_dist;
      const double unit_y = target_y_b / target_dist;
      const double forward_projection = vx * unit_x + vy * unit_y;
      if (forward_projection < 0.0) {
        vx -= forward_projection * unit_x;
        vy -= forward_projection * unit_y;
      }
    }

    return {vx, vy, wz};
  }

  void apply_rate_limits_and_smoothing(double vx, double vy, double wz)
  {
    if (!is_finite_value(vx) || !is_finite_value(vy) || !is_finite_value(wz)) {
      filtered_vx_ = 0.0;
      filtered_vy_ = 0.0;
      filtered_wz_ = 0.0;
      return;
    }

    if (max_linear_cmd_step_ > 0.0) {
      vx = filtered_vx_ + clamp(vx - filtered_vx_, -max_linear_cmd_step_, max_linear_cmd_step_);
      vy = filtered_vy_ + clamp(vy - filtered_vy_, -max_linear_cmd_step_, max_linear_cmd_step_);
    }
    if (max_angular_cmd_step_ > 0.0) {
      wz = filtered_wz_ + clamp(wz - filtered_wz_, -max_angular_cmd_step_, max_angular_cmd_step_);
    }

    if (command_smoothing_factor_ > 0.0) {
      const double alpha = clamp(command_smoothing_factor_, 0.0, 0.999);
      filtered_vx_ = alpha * filtered_vx_ + (1.0 - alpha) * vx;
      filtered_vy_ = alpha * filtered_vy_ + (1.0 - alpha) * vy;
      filtered_wz_ = alpha * filtered_wz_ + (1.0 - alpha) * wz;
    } else {
      filtered_vx_ = vx;
      filtered_vy_ = vy;
      filtered_wz_ = wz;
    }
  }

  void publish_zero()
  {
    geometry_msgs::msg::Twist msg;
    cmd_vel_pub_->publish(msg);
  }

  void publish_local_target(double x_b, double y_b)
  {
    geometry_msgs::msg::PointStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = base_frame_;
    msg.point.x = x_b;
    msg.point.y = y_b;
    msg.point.z = 0.0;
    local_target_pub_->publish(msg);
  }

  std::string world_frame_;
  std::string base_frame_;
  std::string goal_pose_topic_;
  std::string clicked_point_topic_;
  std::string cmd_vel_topic_;
  std::string sport_mode_state_topic_;

  bool use_tf_pose_ = true;
  bool use_sport_mode_state_pose_ = true;
  bool prefer_sport_mode_state_pose_ = true;
  double pose_timeout_sec_ = 0.5;
  double goal_timeout_sec_ = 0.0;
  double update_rate_ = 50.0;

  double velocity_control_stiffness_ = 1.0;
  double heading_control_stiffness_ = 1.5;
  bool only_positive_lin_vel_x_ = true;
  double max_lin_vel_x_ = 0.8;
  double max_lin_vel_y_ = 0.0;
  double max_ang_vel_z_ = 1.0;
  double target_dis_threshold_ = 0.3;
  double target_slowdown_distance_ = 0.6;
  bool enable_soft_target_slowdown_ = true;
  bool enable_heading_speed_gate_ = true;
  double heading_speed_gate_min_ = 0.25;
  bool disallow_reverse_target_component_ = true;
  double lin_vel_threshold_ = 0.02;
  double ang_vel_threshold_ = 0.02;
  double max_linear_cmd_step_ = 0.05;
  double max_angular_cmd_step_ = 0.08;
  double command_smoothing_factor_ = 0.1;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr local_target_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sport_mode_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  bool goal_active_ = false;
  bool goal_is_world_ = true;
  double goal_x_w_ = 0.0;
  double goal_y_w_ = 0.0;
  double local_goal_x_b_ = 0.0;
  double local_goal_y_b_ = 0.0;
  rclcpp::Time goal_stamp_{0, 0, RCL_ROS_TIME};

  bool has_sport_pose_ = false;
  BasePose sport_pose_;
  rclcpp::Time last_sport_pose_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};

  double filtered_vx_ = 0.0;
  double filtered_vy_ = 0.0;
  double filtered_wz_ = 0.0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalToCmdVelNode>());
  rclcpp::shutdown();
  return 0;
}
