#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

namespace
{

using RotationTuple = std::array<int, 4>;

constexpr std::array<int, 4> kNextAxis = {1, 2, 0, 1};

const std::unordered_map<std::string, RotationTuple> kAxesToTuple = {
  {"sxyz", {0, 0, 0, 0}},
  {"sxyx", {0, 0, 1, 0}},
  {"sxzy", {0, 1, 0, 0}},
  {"sxzx", {0, 1, 1, 0}},
  {"syzx", {1, 0, 0, 0}},
  {"syzy", {1, 0, 1, 0}},
  {"syxz", {1, 1, 0, 0}},
  {"syxy", {1, 1, 1, 0}},
  {"szxy", {2, 0, 0, 0}},
  {"szxz", {2, 0, 1, 0}},
  {"szyx", {2, 1, 0, 0}},
  {"szyz", {2, 1, 1, 0}},
  {"rzyx", {0, 0, 0, 1}},
  {"rxyx", {0, 0, 1, 1}},
  {"ryzx", {0, 1, 0, 1}},
  {"rxzx", {0, 1, 1, 1}},
  {"rxzy", {1, 0, 0, 1}},
  {"ryzy", {1, 0, 1, 1}},
  {"rzxy", {1, 1, 0, 1}},
  {"ryxy", {1, 1, 1, 1}},
  {"ryxz", {2, 0, 0, 1}},
  {"rzxz", {2, 0, 1, 1}},
  {"rxyz", {2, 1, 0, 1}},
  {"rzyz", {2, 1, 1, 1}},
};

const std::unordered_map<std::string, std::string> kOrderToAxes = {
  {"extrinsic_xyz", "sxyz"},
  {"extrinsic_xyx", "sxyx"},
  {"extrinsic_xzy", "sxzy"},
  {"extrinsic_xzx", "sxzx"},
  {"extrinsic_yzx", "syzx"},
  {"extrinsic_yzy", "syzy"},
  {"extrinsic_yxz", "syxz"},
  {"extrinsic_yxy", "syxy"},
  {"extrinsic_zxy", "szxy"},
  {"extrinsic_zxz", "szxz"},
  {"extrinsic_zyx", "szyx"},
  {"extrinsic_zyz", "szyz"},
  {"intrinsic_zyx", "rzyx"},
  {"intrinsic_xyx", "rxyx"},
  {"intrinsic_yzx", "ryzx"},
  {"intrinsic_xzx", "rxzx"},
  {"intrinsic_xzy", "rxzy"},
  {"intrinsic_yzy", "ryzy"},
  {"intrinsic_zxy", "rzxy"},
  {"intrinsic_yxy", "ryxy"},
  {"intrinsic_yxz", "ryxz"},
  {"intrinsic_zxz", "rzxz"},
  {"intrinsic_xyz", "rxyz"},
  {"intrinsic_zyz", "rzyz"},
};

struct TransformConfig
{
  std::string parent_frame;
  std::string child_frame;
  std::vector<double> translation_xyz;
  std::vector<double> rotation_angles;
};

struct RotationConfig
{
  std::string rotation_order;
  std::string angle_unit;
};

void validate_vector(
  const std::vector<double> & values,
  const std::string & parameter_name,
  const rclcpp::Logger & logger)
{
  if (values.size() != 3) {
    throw std::invalid_argument(
            parameter_name + " must contain exactly 3 values.");
  }

  for (double value : values) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument(
              parameter_name + " must contain only finite values.");
    }
  }

  RCLCPP_DEBUG(
    logger,
    "Validated parameter %s = [%.6f, %.6f, %.6f]",
    parameter_name.c_str(),
    values[0],
    values[1],
    values[2]);
}

std::vector<double> convert_angles_to_rad(
  const std::vector<double> & angles,
  const std::string & angle_unit)
{
  if (angle_unit == "rad") {
    return angles;
  }

  if (angle_unit != "deg") {
    throw std::invalid_argument(
            "Unsupported angle_unit '" + angle_unit + "'. Use 'rad' or 'deg'.");
  }

  constexpr double kDegToRad = M_PI / 180.0;
  std::vector<double> angles_rad;
  angles_rad.reserve(angles.size());
  for (double angle : angles) {
    angles_rad.push_back(angle * kDegToRad);
  }
  return angles_rad;
}

RotationTuple get_rotation_tuple(const std::string & rotation_order)
{
  const auto order_it = kOrderToAxes.find(rotation_order);
  if (order_it == kOrderToAxes.end()) {
    throw std::invalid_argument(
            "Unsupported rotation_order '" + rotation_order + "'.");
  }

  const auto axes_it = kAxesToTuple.find(order_it->second);
  if (axes_it == kAxesToTuple.end()) {
    throw std::invalid_argument(
            "Internal error: missing axes mapping for rotation_order '" +
            rotation_order + "'.");
  }

  return axes_it->second;
}

tf2::Matrix3x3 euler_to_matrix(
  const std::vector<double> & angles_rad,
  const std::string & rotation_order)
{
  const auto rotation_tuple = get_rotation_tuple(rotation_order);
  int i = rotation_tuple[0];
  int parity = rotation_tuple[1];
  int repetition = rotation_tuple[2];
  int frame = rotation_tuple[3];
  int j = kNextAxis[static_cast<std::size_t>(i + parity)];
  int k = kNextAxis[static_cast<std::size_t>(i - parity + 1)];

  double ai = angles_rad[0];
  double aj = angles_rad[1];
  double ak = angles_rad[2];

  if (frame != 0) {
    std::swap(ai, ak);
  }
  if (parity != 0) {
    ai = -ai;
    aj = -aj;
    ak = -ak;
  }

  const double si = std::sin(ai);
  const double sj = std::sin(aj);
  const double sk = std::sin(ak);
  const double ci = std::cos(ai);
  const double cj = std::cos(aj);
  const double ck = std::cos(ak);
  const double cc = ci * ck;
  const double cs = ci * sk;
  const double sc = si * ck;
  const double ss = si * sk;

  std::array<std::array<double, 3>, 3> matrix = {{
    {{1.0, 0.0, 0.0}},
    {{0.0, 1.0, 0.0}},
    {{0.0, 0.0, 1.0}},
  }};

  if (repetition != 0) {
    matrix[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)] = cj;
    matrix[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] = sj * si;
    matrix[static_cast<std::size_t>(i)][static_cast<std::size_t>(k)] = sj * ci;
    matrix[static_cast<std::size_t>(j)][static_cast<std::size_t>(i)] = sj * sk;
    matrix[static_cast<std::size_t>(j)][static_cast<std::size_t>(j)] = -cj * ss + cc;
    matrix[static_cast<std::size_t>(j)][static_cast<std::size_t>(k)] = -cj * cs - sc;
    matrix[static_cast<std::size_t>(k)][static_cast<std::size_t>(i)] = -sj * ck;
    matrix[static_cast<std::size_t>(k)][static_cast<std::size_t>(j)] = cj * sc + cs;
    matrix[static_cast<std::size_t>(k)][static_cast<std::size_t>(k)] = cj * cc - ss;
  } else {
    matrix[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)] = cj * ck;
    matrix[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] = sj * sc - cs;
    matrix[static_cast<std::size_t>(i)][static_cast<std::size_t>(k)] = sj * cc + ss;
    matrix[static_cast<std::size_t>(j)][static_cast<std::size_t>(i)] = cj * sk;
    matrix[static_cast<std::size_t>(j)][static_cast<std::size_t>(j)] = sj * ss + cc;
    matrix[static_cast<std::size_t>(j)][static_cast<std::size_t>(k)] = sj * cs - sc;
    matrix[static_cast<std::size_t>(k)][static_cast<std::size_t>(i)] = -sj;
    matrix[static_cast<std::size_t>(k)][static_cast<std::size_t>(j)] = cj * si;
    matrix[static_cast<std::size_t>(k)][static_cast<std::size_t>(k)] = cj * ci;
  }

  return tf2::Matrix3x3(
    matrix[0][0], matrix[0][1], matrix[0][2],
    matrix[1][0], matrix[1][1], matrix[1][2],
    matrix[2][0], matrix[2][1], matrix[2][2]);
}

RotationConfig load_rotation_config(rclcpp::Node & node)
{
  RotationConfig config;
  config.rotation_order =
    node.declare_parameter<std::string>("rotation_order", "");
  config.angle_unit =
    node.declare_parameter<std::string>("angle_unit", "");

  if (config.rotation_order.empty()) {
    throw std::invalid_argument("rotation_order must not be empty.");
  }
  if (config.angle_unit.empty()) {
    throw std::invalid_argument("angle_unit must not be empty.");
  }
  if (config.angle_unit != "rad" && config.angle_unit != "deg") {
    throw std::invalid_argument(
            "Unsupported angle_unit '" + config.angle_unit + "'. Use 'rad' or 'deg'.");
  }

  static_cast<void>(get_rotation_tuple(config.rotation_order));
  return config;
}

TransformConfig load_transform_config(
  rclcpp::Node & node,
  const std::string & namespace_prefix)
{
  const auto parent_param = namespace_prefix + ".parent_frame";
  const auto child_param = namespace_prefix + ".child_frame";
  const auto translation_param = namespace_prefix + ".translation_xyz";
  const auto rotation_param = namespace_prefix + ".rotation_angles";

  TransformConfig config;
  config.parent_frame = node.declare_parameter<std::string>(parent_param, "");
  config.child_frame = node.declare_parameter<std::string>(child_param, "");
  config.translation_xyz =
    node.declare_parameter<std::vector<double>>(translation_param, {0.0, 0.0, 0.0});
  config.rotation_angles =
    node.declare_parameter<std::vector<double>>(rotation_param, {0.0, 0.0, 0.0});

  if (config.parent_frame.empty()) {
    throw std::invalid_argument(parent_param + " must not be empty.");
  }
  if (config.child_frame.empty()) {
    throw std::invalid_argument(child_param + " must not be empty.");
  }
  if (config.parent_frame == config.child_frame) {
    throw std::invalid_argument(
            namespace_prefix + " parent and child frames must be different.");
  }

  validate_vector(config.translation_xyz, translation_param, node.get_logger());
  validate_vector(config.rotation_angles, rotation_param, node.get_logger());

  return config;
}

geometry_msgs::msg::TransformStamped to_transform_stamped(
  const rclcpp::Time & stamp,
  const TransformConfig & config,
  const RotationConfig & rotation_config)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = stamp;
  transform.header.frame_id = config.parent_frame;
  transform.child_frame_id = config.child_frame;
  transform.transform.translation.x = config.translation_xyz[0];
  transform.transform.translation.y = config.translation_xyz[1];
  transform.transform.translation.z = config.translation_xyz[2];

  const auto angles_rad =
    convert_angles_to_rad(config.rotation_angles, rotation_config.angle_unit);
  const auto rotation_matrix =
    euler_to_matrix(angles_rad, rotation_config.rotation_order);

  tf2::Quaternion quaternion;
  rotation_matrix.getRotation(quaternion);
  quaternion.normalize();

  if (!std::isfinite(quaternion.x()) ||
    !std::isfinite(quaternion.y()) ||
    !std::isfinite(quaternion.z()) ||
    !std::isfinite(quaternion.w()))
  {
    throw std::invalid_argument(
            "Rotation for transform " + config.parent_frame + " -> " +
            config.child_frame + " produced a non-finite quaternion.");
  }

  transform.transform.rotation.x = quaternion.x();
  transform.transform.rotation.y = quaternion.y();
  transform.transform.rotation.z = quaternion.z();
  transform.transform.rotation.w = quaternion.w();
  return transform;
}

}  // namespace

class LidarStaticTfNode : public rclcpp::Node
{
public:
  LidarStaticTfNode()
  : Node("lidar_static_tf_node"),
    broadcaster_(std::make_unique<tf2_ros::StaticTransformBroadcaster>(this))
  {
    const auto rotation_config = load_rotation_config(*this);
    const auto odom_to_tracking_origin =
      load_transform_config(*this, "odom_to_tracking_origin");
    const auto tracking_body_to_base =
      load_transform_config(*this, "tracking_body_to_base");

    const auto stamp = this->get_clock()->now();
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    transforms.reserve(2);
    transforms.push_back(
      to_transform_stamped(stamp, odom_to_tracking_origin, rotation_config));
    transforms.push_back(
      to_transform_stamped(stamp, tracking_body_to_base, rotation_config));

    broadcaster_->sendTransform(transforms);

    RCLCPP_INFO(
      get_logger(),
      "Published static TF %s -> %s and %s -> %s using rotation_order=%s angle_unit=%s",
      odom_to_tracking_origin.parent_frame.c_str(),
      odom_to_tracking_origin.child_frame.c_str(),
      tracking_body_to_base.parent_frame.c_str(),
      tracking_body_to_base.child_frame.c_str(),
      rotation_config.rotation_order.c_str(),
      rotation_config.angle_unit.c_str());
  }

private:
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<LidarStaticTfNode>();
    rclcpp::spin(node);
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("lidar_static_tf_node"),
      "Failed to start lidar_static_tf_node: %s",
      exception.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
