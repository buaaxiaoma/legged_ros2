/**
 * @file legged_rl_controller.cpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief
 * @version 0.1
 * @date 2026-01-17
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "legged_rl_controller/legged_rl_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

#include "legged_rl_controller/isaaclab/algorithms/algorithms.h"
#include "legged_rl_controller/isaaclab/envs/mdp/actions/joint_actions.h"
#include "legged_rl_controller/isaaclab/envs/mdp/observations/observations.h"

namespace legged
{

controller_interface::CallbackReturn LeggedRLController::on_init()
{
  if (LeggedController::on_init() != controller_interface::CallbackReturn::SUCCESS) {
    return controller_interface::CallbackReturn::ERROR;
  }

  joint_names_ = auto_declare<std::vector<std::string>>(
    "joint_names", std::vector<std::string>());
  imu_names_ = auto_declare<std::vector<std::string>>(
    "imu_names", std::vector<std::string>());

  onnx_model_path_ = auto_declare<std::string>("onnx_model_path", "");
  io_descriptors_path_ = auto_declare<std::string>("io_descriptors_path", "");
  auto_declare<std::string>("cmd_vel_topic", "cmd_vel");
  auto_declare<std::vector<double>>("cmd_vel_range_lin_vel_x", std::vector<double>());
  auto_declare<std::vector<double>>("cmd_vel_range_lin_vel_y", std::vector<double>());
  auto_declare<std::vector<double>>("cmd_vel_range_ang_vel_z", std::vector<double>());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LeggedRLController::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  onnx_model_path_ = get_node()->get_parameter("onnx_model_path").as_string();
  io_descriptors_path_ = get_node()->get_parameter("io_descriptors_path").as_string();
  auto cmd_vel_topic = get_node()->get_parameter("cmd_vel_topic").as_string();

  if (onnx_model_path_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'onnx_model_path' is empty.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (io_descriptors_path_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'io_descriptors_path' is empty.");
    return controller_interface::CallbackReturn::ERROR;
  }

  YAML::Node env_cfg;
  try {
    env_cfg = YAML::LoadFile(io_descriptors_path_);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to load IO descriptors: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  const auto robot_node = env_cfg["articulations"]["robot"];
  if (!robot_node.IsDefined()) {
    RCLCPP_ERROR(get_node()->get_logger(), "IO descriptors missing 'articulations.robot'.");
    return controller_interface::CallbackReturn::ERROR;
  }

  const auto yaml_joint_names = robot_node["joint_names"].as<std::vector<std::string>>();
  if (joint_names_.empty()) {
    joint_names_ = yaml_joint_names;
  } else if (joint_names_.size() != yaml_joint_names.size()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Joint names size mismatch: param=%zu, yaml=%zu.",
      joint_names_.size(), yaml_joint_names.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (LeggedController::on_configure(previous_state) !=
    controller_interface::CallbackReturn::SUCCESS)
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  if (imu_interfaces_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "No IMU interfaces available for RL controller.");
    return controller_interface::CallbackReturn::ERROR;
  }

  cmd_vel_buffer_ = std::make_shared<CmdBuffer>();
  cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic, rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
      cmd_vel_buffer_->writeFromNonRT(msg);
    });

  heightmap_buffer_ = std::make_shared<HeightMapBuffer>();
  heightmap_sub_ = get_node()->create_subscription<unitree_go::msg::HeightMap>(
    "/heightmap", rclcpp::SystemDefaultsQoS(),
    [this](const unitree_go::msg::HeightMap::SharedPtr msg) {
      heightmap_buffer_->writeFromNonRT(msg);
    });

  robot_ = std::make_shared<LeggedArticulation>(
    imu_interfaces_[0], joint_interface_, cmd_vel_buffer_); // Use the first IMU interface

  auto range_lin_vel_x = get_node()->get_parameter("cmd_vel_range_lin_vel_x").as_double_array();
  auto range_lin_vel_y = get_node()->get_parameter("cmd_vel_range_lin_vel_y").as_double_array();
  auto range_ang_vel_z = get_node()->get_parameter("cmd_vel_range_ang_vel_z").as_double_array();

  auto set_range = [](const std::vector<double> &range, std::array<float, 2U> &target) {
    if (range.size() == 2U) {
      target = {static_cast<float>(range[0]), static_cast<float>(range[1])};
    } else {
      target = {
        -std::numeric_limits<float>::infinity(),
        std::numeric_limits<float>::infinity()
      };
    }
  };
  set_range(range_lin_vel_x, robot_->data.velocity_command.range.lin_vel_x);
  set_range(range_lin_vel_y, robot_->data.velocity_command.range.lin_vel_y);
  set_range(range_ang_vel_z, robot_->data.velocity_command.range.ang_vel_z);

  try {
    env_ = std::make_unique<isaaclab::ManagerBasedRLEnv>(env_cfg, robot_);
    env_->alg = std::make_unique<isaaclab::OrtRunner>(onnx_model_path_);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize RL environment: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), "Legged RL Controller configured successfully.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LeggedRLController::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  if (LeggedController::on_activate(previous_state) !=
    controller_interface::CallbackReturn::SUCCESS)
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  if (cmd_vel_buffer_) {
    cmd_vel_buffer_->reset();
  }
  if (heightmap_buffer_) {
    heightmap_buffer_->reset();
  }
  if (env_) {
    env_->reset();
  }

  RCLCPP_INFO(get_node()->get_logger(), "Legged RL Controller activated successfully.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LeggedRLController::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  if (LeggedController::on_deactivate(previous_state) !=
    controller_interface::CallbackReturn::SUCCESS)
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  joint_interface_->set_joint_command(
    std::vector<double>(joint_names_.size(), 0.0),
    std::vector<double>(joint_names_.size(), 0.0),
    std::vector<double>(joint_names_.size(), 0.0),
    std::vector<double>(joint_names_.size(), 0.0),
    std::vector<double>(joint_names_.size(), 0.0));

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type LeggedRLController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!env_ || !env_->alg) {
    RCLCPP_ERROR(get_node()->get_logger(), "RL environment is not initialized.");
    return controller_interface::return_type::ERROR;
  }

  try {
    if (heightmap_buffer_) {
      auto heightmap_msg = *heightmap_buffer_->readFromRT();
      if (heightmap_msg) {
        robot_->data.height_scan.assign(
          heightmap_msg->data.begin(), heightmap_msg->data.end());
      } else {
        robot_->data.height_scan.clear();
      }
    } else {
      robot_->data.height_scan.clear();
    }

    env_->step();
    auto action = env_->action_manager->processed_actions();
    if (action.size() != joint_names_.size()) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Action size mismatch: action=%zu, joints=%zu.",
        action.size(), joint_names_.size());
      return controller_interface::return_type::ERROR;
    }
    for (size_t i = 0; i < action.size(); ++i) {
      if (!std::isfinite(action[i])) {
        RCLCPP_ERROR(
          get_node()->get_logger(),
          "Action contains non-finite value at index %zu.", i);
        return controller_interface::return_type::ERROR;
      }
    }

    joint_interface_->set_joint_command(
      action,
      std::vector<float>(joint_names_.size(), 0.0f),
      std::vector<float>(joint_names_.size(), 0.0f),
      env_->robot->data.joint_stiffness,
      env_->robot->data.joint_damping);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(), "RL update failed: %s", e.what());
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

}  // namespace legged

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  legged::LeggedRLController, controller_interface::ControllerInterface)
