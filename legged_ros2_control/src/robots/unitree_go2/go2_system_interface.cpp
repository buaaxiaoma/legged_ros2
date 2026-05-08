/**
 * @file go2_system_interface.cpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief System interface for Unitree Go2 robot
 * @ref https://github.com/PickNikRobotics/topic_based_ros2_control
 * @version 0.1
 * @date 2025-12-11
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "legged_ros2_control/robots/unitree_go2/go2_system_interface.hpp"
#include <algorithm>
#include <cctype>

namespace legged
{

CallbackReturn Go2SystemInterface::on_init(const hardware_interface::HardwareInfo & info){
  if (LeggedSystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  if(imu_data_.size() != 1){
    RCLCPP_ERROR(*logger_, "Go2SystemInterface only supports one IMU sensor");
    return CallbackReturn::ERROR;
  }

  auto get_topic_parameter = [&info](const std::string & name, const std::string & fallback) {
    auto it = info.hardware_parameters.find(name);
    if (it == info.hardware_parameters.end() || it->second.empty()) {
      return fallback;
    }
    return it->second;
  };

  const auto lowstate_topic = get_topic_parameter("lowstate_topic", "/lowstate");
  const auto lowcmd_topic = get_topic_parameter("lowcmd_topic", "/lowcmd");

  // create lowlevel node for unitree ros2 communication
  unitree_ros2_node_ = std::make_shared<Go2LowLevelNode>(
      "go2_lowlevel_node", lowstate_topic, lowcmd_topic);

  logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("Go2SystemInterface"));


  auto it = info.hardware_parameters.find("enable_lowlevel_write");
  if (it != info.hardware_parameters.end()) {
    std::string value = it->second;
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    enable_lowlevel_write_ = (value == "true");
  } else {
    RCLCPP_WARN(*logger_, "Parameter 'enable_lowlevel_write' not found, default to false");
    enable_lowlevel_write_ = false;
  }

  RCLCPP_INFO(*logger_, "Enable_lowlevel_write_: %s", enable_lowlevel_write_ ? "true" : "false");
  RCLCPP_INFO(*logger_, "Low-level topics: lowstate=%s lowcmd=%s", lowstate_topic.c_str(), lowcmd_topic.c_str());
  RCLCPP_INFO(*logger_, "Go2SystemInterface initialized successfully");


  return CallbackReturn::SUCCESS;
}


bool Go2SystemInterface::build_joint_data_(){
  for(size_t i=0; i<joint_data_.size(); i++){
    const auto & jnt_name = joint_data_[i].name;
    auto it = go2_joint_index_map.find(jnt_name);
    if(it == go2_joint_index_map.end()){
      RCLCPP_ERROR(*logger_, "Joint name %s not found in Go2 joint map", jnt_name.c_str());
      return false;
    }
    joint_data_[i].adr = static_cast<int>(it->second);
  }
  return true;
}


return_type Go2SystemInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){
  
  if(rclcpp::ok()){
    rclcpp::spin_some(unitree_ros2_node_);
  }

  // Read joint data
  for (size_t i=0; i < joint_data_.size(); ++i) {
    joint_data_[i].pos_ = unitree_ros2_node_->lowstate().motor_state[joint_data_[i].adr].q;
    joint_data_[i].vel_ = unitree_ros2_node_->lowstate().motor_state[joint_data_[i].adr].dq;
    joint_data_[i].tau_ = unitree_ros2_node_->lowstate().motor_state[joint_data_[i].adr].tau_est;
  }

  // Read IMU data
  // Only use one IMU here
  // Unitree SDK2 quaternion: w x y z
  // ImuData quaternion: x y z w
  imu_data_[0].quat_[0] = unitree_ros2_node_->lowstate().imu_state.quaternion[1]; // x
  imu_data_[0].quat_[1] = unitree_ros2_node_->lowstate().imu_state.quaternion[2]; // y
  imu_data_[0].quat_[2] = unitree_ros2_node_->lowstate().imu_state.quaternion[3]; // z
  imu_data_[0].quat_[3] = unitree_ros2_node_->lowstate().imu_state.quaternion[0]; // w
  imu_data_[0].ang_vel_[0] = unitree_ros2_node_->lowstate().imu_state.gyroscope[0]; // angular velocity x
  imu_data_[0].ang_vel_[1] = unitree_ros2_node_->lowstate().imu_state.gyroscope[1]; // angular velocity y
  imu_data_[0].ang_vel_[2] = unitree_ros2_node_->lowstate().imu_state.gyroscope[2]; // angular velocity z
  imu_data_[0].lin_acc_[0] = unitree_ros2_node_->lowstate().imu_state.accelerometer[0]; // linear acceleration x
  imu_data_[0].lin_acc_[1] = unitree_ros2_node_->lowstate().imu_state.accelerometer[1]; // linear acceleration y
  imu_data_[0].lin_acc_[2] = unitree_ros2_node_->lowstate().imu_state.accelerometer[2]; // linear acceleration z

  return return_type::OK;
}


return_type Go2SystemInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){

  if(!enable_lowlevel_write_){
    return return_type::OK;
  }

  for(size_t i=0; i < joint_data_.size(); ++i) {
    unitree_ros2_node_->lowcmd().motor_cmd[joint_data_[i].adr].q = joint_data_[i].pos_cmd_;
    unitree_ros2_node_->lowcmd().motor_cmd[joint_data_[i].adr].dq = joint_data_[i].vel_cmd_;
    unitree_ros2_node_->lowcmd().motor_cmd[joint_data_[i].adr].tau = joint_data_[i].ff_cmd_;
    unitree_ros2_node_->lowcmd().motor_cmd[joint_data_[i].adr].kp = joint_data_[i].kp_;
    unitree_ros2_node_->lowcmd().motor_cmd[joint_data_[i].adr].kd = joint_data_[i].kd_;

    // record the desired position and velocity for reading
    joint_data_[i].pos_des_ = joint_data_[i].pos_cmd_;
    joint_data_[i].vel_des_ = joint_data_[i].vel_cmd_;
  }

  unitree_ros2_node_->publish_lowcmd();

  return return_type::OK;
}

    
} // namespace legged

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  legged::Go2SystemInterface, legged::LeggedSystemInterface)
