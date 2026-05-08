/**
 * @file go2_lowlevel_node.cpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2025-12-11
 * 
 * @copyright Copyright (c) 2025
 * 
 */


#include "legged_ros2_control/robots/unitree_go2/go2_lowlevel_node.hpp"

namespace legged
{

Go2LowLevelNode::Go2LowLevelNode(
    const std::string & node_name,
    const std::string & lowstate_topic,
    const std::string & lowcmd_topic,
    const rclcpp::NodeOptions & options) : Node(node_name, options)
{
    init_lowcmd_();

    lowstate_subscriber_ = this->create_subscription<unitree_go::msg::LowState>(
        lowstate_topic, rclcpp::SensorDataQoS(),
        std::bind(&Go2LowLevelNode::lowstate_callback, this, std::placeholders::_1));

    lowcmd_publisher_ = this->create_publisher<unitree_go::msg::LowCmd>(
        lowcmd_topic, 10);

    RCLCPP_INFO(
        this->get_logger(), "Go2 low-level topics: lowstate=%s lowcmd=%s",
        lowstate_topic.c_str(), lowcmd_topic.c_str());
}


void Go2LowLevelNode::lowstate_callback(const unitree_go::msg::LowState::SharedPtr msg){
    lowstate_ = *msg;
}


void Go2LowLevelNode::publish_lowcmd(){
    get_crc(lowcmd_);
    lowcmd_publisher_->publish(lowcmd_);
}

void Go2LowLevelNode::init_lowcmd_(){
    lowcmd_.head[0] = 0xFE;
    lowcmd_.head[1] = 0xEF;
    lowcmd_.level_flag = 0xFF;
    lowcmd_.gpio = 0;

    for (int i = 0; i < 20; i++) {
        lowcmd_.motor_cmd[i].mode = (0x01);  // motor switch to servo (PMSM) mode
        lowcmd_.motor_cmd[i].q = (PosStopF);
        lowcmd_.motor_cmd[i].kp = (0);
        lowcmd_.motor_cmd[i].dq = (VelStopF);
        lowcmd_.motor_cmd[i].kd = (0);
        lowcmd_.motor_cmd[i].tau = (0);
    }
}





}
