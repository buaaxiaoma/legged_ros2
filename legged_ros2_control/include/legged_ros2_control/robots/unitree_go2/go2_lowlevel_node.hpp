/**
 * @file go2_lowlevel_node.hpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief This class is used for low-level communication with Unitree Go2 robot
 *  and it would be integrated into go2_system_interface
 * @version 0.1
 * @date 2025-12-11
 * 
 * @copyright Copyright (c) 2025
 * 
 */


#include "rclcpp/rclcpp.hpp"
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "legged_ros2_control/robots/unitree_go2/motor_crc.h"

namespace legged
{

/// Class for Unitree Go2 low-level communication node
class Go2LowLevelNode : public rclcpp::Node
{
public:
    RCLCPP_SMART_PTR_DEFINITIONS(Go2LowLevelNode)

    explicit Go2LowLevelNode(
        const std::string & node_name = "go2_lowlevel_node",
        const std::string & lowstate_topic = "/lowstate",
        const std::string & lowcmd_topic = "/lowcmd",
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions()
    );

    const unitree_go::msg::LowState& lowstate() const { return lowstate_; }
    unitree_go::msg::LowCmd& lowcmd() { return lowcmd_; }

    void publish_lowcmd();

private:
    void lowstate_callback(const unitree_go::msg::LowState::SharedPtr msg);

    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr lowstate_subscriber_;
    rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr lowcmd_publisher_;

    unitree_go::msg::LowState lowstate_;
    unitree_go::msg::LowCmd lowcmd_;

    void init_lowcmd_();

};

}
