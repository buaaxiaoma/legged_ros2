#!/usr/bin/env bash
source /root/unitree_ros2/setup_local.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/install/setup.bash"

export ROS_DOMAIN_ID=1
echo "Set ROS_DOMAIN_ID to 1"
