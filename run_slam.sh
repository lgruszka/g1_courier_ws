#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

source "$SCRIPT_DIR/install/setup.bash"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros_logs}"
mkdir -p "$ROS_LOG_DIR"
exec ros2 launch g1_courier_bringup mapping_real.launch.py
