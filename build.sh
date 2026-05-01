#!/usr/bin/env bash
set -euo pipefail

# Build the workspace incrementally. Run from anywhere; this script cds first.
HERE="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
cd "$HERE"

# Source ROS2 if not already.
if [ -z "${ROS_DISTRO:-}" ]; then
  for d in humble jazzy iron rolling; do
    if [ -f "/opt/ros/$d/setup.bash" ]; then
      # shellcheck disable=SC1090
      source "/opt/ros/$d/setup.bash"
      break
    fi
  done
fi
echo "ROS_DISTRO=${ROS_DISTRO:-unknown}"

# Skip the upstream unitree_ros2/example demo package — we only want the IDL
# packages (unitree_hg, unitree_api, unitree_go) from that submodule.
colcon build --symlink-install --packages-ignore unitree_ros2_example "$@"
