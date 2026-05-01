#!/usr/bin/env bash
set -eo pipefail

# Build the workspace incrementally. Run from anywhere; this script cds first.
HERE="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
cd "$HERE"

# Source ROS2 if not already. Disable -u temporarily because /opt/ros/*/setup.bash
# touches AMENT_TRACE_SETUP_FILES and other unset-by-default vars.
if [ -z "${ROS_DISTRO:-}" ]; then
  set +u
  for d in humble jazzy iron rolling; do
    if [ -f "/opt/ros/$d/setup.bash" ]; then
      # shellcheck disable=SC1090
      source "/opt/ros/$d/setup.bash"
      break
    fi
  done
  set -u
fi
echo "ROS_DISTRO=${ROS_DISTRO:-unknown}"

# Skip the upstream unitree_ros2/example demo package — we only want the IDL
# packages (unitree_hg, unitree_api, unitree_go) from that submodule.
colcon build --symlink-install --packages-ignore unitree_ros2_example "$@"
