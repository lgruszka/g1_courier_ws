#!/usr/bin/env bash
set -euo pipefail

STACK_TIMEOUT_S="${STACK_TIMEOUT_S:-45}"
MAP_PATH="/home/neo/j2s/maps/floor_only.yaml"

# Post-start quick check (manual):
# 1) ros2 topic hz /livox/lidar
# 2) ros2 topic hz /scan
# 3) ros2 run tf2_ros tf2_echo map odom
# 4) observe logs ~2 min for recurring transformPoseInTargetFrame errors

wait_for_topic() {
  local topic="$1"
  local timeout_s="$2"
  local start_ts
  start_ts=$(date +%s)

  while true; do
    if timeout 4 ros2 topic echo --once "$topic" >/dev/null 2>&1; then
      echo "[preflight] topic ready: $topic"
      return 0
    fi

    if (( $(date +%s) - start_ts >= timeout_s )); then
      echo "[preflight] TIMEOUT waiting for topic: $topic" >&2
      return 1
    fi
    sleep 0.5
  done
}

wait_for_tf() {
  local target="$1"
  local source="$2"
  local timeout_s="$3"
  local start_ts
  start_ts=$(date +%s)

  while true; do
    if timeout 2 ros2 run tf2_ros tf2_echo "$target" "$source" >/dev/null 2>&1; then
      echo "[preflight] tf ready: $target -> $source"
      return 0
    fi

    if (( $(date +%s) - start_ts >= timeout_s )); then
      echo "[preflight] TIMEOUT waiting for tf: $target -> $source" >&2
      return 1
    fi
    sleep 0.5
  done
}

echo "[run_nav2] building packages..."
colcon build --packages-select \
  g1_courier_bringup \
  g1_courier_msgs \
  g1_courier_docking \
  g1_courier_arm_skills \
  g1_courier_mission \
  g1_courier_safety

set +u
source install/setup.bash
set -u

echo "[run_nav2] preflight start (timeout=${STACK_TIMEOUT_S}s)..."
wait_for_topic "/livox/lidar" "$STACK_TIMEOUT_S"
wait_for_topic "/dog_odom" "$STACK_TIMEOUT_S"

echo "[run_nav2] launching nav stack..."
exec ros2 launch g1_courier_bringup real.launch.py \
  map:="$MAP_PATH" \
  enable_mission:=false
