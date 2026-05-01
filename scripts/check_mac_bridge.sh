#!/usr/bin/env bash
# Diagnostic: verify the DDS bridge from macOS unitree_mujoco -> ROS2 here.
#
# Run AFTER `unitree_mujoco.py` is started on the macOS side.
# Prints what we see: RMW in use, visible topics, sample LowState payload
# if any arrives within `WAIT_S` seconds.
set -u

WAIT_S="${WAIT_S:-5}"

# Source ROS + workspace if not already current.
if [ -z "${ROS_DISTRO:-}" ]; then
    set +u
    source /opt/ros/jazzy/setup.bash
    set -u
fi
if [ -f /home/parallels/g1_courier_ws/install/setup.bash ]; then
    set +u
    source /home/parallels/g1_courier_ws/install/setup.bash
    set -u
fi

echo "=== env ==="
echo "ROS_DISTRO          = ${ROS_DISTRO:-?}"
echo "ROS_DOMAIN_ID       = ${ROS_DOMAIN_ID:-0 (default)}"
echo "RMW_IMPLEMENTATION  = ${RMW_IMPLEMENTATION:-?}"
echo "CYCLONEDDS_URI      = ${CYCLONEDDS_URI:-(unset)}"

echo
echo "=== visible topics related to lowstate / lowcmd / unitree ==="
ros2 topic list 2>&1 | grep -E "lowstate|lowcmd|rt/|unitree" || echo "(none yet — mac side may not be publishing)"

echo
echo "=== detailed info on /lowstate (or rt/lowstate) if present ==="
for t in /lowstate rt/lowstate /rt/lowstate; do
    if ros2 topic list 2>&1 | grep -qx "$t"; then
        ros2 topic info -v "$t" 2>&1 | head -25
        TOPIC="$t"
        break
    fi
done

if [ -z "${TOPIC:-}" ]; then
    echo
    echo "No lowstate topic visible. If macOS side IS publishing:"
    echo "  1. Confirm same ROS_DOMAIN_ID on both sides."
    echo "  2. Confirm RMW_IMPLEMENTATION=rmw_cyclonedds_cpp on both sides."
    echo "  3. Multicast may be filtered by Parallels Shared Network — the"
    echo "     CYCLONEDDS_URI here already declares 10.211.55.1 (mac host)"
    echo "     as a unicast peer. Mac side should declare 10.211.55.11 (us)."
    echo "  4. Topic name from unitree_mujoco is 'rt/lowstate' (DDS native);"
    echo "     ROS2 sees it as 'rt/lowstate' or 'lowstate' depending on remap."
    exit 1
fi

echo
echo "=== try receiving 1 message from $TOPIC (timeout ${WAIT_S}s) ==="
timeout "$WAIT_S" ros2 topic echo --once "$TOPIC" 2>&1 | head -40 || {
    echo
    echo "Timed out waiting for a message on $TOPIC."
    echo "Topic discovered but no data flowing. Check IDL match between"
    echo "macOS unitree_sdk2py and our unitree_hg/LowState (type hash should match)."
    exit 2
}

echo
echo "=== bridge OK ==="
