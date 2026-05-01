#!/usr/bin/env python3
"""Publish a LowCmd holding a static arm/torso pose for N seconds.

Validates the OPPOSITE direction of the bridge — Linux ROS2 -> mac MuJoCo.
If this works you should see, in the macOS MuJoCo viewer, the G1's arms
slowly settle into the P0 keyframe pose (relaxed, slightly raised
shoulders) regardless of how the robot's base is oriented.

Caveats:
  - On the REAL robot, arm control runs on /arm_sdk; legs/torso are
    handled by the sport-mode base controller. unitree_mujoco has NO
    base controller; commanding only the 17 arm/torso joints means the
    legs receive no torque and the robot keeps falling. Since the robot
    already fell over (no controller running on either side), this is
    fine for the test — we only care about arm joint motion visible.
  - We must set CRC; unitree_sdk2py on the mac side checks it and drops
    malformed LowCmd silently. We use the same lowcmd_crc helper that
    arm_controller.py uses on the real robot.

Usage:
  source /opt/ros/jazzy/setup.bash
  source ~/g1_courier_ws/install/setup.bash
  python3 ~/g1_courier_ws/scripts/publish_lowcmd_pose.py [seconds] [pose_name]
    seconds   default 10
    pose_name P0|P1|P2|P3|P4|P5|P6 (default P0)
"""
from __future__ import annotations

import sys
import time

import rclpy
from rclpy.node import Node
from unitree_hg.msg import LowCmd

from g1_courier_arm_skills import keyframes
from g1_courier_arm_skills.keyframes import ARM_JOINTS, ARM_ENABLE_JOINT
from g1_courier_arm_skills.lowcmd_crc import LowCmdCrc


KP = 80.0
KD = 2.0
RATE_HZ = 100.0


def main() -> int:
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    pose_name = sys.argv[2] if len(sys.argv) > 2 else 'P0'
    pose = getattr(keyframes, pose_name)
    if len(pose) != len(ARM_JOINTS):
        print(f'pose {pose_name} length {len(pose)} != ARM_JOINTS {len(ARM_JOINTS)}')
        return 1

    rclpy.init()
    node = rclpy.create_node('publish_lowcmd_pose')
    pub = node.create_publisher(LowCmd, '/lowcmd', 10)
    crc = LowCmdCrc()

    print(f'publishing {pose_name} on /lowcmd @ {RATE_HZ:.0f} Hz for {duration:.1f}s')
    print(f'  arm_enable joint[{ARM_ENABLE_JOINT}].q = 1.0 (full takeover)')
    print(f'  kp={KP} kd={KD}')

    dt = 1.0 / RATE_HZ
    deadline = time.monotonic() + duration
    sent = 0
    try:
        while time.monotonic() < deadline:
            cmd = LowCmd()
            # Switch arm_sdk on; weight=1.0 means our pose fully replaces
            # whatever arm controller would otherwise be active.
            cmd.motor_cmd[ARM_ENABLE_JOINT].q = 1.0
            for idx, joint in enumerate(ARM_JOINTS):
                m = cmd.motor_cmd[joint]
                m.tau = 0.0
                m.q = float(pose[idx])
                m.dq = 0.0
                m.kp = KP
                m.kd = KD
            cmd.crc = crc.Crc(cmd)
            pub.publish(cmd)
            sent += 1
            time.sleep(dt)
    except KeyboardInterrupt:
        pass

    print(f'sent {sent} commands. Look in MuJoCo viewer on macOS — arms should '
          f'have moved toward {pose_name} pose.')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
