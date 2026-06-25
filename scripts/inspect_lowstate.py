#!/usr/bin/env python3
"""Inspect LowState messages flowing in from macOS unitree_mujoco.

Validates two things:
  1. Our workspace `unitree_hg/msg/LowState` IDL deserializes the data
     published from the mac (cross-OS, cross-implementation roundtrip).
  2. The data is plausibly live (tick rising, gravity in IMU, joint angles
     not all zero — would indicate a stuck publisher or fixture rather
     than the real MuJoCo physics).

Usage:
  ros2 run ... no — just:
    source /opt/ros/jazzy/setup.bash
    source ~/g1_courier_ws/install/setup.bash
    python3 ~/g1_courier_ws/scripts/inspect_lowstate.py [seconds]

Default observation window: 5 seconds.
"""
from __future__ import annotations

import math
import sys
import time

import rclpy
from rclpy.node import Node
from unitree_hg.msg import LowState

# Joint index map from g1_courier_arm_skills/keyframes.py:
#   left arm  joints 15..21
#   right arm joints 22..28
#   torso     joints 12..14
ARM_JOINTS = (15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 12, 13, 14)


class LowStateInspector(Node):
    def __init__(self) -> None:
        super().__init__('lowstate_inspector')
        self._first_tick = None
        self._last_tick = None
        self._count = 0
        self._first_msg_time = None
        self._latest: LowState | None = None
        self.create_subscription(LowState, '/lf/lowstate', self._on, 10)

    def _on(self, msg: LowState) -> None:
        self._count += 1
        if self._first_tick is None:
            self._first_tick = msg.tick
            self._first_msg_time = time.monotonic()
        self._last_tick = msg.tick
        self._latest = msg

    def report(self, elapsed_s: float) -> None:
        if self._latest is None:
            print('NO MESSAGES received in observation window.')
            return
        m = self._latest
        rate = self._count / max(0.001, elapsed_s)
        print()
        print('=== bridge data flow ===')
        print(f'messages received: {self._count} in {elapsed_s:.1f}s -> {rate:.1f} Hz')
        if self._first_tick is not None and self._last_tick is not None:
            tick_delta = (self._last_tick - self._first_tick) & 0xFFFFFFFF
            print(f'tick advanced: {tick_delta} (first={self._first_tick}, last={self._last_tick})')

        print()
        print('=== IMU ===')
        q = m.imu_state.quaternion
        a = m.imu_state.accelerometer
        g = m.imu_state.gyroscope
        print(f'  quaternion (w,x,y,z): {q[0]:+.3f} {q[1]:+.3f} {q[2]:+.3f} {q[3]:+.3f}')
        print(f'  accel  (m/s^2)     : {a[0]:+.3f} {a[1]:+.3f} {a[2]:+.3f}  '
              f'(|a|={math.sqrt(a[0]**2+a[1]**2+a[2]**2):.3f})')
        print(f'  gyro   (rad/s)     : {g[0]:+.4f} {g[1]:+.4f} {g[2]:+.4f}')

        print()
        print('=== arm/torso joint angles (rad) ===')
        labels = {
            12: 'waist_yaw', 13: 'waist_roll', 14: 'waist_pitch',
            15: 'L_shoulder_pitch', 16: 'L_shoulder_roll', 17: 'L_shoulder_yaw',
            18: 'L_elbow', 19: 'L_wrist_roll', 20: 'L_wrist_pitch', 21: 'L_wrist_yaw',
            22: 'R_shoulder_pitch', 23: 'R_shoulder_roll', 24: 'R_shoulder_yaw',
            25: 'R_elbow', 26: 'R_wrist_roll', 27: 'R_wrist_pitch', 28: 'R_wrist_yaw',
        }
        for j in ARM_JOINTS:
            ms = m.motor_state[j]
            print(f'  joint[{j:2d}] {labels.get(j, "?"):20s}  '
                  f'q={ms.q:+7.3f}  dq={ms.dq:+7.3f}  tau_est={ms.tau_est:+7.3f}')

        print()
        print('=== sanity verdict ===')
        a_mag = math.sqrt(a[0]**2 + a[1]**2 + a[2]**2)
        gravity_ok = 9.0 < a_mag < 10.5
        any_nonzero_q = any(abs(m.motor_state[j].q) > 1e-3 for j in ARM_JOINTS)
        print(f'  IMU accel magnitude in [9.0, 10.5] m/s^2 (gravity expected): '
              f'{a_mag:.2f}  -> {"OK" if gravity_ok else "WRONG"}')
        print(f'  at least one arm/torso joint q != 0 (real model, not fixture): '
              f'{"OK" if any_nonzero_q else "ALL ZERO (suspicious)"}')
        print(f'  tick advancing: '
              f'{"OK" if self._first_tick != self._last_tick else "STUCK"}')


def main() -> int:
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
    rclpy.init()
    node = LowStateInspector()
    t0 = time.monotonic()
    try:
        while time.monotonic() - t0 < duration:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    elapsed = time.monotonic() - t0
    node.report(elapsed)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
