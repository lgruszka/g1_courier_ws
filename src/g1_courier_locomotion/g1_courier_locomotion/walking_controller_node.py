"""Phase 1.3 alpha — standing PD controller + arm command passthrough.

This is the simplest viable walking controller (variant α from CLAUDE.md
"Sub-fazy Fazy 1"): the robot does NOT actually walk, it just stands still
in a fixed soft-knee pose, holding the legs against gravity with PD. The
existing arm action servers can run while it stands.

Architecture (sim-only — on the real robot you'd use /arm_sdk + sport-mode):

  /lowstate (mac MuJoCo bridge) ───┐
  /cmd_vel (cmd_vel_arbiter)   ───┤
  /arm_intent (arm_action)     ───┤
                                  ▼
                    walking_controller_node
                                  ▼
                              /lowcmd  ───► mac MuJoCo bridge ► motor torques

Node fills a single LowCmd per frame:
  motor_cmd[0..11]  — legs: PD toward STAND_POSE, kp/kd from params (default
                       150 / 5).
  motor_cmd[12..14] — waist: passthrough from /arm_intent if fresh, else PD
                       to zero (keyframes.ARM_JOINTS includes waist 12..14
                       so the arm controller already drives them).
  motor_cmd[15..28] — arms: passthrough from /arm_intent if fresh, else
                       zero (no torque, arms fall under gravity).
  motor_cmd[29]     — ARM_ENABLE: passthrough from /arm_intent.

`/cmd_vel` is read but currently unused — variant α holds standing pose only.
A later variant (β) would tilt the pelvis or take steps.

Real-robot path: this node is sim-only. On the real robot the firmware sport
mode handles legs and the arm controller publishes directly to /arm_sdk.
"""
from __future__ import annotations

import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from unitree_hg.msg import LowCmd, LowState

from g1_courier_arm_skills.lowcmd_crc import LowCmdCrc
from g1_courier_arm_skills.keyframes import ARM_JOINTS, ARM_ENABLE_JOINT


# Joint indices on the G1 (matches g1_29dof.xml ordering).
LEG_JOINTS = (0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11)
WAIST_JOINTS = (12, 13, 14)

# Soft-knee standing pose (rad). Hip pitch + knee + ankle pitch balance so
# the centre of mass sits over the feet with knees slightly bent.
STAND_POSE_LEGS = {
    0:  -0.10,   # left_hip_pitch
    1:   0.00,   # left_hip_roll
    2:   0.00,   # left_hip_yaw
    3:   0.30,   # left_knee
    4:  -0.20,   # left_ankle_pitch
    5:   0.00,   # left_ankle_roll
    6:  -0.10,   # right_hip_pitch
    7:   0.00,
    8:   0.00,
    9:   0.30,   # right_knee
    10: -0.20,
    11:  0.00,
}


class WalkingController(Node):
    def __init__(self) -> None:
        super().__init__('walking_controller_node')

        self.declare_parameter('lowstate_topic', '/lowstate')
        self.declare_parameter('lowcmd_topic', '/lowcmd')
        self.declare_parameter('arm_intent_topic', '/arm_intent')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('publish_rate_hz', 100.0)
        self.declare_parameter('leg_kp', 150.0)
        self.declare_parameter('leg_kd', 5.0)
        self.declare_parameter('waist_kp', 80.0)
        self.declare_parameter('waist_kd', 2.0)
        self.declare_parameter('arm_intent_timeout_s', 0.5)

        self._timeout_ns = int(
            float(self.get_parameter('arm_intent_timeout_s').value) * 1e9
        )
        self._crc = LowCmdCrc()

        self._lock = threading.Lock()
        self._latest_arm: Optional[LowCmd] = None
        self._latest_arm_stamp_ns = 0
        self._latest_cmd_vel = Twist()
        self._latest_cmd_vel_stamp_ns = 0
        self._lowstate: Optional[LowState] = None

        self._pub = self.create_publisher(
            LowCmd, str(self.get_parameter('lowcmd_topic').value), 10,
        )
        self.create_subscription(
            LowState, str(self.get_parameter('lowstate_topic').value),
            self._on_lowstate, 10,
        )
        self.create_subscription(
            LowCmd, str(self.get_parameter('arm_intent_topic').value),
            self._on_arm_intent, 10,
        )
        self.create_subscription(
            Twist, str(self.get_parameter('cmd_vel_topic').value),
            self._on_cmd_vel, 10,
        )

        rate_hz = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(
            f'walking_controller (alpha standing PD) ready @ {rate_hz:.0f} Hz'
        )

    # ---------- callbacks ----------

    def _on_lowstate(self, msg: LowState) -> None:
        with self._lock:
            self._lowstate = msg

    def _on_arm_intent(self, msg: LowCmd) -> None:
        with self._lock:
            self._latest_arm = msg
            self._latest_arm_stamp_ns = self.get_clock().now().nanoseconds

    def _on_cmd_vel(self, msg: Twist) -> None:
        with self._lock:
            self._latest_cmd_vel = msg
            self._latest_cmd_vel_stamp_ns = self.get_clock().now().nanoseconds

    # ---------- main loop ----------

    def _tick(self) -> None:
        kp_leg = float(self.get_parameter('leg_kp').value)
        kd_leg = float(self.get_parameter('leg_kd').value)
        kp_waist = float(self.get_parameter('waist_kp').value)
        kd_waist = float(self.get_parameter('waist_kd').value)

        cmd = LowCmd()

        # Legs: standing PD.
        for j, q_target in STAND_POSE_LEGS.items():
            m = cmd.motor_cmd[j]
            m.q = float(q_target)
            m.dq = 0.0
            m.kp = kp_leg
            m.kd = kd_leg
            m.tau = 0.0

        with self._lock:
            arm = self._latest_arm
            arm_age_ns = (self.get_clock().now().nanoseconds
                          - self._latest_arm_stamp_ns)

        arm_fresh = (arm is not None and arm_age_ns < self._timeout_ns)

        if arm_fresh:
            # Passthrough: copy arm part (which keyframes.ARM_JOINTS covers,
            # i.e. 15..28 for arms + 12..14 for waist) verbatim from the
            # arm action server's intent.
            for j in ARM_JOINTS:
                src = arm.motor_cmd[j]
                m = cmd.motor_cmd[j]
                m.q = float(src.q)
                m.dq = float(src.dq)
                m.kp = float(src.kp)
                m.kd = float(src.kd)
                m.tau = float(src.tau)
            cmd.motor_cmd[ARM_ENABLE_JOINT].q = float(
                arm.motor_cmd[ARM_ENABLE_JOINT].q
            )
        else:
            # No fresh arm intent — hold waist at zero with PD; arms stay
            # at zero torque (no command), which means they hang under
            # gravity. That's fine while the BT is idle.
            for j in WAIST_JOINTS:
                m = cmd.motor_cmd[j]
                m.q = 0.0
                m.dq = 0.0
                m.kp = kp_waist
                m.kd = kd_waist
                m.tau = 0.0
            cmd.motor_cmd[ARM_ENABLE_JOINT].q = 0.0

        cmd.crc = self._crc.Crc(cmd)
        self._pub.publish(cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WalkingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
