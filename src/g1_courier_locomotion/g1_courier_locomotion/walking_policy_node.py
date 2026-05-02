"""Phase 1.3 beta — pretrained RL walking policy from unitree_rl_gym.

Loads a TorchScript policy trained in legged_gym for the G1 humanoid and
runs it at 50 Hz against the live `/lowstate` stream coming from the macOS
`unitree_mujoco` bridge. Publishes `/lowcmd` with PD targets for the 12
leg joints; arm/torso/wrist joints are passed through from `/arm_intent`
(written by `pick_action_server` / `place_action_server`).

Reference: deploy/deploy_mujoco/deploy_mujoco.py from
https://github.com/unitreerobotics/unitree_rl_gym (commit master @ clone).
The MuJoCo step there is replaced here by DDS subscribe / publish — the
observation construction and action mapping are otherwise identical.

Topology:

  /lowstate  (mac MuJoCo bridge -> us)        \\
  /cmd_vel   (cmd_vel_arbiter   -> us)         |  WalkingPolicyNode
  /arm_intent (arm action srv   -> us)         |  (50 Hz inference)
                                              /
                                              \\
                            /lowcmd  (us -> mac MuJoCo bridge)

Observation (47 dim, scales from config):
  obs[0:3]   = imu_state.gyroscope * ang_vel_scale
  obs[3:6]   = gravity-in-body from imu_state.quaternion (wxyz)
  obs[6:9]   = cmd_vel * cmd_scale  (vx, vy, wz)
  obs[9:21]  = (q[0..11] - default_angles) * dof_pos_scale
  obs[21:33] = dq[0..11] * dof_vel_scale
  obs[33:45] = previous action
  obs[45:47] = sin / cos of phase clock (period 0.8 s)

Action (12 dim) → target_q = action * action_scale + default_angles, then
mapped to motor_cmd[0..11] with per-joint kp / kd from the config.
"""
from __future__ import annotations

import math
import os
import threading
import time
from typing import Optional

import numpy as np
import rclpy
import torch
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from unitree_hg.msg import LowCmd, LowState

from g1_courier_arm_skills.lowcmd_crc import LowCmdCrc
from g1_courier_arm_skills.keyframes import ARM_JOINTS, ARM_ENABLE_JOINT


WAIST_JOINTS = (12, 13, 14)
GAIT_PERIOD_S = 0.8


def _gravity_in_body(quat_wxyz: np.ndarray) -> np.ndarray:
    """Project world gravity (-z) into body frame using quaternion (w, x, y, z)."""
    qw, qx, qy, qz = quat_wxyz
    g = np.empty(3, dtype=np.float32)
    g[0] = 2.0 * (-qz * qx + qw * qy)
    g[1] = -2.0 * (qz * qy + qw * qx)
    g[2] = 1.0 - 2.0 * (qw * qw + qz * qz)
    return g


class WalkingPolicyNode(Node):
    def __init__(self) -> None:
        super().__init__('walking_policy_node')

        share = get_package_share_directory('g1_courier_locomotion')
        default_cfg = os.path.join(share, 'config', 'g1_walking.yaml')
        default_policy = os.path.join(share, 'policies', 'g1_walking.pt')

        self.declare_parameter('config_path', default_cfg)
        self.declare_parameter('policy_path', default_policy)
        self.declare_parameter('lowstate_topic', '/lowstate')
        self.declare_parameter('lowcmd_topic', '/lowcmd')
        self.declare_parameter('arm_intent_topic', '/arm_intent')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('arm_intent_timeout_s', 0.5)
        self.declare_parameter('waist_kp', 80.0)
        self.declare_parameter('waist_kd', 2.0)

        cfg_path = str(self.get_parameter('config_path').value)
        policy_path = str(self.get_parameter('policy_path').value)

        with open(cfg_path, 'r') as f:
            cfg = yaml.safe_load(f)
        self._kps = np.array(cfg['kps'], dtype=np.float32)
        self._kds = np.array(cfg['kds'], dtype=np.float32)
        self._default_angles = np.array(cfg['default_angles'], dtype=np.float32)
        self._ang_vel_scale = float(cfg['ang_vel_scale'])
        self._dof_pos_scale = float(cfg['dof_pos_scale'])
        self._dof_vel_scale = float(cfg['dof_vel_scale'])
        self._action_scale = float(cfg['action_scale'])
        self._cmd_scale = np.array(cfg['cmd_scale'], dtype=np.float32)
        self._num_actions = int(cfg['num_actions'])
        self._num_obs = int(cfg['num_obs'])
        # control_decimation×simulation_dt = our control period (50 Hz typ.)
        self._control_period_s = float(cfg['simulation_dt']) * int(cfg['control_decimation'])

        if self._num_actions != 12:
            self.get_logger().warn(
                f'config says num_actions={self._num_actions}, expected 12 '
                f'(G1 leg joints). Continuing but motor mapping may be off.'
            )

        self._policy = torch.jit.load(policy_path)
        self._policy.eval()
        self.get_logger().info(
            f'loaded policy from {policy_path} (obs={self._num_obs}, '
            f'act={self._num_actions}, control={1.0/self._control_period_s:.0f} Hz)'
        )

        self._action = np.zeros(self._num_actions, dtype=np.float32)
        self._obs = np.zeros(self._num_obs, dtype=np.float32)
        self._t0 = time.monotonic()

        self._lock = threading.Lock()
        self._lowstate: Optional[LowState] = None
        self._cmd = np.zeros(3, dtype=np.float32)
        self._latest_arm: Optional[LowCmd] = None
        self._latest_arm_stamp_ns = 0
        self._timeout_ns = int(
            float(self.get_parameter('arm_intent_timeout_s').value) * 1e9
        )

        self._crc = LowCmdCrc()
        self._pub = self.create_publisher(
            LowCmd, str(self.get_parameter('lowcmd_topic').value), 10,
        )
        self.create_subscription(
            LowState, str(self.get_parameter('lowstate_topic').value),
            self._on_lowstate, qos_profile_sensor_data,
        )
        self.create_subscription(
            Twist, str(self.get_parameter('cmd_vel_topic').value),
            self._on_cmd_vel, 10,
        )
        self.create_subscription(
            LowCmd, str(self.get_parameter('arm_intent_topic').value),
            self._on_arm_intent, 10,
        )

        self.create_timer(self._control_period_s, self._tick)
        self.get_logger().info('walking_policy_node (beta) ready')

    # ---------- callbacks ----------

    def _on_lowstate(self, msg: LowState) -> None:
        with self._lock:
            self._lowstate = msg

    def _on_cmd_vel(self, msg: Twist) -> None:
        with self._lock:
            self._cmd[0] = float(msg.linear.x)
            self._cmd[1] = float(msg.linear.y)
            self._cmd[2] = float(msg.angular.z)

    def _on_arm_intent(self, msg: LowCmd) -> None:
        with self._lock:
            self._latest_arm = msg
            self._latest_arm_stamp_ns = self.get_clock().now().nanoseconds

    # ---------- control loop ----------

    def _tick(self) -> None:
        with self._lock:
            ls = self._lowstate
            cmd = self._cmd.copy()
            arm = self._latest_arm
            arm_age_ns = (self.get_clock().now().nanoseconds
                          - self._latest_arm_stamp_ns)
        if ls is None:
            return  # waiting for first lowstate

        # ----- observation -----
        q = np.fromiter((ls.motor_state[i].q for i in range(12)),
                        dtype=np.float32, count=12)
        dq = np.fromiter((ls.motor_state[i].dq for i in range(12)),
                         dtype=np.float32, count=12)
        quat = np.array(ls.imu_state.quaternion, dtype=np.float32)  # wxyz
        omega = np.array(ls.imu_state.gyroscope, dtype=np.float32)

        qj = (q - self._default_angles) * self._dof_pos_scale
        dqj = dq * self._dof_vel_scale
        omega_obs = omega * self._ang_vel_scale
        gravity = _gravity_in_body(quat)

        t = time.monotonic() - self._t0
        phase = (t % GAIT_PERIOD_S) / GAIT_PERIOD_S
        sin_phase = math.sin(2.0 * math.pi * phase)
        cos_phase = math.cos(2.0 * math.pi * phase)

        n = self._num_actions
        self._obs[0:3] = omega_obs
        self._obs[3:6] = gravity
        self._obs[6:9] = cmd * self._cmd_scale
        self._obs[9:9 + n] = qj
        self._obs[9 + n:9 + 2 * n] = dqj
        self._obs[9 + 2 * n:9 + 3 * n] = self._action
        self._obs[9 + 3 * n:9 + 3 * n + 2] = (sin_phase, cos_phase)

        # ----- inference -----
        with torch.no_grad():
            obs_t = torch.from_numpy(self._obs).unsqueeze(0)
            act_t = self._policy(obs_t)
        self._action = act_t.detach().numpy().squeeze().astype(np.float32)

        target_q = self._action * self._action_scale + self._default_angles

        # ----- LowCmd -----
        out = LowCmd()
        for i in range(12):
            m = out.motor_cmd[i]
            m.q = float(target_q[i])
            m.dq = 0.0
            m.kp = float(self._kps[i])
            m.kd = float(self._kds[i])
            m.tau = 0.0

        arm_fresh = (arm is not None and arm_age_ns < self._timeout_ns)
        if arm_fresh:
            for j in ARM_JOINTS:
                src = arm.motor_cmd[j]
                m = out.motor_cmd[j]
                m.q = float(src.q)
                m.dq = float(src.dq)
                m.kp = float(src.kp)
                m.kd = float(src.kd)
                m.tau = float(src.tau)
            out.motor_cmd[ARM_ENABLE_JOINT].q = float(
                arm.motor_cmd[ARM_ENABLE_JOINT].q
            )
        else:
            wkp = float(self.get_parameter('waist_kp').value)
            wkd = float(self.get_parameter('waist_kd').value)
            for j in WAIST_JOINTS:
                m = out.motor_cmd[j]
                m.q = 0.0
                m.dq = 0.0
                m.kp = wkp
                m.kd = wkd
                m.tau = 0.0
            out.motor_cmd[ARM_ENABLE_JOINT].q = 0.0

        out.crc = self._crc.Crc(out)
        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WalkingPolicyNode()
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
