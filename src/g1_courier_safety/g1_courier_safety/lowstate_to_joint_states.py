"""Adapter: /lowstate -> /joint_states for robot_state_publisher.

Unitree firmware publishes /lowstate (unitree_hg/LowState) with motor
positions in motor_state[i].q. robot_state_publisher needs /joint_states
(sensor_msgs/JointState) keyed by URDF joint names to compute TF.

This node maps motor index -> joint name per the G1 29-DoF convention
matching unitree_ros/robots/g1_description URDF. For other Unitree
models (H1, etc.), edit JOINT_NAMES below or pass `joint_names_yaml`
parameter pointing to a YAML override.
"""
from __future__ import annotations

import sys
from typing import List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

try:
    from unitree_hg.msg import LowState
    UNITREE_HG_OK = True
    UNITREE_HG_ERR: Optional[Exception] = None
except ImportError as exc:
    LowState = None  # type: ignore[assignment]
    UNITREE_HG_OK = False
    UNITREE_HG_ERR = exc


# G1 29-DoF motor index -> joint name (matches g1_description URDF in
# unitree_ros/robots/g1_description/g1_29dof.urdf).
DEFAULT_JOINT_NAMES: List[str] = [
    'left_hip_pitch_joint',          # 0
    'left_hip_roll_joint',           # 1
    'left_hip_yaw_joint',            # 2
    'left_knee_joint',               # 3
    'left_ankle_pitch_joint',        # 4
    'left_ankle_roll_joint',         # 5
    'right_hip_pitch_joint',         # 6
    'right_hip_roll_joint',          # 7
    'right_hip_yaw_joint',           # 8
    'right_knee_joint',              # 9
    'right_ankle_pitch_joint',       # 10
    'right_ankle_roll_joint',        # 11
    'waist_yaw_joint',               # 12
    'waist_roll_joint',              # 13
    'waist_pitch_joint',             # 14
    'left_shoulder_pitch_joint',     # 15
    'left_shoulder_roll_joint',      # 16
    'left_shoulder_yaw_joint',       # 17
    'left_elbow_joint',              # 18
    'left_wrist_roll_joint',         # 19
    'left_wrist_pitch_joint',        # 20
    'left_wrist_yaw_joint',          # 21
    'right_shoulder_pitch_joint',    # 22
    'right_shoulder_roll_joint',     # 23
    'right_shoulder_yaw_joint',      # 24
    'right_elbow_joint',             # 25
    'right_wrist_roll_joint',        # 26
    'right_wrist_pitch_joint',       # 27
    'right_wrist_yaw_joint',         # 28
]


class LowStateToJointStates(Node):
    def __init__(self) -> None:
        super().__init__('lowstate_to_joint_states')
        self.declare_parameter('lowstate_topic', '/lf/lowstate')
        self.declare_parameter('joint_states_topic', '/joint_states')
        # Comma-separated override for non-G1-29DoF; empty = use DEFAULT_JOINT_NAMES.
        self.declare_parameter('joint_names_csv', '')

        in_topic = str(self.get_parameter('lowstate_topic').value)
        out_topic = str(self.get_parameter('joint_states_topic').value)
        csv = str(self.get_parameter('joint_names_csv').value).strip()
        self._joint_names = (
            [name.strip() for name in csv.split(',') if name.strip()]
            if csv else DEFAULT_JOINT_NAMES
        )

        self._pub = self.create_publisher(JointState, out_topic, 10)
        self._sub = self.create_subscription(LowState, in_topic, self._on_lowstate, 10)
        self.get_logger().info(
            f'lowstate_to_joint_states ready '
            f'({in_topic} -> {out_topic}, {len(self._joint_names)} joints)'
        )

    def _on_lowstate(self, msg) -> None:
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = list(self._joint_names)
        n = len(self._joint_names)
        out.position = [float(msg.motor_state[i].q) for i in range(n)]
        out.velocity = [float(msg.motor_state[i].dq) for i in range(n)]
        out.effort = [float(msg.motor_state[i].tau_est) for i in range(n)]
        self._pub.publish(out)


def main(args=None) -> None:
    if not UNITREE_HG_OK:
        sys.stderr.write(
            f'lowstate_to_joint_states requires unitree_hg messages. '
            f'Import error: {UNITREE_HG_ERR}\n'
        )
        return
    rclpy.init(args=args)
    node = LowStateToJointStates()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
