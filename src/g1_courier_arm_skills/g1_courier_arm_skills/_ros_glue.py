"""Common ROS glue shared by pick/place action servers.

Lives separately so the controller layer stays ROS-free and testable.
"""
from __future__ import annotations

import threading
from typing import Optional

from rclpy.node import Node

try:
    from unitree_hg.msg import LowCmd, LowState
    UNITREE_HG_OK = True
    UNITREE_HG_ERR: Optional[Exception] = None
except ImportError as exc:  # pragma: no cover
    LowCmd = None  # type: ignore[assignment]
    LowState = None  # type: ignore[assignment]
    UNITREE_HG_OK = False
    UNITREE_HG_ERR = exc

from .arm_controller import ArmController, ArmControllerConfig
from .grasp_verifier import GraspVerifier
from .keyframes import ARM_JOINTS
from .lowcmd_crc import LowCmdCrc


class _Publisher:
    def __init__(self, ros_pub):
        self._pub = ros_pub

    def Write(self, msg) -> None:
        self._pub.publish(msg)


class ArmRosBundle:
    """Holds publisher + low_state subscription + ArmController + GraspVerifier
    so the action servers don't duplicate setup code."""

    def __init__(self, node: Node, *, arm_topic: str, lowstate_topic: str,
                 grasp_threshold_nm: float, controller_config: ArmControllerConfig) -> None:
        if not UNITREE_HG_OK:
            raise RuntimeError(
                f'unitree_hg messages not available: {UNITREE_HG_ERR}'
            )
        self._node = node
        self._lock = threading.Lock()
        self._low_state: Optional[object] = None

        self._arm_pub = node.create_publisher(LowCmd, arm_topic, 10)
        self._low_sub = node.create_subscription(
            LowState, lowstate_topic, self._on_low_state, 10,
        )
        self.controller = ArmController(
            low_cmd_ctor=LowCmd,
            arm_publisher=_Publisher(self._arm_pub),
            crc=LowCmdCrc(),
            get_low_state=self.get_low_state,
            log_fn=lambda msg: node.get_logger().info(f'[arm] {msg}'),
            config=controller_config,
        )
        self.verifier = GraspVerifier(
            get_low_state=self.get_low_state,
            joint_indices=ARM_JOINTS[:14],  # both arms (pick is bimanual; either side may slip)
            threshold_nm=grasp_threshold_nm,
            log_fn=lambda msg: node.get_logger().info(f'[verify] {msg}'),
        )

    def _on_low_state(self, msg) -> None:
        with self._lock:
            self._low_state = msg

    def get_low_state(self):
        with self._lock:
            return self._low_state
