"""Common ROS glue shared by pick/place action servers.

Lives separately so the controller layer stays ROS-free and testable.
"""
from __future__ import annotations

import threading
from typing import Optional

from rclpy.node import Node
from std_msgs.msg import String

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


# Topic used to coordinate hand-off between arm action servers running in
# separate processes. When a server is about to start a new sequence, it
# publishes its node name; every other arm server's hold thread stops
# immediately so it doesn't fight the freshly-started sequence over /lowcmd.
ARM_TAKE_CONTROL_TOPIC = '/arms/take_control'


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
        self._self_name = node.get_name()

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

        # Cross-process release channel. When another arm server announces
        # "I'm taking control" we drop our hold thread so it doesn't fight
        # the new sequence over /lowcmd. Our own announcements are ignored.
        self._take_control_pub = node.create_publisher(
            String, ARM_TAKE_CONTROL_TOPIC, 10,
        )
        node.create_subscription(
            String, ARM_TAKE_CONTROL_TOPIC, self._on_take_control, 10,
        )

    def announce_take_control(self) -> None:
        """Call before run_sequence() to silence other servers' hold loops."""
        msg = String()
        msg.data = self._self_name
        self._take_control_pub.publish(msg)

    def _on_take_control(self, msg) -> None:
        if msg.data == self._self_name:
            return  # our own announcement; nothing to do
        # Stop our hold loop. Safe to call when no hold is running.
        try:
            self.controller._stop_hold()
        except Exception:
            pass

    def _on_low_state(self, msg) -> None:
        with self._lock:
            self._low_state = msg

    def get_low_state(self):
        with self._lock:
            return self._low_state
