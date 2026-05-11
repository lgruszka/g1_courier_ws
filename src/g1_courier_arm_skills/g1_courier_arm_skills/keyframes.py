"""Arm keyframe library extracted from j2s-light_tracking.

Each pose is 17 floats:
- 0..6   : left arm (joints 15..21)
- 7..13  : right arm (joints 22..28)
- 14..16 : torso/waist (joints 12, 13, 14)
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Tuple

ARM_JOINTS: Tuple[int, ...] = (
    15, 16, 17, 18, 19, 20, 21,
    22, 23, 24, 25, 26, 27, 28,
    12, 13, 14,
)
ARM_ENABLE_JOINT: int = 29
NUM_DOF: int = len(ARM_JOINTS)

# Keyframes from rece_pozycja_7_5_pobranie.py (P0..P6) - validated on real robot.
P0 = [
    +0.2910, +0.0000, +0.0000, +0.0000, +0.0000, +0.0000, +0.0000,
    +0.2390, +0.0000, +0.0000, +0.0000, +0.0000, +0.0000, +0.0000,
    +0.0000, +0.0000, +0.0000,
]
P1 = [
    +0.6330, +0.3020, +0.1090, -0.6580, -0.3620, +0.0600, +0.1150,
    +0.6330, -0.3020, -0.1090, -0.6580, +0.3620, +0.0600, -0.1150,
    +0.0000, +0.0000, +0.0000,
]
P2 = [
    -0.7280, +0.2980, -0.1620, +0.6090, +0.0740, +0.1560, +0.1960,
    -0.7280, -0.2980, +0.1620, +0.6090, -0.0740, +0.1560, -0.1960,
    +0.0000, +0.0000, +0.0000,
]
P3 = [
    -0.8280, +0.2480, -0.1620, +0.6090, +0.0740, +0.1560, +0.5960,
    -0.7780, -0.0480, +0.1620, +0.6090, -0.0740, +0.1560, -0.8960,
    +0.0000, +0.0000, +0.0000,
]
P4 = [
    -0.8280, +0.0480, -0.1620, +0.6090, +0.0740, +0.1560, +0.1960,
    -0.7780, +0.0020, +0.1620, +0.6090, -0.0740, +0.1560, -0.1960,
    +0.0000, +0.0000, +0.0000,
]
P5 = [
    -0.8780, +0.0480, -0.2620, +0.3090, +0.0740, +0.3560, +0.4960,
    -0.8780, -0.0480, +0.2620, +0.3090, -0.0740, +0.3560, -0.4960,
    +0.0000, +0.0000, +0.0000,
]
P6 = [
    -0.4780, +0.0480, -0.2620, -0.1910, +0.0740, +0.3560, +0.4960,
    -0.4780, -0.0480, +0.2620, -0.1910, -0.0740, +0.3560, -0.4960,
    +0.0000, +0.0000, +0.0000,
]
ZERO = [0.0] * NUM_DOF


@dataclass
class TrajectoryStage:
    """One leg of an arm sequence: from previous pose to target with weight ramp."""
    target: List[float]
    duration_s: float
    weight_start: float = 1.0
    weight_end: float = 1.0
    label: str = ''


@dataclass
class ArmSequence:
    """A named sequence of TrajectoryStages."""
    name: str
    stages: List[TrajectoryStage] = field(default_factory=list)
    end_pose: List[float] = field(default_factory=lambda: list(ZERO))
    final_weight: float = 1.0


# rece_pozycja_7_5_pobranie: q_act -> P0 -> ... -> P6, hold weight=1
PICK_BOX = ArmSequence(
    name='pick_box',
    stages=[
        TrajectoryStage(P0, 2.0, label='to_P0'),
        TrajectoryStage(P1, 2.0, label='to_P1'),
        TrajectoryStage(P2, 2.0, label='to_P2'),
        TrajectoryStage(P3, 2.0, label='approach'),
        TrajectoryStage(P4, 2.0, label='grasp'),
        TrajectoryStage(P5, 2.0, label='lift'),
        TrajectoryStage(P6, 2.0, label='carry_pose'),
    ],
    end_pose=P6,
    final_weight=1.0,
)

# rece_pozycja_8_5_odlozenie: q_act -> P5 (snap) -> P5 (hold) -> P4 -> P3 ->
# P2 -> 0 -> release. Matches j2s-light_tracking ArmSkillController:
# PLACE_T_SNAP_S=1.0 + PLACE_POSITIONS=[P5, P4, P3, P2] iterated with
# previous=P5 so first iter is a 2 s hold at P5 before lowering.
PLACE_BOX = ArmSequence(
    name='place_box',
    stages=[
        TrajectoryStage(P5, 1.0, label='snap_to_P5'),
        TrajectoryStage(P5, 2.0, label='hold_P5'),
        TrajectoryStage(P4, 2.0, label='lower'),
        TrajectoryStage(P3, 2.0, label='release_height'),
        TrajectoryStage(P2, 2.0, label='retract'),
        TrajectoryStage(ZERO, 1.0, label='to_zero'),
        TrajectoryStage(ZERO, 1.0, weight_end=0.0, label='handoff_to_fsm'),
    ],
    end_pose=ZERO,
    final_weight=0.0,
)

LIBRARY: Dict[str, ArmSequence] = {
    PICK_BOX.name: PICK_BOX,
    PLACE_BOX.name: PLACE_BOX,
}
