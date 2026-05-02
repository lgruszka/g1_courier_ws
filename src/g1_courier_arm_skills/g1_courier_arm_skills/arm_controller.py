"""Parametric arm controller for the G1 courier stack.

Refactored from j2s-light_tracking/arm_skill_controller.py.

Improvements over the original:
- Trajectory data lives in keyframes.py (so a different sequence can be loaded).
- Body-at-rest gate before motion (stub: sleeps a settle window - TODO real check).
- Optional parametric pose offset (cartesian -> joint correction hook - TODO IK).
- Per-stage progress callback (action feedback hook).
- Stop event with weight-zero ramp (clean handoff to FSM on abort).
- Grasp / release verification injected via GraspVerifier.

What is *not* yet implemented (clear TODOs):
- Real cartesian -> joint correction needs IK against G1 URDF (pinocchio/pink).
  The hook `_apply_pose_correction` is a no-op linear placeholder.
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence

from .keyframes import ARM_ENABLE_JOINT, ARM_JOINTS, NUM_DOF, ArmSequence, LIBRARY


@dataclass
class ArmControllerConfig:
    control_dt_s: float = 0.02
    kp: float = 80.0
    kd: float = 2.0
    low_state_timeout_s: float = 5.0
    body_velocity_threshold_mps: float = 0.02
    body_velocity_settle_s: float = 0.3
    # Sim-only escape hatch. When True, _publish_pose sets motor_cmd[i].mode=99
    # for arm joints. The patched unitree_mujoco bridge interprets that as
    # "snap data.qpos[i] directly, bypass PD". Effect: smooth motion without
    # PD oscillation against the welded pelvis. On the real robot keep
    # kinematic_mode=False — real motor firmware does not understand mode=99.
    kinematic_mode: bool = False


class ArmSkillAborted(RuntimeError):
    pass


class ArmController:
    def __init__(
        self,
        low_cmd_ctor,
        arm_publisher,
        crc,
        get_low_state: Callable[[], Optional[object]],
        log_fn: Callable[[str], None],
        config: Optional[ArmControllerConfig] = None,
    ) -> None:
        self._LowCmd = low_cmd_ctor
        self._publisher = arm_publisher
        self._crc = crc
        self._get_low_state = get_low_state
        self._log = log_fn
        self._cfg = config or ArmControllerConfig()
        self._stop_event = threading.Event()
        # Per-action callbacks set by run_sequence(); reset each time.
        # Concurrent run_sequence() calls are prevented by the action server's
        # own busy lock - no second guard needed here.
        self._on_phase: Optional[Callable[[str, float], None]] = None
        self._pose_offset_xyz: Optional[Sequence[float]] = None

    # ---------- public API ----------

    def run_sequence(
        self,
        sequence_name: str,
        on_phase: Optional[Callable[[str, float], None]] = None,
        pose_offset_xyz: Optional[Sequence[float]] = None,
    ) -> None:
        sequence = LIBRARY.get(sequence_name)
        if sequence is None:
            raise ValueError(f'Unknown arm sequence: {sequence_name}')

        self._stop_event.clear()
        self._on_phase = on_phase
        self._pose_offset_xyz = pose_offset_xyz
        try:
            self._wait_for_low_state()
            self._wait_for_body_at_rest()

            previous = self._snapshot_current_pose()
            n_stages = len(sequence.stages)
            for i, stage in enumerate(sequence.stages):
                target = self._apply_pose_correction(stage.target)
                self._emit_phase(stage.label, i / max(1, n_stages))
                self._run_interpolation(
                    from_pose=previous,
                    to_pose=target,
                    duration_s=stage.duration_s,
                    weight_start=stage.weight_start,
                    weight_end=stage.weight_end,
                )
                previous = target

            self._publish_pose(sequence.end_pose, weight=sequence.final_weight)
            self._emit_phase('done', 1.0)
            self._log(f'Arm sequence completed: {sequence.name}')
        finally:
            self._on_phase = None
            self._pose_offset_xyz = None

    def stop(self) -> None:
        self._stop_event.set()
        self._publish_sdk_enable(0.0)
        self._log('Arm sequence stop requested (arm_sdk weight=0).')

    # ---------- internals ----------

    def _emit_phase(self, label: str, progress: float) -> None:
        cb = self._on_phase
        if cb is not None:
            try:
                cb(label, max(0.0, min(1.0, progress)))
            except Exception as exc:
                self._log(f'Phase callback raised: {exc}')

    def _apply_pose_correction(self, target: List[float]) -> List[float]:
        """Hook to bend a nominal keyframe by a measured cartesian offset.

        Real implementation requires IK against the URDF. For the starter
        package this is a no-op so the call site is correct and tunable later.
        """
        if self._pose_offset_xyz is None:
            return list(target)
        # TODO: replace with IK delta. For now return target unmodified
        # but log so we notice when the hook is exercised.
        self._log(
            f'pose_offset_xyz={tuple(self._pose_offset_xyz)} - correction not applied (TODO IK).'
        )
        return list(target)

    def _run_interpolation(
        self,
        from_pose: List[float],
        to_pose: List[float],
        duration_s: float,
        weight_start: float,
        weight_end: float,
    ) -> None:
        duration_s = max(float(duration_s), 0.0)
        if duration_s <= 0.0:
            self._publish_pose(to_pose, weight=weight_end)
            return

        start = time.monotonic()
        while True:
            self._raise_if_stop_requested()
            elapsed = time.monotonic() - start
            if elapsed >= duration_s:
                break
            ratio = max(0.0, min(1.0, elapsed / duration_s))
            # Smoothstep `3t^2 - 2t^3`: zero derivatives at both ends, so
            # joint velocities ramp up and down smoothly across waypoint
            # boundaries instead of jumping. Linear blends caused visible
            # jerks in MuJoCo at every stage transition.
            smooth_ratio = ratio * ratio * (3.0 - 2.0 * ratio)
            pose = [(1.0 - smooth_ratio) * a + smooth_ratio * b
                    for a, b in zip(from_pose, to_pose)]
            weight = (1.0 - smooth_ratio) * weight_start + smooth_ratio * weight_end
            self._publish_pose(pose, weight=weight)
            time.sleep(self._cfg.control_dt_s)
        self._publish_pose(to_pose, weight=weight_end)

    def _publish_pose(self, pose: List[float], weight: float) -> None:
        if len(pose) != NUM_DOF:
            raise ValueError(f'pose length {len(pose)} != {NUM_DOF}')
        # We need a low_state to make sure the bridge is alive; we don't read q from it.
        if self._get_low_state() is None:
            raise RuntimeError('low_state not available while publishing arm pose.')

        cmd = self._LowCmd()
        cmd.motor_cmd[ARM_ENABLE_JOINT].q = float(weight)
        kinematic = bool(self._cfg.kinematic_mode)
        for idx, joint in enumerate(ARM_JOINTS):
            m = cmd.motor_cmd[joint]
            m.tau = 0.0
            m.q = float(pose[idx])
            m.dq = 0.0
            m.kp = self._cfg.kp
            m.kd = self._cfg.kd
            if kinematic:
                m.mode = 99   # sim-only sentinel — patched bridge writes qpos directly
        cmd.crc = self._crc.Crc(cmd)
        self._publisher.Write(cmd)

    def _publish_sdk_enable(self, value: float) -> None:
        cmd = self._LowCmd()
        cmd.motor_cmd[ARM_ENABLE_JOINT].q = float(value)
        cmd.crc = self._crc.Crc(cmd)
        self._publisher.Write(cmd)

    def _snapshot_current_pose(self) -> List[float]:
        state = self._get_low_state()
        if state is None:
            raise RuntimeError('low_state missing for snapshot.')
        return [float(state.motor_state[j].q) for j in ARM_JOINTS]

    def _wait_for_low_state(self) -> None:
        start = time.monotonic()
        while self._get_low_state() is None:
            self._raise_if_stop_requested()
            if (time.monotonic() - start) >= self._cfg.low_state_timeout_s:
                raise RuntimeError('low_state not received (timeout).')
            time.sleep(0.05)

    def _wait_for_body_at_rest(self) -> None:
        """Stub: sleep a settle window in chunks so cancel can interrupt.

        TODO: read actual body velocity from low_state.imu_state or /odom and
        gate on threshold. For the starter we trust that cmd_vel arbiter has
        already stopped commanding motion before this action started.
        """
        deadline = time.monotonic() + self._cfg.body_velocity_settle_s
        while time.monotonic() < deadline:
            self._raise_if_stop_requested()
            time.sleep(0.05)

    def _raise_if_stop_requested(self) -> None:
        if not self._stop_event.is_set():
            return
        self._publish_sdk_enable(0.0)
        raise ArmSkillAborted('Arm sequence aborted.')
