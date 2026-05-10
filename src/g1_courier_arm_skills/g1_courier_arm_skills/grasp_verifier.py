"""Grasp verification by joint torque deviation.

The principle: take a baseline of joint torques on the carrying arm just before
contact, then after lift compare. A held box produces a sustained tau shift
proportional to its mass and the arm geometry. No load -> tau returns to baseline.
"""
from __future__ import annotations

from typing import Callable, List, Optional, Sequence


class GraspVerifier:
    def __init__(
        self,
        get_low_state: Callable[[], Optional[object]],
        joint_indices: Sequence[int],
        threshold_nm: float,
        log_fn: Callable[[str], None],
        sample_count: int = 5,
    ) -> None:
        self._get_low_state = get_low_state
        self._joints = list(joint_indices)
        self._threshold = abs(float(threshold_nm))
        self._log = log_fn
        self._sample_count = max(1, int(sample_count))
        self._baseline: Optional[List[float]] = None

    def capture_baseline(self) -> None:
        self._baseline = self._sample_taus()
        self._log(f'grasp baseline tau: {self._baseline}')

    def verify_grasp(self) -> bool:
        if self._baseline is None:
            self._log('grasp verifier: no baseline -> cannot verify, returning True optimistically')
            return True
        current = self._sample_taus()
        max_delta = max(abs(c - b) for c, b in zip(current, self._baseline))
        ok = max_delta >= self._threshold
        self._log(f'grasp verify: max |dtau|={max_delta:.3f} threshold={self._threshold:.3f} -> {ok}')
        return ok

    def verify_release(self) -> bool:
        """Conjugate of verify_grasp - true if tau dropped near baseline."""
        if self._baseline is None:
            return True
        current = self._sample_taus()
        max_delta = max(abs(c - b) for c, b in zip(current, self._baseline))
        ok = max_delta < self._threshold
        self._log(f'release verify: max |dtau|={max_delta:.3f} threshold={self._threshold:.3f} -> {ok}')
        return ok

    def _sample_taus(self) -> List[float]:
        sums = [0.0] * len(self._joints)
        valid = 0
        for _ in range(self._sample_count):
            state = self._get_low_state()
            if state is None:
                continue
            for i, joint in enumerate(self._joints):
                sums[i] += float(state.motor_state[joint].tau_est)
            valid += 1
        if valid == 0:
            raise RuntimeError('GraspVerifier: no low_state samples available')
        return [s / valid for s in sums]
