"""Phase 0 fixture: publish a constant `unitree_hg/LowState` on `/lowstate`.

The arm skills (`pick_box`, `place_box`) refuse to start until they receive at
least one LowState within `low_state_timeout_s`. On the real robot this stream
comes from the G1 firmware at ~500 Hz; in MuJoCo from `unitree_mujoco`'s bridge.
For Phase 0 we don't have either, so we publish an idle state — all 35 motors
at q=0, dq=0, tau_est=0 — so the arm controller can wake up and run sequences
against fake encoders.

What this fixture does NOT do:
  - Track LowCmd output (motors don't actually move). `_wait_for_body_at_rest`
    is currently a stub sleep, so nothing checks convergence to the target.
  - Simulate grasp torque. `grasp_verifier` will read tau_est=0 everywhere and
    decide "no grasp" - which is fine for Phase 0 BT smoke tests as long as
    pick/place actions are mocked or `grasp_required` is false.

Topic: /lowstate (geometry: unitree_hg/LowState)
Rate:  200 Hz (configurable via `rate_hz` param)
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from unitree_hg.msg import LowState


class SimLowStatePublisher(Node):
    def __init__(self) -> None:
        super().__init__('sim_lowstate_publisher_node')
        self.declare_parameter('lowstate_topic', '/lowstate')
        self.declare_parameter('rate_hz', 200.0)

        rate_hz = max(1.0, float(self.get_parameter('rate_hz').value))
        topic = str(self.get_parameter('lowstate_topic').value)

        self._tick = 0
        self._pub = self.create_publisher(LowState, topic, 10)
        # Pre-build the message once; only `tick` changes per publish.
        self._msg = self._build_idle_lowstate()
        self.create_timer(1.0 / rate_hz, self._publish)
        self.get_logger().info(
            f'sim_lowstate_publisher ready @ {rate_hz:.0f} Hz on {topic} '
            f'({len(self._msg.motor_state)} motors, all idle)'
        )

    @staticmethod
    def _build_idle_lowstate() -> LowState:
        msg = LowState()
        # All motor_state entries default-construct to zeros, which matches our
        # "robot at rest, no torque, no grasp" intent. IMU quaternion left at
        # default (zero); arm controller does not read IMU in the current code.
        for m in msg.motor_state:
            m.q = 0.0
            m.dq = 0.0
            m.tau_est = 0.0
        return msg

    def _publish(self) -> None:
        self._tick = (self._tick + 1) & 0xFFFFFFFF
        self._msg.tick = self._tick
        self._pub.publish(self._msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimLowStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
