"""cmd_vel arbiter.

Inputs (priority order, top wins):
  1. /cmd_vel_estop            - if any source latched true, output zero (no override).
  2. freeze flag (service)     - mission asks loco to hold still during arm motion.
  3. /cmd_vel_dock             - dock action while servoing.
  4. /cmd_vel_retreat          - retreat action while backing off.
  5. /cmd_vel_nav              - nav2 controller output.

Output:
  /cmd_vel  -> consumed by unitree_cmd_vel_bridge_node.

Modes:
  - carry_mode (service `/safety/set_carry_mode`): when on, output is clamped
    to lower max_vx / max_vy / max_vyaw so the box stays steady on the arms.

Inactivity:
  - If no fresh input from any source for `cmd_timeout_s`, publishes zero.
"""
from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

from g1_courier_msgs.srv import SetCarryMode, SetFreeze

# Latched profile so a late-joining arbiter still sees the last e-stop value.
_ESTOP_QOS = QoSProfile(
    depth=1, reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST,
)


@dataclass
class _Source:
    msg: Optional[Twist] = None
    last_stamp_ns: int = 0


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _floor(v: float, min_v: float) -> float:
    """Boost |v| to min_v while keeping sign — but only when v is intentionally
    non-zero. v=0 stays 0 (freeze / no source). Used for hardware that has a
    minimum velocity threshold (G1 sport API ~0.11 m/s below which the firmware
    refuses to step). Set min_v=0.0 to disable."""
    import math
    if min_v <= 0.0:
        return v
    if abs(v) < 1e-6:
        return 0.0
    return math.copysign(max(abs(v), min_v), v)


class CmdVelArbiter(Node):
    def __init__(self) -> None:
        super().__init__('cmd_vel_arbiter')

        self.declare_parameter('output_topic', '/cmd_vel')
        # Zrodlo komend nawigacji. nav2 publikuje surowe wyjscie kontrolera na
        # /cmd_vel (u nas remapowane na /cmd_vel_nav), a jego velocity_smoother
        # czyta to samo i wypuszcza /cmd_vel_smoothed. Jesli arbiter czyta
        # /cmd_vel_nav, to smoother publikuje W PROZNIE (zmierzone: 0
        # subskrybentow) i skonfigurowane w nav2_params limity przyspieszenia
        # NIE dzialaja — komendy do robota sa schodkowe (zmierzone 5.88 m/s2,
        # czyli skok pelnozakresowy w jednym ticku 20 Hz, przy limicie 2.5).
        # Dla paczki trzymanej w rekach szarpniecie jest grozniejsze niz zbyt
        # duza predkosc, dlatego warto ustawic /cmd_vel_smoothed.
        # Default zostaje /cmd_vel_nav = zachowanie bez zmian.
        self.declare_parameter('nav_topic', '/cmd_vel_nav')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('cmd_timeout_s', 0.4)
        # When nav2 owns /cmd_vel directly (Faza 1.5+), set this False so the
        # arbiter still hosts /safety/set_carry_mode + /safety/set_freeze
        # services (mission BT needs them) but doesn't fight nav2 on /cmd_vel.
        # Dock/retreat in that mode publish straight to /cmd_vel via their
        # own cmd_vel_topic params.
        self.declare_parameter('enable_publish', True)
        # normal limits
        self.declare_parameter('max_vx_normal', 0.6)
        self.declare_parameter('max_vy_normal', 0.4)
        self.declare_parameter('max_vyaw_normal', 0.8)
        # carry-mode limits (box held)
        self.declare_parameter('max_vx_carry', 0.3)
        self.declare_parameter('max_vy_carry', 0.2)
        self.declare_parameter('max_vyaw_carry', 0.4)
        # Minimum-velocity floor (hardware threshold). Below this the G1 sport
        # API refuses to step. Set 0.0 to disable. Default disabled — enable
        # per-platform in safety.yaml.
        self.declare_parameter('min_vx_threshold', 0.0)
        self.declare_parameter('min_vy_threshold', 0.0)
        self.declare_parameter('min_vyaw_threshold', 0.0)

        self._timeout_ns = int(float(self.get_parameter('cmd_timeout_s').value) * 1e9)
        self._max_normal = (
            float(self.get_parameter('max_vx_normal').value),
            float(self.get_parameter('max_vy_normal').value),
            float(self.get_parameter('max_vyaw_normal').value),
        )
        self._max_carry = (
            float(self.get_parameter('max_vx_carry').value),
            float(self.get_parameter('max_vy_carry').value),
            float(self.get_parameter('max_vyaw_carry').value),
        )
        self._min_floor = (
            float(self.get_parameter('min_vx_threshold').value),
            float(self.get_parameter('min_vy_threshold').value),
            float(self.get_parameter('min_vyaw_threshold').value),
        )

        # State.
        self._lock = threading.Lock()
        self._dock = _Source()
        self._retreat = _Source()
        self._nav = _Source()
        self._estop = False
        self._freeze = False
        self._carrying = False

        # IO.
        self._pub = self.create_publisher(
            Twist, str(self.get_parameter('output_topic').value), 10,
        )
        self.create_subscription(Twist, '/cmd_vel_dock', self._on_dock, 10)
        self.create_subscription(Twist, '/cmd_vel_retreat', self._on_retreat, 10)
        nav_topic = str(self.get_parameter('nav_topic').value)
        self.create_subscription(Twist, nav_topic, self._on_nav, 10)
        self.create_subscription(Bool, '/cmd_vel_estop', self._on_estop, _ESTOP_QOS)
        self.create_service(SetCarryMode, '/safety/set_carry_mode', self._on_set_carry)
        self.create_service(SetFreeze, '/safety/set_freeze', self._on_set_freeze)

        rate_hz = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(
            f"cmd_vel_arbiter ready (nav: '{nav_topic}'"
            f"{' — smoother W TORZE' if 'smoothed' in nav_topic else ''})")

    # ---------- callbacks ----------

    def _stamp(self, src: _Source, msg: Twist) -> None:
        with self._lock:
            src.msg = msg
            src.last_stamp_ns = self.get_clock().now().nanoseconds

    def _on_dock(self, msg: Twist) -> None: self._stamp(self._dock, msg)
    def _on_retreat(self, msg: Twist) -> None: self._stamp(self._retreat, msg)
    def _on_nav(self, msg: Twist) -> None: self._stamp(self._nav, msg)

    def _on_estop(self, msg: Bool) -> None:
        with self._lock:
            self._estop = bool(msg.data)
        if msg.data:
            self.get_logger().warn('E-STOP latched -> zero cmd_vel')

    def _on_set_carry(self, request: SetCarryMode.Request, response: SetCarryMode.Response):
        with self._lock:
            self._carrying = bool(request.carrying)
        response.success = True
        response.message = f'carry_mode={self._carrying}'
        self.get_logger().info(response.message)
        return response

    def _on_set_freeze(self, request: SetFreeze.Request, response: SetFreeze.Response):
        with self._lock:
            self._freeze = bool(request.freeze)
        response.success = True
        response.message = f'freeze={self._freeze}'
        self.get_logger().info(response.message)
        return response

    # ---------- selection ----------

    def _is_fresh(self, src: _Source, now_ns: int) -> bool:
        return src.msg is not None and (now_ns - src.last_stamp_ns) <= self._timeout_ns

    def _select(self, now_ns: int) -> Optional[Twist]:
        if self._estop or self._freeze:
            return Twist()
        if self._is_fresh(self._dock, now_ns):
            return self._dock.msg
        if self._is_fresh(self._retreat, now_ns):
            return self._retreat.msg
        if self._is_fresh(self._nav, now_ns):
            return self._nav.msg
        return None

    def _apply_caps(self, cmd: Twist) -> Twist:
        vx_max, vy_max, vyaw_max = self._max_carry if self._carrying else self._max_normal
        vx_min, vy_min, vyaw_min = self._min_floor
        out = Twist()
        out.linear.x = _floor(_clamp(cmd.linear.x, -vx_max, vx_max), vx_min)
        out.linear.y = _floor(_clamp(cmd.linear.y, -vy_max, vy_max), vy_min)
        out.angular.z = _floor(_clamp(cmd.angular.z, -vyaw_max, vyaw_max), vyaw_min)
        return out

    # ---------- main loop ----------

    def _tick(self) -> None:
        if not bool(self.get_parameter('enable_publish').value):
            return  # service-only mode (Faza 1.5+ where nav2 owns /cmd_vel)
        now_ns = self.get_clock().now().nanoseconds
        with self._lock:
            chosen = self._select(now_ns)
        if chosen is None:
            self._pub.publish(Twist())
            return
        self._pub.publish(self._apply_caps(chosen))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelArbiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
