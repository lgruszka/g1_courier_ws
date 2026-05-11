from __future__ import annotations

import json
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool
from unitree_api.msg import Request, Response

class UnitreeCmdVelBridgeNode(Node):
    """
    Cel: Ta klasa realizuje odpowiedzialność `UnitreeCmdVelBridgeNode` w aktualnym module.
    Dlaczego tak: Wydzielenie tej jednostki upraszcza debugowanie i chroni krytyczne ścieżki przed niekontrolowanymi zmianami.
    """
    def __init__(self) -> None:
        """
        Cel: Ta metoda realizuje odpowiedzialność `__init__` w aktualnym module.
        Dlaczego tak: Wydzielenie tej jednostki upraszcza debugowanie i chroni krytyczne ścieżki przed niekontrolowanymi zmianami.
        """
        super().__init__('unitree_cmd_vel_bridge_node')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('unitree_request_topic', 'api/sport/request')
        self.declare_parameter('max_vx', 0.3)
        self.declare_parameter('max_vy', 0.3)
        self.declare_parameter('max_vyaw', 0.8)
        self.declare_parameter('cmd_timeout_s', 0.5)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('api_response_topic', '/api/sport/response')
        self.declare_parameter('switch_to_normal', False)
        self.declare_parameter('startup_delay_s', 1.5)
        self.declare_parameter('start_fsm_id', 500)
        self.declare_parameter('enable_balance_mode', True)
        self.declare_parameter('balance_mode', 1)
        self.declare_parameter('velocity_duration_s', 0.2)
        self.declare_parameter('log_cmd_vel_rx', True)
        self.declare_parameter('log_cmd_vel_tx', True)
        self.declare_parameter('log_subscribers', True)
        self.declare_parameter('log_interval_s', 1.0)
        # [AI-CHANGE | 2026-04-20 06:28 UTC | v0.135]
        # CO ZMIENIONO: Dodano konfigurację wejścia E-STOP i ograniczenie częstotliwości wysyłki hard-stop:
        #   - `estop_active_topic`,
        #   - `stop_command_min_interval_s`.
        # DLACZEGO: Samo zerowanie `cmd_vel` może nie wystarczyć przy potencjalnym kolejkowaniu zadań po stronie
        #   wykonawczej robota; potrzebny jest jawny sygnał stop i aktywne wysłanie komendy stop.
        # JAK TO DZIAŁA: Gdy na `estop_active_topic` pojawia się True, bridge wymusza lokalny stan STOP i
        #   wysyła `api_id_stop` z ograniczeniem częstotliwości, aby nie spamować magistrali.
        # TODO: Dodać potwierdzenie ACK stopu (code==0) jako warunek wyjścia z trybu awaryjnego.
        self.declare_parameter('estop_active_topic', '/emergency_stop/active')
        self.declare_parameter('stop_command_min_interval_s', 0.2)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.unitree_request_topic = self.get_parameter(
            'unitree_request_topic'
        ).get_parameter_value().string_value
        self.max_vx = float(self.get_parameter('max_vx').get_parameter_value().double_value)
        self.max_vy = float(self.get_parameter('max_vy').get_parameter_value().double_value)
        self.max_vyaw = float(self.get_parameter('max_vyaw').get_parameter_value().double_value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout_s').get_parameter_value().double_value)
        publish_rate = float(self.get_parameter('publish_rate_hz').get_parameter_value().double_value)
        self.api_response_topic = self.get_parameter(
            'api_response_topic'
        ).get_parameter_value().string_value
        self.switch_to_normal = self.get_parameter('switch_to_normal').get_parameter_value().bool_value
        self.startup_delay_s = float(self.get_parameter('startup_delay_s').get_parameter_value().double_value)
        self.start_fsm_id = int(self.get_parameter('start_fsm_id').get_parameter_value().integer_value)
        self.enable_balance_mode = self.get_parameter('enable_balance_mode').get_parameter_value().bool_value
        self.balance_mode = int(self.get_parameter('balance_mode').get_parameter_value().integer_value)
        self.velocity_duration_s = float(
            self.get_parameter('velocity_duration_s').get_parameter_value().double_value
        )
        self.log_cmd_vel_rx = self.get_parameter('log_cmd_vel_rx').get_parameter_value().bool_value
        self.log_cmd_vel_tx = self.get_parameter('log_cmd_vel_tx').get_parameter_value().bool_value
        self.log_subscribers = self.get_parameter('log_subscribers').get_parameter_value().bool_value
        self.log_interval_s = float(self.get_parameter('log_interval_s').get_parameter_value().double_value)
        self.estop_active_topic = self.get_parameter(
            'estop_active_topic'
        ).get_parameter_value().string_value
        self.stop_command_min_interval_s = float(
            self.get_parameter('stop_command_min_interval_s').get_parameter_value().double_value
        )

        # G1 loco API on /api/sport/request
        self.api_id_move = 7105
        self.api_id_set_fsm = 7101
        self.api_id_set_balance_mode = 7102
        self.api_id_motion_release = 1003
        self.api_id_stop = 1003
        self.api_id_enable_obstacle_avoidance = 2048

        self.last_twist = Twist()
        self.last_cmd_time = self.get_clock().now()
        self.request_id = 0
        self.is_moving = False
        # [AI-CHANGE | 2026-04-29 13:35 UTC | v0.333]
        # CO ZMIENIONO: Dodano jawny typ mapy `sent_ids`.
        # DLACZEGO: Pełna analiza `mypy` wymaga typu pustego słownika; bez niego nie wiadomo, jakie identyfikatory
        #           odpowiedzi Unitree są śledzone i łatwo przeoczyć błędne porównanie typów.
        # JAK TO DZIAŁA: Kluczem jest request id wysłany do API, a wartością nazwa/etykieta komendy używana w logice odpowiedzi.
        # TODO: Zastąpić wartość tekstową strukturą z czasem wysłania, aby wykrywać przeterminowane odpowiedzi API.
        self.sent_ids: dict[int, str] = {}
        self._last_rx_log_time = None
        self._last_tx_log_time = None
        self._last_subscribers_log_time = None
        self._estop_active = False
        self._last_stop_send_monotonic = 0.0

        self.cmd_sub = self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_vel_callback, 10)
        self.estop_sub = self.create_subscription(
            Bool, self.estop_active_topic, self.estop_callback, 10
        )
        self.unitree_pub = self.create_publisher(Request, self.unitree_request_topic, 10)
        self.response_sub = self.create_subscription(
            Response, self.api_response_topic, self._on_response, 10
        )
        self.timer = self.create_timer(1.0 / max(publish_rate, 1.0), self.send_move)

        self.get_logger().info(
            f'Bridge online: {self.cmd_vel_topic} -> {self.unitree_request_topic}, rate={publish_rate}Hz'
        )

        self._send_startup_sequence()

    def _publish_api(self, api_id: int, payload: dict | None, tag: str) -> int:
        """
        Cel: Ta metoda realizuje odpowiedzialność `_publish_api` w aktualnym module.
        Dlaczego tak: Wydzielenie tej jednostki upraszcza debugowanie i chroni krytyczne ścieżki przed niekontrolowanymi zmianami.
        """
        req = Request()
        req_id = self.get_next_id()
        req.header.identity.id = req_id
        req.header.identity.api_id = api_id
        req.parameter = json.dumps(payload) if payload is not None else ''
        self.unitree_pub.publish(req)
        self.sent_ids[req_id] = tag
        return req_id

    def _send_startup_sequence(self) -> None:
        """
        Cel: Ta metoda realizuje odpowiedzialność `_send_startup_sequence` w aktualnym module.
        Dlaczego tak: Wydzielenie tej jednostki upraszcza debugowanie i chroni krytyczne ścieżki przed niekontrolowanymi zmianami.
        """
        if self.switch_to_normal:
            req_id = self._publish_api(self.api_id_motion_release, {}, 'motion_release')
            self.get_logger().info(
                f'ReleaseMode sent (api_id={self.api_id_motion_release}, req_id={req_id}), waiting {self.startup_delay_s:.1f}s'
            )
            time.sleep(max(0.0, self.startup_delay_s))

        req_id = self._publish_api(self.api_id_set_fsm, {'data': self.start_fsm_id}, 'set_fsm')
        self.get_logger().info(
            f'SetFsmId sent (api_id={self.api_id_set_fsm}, req_id={req_id}, fsm_id={self.start_fsm_id})'
        )

        if self.enable_balance_mode:
            req_id = self._publish_api(
                self.api_id_set_balance_mode, {'data': self.balance_mode}, 'set_balance_mode'
            )
            self.get_logger().info(
                'SetBalanceMode sent '
                f'(api_id={self.api_id_set_balance_mode}, req_id={req_id}, mode={self.balance_mode})'
            )

    def get_next_id(self) -> int:
        """
        Cel: Ta metoda realizuje odpowiedzialność `get_next_id` w aktualnym module.
        Dlaczego tak: Wydzielenie tej jednostki upraszcza debugowanie i chroni krytyczne ścieżki przed niekontrolowanymi zmianami.
        """
        self.request_id += 1
        return self.request_id

    @staticmethod
    def clamp(value: float, min_val: float, max_val: float) -> float:
        """
        Cel: Ta metoda realizuje odpowiedzialność `clamp` w aktualnym module.
        Dlaczego tak: Wydzielenie tej jednostki upraszcza debugowanie i chroni krytyczne ścieżki przed niekontrolowanymi zmianami.
        """
        return max(min_val, min(max_val, value))

    def cmd_vel_callback(self, msg: Twist) -> None:
        """
        Cel: Ta metoda realizuje odpowiedzialność `cmd_vel_callback` w aktualnym module.
        Dlaczego tak: Wydzielenie tej jednostki upraszcza debugowanie i chroni krytyczne ścieżki przed niekontrolowanymi zmianami.
        """
        self.last_twist = msg
        self.last_cmd_time = self.get_clock().now()
        if self.log_cmd_vel_rx:
            now = self.get_clock().now()
            if self._last_rx_log_time is None:
                self._last_rx_log_time = now
                self.get_logger().info(
                    f'cmd_vel rx: vx={msg.linear.x:.3f}, vy={msg.linear.y:.3f}, wz={msg.angular.z:.3f}'
                )
            else:
                elapsed = (now - self._last_rx_log_time).nanoseconds / 1e9
                if elapsed >= self.log_interval_s:
                    self._last_rx_log_time = now
                    self.get_logger().info(
                        f'cmd_vel rx: vx={msg.linear.x:.3f}, vy={msg.linear.y:.3f}, wz={msg.angular.z:.3f}'
                    )

    def send_move(self) -> None:
        """
        Cel: Ta metoda realizuje odpowiedzialność `send_move` w aktualnym module.
        Dlaczego tak: Wydzielenie tej jednostki upraszcza debugowanie i chroni krytyczne ścieżki przed niekontrolowanymi zmianami.
        """
        self._maybe_log_topic_subscribers()
        if self._estop_active:
            self.last_twist = Twist()
            self.last_cmd_time = self.get_clock().now()
            self.send_stop()
            return

        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9

        if elapsed > self.cmd_timeout:
            if self.is_moving:
                self.get_logger().info('cmd_vel timeout -> stop')
                self.is_moving = False
            vx, vy, vyaw = 0.0, 0.0, 0.0
        else:
            vx = self.clamp(self.last_twist.linear.x, -self.max_vx, self.max_vx)
            vy = self.clamp(self.last_twist.linear.y, -self.max_vy, self.max_vy)
            vyaw = self.clamp(self.last_twist.angular.z, -self.max_vyaw, self.max_vyaw)

            if not self.is_moving and (vx != 0.0 or vy != 0.0 or vyaw != 0.0):
                self.get_logger().info('Robot motion command active')
                self.is_moving = True

        duration = max(0.1, self.velocity_duration_s)
        params = {'velocity': [float(vx), float(vy), float(vyaw)], 'duration': duration}
        req_id = self._publish_api(self.api_id_move, params, 'set_velocity')
        self._maybe_log_tx(req_id, elapsed, vx, vy, vyaw, duration)

    def _on_response(self, msg: Response) -> None:
        """
        Cel: Ta metoda realizuje odpowiedzialność `_on_response` w aktualnym module.
        Dlaczego tak: Wydzielenie tej jednostki upraszcza debugowanie i chroni krytyczne ścieżki przed niekontrolowanymi zmianami.
        """
        req_id = int(msg.header.identity.id)
        tag = self.sent_ids.pop(req_id, None)
        if tag is None:
            return

        code = int(msg.header.status.code)
        if code != 0:
            self.get_logger().warn(
                'Unitree API error: '
                f'tag={tag}, req_id={req_id}, api_id={msg.header.identity.api_id}, '
                f'code={code}, data={msg.data}'
            )

    def _maybe_log_topic_subscribers(self) -> None:
        """
        Cel: Ta metoda realizuje odpowiedzialność `_maybe_log_topic_subscribers` w aktualnym module.
        Dlaczego tak: Wydzielenie tej jednostki upraszcza debugowanie i chroni krytyczne ścieżki przed niekontrolowanymi zmianami.
        """
        if not self.log_subscribers:
            return

        now = self.get_clock().now()
        if self._last_subscribers_log_time is not None:
            elapsed = (now - self._last_subscribers_log_time).nanoseconds / 1e9
            if elapsed < self.log_interval_s:
                return

        self._last_subscribers_log_time = now
        count = self.unitree_pub.get_subscription_count()
        level = self.get_logger().warn if count == 0 else self.get_logger().info
        level(f'{self.unitree_request_topic} subscribers={count}')

    def _maybe_log_tx(
        self, req_id: int, cmd_age_s: float, vx: float, vy: float, vyaw: float, duration: float
    ) -> None:
        """
        Cel: Ta metoda realizuje odpowiedzialność `_maybe_log_tx` w aktualnym module.
        Dlaczego tak: Wydzielenie tej jednostki upraszcza debugowanie i chroni krytyczne ścieżki przed niekontrolowanymi zmianami.
        """
        if not self.log_cmd_vel_tx:
            return

        now = self.get_clock().now()
        if self._last_tx_log_time is not None:
            elapsed = (now - self._last_tx_log_time).nanoseconds / 1e9
            if elapsed < self.log_interval_s:
                return

        self._last_tx_log_time = now
        self.get_logger().info(
            'unitree tx: '
            f'req_id={req_id}, api_id={self.api_id_move}, age={cmd_age_s:.3f}s, '
            f'vel=[{vx:.3f},{vy:.3f},{vyaw:.3f}], duration={duration:.3f}s'
        )

    def send_stop(self) -> None:
        """
        Cel: Ta metoda realizuje odpowiedzialność `send_stop` w aktualnym module.
        Dlaczego tak: Wydzielenie tej jednostki upraszcza debugowanie i chroni krytyczne ścieżki przed niekontrolowanymi zmianami.
        """
        now_monotonic = time.monotonic()
        if now_monotonic - self._last_stop_send_monotonic < max(self.stop_command_min_interval_s, 0.01):
            return
        self._last_stop_send_monotonic = now_monotonic
        req = Request()
        req.header.identity.id = self.get_next_id()
        req.header.identity.api_id = self.api_id_stop
        req.parameter = ''
        self.unitree_pub.publish(req)
        self.get_logger().info(f'Stop sent (api_id={self.api_id_stop})')

    def estop_callback(self, msg: Bool) -> None:
        """
        Cel: Ta metoda realizuje odpowiedzialność `estop_callback` w aktualnym module.
        Dlaczego tak: Wydzielenie tej jednostki upraszcza debugowanie i chroni krytyczne ścieżki przed niekontrolowanymi zmianami.
        """
        self._estop_active = bool(msg.data)
        if self._estop_active:
            self.last_twist = Twist()
            self.last_cmd_time = self.get_clock().now()
            self.send_stop()


def main(args=None) -> None:
    """
    Cel: Ta funkcja realizuje odpowiedzialność `main` w aktualnym module.
    Dlaczego tak: Wydzielenie tej jednostki upraszcza debugowanie i chroni krytyczne ścieżki przed niekontrolowanymi zmianami.
    """
    rclpy.init(args=args)
    node = UnitreeCmdVelBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.send_stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
