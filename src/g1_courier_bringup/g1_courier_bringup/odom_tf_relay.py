from __future__ import annotations

import math
import time
from collections import deque

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from tf2_ros import TransformBroadcaster


class OdomTfRelay(Node):
    """Relay Odometry pose into a dynamic TF transform."""

    def __init__(self) -> None:
        super().__init__('odom_tf_relay')

        self.declare_parameter('odom_topic', '/dog_odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('use_msg_frame_ids', False)
        self.declare_parameter('use_msg_stamp', True)
        # Gorny limit publikacji TF [Hz]; 0 = bez limitu (1:1 z odom).
        # /dog_odom na realnym G1 to ~950 Hz — TF w tym tempie kosztuje CPU
        # (zmierzone ~82%) i zatyka konsumentow (np. websocket do Foxglove
        # dropuje wtedy rzadsze TF stawow); nav wystarcza <=50 Hz.
        self.declare_parameter('max_rate', 0.0)
        # Rzut 2D dla base_footprint (REP-120): zeruje z oraz roll/pitch
        # (zostaje x, y, yaw). Firmware G1 daje w odom poze PELVIS (z~0.75,
        # z przechylami tulowia) — do nawigacji 2D ramka bazowa ma lezec
        # plasko na podlodze, inaczej AMCL klei plaszczyzne mapy do bioder.
        self.declare_parameter('flatten', False)
        # Korekta skali translacji odometrii (PER-ROBOT). Firmware biepa G1
        # potrafi grubo niedoszacowywac dystans do przodu (zmierzone
        # narzedziem verdict: odom 1.14 m przy realnych 2.15 m => 0.53,
        # trans_scale=1/0.53~1.89). AMCL bierze odometrie z tego TF, wiec
        # skalujemy delty tu — u zrodla — zamiast maskowac szumem (alpha).
        # Skala wzgledem pierwszej pozycji => absolutny offset bez zmian.
        self.declare_parameter('trans_scale', 1.0)
        # ZUPT (zero-velocity update) — kompensacja PELZANIA odometrii na
        # postoju. Firmware bipeda calkuje kinematyke nog takze gdy robot
        # stoi: zmierzone na InsideBot ~2.6 mm/s stalego biasu do przodu
        # (15.6 cm/min surowo, x trans_scale => ~29 cm/min w TF), przy czym
        # samo kolysanie kostek ma tylko ~1 cm peak-to-peak. AMCL byl wleczony
        # za tym fikcyjnym ruchem i pozycja na mapie systematycznie uciekala.
        #
        # Detektor patrzy na ROZPIETOSC okna czasowego, nie na predkosc
        # chwilowa: bias siedzi WEWNATRZ kolysania (±1 cm daje chwilowo
        # ~0.05 m/s, wiec prog predkosci przepuscilby go razem z sway).
        # Rozpietosc yaw jest w warunku, by obrotu w miejscu (RotationShim
        # na starcie kazdej sciezki) nie uznac za bezruch.
        # still_dist = 0 wylacza mechanizm (domyslnie OFF).
        self.declare_parameter('still_dist', 0.0)     # [m] prog rozpietosci xy
        self.declare_parameter('still_angle', 0.09)   # [rad] prog rozpietosci yaw
        self.declare_parameter('still_window', 1.0)   # [s] dlugosc okna

        self._odom_frame = str(self.get_parameter('odom_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._use_msg_frame_ids = bool(self.get_parameter('use_msg_frame_ids').value)
        self._use_msg_stamp = bool(self.get_parameter('use_msg_stamp').value)
        max_rate = float(self.get_parameter('max_rate').value)
        self._min_period = (1.0 / max_rate) if max_rate > 0.0 else 0.0
        self._last_pub = 0.0
        self._flatten = bool(self.get_parameter('flatten').value)
        self._trans_scale = float(self.get_parameter('trans_scale').value)
        self._origin = None          # pierwsza pozycja odom (x,y) — baza skali
        self._still_dist = float(self.get_parameter('still_dist').value)
        self._still_angle = float(self.get_parameter('still_angle').value)
        self._still_window = float(self.get_parameter('still_window').value)
        self._win: deque = deque()   # (t, x, y, yaw) surowe, okno detektora
        self._prev_raw = None        # poprzednia surowa pozycja (x,y)
        self._bias = (0.0, 0.0)      # pochloniete pelzanie z postojow
        self._was_still = False
        odom_topic = str(self.get_parameter('odom_topic').value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        self._tf_broadcaster = TransformBroadcaster(self)
        # Subskrypcja SUROWA (raw=True): callback dostaje bajty, a nie
        # zdeserializowana wiadomosc. Przy max_rate dlawik odrzuca ~95%
        # ramek, wiec deserializacja ich wszystkich to czysta strata —
        # /dog_odom na G1 ma ~1000 Hz i zmierzone 71% RDZENIA szlo wlasnie
        # w rozpakowywanie ramek wyrzucanych milisekunde pozniej.
        # Dlawik dziala na time.monotonic(), nie na stemplu wiadomosci,
        # wiec da sie go sprawdzic PRZED deserializacja.
        self._sub = self.create_subscription(
            Odometry, odom_topic, self._on_odom_raw, qos, raw=True)

        self.get_logger().info(
            f"Relaying '{odom_topic}' into TF using fallback frames "
            f"'{self._odom_frame}' -> '{self._base_frame}'"
        )

    def _is_still(self, now: float) -> bool:
        """Bezruch = mala rozpietosc xy ORAZ yaw w calym oknie czasowym."""
        while self._win and now - self._win[0][0] > self._still_window:
            self._win.popleft()
        if len(self._win) < 5 or now - self._win[0][0] < 0.5 * self._still_window:
            return False           # okno jeszcze niepelne — nie zgaduj
        xs = [w[1] for w in self._win]
        ys = [w[2] for w in self._win]
        span_xy = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
        if span_xy >= self._still_dist:
            return False
        # rozpietosc kata liczona wzgledem pierwszej probki (odporne na zawijanie ±pi)
        a0 = self._win[0][3]
        rel = [math.atan2(math.sin(w[3] - a0), math.cos(w[3] - a0)) for w in self._win]
        return (max(rel) - min(rel)) < self._still_angle

    def _compensate_creep(self, px: float, py: float, yaw: float):
        """Na postoju pochlania delty w biasie => wyjscie zamarza, a przy
        wznowieniu ruchu nie ma skoku (odrzucone pelzanie nie wraca)."""
        now = time.monotonic()
        self._win.append((now, px, py, yaw))
        still = self._is_still(now)
        if still and self._prev_raw is not None:
            self._bias = (self._bias[0] + (px - self._prev_raw[0]),
                          self._bias[1] + (py - self._prev_raw[1]))
        if still != self._was_still:
            self._was_still = still
            self.get_logger().info(
                'ZUPT: %s (pochloniete pelzanie: %.1f cm)'
                % ('BEZRUCH — mrozenie translacji' if still else 'RUCH — calkowanie',
                   math.hypot(*self._bias) * 100.0))
        self._prev_raw = (px, py)
        return px - self._bias[0], py - self._bias[1]

    def _on_odom_raw(self, data: bytes) -> None:
        """Dlawik PRZED deserializacja — tu jest cala oszczednosc CPU."""
        if self._min_period > 0.0:
            now = time.monotonic()
            if now - self._last_pub < self._min_period:
                return
            self._last_pub = now
        self._on_odom(deserialize_message(data, Odometry))

    def _on_odom(self, msg: Odometry) -> None:
        parent = self._odom_frame
        child = self._base_frame

        if self._use_msg_frame_ids:
            msg_parent = msg.header.frame_id.strip()
            msg_child = msg.child_frame_id.strip()
            if msg_parent:
                parent = msg_parent
            if msg_child:
                child = msg_child

        if parent == child:
            self.get_logger().warn(
                f"Skipping TF publish because parent and child frame are identical ('{parent}')."
            )
            return

        t = TransformStamped()
        t.header.frame_id = parent
        t.child_frame_id = child

        if self._use_msg_stamp and (msg.header.stamp.sec != 0 or msg.header.stamp.nanosec != 0):
            t.header.stamp = msg.header.stamp
        else:
            t.header.stamp = self.get_clock().now().to_msg()

        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        if self._still_dist > 0.0:
            px, py = self._compensate_creep(px, py, yaw)

        # Skala translacji wzgledem pierwszej pozycji: delty *= trans_scale,
        # absolutny offset zachowany (AMCL i tak wchlania stala przez map->odom).
        if self._origin is None:
            self._origin = (px, py)
        sx = self._origin[0] + self._trans_scale * (px - self._origin[0])
        sy = self._origin[1] + self._trans_scale * (py - self._origin[1])

        t.transform.translation.x = sx
        t.transform.translation.y = sy
        if self._flatten:
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = math.sin(yaw / 2.0)
            t.transform.rotation.w = math.cos(yaw / 2.0)
        else:
            t.transform.translation.z = msg.pose.pose.position.z
            t.transform.rotation = msg.pose.pose.orientation

        self._tf_broadcaster.sendTransform(t)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = OdomTfRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
