"""Operator GUI dla G1 courier — wywoływanie zadań w trybie enable_mission:=false.

Jeden plik, PyQt6 + rclpy. Po `ros2 launch g1_courier_bringup real.launch.py
enable_mission:=false` odpalasz `ros2 run g1_courier_bringup operator_gui` w
drugim terminalu.

Layout:
  ┌─ STATUS ─────────┬─ AKCJE ──────────┬─ LOG ────────┐
  │ AMCL pose        │ Navigate         │ event stream  │
  │ topic Hz         │ Dock             │ feedback      │
  │ lifecycle        │ Pick / Place     │ results       │
  │ Active goal      │ Retreat          │              │
  │ Set Initial Pose │ E-Stop           │              │
  └──────────────────┴──────────────────┴──────────────┘

Wymagania: pip install PyQt5 (opcjonalnie PyQt6)
"""
from __future__ import annotations

import json
import math
import os
import sys
import threading
import time
from typing import Optional

# Plik trwałego zapisu kotwic cyklu (przeżywa restart GUI).
CYCLE_WP_PATH = os.path.expanduser('~/maps/cycle_waypoints.json')

import yaml

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

try:
    from PyQt5.QtCore import QObject, pyqtSignal
    from PyQt5.QtGui import QFont
    from PyQt5.QtWidgets import (
        QApplication, QButtonGroup, QComboBox, QDoubleSpinBox, QFormLayout,
        QGridLayout, QGroupBox, QHBoxLayout, QLabel, QMainWindow,
        QPlainTextEdit, QPushButton, QRadioButton, QSpinBox, QVBoxLayout, QWidget,
    )
    HAVE_PYQT6 = False
except ImportError:
    from PyQt6.QtCore import QObject, pyqtSignal
    from PyQt6.QtGui import QFont
    from PyQt6.QtWidgets import (
        QApplication, QButtonGroup, QComboBox, QDoubleSpinBox, QFormLayout,
        QGridLayout, QGroupBox, QHBoxLayout, QLabel, QMainWindow,
        QPlainTextEdit, QPushButton, QRadioButton, QSpinBox, QVBoxLayout, QWidget,
    )
    HAVE_PYQT6 = True

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist, Vector3Stamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from g1_courier_msgs.action import (
    DockToTable, NavigateToPose as CourierNavigate, PickBox, PlaceBox, Retreat,
)
from g1_courier_msgs.srv import SetFreeze

try:
    from unitree_hg.msg import LowState
    HAVE_LOWSTATE = True
except ImportError:
    HAVE_LOWSTATE = False


# ----- ROS node (w osobnym wątku) -----


class RosBridge(QObject):
    """ROS2 client zdarzeń. Sygnały Qt do bezpiecznego update GUI thread."""

    amcl_pose_changed = pyqtSignal(float, float, float)            # x, y, yaw_deg
    hz_changed = pyqtSignal(str, float)                            # topic, hz
    log_message = pyqtSignal(str, str)                             # level, text
    active_goal_changed = pyqtSignal(str, str)                     # action, info
    goal_finished = pyqtSignal(bool, str)                          # success, message
    dock_errors_changed = pyqtSignal(float, float, float)          # dx_m, dy_m, dyaw_rad
    cycle_phase_changed = pyqtSignal(str)                          # opis bieżącej fazy cyklu
    cycle_finished = pyqtSignal(bool, str)                         # cykl zatrzymany (ok, powód)

    def __init__(self) -> None:
        super().__init__()
        self.node = rclpy.create_node('operator_gui')
        self._lock = threading.Lock()
        self._active_goal_handle = None
        self._active_action: Optional[str] = None
        # Stan cyklu misji.
        self._last_amcl: Optional[tuple] = None     # (x, y, yaw_rad) najświeższa poza AMCL
        self._cycle_thread: Optional[threading.Thread] = None
        self._cycle_stop = threading.Event()

        # Action clients.
        self._nav = ActionClient(self.node, CourierNavigate, '/courier/navigate_to_pose')
        self._dock = ActionClient(self.node, DockToTable, '/dock_to_table')
        self._pick = ActionClient(self.node, PickBox, '/pick_box')
        self._place = ActionClient(self.node, PlaceBox, '/place_box')
        self._retreat = ActionClient(self.node, Retreat, '/retreat')

        # Publisher dla Initial Pose.
        self._initialpose_pub = self.node.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10,
        )
        # Publisher dla awaryjnego zero cmd_vel (E-stop fallback).
        self._cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        # Service client dla freeze.
        self._freeze_client = self.node.create_client(SetFreeze, '/safety/set_freeze')

        # Subscribers — status.
        self.node.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl, 10,
        )
        # Live diagnostic dx/dy/dyaw z dock_action_server (publikuje co tick
        # podczas servoing/settle, milknie gdy dock idle).
        self.node.create_subscription(
            Vector3Stamped, '/dock/errors', self._on_dock_errors, 10,
        )
        self._scan_count = 0
        self._scan_t0 = time.monotonic()
        self.node.create_subscription(
            LaserScan, '/scan', self._on_scan, qos_profile_sensor_data,
        )

        if HAVE_LOWSTATE:
            self._lowstate_count = 0
            self._lowstate_t0 = time.monotonic()
            self.node.create_subscription(LowState, '/lowstate', self._on_lowstate, 10)

        # Timer Hz reporting (co 2 s).
        self._hz_timer = self.node.create_timer(2.0, self._report_hz)

    # ---- subscribers ----

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose
        q = p.orientation
        # quaternion → yaw
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self._lock:
            self._last_amcl = (p.position.x, p.position.y, yaw)
        self.amcl_pose_changed.emit(p.position.x, p.position.y, math.degrees(yaw))

    def current_pose(self) -> Optional[tuple]:
        """Najświeższa poza AMCL (x, y, yaw_rad) albo None — do teachowania kotwic."""
        with self._lock:
            return self._last_amcl

    def _on_dock_errors(self, msg: Vector3Stamped) -> None:
        self.dock_errors_changed.emit(msg.vector.x, msg.vector.y, msg.vector.z)

    def _on_scan(self, _msg: LaserScan) -> None:
        self._scan_count += 1

    def _on_lowstate(self, _msg) -> None:
        self._lowstate_count += 1

    def _report_hz(self) -> None:
        now = time.monotonic()
        dt = now - self._scan_t0
        if dt > 0:
            self.hz_changed.emit('scan', self._scan_count / dt)
        self._scan_count = 0
        self._scan_t0 = now

        if HAVE_LOWSTATE:
            dt2 = now - self._lowstate_t0
            if dt2 > 0:
                self.hz_changed.emit('lowstate', self._lowstate_count / dt2)
            self._lowstate_count = 0
            self._lowstate_t0 = now

    # ---- generic goal handler ----

    def _send_action(self, client: ActionClient, goal_msg, action_name: str,
                     info: str, feedback_fmt) -> None:
        """Wspólna ścieżka wysyłania goal'a. Single-flight."""
        with self._lock:
            if self._active_goal_handle is not None:
                self.log_message.emit('warn', f'BUSY — odrzucam {action_name}')
                return
            self._active_action = action_name

        self.log_message.emit('info', f'→ {action_name}: {info}')
        self.active_goal_changed.emit(action_name, info)

        if not client.wait_for_server(timeout_sec=2.0):
            self.log_message.emit('error', f'{action_name}: server niedostępny')
            with self._lock:
                self._active_goal_handle = None
                self._active_action = None
            self.active_goal_changed.emit('', '')
            return

        def feedback_cb(fb):
            try:
                txt = feedback_fmt(fb.feedback)
                if txt:
                    self.log_message.emit('fb', f'{action_name}: {txt}')
            except Exception:
                pass

        send_future = client.send_goal_async(goal_msg, feedback_callback=feedback_cb)
        send_future.add_done_callback(
            lambda f: self._on_goal_response(f, action_name)
        )

    def _on_goal_response(self, future, action_name: str) -> None:
        handle = future.result()
        if handle is None or not handle.accepted:
            self.log_message.emit('error', f'{action_name}: REJECTED')
            with self._lock:
                self._active_goal_handle = None
                self._active_action = None
            self.active_goal_changed.emit('', '')
            return

        with self._lock:
            self._active_goal_handle = handle

        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self._on_goal_result(f, action_name)
        )

    def _on_goal_result(self, future, action_name: str) -> None:
        try:
            wrapper = future.result()
            status = wrapper.status
            result = wrapper.result
            ok = bool(getattr(result, 'success', False))
            msg = getattr(result, 'message', '') or f'status={status}'
        except Exception as exc:
            ok = False
            msg = f'exception: {exc}'

        self.log_message.emit('ok' if ok else 'error',
                              f'{action_name}: {"OK" if ok else "FAIL"} — {msg}')
        self.goal_finished.emit(ok, msg)
        with self._lock:
            self._active_goal_handle = None
            self._active_action = None
        self.active_goal_changed.emit('', '')

    # ---- public goal methods ----

    def send_nav(self, x: float, y: float, yaw_rad: float, waypoint_name: str = '') -> None:
        goal = CourierNavigate.Goal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        goal.target_pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
        goal.waypoint_name = waypoint_name or f'({x:.2f},{y:.2f})'

        def fb(f):
            return f'd={f.distance_remaining_m:.2f}m phase={f.phase}'

        self._send_action(
            self._nav, goal, 'navigate',
            f'x={x:.2f} y={y:.2f} yaw={math.degrees(yaw_rad):.0f}° ({waypoint_name or "ad-hoc"})',
            fb,
        )

    def send_dock(self, mode: int, tag_id: int, target_xyz_yaw, xy_tol: float,
                  yaw_tol: float) -> None:
        goal = DockToTable.Goal()
        goal.mode = mode
        goal.apriltag_id = tag_id
        x, y, z, yaw = target_xyz_yaw
        if mode == DockToTable.Goal.MODE_APRILTAG:
            goal.target_pose.header.frame_id = f'tag_{tag_id}'
        else:
            goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = z
        goal.target_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.target_pose.pose.orientation.w = math.cos(yaw / 2.0)
        goal.xy_tolerance_m = xy_tol
        goal.yaw_tolerance_rad = yaw_tol
        goal.timeout_s = 30.0

        def fb(f):
            return f'phase={f.phase} xy_err={f.xy_error_m:.3f} yaw_err={f.yaw_error_rad:.3f}'

        mode_str = {0: 'APRILTAG', 1: 'LIDAR_LINE', 2: 'AMCL_ONLY'}[mode]
        info = (f'mode={mode_str}' + (f' tag={tag_id}' if mode == 0 else '') +
                f' xy_tol={xy_tol:.2f}')
        self._send_action(self._dock, goal, 'dock', info, fb)

    def send_pick(self) -> None:
        goal = PickBox.Goal()
        goal.sequence_name = 'pick_box'
        goal.timeout_s = 60.0
        self._send_action(self._pick, goal, 'pick', 'sequence=pick_box',
                          lambda f: f'phase={f.phase} progress={f.progress:.2f}')

    def send_place(self) -> None:
        goal = PlaceBox.Goal()
        goal.timeout_s = 60.0
        self._send_action(self._place, goal, 'place', 'default',
                          lambda f: f'phase={f.phase} progress={f.progress:.2f}')

    def send_retreat(self, dist: float, speed: float) -> None:
        goal = Retreat.Goal()
        goal.distance_m = dist
        goal.speed_mps = speed
        goal.timeout_s = 15.0
        self._send_action(
            self._retreat, goal, 'retreat',
            f'd={dist:.2f}m v={speed:.2f}m/s',
            lambda f: f'traveled={f.distance_traveled_m:.2f}m',
        )

    # ---- cykl misji (synchroniczna orkiestracja w wątku roboczym) ----

    def _exec_sync(self, client, goal, name: str, feedback_fmt=None) -> tuple:
        """Wyślij goal i ZABLOKUJ aż do wyniku. Zwraca (ok, msg). Executor kręci
        w osobnym wątku ROS — tu tylko czekamy na Eventy z callbacków."""
        if self._cycle_stop.is_set():
            return False, 'stop'
        self.cycle_phase_changed.emit(name)
        self.log_message.emit('info', f'[cykl] → {name}')
        if not client.wait_for_server(timeout_sec=3.0):
            return False, f'{name}: server niedostępny'

        def fb_cb(fb):
            if feedback_fmt:
                try:
                    txt = feedback_fmt(fb.feedback)
                    if txt:
                        self.log_message.emit('fb', f'{name}: {txt}')
                except Exception:
                    pass

        send_fut = client.send_goal_async(goal, feedback_callback=fb_cb)
        ev = threading.Event(); box = {}
        send_fut.add_done_callback(lambda f: (box.__setitem__('h', f.result()), ev.set()))
        if not ev.wait(timeout=10.0):
            return False, f'{name}: brak odpowiedzi serwera'
        handle = box.get('h')
        if handle is None or not handle.accepted:
            return False, f'{name}: REJECTED'

        with self._lock:
            self._active_goal_handle = handle
            self._active_action = name
        self.active_goal_changed.emit(name, '(cykl)')

        res_fut = handle.get_result_async()
        ev2 = threading.Event(); box2 = {}
        res_fut.add_done_callback(lambda f: (box2.__setitem__('w', f.result()), ev2.set()))
        ev2.wait()   # blokuje aż do wyniku LUB cancel (stop_cycle → cancel_active)

        with self._lock:
            self._active_goal_handle = None
            self._active_action = None
        self.active_goal_changed.emit('', '')

        wrap = box2.get('w')
        if wrap is None:
            return False, f'{name}: brak wyniku'
        ok = bool(getattr(wrap.result, 'success', False))
        msg = getattr(wrap.result, 'message', '') or f'status={wrap.status}'
        self.log_message.emit('ok' if ok else 'error',
                              f'[cykl] {name}: {"OK" if ok else "FAIL"} — {msg}')
        return ok, msg

    @staticmethod
    def _predock(anchor: tuple, standoff: float) -> tuple:
        """Cofnij kotwicę (poza przy biurku) o `standoff` wzdłuż osi patrzenia
        → punkt PRZED biurkiem. yaw bez zmian (dalej twarzą do biurka)."""
        x, y, yaw = anchor
        return (x - standoff * math.cos(yaw), y - standoff * math.sin(yaw), yaw)

    def _g_nav(self, x, y, yaw, name):
        g = CourierNavigate.Goal()
        g.target_pose.header.frame_id = 'map'
        g.target_pose.pose.position.x = x
        g.target_pose.pose.position.y = y
        g.target_pose.pose.orientation.z = math.sin(yaw / 2.0)
        g.target_pose.pose.orientation.w = math.cos(yaw / 2.0)
        g.waypoint_name = name
        g.timeout_s = 120.0
        return g

    def _g_dock(self, mode, p):
        g = DockToTable.Goal()
        g.mode = mode
        g.apriltag_id = int(p['tag_id'])
        if mode == DockToTable.Goal.MODE_APRILTAG:
            g.target_pose.header.frame_id = f"tag_{int(p['tag_id'])}"
        else:
            g.target_pose.header.frame_id = 'map'
        g.target_pose.pose.position.z = float(p['dock_z'])
        g.target_pose.pose.orientation.w = 1.0
        g.xy_tolerance_m = float(p['xy_tol'])
        g.yaw_tolerance_rad = float(p['yaw_tol'])
        g.timeout_s = 40.0
        return g

    def start_cycle(self, params: dict) -> None:
        if self._cycle_thread is not None and self._cycle_thread.is_alive():
            self.log_message.emit('warn', 'cykl już aktywny')
            return
        self._cycle_stop.clear()
        self._cycle_thread = threading.Thread(
            target=self._cycle_loop, args=(params,), daemon=True)
        self._cycle_thread.start()

    def stop_cycle(self) -> None:
        if self._cycle_thread is None or not self._cycle_thread.is_alive():
            return
        self.log_message.emit('warn', '[cykl] STOP — przerywam po bieżącym kroku')
        self._cycle_stop.set()
        self.cancel_active()   # odblokuj _exec_sync czekający na wynik

    def _cycle_loop(self, p: dict) -> None:
        anchors = {'A': p['anchor_a'], 'B': p['anchor_b']}
        legs = [('A', 'B'), ('B', 'A')]   # (pobierz, odłóż)
        ok_all, reason = True, 'zatrzymano przez operatora'
        try:
            while not self._cycle_stop.is_set():
                aborted = False
                for pick_t, place_t in legs:
                    if self._cycle_stop.is_set():
                        break
                    if not self._run_leg(p, anchors, pick_t, place_t):
                        ok_all, reason = False, 'krok nieudany — cykl przerwany'
                        aborted = True
                        break
                if aborted:
                    break
        finally:
            self.cycle_finished.emit(ok_all, reason)

    def _run_leg(self, p, anchors, pick_t, place_t) -> bool:
        st = self._cycle_stop
        # 1. nav PRZED biurko pick (kotwica - pick_standoff)
        x, y, yaw = self._predock(anchors[pick_t], p['pick_standoff'])
        ok, _ = self._exec_sync(self._nav, self._g_nav(x, y, yaw, f'pick_{pick_t}'),
                                f'nav→{pick_t} (pick {p["pick_standoff"]:.1f}m przed)',
                                lambda f: f'd={f.distance_remaining_m:.2f}m')
        if not ok or st.is_set():
            return False
        # 2. dok APRILTAG na karton
        ok, _ = self._exec_sync(self._dock, self._g_dock(DockToTable.Goal.MODE_APRILTAG, p),
                                f'dock APRILTAG @{pick_t}',
                                lambda f: f'{f.phase} xy={f.xy_error_m:.3f}')
        if not ok or st.is_set():
            return False
        # 3. pick
        pg = PickBox.Goal(); pg.sequence_name = 'pick_box'; pg.timeout_s = 60.0
        ok, _ = self._exec_sync(self._pick, pg, f'pick @{pick_t}',
                                lambda f: f'{f.phase} {f.progress:.0%}')
        if not ok or st.is_set():
            return False
        # 4. clearance retreat (żeby planner wyszedł ze strefy biurka)
        if p['post_pick'] > 0.01:
            rg = Retreat.Goal(); rg.distance_m = p['post_pick']
            rg.speed_mps = p['retreat_speed']; rg.timeout_s = 15.0
            self._exec_sync(self._retreat, rg, f'clearance {p["post_pick"]:.1f}m')
            if st.is_set():
                return False
        # 5. nav BLISKO biurko place (kotwica - place_standoff)
        x, y, yaw = self._predock(anchors[place_t], p['place_standoff'])
        ok, _ = self._exec_sync(self._nav, self._g_nav(x, y, yaw, f'place_{place_t}'),
                                f'nav→{place_t} (place {p["place_standoff"]:.1f}m przed)',
                                lambda f: f'd={f.distance_remaining_m:.2f}m')
        if not ok or st.is_set():
            return False
        # 6. metoda odkładania
        if p['place_method'] == 'lidar':
            ok, _ = self._exec_sync(self._dock, self._g_dock(DockToTable.Goal.MODE_LIDAR_LINE, p),
                                    f'dock LIDAR @{place_t}',
                                    lambda f: f'{f.phase} xy={f.xy_error_m:.3f}')
            if not ok or st.is_set():
                return False
        elif p['place_method'] == 'apriltag':
            ok, _ = self._exec_sync(self._dock, self._g_dock(DockToTable.Goal.MODE_APRILTAG, p),
                                    f'dock APRILTAG @{place_t}',
                                    lambda f: f'{f.phase} xy={f.xy_error_m:.3f}')
            if not ok or st.is_set():
                return False
        # else 'nav' — bez dokowania, odkładamy z nawigacji
        plg = PlaceBox.Goal(); plg.timeout_s = 60.0
        ok, _ = self._exec_sync(self._place, plg, f'place @{place_t}',
                                lambda f: f'{f.phase} {f.progress:.0%}')
        if not ok or st.is_set():
            return False
        # 7. retreat (parametr z GUI, np. 2m)
        rg = Retreat.Goal(); rg.distance_m = p['retreat_dist']
        rg.speed_mps = p['retreat_speed']; rg.timeout_s = 20.0
        ok, _ = self._exec_sync(self._retreat, rg, f'retreat {p["retreat_dist"]:.1f}m')
        return ok and not st.is_set()

    def cancel_active(self) -> None:
        with self._lock:
            handle = self._active_goal_handle
        if handle is None:
            self.log_message.emit('warn', 'CANCEL: brak aktywnej akcji')
            return
        self.log_message.emit('warn', 'CANCEL → wysyłam')
        handle.cancel_goal_async()

    def estop(self) -> None:
        # Cancel + zero cmd_vel + freeze service (jeśli dostępny).
        self._cycle_stop.set()   # zatrzymaj cykl jeśli aktywny (nie tylko bieżący goal)
        self.cancel_active()
        zero = Twist()
        for _ in range(5):
            self._cmd_vel_pub.publish(zero)
        if self._freeze_client.wait_for_service(timeout_sec=0.5):
            req = SetFreeze.Request()
            req.freeze = True
            self._freeze_client.call_async(req)
            self.log_message.emit('error', 'E-STOP: cancel + freeze=true')
        else:
            self.log_message.emit('error', 'E-STOP: cancel + zero cmd_vel (brak /safety/set_freeze)')

    def unfreeze(self) -> None:
        if self._freeze_client.wait_for_service(timeout_sec=0.5):
            req = SetFreeze.Request()
            req.freeze = False
            self._freeze_client.call_async(req)
            self.log_message.emit('info', 'freeze=false')

    def set_initial_pose(self, x: float, y: float, yaw_rad: float) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
        # Domyślna kowariancja (rozsądna dla manual estimate ~30 cm / ~15°).
        msg.pose.covariance = [
            0.25, 0, 0, 0, 0, 0,
            0, 0.25, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0.0685,  # ~15° yaw std
        ]
        self._initialpose_pub.publish(msg)
        self.log_message.emit(
            'info', f'/initialpose: x={x:.2f} y={y:.2f} yaw={math.degrees(yaw_rad):.0f}°',
        )


# ----- GUI -----


def _load_waypoints() -> dict:
    try:
        path = os.path.join(
            get_package_share_directory('g1_courier_mission'),
            'config', 'waypoints.yaml',
        )
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        return data.get('tables', {})
    except Exception:
        return {}


class OperatorWindow(QMainWindow):
    def __init__(self, bridge: RosBridge) -> None:
        super().__init__()
        self.bridge = bridge
        self.setWindowTitle('G1 Courier — Operator')
        self.resize(1280, 720)

        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)

        root.addWidget(self._build_status_panel(), stretch=1)
        root.addWidget(self._build_actions_panel(), stretch=2)
        root.addWidget(self._build_log_panel(), stretch=2)

        # Połącz sygnały z ROS bridge.
        bridge.amcl_pose_changed.connect(self._on_amcl)
        bridge.dock_errors_changed.connect(self._on_dock_errors)
        bridge.hz_changed.connect(self._on_hz)
        bridge.log_message.connect(self._on_log)
        bridge.active_goal_changed.connect(self._on_active_goal)
        # Goal end (any action) → reset dock error label do (idle), żeby nie
        # zamrażało się na 'NO TAG' po timeout/abort.
        bridge.goal_finished.connect(self._reset_dock_err_label)
        bridge.cycle_phase_changed.connect(self._on_cycle_phase)
        bridge.cycle_finished.connect(self._on_cycle_finished)

    # ----- panele -----

    def _build_status_panel(self) -> QWidget:
        box = QGroupBox('Status')
        layout = QFormLayout(box)

        self.lbl_amcl = QLabel('—')
        self.lbl_scan = QLabel('—')
        self.lbl_lowstate = QLabel('—')
        self.lbl_active = QLabel('(none)')
        self.lbl_active.setStyleSheet('color: #888;')
        for lbl in (self.lbl_amcl, self.lbl_scan, self.lbl_lowstate, self.lbl_active):
            lbl.setFont(QFont('Monospace'))

        layout.addRow('AMCL pose:', self.lbl_amcl)
        layout.addRow('/scan Hz:', self.lbl_scan)
        layout.addRow('/lowstate Hz:', self.lbl_lowstate)
        layout.addRow('Active goal:', self.lbl_active)

        # Initial pose section.
        ip_box = QGroupBox('Set Initial Pose (AMCL)')
        ip_form = QFormLayout(ip_box)
        self.ip_x = QDoubleSpinBox(); self.ip_x.setRange(-100, 100); self.ip_x.setSingleStep(0.1); self.ip_x.setDecimals(2)
        self.ip_y = QDoubleSpinBox(); self.ip_y.setRange(-100, 100); self.ip_y.setSingleStep(0.1); self.ip_y.setDecimals(2)
        self.ip_yaw = QDoubleSpinBox(); self.ip_yaw.setRange(-360, 360); self.ip_yaw.setSingleStep(15); self.ip_yaw.setDecimals(0); self.ip_yaw.setSuffix(' °')
        ip_form.addRow('x [m]:', self.ip_x)
        ip_form.addRow('y [m]:', self.ip_y)
        ip_form.addRow('yaw:', self.ip_yaw)
        btn_ip = QPushButton('Set Initial Pose')
        btn_ip.clicked.connect(self._on_set_initial_pose)
        ip_form.addRow(btn_ip)
        layout.addRow(ip_box)

        # E-stop.
        btn_estop = QPushButton('⛔  E-STOP')
        btn_estop.setStyleSheet(
            'background-color: #cc0000; color: white; font-weight: bold; '
            'font-size: 18pt; padding: 16px;'
        )
        btn_estop.clicked.connect(self.bridge.estop)
        layout.addRow(btn_estop)

        btn_unfreeze = QPushButton('Unfreeze (po E-stopie)')
        btn_unfreeze.clicked.connect(self.bridge.unfreeze)
        layout.addRow(btn_unfreeze)

        # Cancel current goal (mniej drastyczne niż E-stop).
        btn_cancel = QPushButton('Cancel active goal')
        btn_cancel.setStyleSheet('background-color: #cc8800; color: white;')
        btn_cancel.clicked.connect(self.bridge.cancel_active)
        layout.addRow(btn_cancel)

        return box

    def _build_actions_panel(self) -> QWidget:
        outer = QWidget()
        layout = QVBoxLayout(outer)
        layout.addWidget(self._build_cycle_group())
        layout.addWidget(self._build_navigate_group())
        layout.addWidget(self._build_dock_group())
        layout.addWidget(self._build_pick_place_group())
        layout.addWidget(self._build_retreat_group())
        layout.addStretch()
        return outer

    def _build_cycle_group(self) -> QGroupBox:
        box = QGroupBox('Cykl misji A↔B (auto)')
        layout = QGridLayout(box)

        # Kotwice = poza robota PRZY biurku (twarzą do niego). Dystanse niżej
        # liczone WSTECZ od kotwicy → punkt przed biurkiem.
        self._anchor_a = None
        self._anchor_b = None
        self.lbl_anchor_a = QLabel('A: —'); self.lbl_anchor_a.setFont(QFont('Monospace', 9))
        self.lbl_anchor_b = QLabel('B: —'); self.lbl_anchor_b.setFont(QFont('Monospace', 9))
        btn_teach_a = QPushButton('Zapisz A = aktualna poza')
        btn_teach_a.clicked.connect(lambda: self._on_teach('A'))
        btn_teach_b = QPushButton('Zapisz B = aktualna poza')
        btn_teach_b.clicked.connect(lambda: self._on_teach('B'))
        layout.addWidget(btn_teach_a, 0, 0, 1, 2); layout.addWidget(self.lbl_anchor_a, 0, 2, 1, 2)
        layout.addWidget(btn_teach_b, 1, 0, 1, 2); layout.addWidget(self.lbl_anchor_b, 1, 2, 1, 2)

        # Parametry standoffów (mierzone wstecz od kotwicy=biurka).
        self.cyc_pick = QDoubleSpinBox(); self.cyc_pick.setRange(0.2, 3.0); self.cyc_pick.setValue(1.0); self.cyc_pick.setSingleStep(0.1); self.cyc_pick.setDecimals(2)
        self.cyc_place = QDoubleSpinBox(); self.cyc_place.setRange(0.0, 2.0); self.cyc_place.setValue(0.2); self.cyc_place.setSingleStep(0.05); self.cyc_place.setDecimals(2)
        self.cyc_retreat = QDoubleSpinBox(); self.cyc_retreat.setRange(0.1, 3.0); self.cyc_retreat.setValue(2.0); self.cyc_retreat.setSingleStep(0.1); self.cyc_retreat.setDecimals(1)
        self.cyc_postpick = QDoubleSpinBox(); self.cyc_postpick.setRange(0.0, 1.5); self.cyc_postpick.setValue(0.5); self.cyc_postpick.setSingleStep(0.1); self.cyc_postpick.setDecimals(1)
        layout.addWidget(QLabel('pick (m przed):'), 2, 0); layout.addWidget(self.cyc_pick, 2, 1)
        layout.addWidget(QLabel('place (m przed):'), 2, 2); layout.addWidget(self.cyc_place, 2, 3)
        layout.addWidget(QLabel('retreat [m]:'), 3, 0); layout.addWidget(self.cyc_retreat, 3, 1)
        layout.addWidget(QLabel('clearance po pick [m]:'), 3, 2); layout.addWidget(self.cyc_postpick, 3, 3)

        self.cyc_tag = QSpinBox(); self.cyc_tag.setRange(0, 99); self.cyc_tag.setValue(10)
        self.cyc_place_method = QComboBox()
        self.cyc_place_method.addItem('nav blisko (bez doku)', 'nav')
        self.cyc_place_method.addItem('dok LIDAR_LINE', 'lidar')
        self.cyc_place_method.addItem('dok APRILTAG', 'apriltag')
        layout.addWidget(QLabel('tag_id:'), 4, 0); layout.addWidget(self.cyc_tag, 4, 1)
        layout.addWidget(QLabel('place:'), 4, 2); layout.addWidget(self.cyc_place_method, 4, 3)

        self.btn_cycle_start = QPushButton('▶ START CYKL')
        self.btn_cycle_start.setStyleSheet('background-color: #117711; color: white; font-weight: bold; padding: 8px;')
        self.btn_cycle_start.clicked.connect(self._on_cycle_start)
        self.btn_cycle_stop = QPushButton('■ STOP CYKL')
        self.btn_cycle_stop.setStyleSheet('background-color: #883333; color: white; padding: 8px;')
        self.btn_cycle_stop.clicked.connect(self.bridge.stop_cycle)
        self.btn_cycle_stop.setEnabled(False)
        layout.addWidget(self.btn_cycle_start, 5, 0, 1, 2)
        layout.addWidget(self.btn_cycle_stop, 5, 2, 1, 2)

        self.lbl_cycle_phase = QLabel('(bezczynny)')
        self.lbl_cycle_phase.setStyleSheet('font-family: monospace; color: #888;')
        layout.addWidget(QLabel('faza:'), 6, 0); layout.addWidget(self.lbl_cycle_phase, 6, 1, 1, 3)

        self._load_anchors()
        return box

    def _build_navigate_group(self) -> QGroupBox:
        box = QGroupBox('Navigate')
        layout = QGridLayout(box)

        # Waypoint dropdown.
        self.nav_combo = QComboBox()
        self.nav_combo.addItem('(ad-hoc)')
        for name, cfg in _load_waypoints().items():
            self.nav_combo.addItem(name, cfg)
        self.nav_combo.currentIndexChanged.connect(self._on_waypoint_selected)
        layout.addWidget(QLabel('Waypoint:'), 0, 0)
        layout.addWidget(self.nav_combo, 0, 1, 1, 3)

        self.nav_x = QDoubleSpinBox(); self.nav_x.setRange(-100, 100); self.nav_x.setSingleStep(0.1); self.nav_x.setDecimals(2)
        self.nav_y = QDoubleSpinBox(); self.nav_y.setRange(-100, 100); self.nav_y.setSingleStep(0.1); self.nav_y.setDecimals(2)
        self.nav_yaw = QDoubleSpinBox(); self.nav_yaw.setRange(-360, 360); self.nav_yaw.setSingleStep(15); self.nav_yaw.setDecimals(0); self.nav_yaw.setSuffix(' °')
        layout.addWidget(QLabel('x [m]:'), 1, 0); layout.addWidget(self.nav_x, 1, 1)
        layout.addWidget(QLabel('y [m]:'), 1, 2); layout.addWidget(self.nav_y, 1, 3)
        layout.addWidget(QLabel('yaw:'), 2, 0); layout.addWidget(self.nav_yaw, 2, 1)

        btn = QPushButton('GO →')
        btn.setStyleSheet('background-color: #2266aa; color: white; padding: 8px;')
        btn.clicked.connect(self._on_navigate_clicked)
        layout.addWidget(btn, 2, 2, 1, 2)
        return box

    def _build_dock_group(self) -> QGroupBox:
        box = QGroupBox('Dock to table')
        layout = QGridLayout(box)

        self.dock_mode_group = QButtonGroup(self)
        self.dock_apriltag = QRadioButton('APRILTAG (pickup)')
        self.dock_lidar = QRadioButton('LIDAR_LINE (place)')
        self.dock_amcl = QRadioButton('AMCL_ONLY')
        self.dock_apriltag.setChecked(True)
        for i, rb in enumerate((self.dock_apriltag, self.dock_lidar, self.dock_amcl)):
            self.dock_mode_group.addButton(rb, i)
            layout.addWidget(rb, 0, i)

        self.dock_tag = QSpinBox(); self.dock_tag.setRange(0, 99); self.dock_tag.setValue(10)
        layout.addWidget(QLabel('tag_id:'), 1, 0); layout.addWidget(self.dock_tag, 1, 1)
        self.dock_z = QDoubleSpinBox(); self.dock_z.setRange(0.05, 2.0); self.dock_z.setValue(0.30); self.dock_z.setSingleStep(0.05); self.dock_z.setDecimals(2)
        layout.addWidget(QLabel('target z [m]:'), 1, 2); layout.addWidget(self.dock_z, 1, 3)

        self.dock_xy_tol = QDoubleSpinBox(); self.dock_xy_tol.setRange(0.01, 0.5); self.dock_xy_tol.setValue(0.10); self.dock_xy_tol.setSingleStep(0.01); self.dock_xy_tol.setDecimals(2)
        layout.addWidget(QLabel('xy_tol [m]:'), 2, 0); layout.addWidget(self.dock_xy_tol, 2, 1)
        self.dock_yaw_tol = QDoubleSpinBox(); self.dock_yaw_tol.setRange(0.01, 1.5); self.dock_yaw_tol.setValue(0.15); self.dock_yaw_tol.setSingleStep(0.05); self.dock_yaw_tol.setDecimals(2)
        layout.addWidget(QLabel('yaw_tol [rad]:'), 2, 2); layout.addWidget(self.dock_yaw_tol, 2, 3)

        btn = QPushButton('DOCK →')
        btn.setStyleSheet('background-color: #aa6622; color: white; padding: 8px;')
        btn.clicked.connect(self._on_dock_clicked)
        layout.addWidget(btn, 3, 0, 1, 4)

        # Live dx/dy/dyaw z /dock/errors — aktywne tylko podczas dock servo.
        # Idle = "(idle)". Po convergence i kolejnym dock'u resetuje się.
        layout.addWidget(QLabel('errors:'), 4, 0)
        self.lbl_dock_err = QLabel('(idle)')
        self.lbl_dock_err.setStyleSheet('font-family: monospace; color: #888;')
        layout.addWidget(self.lbl_dock_err, 4, 1, 1, 3)
        return box

    def _build_pick_place_group(self) -> QGroupBox:
        box = QGroupBox('Arm skills')
        layout = QHBoxLayout(box)
        btn_pick = QPushButton('PICK box')
        btn_pick.setStyleSheet('background-color: #228855; color: white; padding: 8px;')
        btn_pick.clicked.connect(self.bridge.send_pick)
        btn_place = QPushButton('PLACE box')
        btn_place.setStyleSheet('background-color: #885522; color: white; padding: 8px;')
        btn_place.clicked.connect(self.bridge.send_place)
        layout.addWidget(btn_pick)
        layout.addWidget(btn_place)
        return box

    def _build_retreat_group(self) -> QGroupBox:
        box = QGroupBox('Retreat (open-loop backup)')
        layout = QGridLayout(box)
        self.retreat_dist = QDoubleSpinBox(); self.retreat_dist.setRange(0.1, 2.0); self.retreat_dist.setValue(0.5); self.retreat_dist.setSingleStep(0.1); self.retreat_dist.setDecimals(1)
        self.retreat_speed = QDoubleSpinBox(); self.retreat_speed.setRange(0.05, 0.3); self.retreat_speed.setValue(0.12); self.retreat_speed.setSingleStep(0.01); self.retreat_speed.setDecimals(2)
        layout.addWidget(QLabel('dist [m]:'), 0, 0); layout.addWidget(self.retreat_dist, 0, 1)
        layout.addWidget(QLabel('speed [m/s]:'), 0, 2); layout.addWidget(self.retreat_speed, 0, 3)
        btn = QPushButton('RETREAT ←')
        btn.setStyleSheet('background-color: #555555; color: white; padding: 8px;')
        btn.clicked.connect(self._on_retreat_clicked)
        layout.addWidget(btn, 1, 0, 1, 4)
        return box

    def _build_log_panel(self) -> QWidget:
        box = QGroupBox('Log')
        layout = QVBoxLayout(box)
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(500)
        self.log.setFont(QFont('Monospace', 9))
        layout.addWidget(self.log)
        return box

    # ----- handlers -----

    def _on_waypoint_selected(self, idx: int) -> None:
        cfg = self.nav_combo.itemData(idx)
        if not cfg:
            return
        self.nav_x.setValue(float(cfg.get('predock_x', 0.0)))
        self.nav_y.setValue(float(cfg.get('predock_y', 0.0)))
        self.nav_yaw.setValue(math.degrees(float(cfg.get('predock_yaw', 0.0))))

    def _on_navigate_clicked(self) -> None:
        name = self.nav_combo.currentText() if self.nav_combo.currentIndex() > 0 else ''
        self.bridge.send_nav(
            self.nav_x.value(), self.nav_y.value(),
            math.radians(self.nav_yaw.value()), name,
        )

    def _on_dock_clicked(self) -> None:
        mode = self.dock_mode_group.checkedId()
        self.bridge.send_dock(
            mode=mode,
            tag_id=self.dock_tag.value(),
            target_xyz_yaw=(0.0, 0.0, self.dock_z.value(), 0.0),
            xy_tol=self.dock_xy_tol.value(),
            yaw_tol=self.dock_yaw_tol.value(),
        )

    def _on_retreat_clicked(self) -> None:
        self.bridge.send_retreat(self.retreat_dist.value(), self.retreat_speed.value())

    def _on_set_initial_pose(self) -> None:
        self.bridge.set_initial_pose(
            self.ip_x.value(), self.ip_y.value(),
            math.radians(self.ip_yaw.value()),
        )

    # ----- cykl: teach + start/stop -----

    def _fmt_anchor(self, a) -> str:
        if a is None:
            return '—'
        return f'x={a[0]:+.2f} y={a[1]:+.2f} yaw={math.degrees(a[2]):+.0f}°'

    def _on_teach(self, which: str) -> None:
        pose = self.bridge.current_pose()
        if pose is None:
            self.bridge.log_message.emit('error', 'teach: brak /amcl_pose — czy AMCL zlokalizowany?')
            return
        if which == 'A':
            self._anchor_a = pose
            self.lbl_anchor_a.setText(f'A: {self._fmt_anchor(pose)}')
        else:
            self._anchor_b = pose
            self.lbl_anchor_b.setText(f'B: {self._fmt_anchor(pose)}')
        self.bridge.log_message.emit('ok', f'kotwica {which} = {self._fmt_anchor(pose)} (PRZY biurku)')
        self._save_anchors()

    def _save_anchors(self) -> None:
        try:
            os.makedirs(os.path.dirname(CYCLE_WP_PATH), exist_ok=True)
            with open(CYCLE_WP_PATH, 'w') as f:
                json.dump({'A': self._anchor_a, 'B': self._anchor_b}, f)
        except Exception as exc:
            self.bridge.log_message.emit('warn', f'zapis kotwic nieudany: {exc}')

    def _load_anchors(self) -> None:
        try:
            with open(CYCLE_WP_PATH) as f:
                d = json.load(f)
            if d.get('A'):
                self._anchor_a = tuple(d['A']); self.lbl_anchor_a.setText(f'A: {self._fmt_anchor(self._anchor_a)}')
            if d.get('B'):
                self._anchor_b = tuple(d['B']); self.lbl_anchor_b.setText(f'B: {self._fmt_anchor(self._anchor_b)}')
        except FileNotFoundError:
            pass
        except Exception as exc:
            self.bridge.log_message.emit('warn', f'wczytanie kotwic nieudane: {exc}')

    def _on_cycle_start(self) -> None:
        if self._anchor_a is None or self._anchor_b is None:
            self.bridge.log_message.emit('error', 'cykl: zapisz najpierw kotwice A i B')
            return
        params = {
            'anchor_a': self._anchor_a, 'anchor_b': self._anchor_b,
            'pick_standoff': self.cyc_pick.value(),
            'place_standoff': self.cyc_place.value(),
            'retreat_dist': self.cyc_retreat.value(),
            'retreat_speed': 0.12,
            'post_pick': self.cyc_postpick.value(),
            'tag_id': self.cyc_tag.value(),
            'dock_z': 0.30, 'xy_tol': 0.05, 'yaw_tol': 0.10,
            'place_method': self.cyc_place_method.currentData(),
        }
        self.btn_cycle_start.setEnabled(False)
        self.btn_cycle_stop.setEnabled(True)
        self.bridge.log_message.emit('ok', '[cykl] START')
        self.bridge.start_cycle(params)

    # ----- ROS bridge slots -----

    def _on_amcl(self, x: float, y: float, yaw_deg: float) -> None:
        self.lbl_amcl.setText(f'x={x:+.2f} y={y:+.2f} yaw={yaw_deg:+.0f}°')

    def _on_cycle_phase(self, phase: str) -> None:
        self.lbl_cycle_phase.setText(phase)
        self.lbl_cycle_phase.setStyleSheet('font-family: monospace; color: #117711; font-weight: bold;')

    def _on_cycle_finished(self, ok: bool, reason: str) -> None:
        self.btn_cycle_start.setEnabled(True)
        self.btn_cycle_stop.setEnabled(False)
        self.lbl_cycle_phase.setText(f'koniec: {reason}')
        self.lbl_cycle_phase.setStyleSheet(
            f'font-family: monospace; color: {"#888" if ok else "#cc2222"};')

    def _reset_dock_err_label(self, _ok: bool, _msg: str) -> None:
        """Reset dock errors label do '(idle)' po zakończeniu dowolnego action.
        Wywoływane przez goal_finished signal (timeout, success, cancel, abort)."""
        self.lbl_dock_err.setText('(idle)')
        self.lbl_dock_err.setStyleSheet('font-family: monospace; color: #888;')

    def _on_dock_errors(self, dx: float, dy: float, dyaw: float) -> None:
        # NaN sygnalizuje "tag/line lost" — dock servo żyje ale aligner nie ma
        # feature do dopasowania. Czerwone "NO TAG" żeby od razu było widać.
        if math.isnan(dx) or math.isnan(dyaw):
            self.lbl_dock_err.setText('NO TAG / LINE — server alive, no detection')
            self.lbl_dock_err.setStyleSheet('font-family: monospace; color: #cc0000; font-weight: bold;')
            return
        # Kolor: zielony jak każdy z błędów < threshold convergence (z dock_action_server
        # default xy_tol=0.03, yaw_tol=0.05). Pomarańcz medium. Czerwony duży błąd.
        max_xy = max(abs(dx), abs(dy))
        if max_xy < 0.03 and abs(dyaw) < 0.05:
            color = '#22aa22'   # converged
        elif max_xy < 0.10 and abs(dyaw) < 0.15:
            color = '#cc6600'   # close
        else:
            color = '#cc0000'   # far
        dyaw_deg = math.degrees(dyaw)
        self.lbl_dock_err.setText(
            f'dx={dx:+.3f}m  dy={dy:+.3f}m  dyaw={dyaw_deg:+.1f}°'
        )
        self.lbl_dock_err.setStyleSheet(f'font-family: monospace; color: {color};')

    def _on_hz(self, topic: str, hz: float) -> None:
        color = '#22aa22' if hz > 1 else '#cc6600' if hz > 0.1 else '#cc0000'
        text = f'{hz:.1f} Hz'
        lbl = {'scan': self.lbl_scan, 'lowstate': self.lbl_lowstate}.get(topic)
        if lbl:
            lbl.setText(text)
            lbl.setStyleSheet(f'color: {color}; font-family: monospace;')

    def _on_log(self, level: str, text: str) -> None:
        color = {
            'info': '#aaaaaa', 'fb': '#5588ff', 'warn': '#dd8800',
            'error': '#dd2222', 'ok': '#22aa22',
        }.get(level, '#cccccc')
        t = time.strftime('%H:%M:%S')
        self.log.appendHtml(
            f'<span style="color: #666;">{t}</span> '
            f'<span style="color: {color};">[{level}]</span> {text}'
        )

    def _on_active_goal(self, action: str, info: str) -> None:
        if action:
            self.lbl_active.setText(f'{action} {info}')
            self.lbl_active.setStyleSheet('color: #2266aa; font-weight: bold;')
        else:
            self.lbl_active.setText('(none)')
            self.lbl_active.setStyleSheet('color: #888;')


# ----- main -----


def main() -> None:
    rclpy.init()
    bridge = RosBridge()

    # ROS w osobnym wątku.
    executor = MultiThreadedExecutor()
    executor.add_node(bridge.node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    app = QApplication(sys.argv)
    window = OperatorWindow(bridge)
    window.show()
    # PyQt5 używa exec_(), PyQt6 ma exec().
    exit_code = app.exec() if HAVE_PYQT6 else app.exec_()

    executor.shutdown()
    bridge.node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
