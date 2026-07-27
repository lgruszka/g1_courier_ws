#!/usr/bin/env python3
"""Pomiar dryfu przy STANIU W MIEJSCU.

Zbiera ~60 s:
  /dog_odom            -> surowa odometria firmware (przed trans_scale)
  /amcl_pose           -> pozycja na mapie (po AMCL)
  TF odom->base_footprint -> wyjscie odom_tf_relay (po trans_scale+flatten)
  /lowstate IMU pitch  -> amplituda kolysania (jesli typ dostepny)

Wynik: ile kazdy tor "uciekl" i jaka jest amplituda sway.
Uruchamiac gdy robot STOI (bez celu nav).
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from tf2_ros import Buffer, TransformListener

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def pitch_of(q):
    s = 2.0 * (q.w * q.y - q.z * q.x)
    return math.asin(max(-1.0, min(1.0, s)))


class Diag(Node):
    def __init__(self):
        super().__init__('standstill_diag')
        self.odom = []      # (t, x, y, yaw, pitch)
        self.amcl = []      # (t, x, y, yaw)
        self.tfs = []       # (t, x, y, yaw)
        self.create_subscription(Odometry, '/dog_odom', self.cb_odom,
                                 qos_profile_sensor_data)
        latched = QoSProfile(depth=5,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self.cb_amcl, latched)
        self.buf = Buffer()
        self.tl = TransformListener(self.buf, self)
        self.t0 = time.time()
        self.create_timer(0.1, self.tick)

    def cb_odom(self, m):
        p = m.pose.pose
        self.odom.append((time.time() - self.t0, p.position.x, p.position.y,
                          yaw_of(p.orientation), pitch_of(p.orientation)))

    def cb_amcl(self, m):
        p = m.pose.pose
        self.amcl.append((time.time() - self.t0, p.position.x, p.position.y,
                          yaw_of(p.orientation)))

    def tick(self):
        try:
            t = self.buf.lookup_transform('odom', 'base_footprint',
                                          rclpy.time.Time())
            tr = t.transform.translation
            self.tfs.append((time.time() - self.t0, tr.x, tr.y,
                             yaw_of(t.transform.rotation)))
        except Exception:
            pass


def stats(name, rows, has_pitch=False):
    if len(rows) < 5:
        print(f'{name}: BRAK DANYCH ({len(rows)} probek)')
        return
    t0, x0, y0 = rows[0][0], rows[0][1], rows[0][2]
    tN, xN, yN = rows[-1][0], rows[-1][1], rows[-1][2]
    dt = tN - t0
    drift = math.hypot(xN - x0, yN - y0)
    dyaw = math.degrees(rows[-1][3] - rows[0][3])
    # rozpietosc chwilowa (peak-to-peak) w oknach 2 s -> amplituda sway
    pp = 0.0
    w = []
    for r in rows:
        w = [q for q in w if r[0] - q[0] <= 2.0] + [r]
        if len(w) > 3:
            pp = max(pp, math.hypot(max(q[1] for q in w) - min(q[1] for q in w),
                                    max(q[2] for q in w) - min(q[2] for q in w)))
    # dlugosc calkowitej sciezki (ile "przejechal" wg tego toru)
    path = sum(math.hypot(rows[i][1] - rows[i - 1][1], rows[i][2] - rows[i - 1][2])
               for i in range(1, len(rows)))
    print(f'{name}:')
    print(f'   probek={len(rows)}  ({len(rows)/max(dt,1e-6):.0f} Hz)  okno={dt:.1f} s')
    print(f'   DRYF netto = {drift*100:+.1f} cm   dyaw = {dyaw:+.2f} st')
    print(f'   dlugosc sciezki (suma |delta|) = {path*100:.1f} cm  '
          f'<-- to widzi nav jako "ruch"')
    print(f'   sway peak-to-peak (okno 2 s) = {pp*100:.1f} cm')
    if has_pitch:
        pitches = [math.degrees(r[4]) for r in rows]
        print(f'   pitch miednicy: {min(pitches):+.2f} .. {max(pitches):+.2f} st '
              f'(pp {max(pitches)-min(pitches):.2f} st)')


def main():
    rclpy.init()
    n = Diag()
    print(f'zbieram {DUR:.0f} s (robot MUSI STAC, bez celu nav)...', flush=True)
    end = time.time() + DUR
    while time.time() < end and rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.2)
    print('=' * 64)
    stats('/dog_odom  (SUROWA odometria firmware)', n.odom, has_pitch=True)
    print('-' * 64)
    stats('TF odom->base_footprint (PO trans_scale=1.89 + flatten)', n.tfs)
    print('-' * 64)
    stats('/amcl_pose (pozycja na MAPIE)', n.amcl)
    print('=' * 64)
    if len(n.odom) > 5 and len(n.tfs) > 5:
        po = sum(math.hypot(n.odom[i][1]-n.odom[i-1][1], n.odom[i][2]-n.odom[i-1][2])
                 for i in range(1, len(n.odom)))
        pt = sum(math.hypot(n.tfs[i][1]-n.tfs[i-1][1], n.tfs[i][2]-n.tfs[i-1][2])
                 for i in range(1, len(n.tfs)))
        print(f'WZMOCNIENIE SZUMU przez relay: sciezka TF / sciezka odom = '
              f'{pt/max(po,1e-9):.2f}x  (oczekiwane ~1.89 jesli szum jest skalowany)')
    print('=' * 64, flush=True)


if __name__ == '__main__':
    main()
