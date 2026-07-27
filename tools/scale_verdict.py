#!/usr/bin/env python3
"""Weryfikacja skali odometrii POD RUCHEM + ile AMCL musi walczyc z odometria.

  python3 scale_verdict.py [max_sekund_oczekiwania]

Startuje sam, gdy pojawi sie ruch. Mierzy rownolegle:
  raw   : /dog_odom            — co zglasza firmware
  tf    : odom->base_footprint — po trans_scale (to widzi AMCL jako ruch)
  amcl  : /amcl_pose           — gdzie robot FAKTYCZNIE jest (zakotwiczone skanem)
  m->o  : map->odom            — ile AMCL musial podsunac, by nadgonic blad

Iloraz amcl/raw = SKALA, ktora powinna byc w trans_scale.
Dryf map->odom na metr drogi = ile bledu AMCL musi kasowac (to jest to, co widac
jako "skan ucieka od mapy" miedzy korektami).
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy,
                       qos_profile_sensor_data)
from tf2_ros import Buffer, TransformListener

WAIT = float(sys.argv[1]) if len(sys.argv) > 1 else 200.0
IDLE_STOP_S = 4.0
MOVE_EPS_M = 0.05


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


class V(Node):
    def __init__(self):
        super().__init__('scale_verdict')
        self.raw = []
        self.amcl = []
        self.tf = []
        self.mo = []
        self.create_subscription(Odometry, '/dog_odom', self.cb_raw,
                                 qos_profile_sensor_data)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.cb_amcl,
            QoSProfile(depth=5, reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.buf = Buffer()
        self.tl = TransformListener(self.buf, self)
        self.create_timer(0.1, self.tick)

    def cb_raw(self, m):
        p = m.pose.pose
        self.raw.append((time.time(), p.position.x, p.position.y,
                         yaw_of(p.orientation)))

    def cb_amcl(self, m):
        p = m.pose.pose
        self.amcl.append((time.time(), p.position.x, p.position.y,
                          yaw_of(p.orientation)))

    def tick(self):
        for parent, child, sink in (('odom', 'base_footprint', self.tf),
                                    ('map', 'odom', self.mo)):
            try:
                t = self.buf.lookup_transform(parent, child, rclpy.time.Time())
                tr = t.transform.translation
                sink.append((time.time(), tr.x, tr.y, yaw_of(t.transform.rotation)))
            except Exception:
                pass


def net(rows, a, b):
    """netto, dyaw, dlugosc sciezki, najwieksza nieciaglosc (skok) + jej czas."""
    w = [r for r in rows if a <= r[0] <= b]
    if len(w) < 2:
        return None
    d = math.hypot(w[-1][1] - w[0][1], w[-1][2] - w[0][2])
    dyaw = math.degrees(math.atan2(math.sin(w[-1][3] - w[0][3]),
                                   math.cos(w[-1][3] - w[0][3])))
    path = 0.0
    jump, jump_t = 0.0, None
    for i in range(1, len(w)):
        step = math.hypot(w[i][1] - w[i-1][1], w[i][2] - w[i-1][2])
        path += step
        if step > jump:
            jump, jump_t = step, w[i][0]
    return d, dyaw, path, jump, jump_t


def main():
    rclpy.init()
    n = V()
    print(f'czekam na ruch (WYSLIJ CEL, najlepiej prosto), do {WAIT:.0f} s...',
          flush=True)
    t_end = time.time() + WAIT
    start = None
    last = None
    ref = None
    while time.time() < t_end and rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.2)
        if not n.raw:
            continue
        cur = n.raw[-1]
        if ref is None:
            ref = cur
        if math.hypot(cur[1] - ref[1], cur[2] - ref[2]) > MOVE_EPS_M:
            if start is None:
                start = ref[0]
                print('>>> RUCH — mierze', flush=True)
            last = cur[0]
            ref = cur
        elif start and last and time.time() - last > IDLE_STOP_S:
            print('>>> STOP', flush=True)
            break

    if start is None:
        print('BRAK RUCHU — cel nie dotarl.')
        return
    b = last

    # Pose estimate operatora TELEPORTUJE AMCL => okno musi zaczynac sie PO
    # ostatnim skoku, inaczej "prawda" zawiera reczne korekty (tak zepsul sie
    # pomiar 2026-07-27: okno 101.9 s objelo dwa Setting pose).
    JUMP_M = 0.30
    jumps = [n.amcl[i][0] for i in range(1, len(n.amcl))
             if math.hypot(n.amcl[i][1] - n.amcl[i-1][1],
                           n.amcl[i][2] - n.amcl[i-1][2]) > JUMP_M
             and start <= n.amcl[i][0] <= b]
    if jumps:
        print(f'\n!!! WYKRYTO {len(jumps)} SKOK(OW) /amcl_pose > {JUMP_M} m '
              f'(Pose estimate albo relokalizacja).')
        if b - jumps[-1] < 8.0:
            print('!!! Po ostatnim skoku zostalo <8 s ruchu — POMIAR NIEWAZNY.')
            print('!!! Ustaw Pose estimate PRZED startem sondy i nie ruszaj go.')
            return
        print(f'    przycinam okno do czasu PO ostatnim skoku '
              f'(tracac {jumps[-1] - start:.1f} s)')
        start = jumps[-1] + 0.5

    r = net(n.raw, start, b)
    t = net(n.tf, start, b)
    a = net(n.amcl, start, b)
    m = net(n.mo, start, b)
    if not (r and t and a):
        print('BRAK DANYCH w oknie po przycieciu.')
        return
    r_d, r_yaw, r_path, r_jump, r_jt = r
    t_d, t_yaw, t_path, _, _ = t
    a_d, a_yaw, a_path, a_jump, _ = a
    m_d, m_yaw = m[0], m[1]

    if r_jump > 0.05:
        print(f'\n!!! NIECIAGLOSC /dog_odom: skok {r_jump*100:.1f} cm miedzy '
              f'kolejnymi probkami (~1 kHz) => firmware ZRESETOWAL odometrie.')
        print('!!! Skala policzona z netto bedzie bezsensowna.')

    print('\n' + '=' * 66)
    print('SKALA ODOMETRII POD RUCHEM')
    print('=' * 66)
    print(f'czas ruchu: {b - start:.1f} s')
    print(f'{'tor':<26} {'dystans netto':>14} {'dyaw':>9}  {'sciezka':>13}')
    print('-' * 66)
    for lbl, d, y, pth in (('/dog_odom (surowa)', r_d, r_yaw, r_path),
                           ('TF po trans_scale', t_d, t_yaw, t_path),
                           ('/amcl_pose (prawda)', a_d, a_yaw, a_path)):
        print(f'{lbl:<26} {d:>13.3f} m {y:>+8.2f} st  sciezka {pth:6.2f} m')
    if r_d > 0.05 and r_path / max(r_d, 1e-6) > 1.6:
        print('  UWAGA: sciezka >> netto => trasa zawracala; iloraz netto'
              ' jest wtedy slaba miara skali.')
    print('-' * 66)
    if r_d and t_d:
        print(f'zastosowana skala (TF/raw)   = {t_d / r_d:.3f}'
              f'   [w configu: 1.890]')
    if r_d and a_d and r_d > 0.05:
        need = a_d / r_d
        print(f'SKALA WYMAGANA (amcl/raw)    = {need:.3f}   <== to powinno byc'
              f' w trans_scale')
        if t_d:
            err = (t_d - a_d) / max(a_d, 1e-6) * 100
            print(f'nadmiar/niedobor drogi w TF  = {err:+.1f}% '
                  f'({t_d - a_d:+.3f} m na tym odcinku)')
    if m_d is not None and a_d:
        print(f'dryf map->odom               = {m_d:.3f} m, {m_yaw:+.2f} st')
        print(f'  czyli {m_d / max(a_d, 1e-6):.3f} m korekty na metr drogi'
              f'  <== to widac jako "skan ucieka"')
    print('=' * 66, flush=True)


if __name__ == '__main__':
    main()
