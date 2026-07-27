#!/usr/bin/env python3
"""A1: rozrzut dojazdu — dokladnosc i POWTARZALNOSC osiagania celu.

  python3 goal_spread.py [liczba_prob] [max_sekund]

Protokol: wyslij TEN SAM cel N razy, kazdorazowo podjezdzajac z innej strony
(odjedz pilotem albo wyslij cel pomocniczy, potem wroc na cel mierzony).
Sonda sama rozpoznaje kolejne proby: nowy /goal_pose -> ruch -> postoj 3 s.

Raportuje ROZDZIELNIE dwie rzeczy, ktore lacznie sie myli:
  DOKLADNOSC    — odleglosc konca od ZADANEGO punktu (limit: xy_goal_tolerance)
  POWTARZALNOSC — rozrzut koncow wzgledem ich wlasnego srodka (to jest liczba,
                  od ktorej powinny zalezec tolerancje dokowania)

Cele oddalone od siebie > GROUP_M traktuje jako OSOBNE punkty i grupuje osobno,
zeby przypadkowy inny cel nie zafalszowal rozrzutu.
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy,
                       qos_profile_sensor_data)
from geometry_msgs.msg import PoseWithCovarianceStamped

N_TARGET = int(sys.argv[1]) if len(sys.argv) > 1 else 5
MAX_S = float(sys.argv[2]) if len(sys.argv) > 2 else 1200.0
IDLE_S = 3.0
MOVE_EPS = 0.02
GROUP_M = 0.5


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def ang_diff(a, b):
    return math.degrees(math.atan2(math.sin(a - b), math.cos(a - b)))


class Spread(Node):
    def __init__(self):
        super().__init__('goal_spread')
        self.goal = None          # (x, y, yaw) aktualnie zadany
        self.amcl = None          # (x, y, yaw) ostatnia poza
        self.start_pose = None    # poza w momencie przyjecia celu (kierunek podejscia)
        self.last_move = None
        self.moved = False
        self.rows = []            # (goal, final, start_pose)
        self.create_subscription(PoseStamped, '/goal_pose', self.cb_goal, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.cb_amcl,
            QoSProfile(depth=5, reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Twist, '/cmd_vel', self.cb_cmd, 10)

    def cb_goal(self, m):
        self.goal = (m.pose.position.x, m.pose.position.y, yaw_of(m.pose.orientation))
        self.start_pose = self.amcl
        self.moved = False
        self.last_move = None
        print(f'\n--- proba {len(self.rows)+1}: cel ({self.goal[0]:+.3f}, '
              f'{self.goal[1]:+.3f})', flush=True)

    def cb_amcl(self, m):
        p = m.pose.pose
        self.amcl = (p.position.x, p.position.y, yaw_of(p.orientation))

    def cb_cmd(self, m):
        if max(abs(m.linear.x), abs(m.linear.y), abs(m.angular.z)) > MOVE_EPS:
            self.moved = True
            self.last_move = time.time()

    def maybe_close(self):
        if not (self.goal and self.moved and self.last_move and self.amcl):
            return
        if time.time() - self.last_move < IDLE_S:
            return
        g, f, s = self.goal, self.amcl, self.start_pose
        self.rows.append((g, f, s))
        exy = math.hypot(f[0] - g[0], f[1] - g[1])
        eyaw = ang_diff(f[2], g[2])
        appr = ''
        if s:
            appr = f'  podejscie z kierunku {math.degrees(math.atan2(g[1]-s[1], g[0]-s[0])):+7.1f} st'
        print(f'    koniec ({f[0]:+.3f}, {f[1]:+.3f})  blad xy {exy*100:6.1f} cm'
              f'  dyaw {eyaw:+6.1f} st{appr}', flush=True)
        self.goal = None
        self.moved = False
        if len(self.rows) >= 2:
            report(self.rows)       # raport po KAZDEJ probie — przebieg moze sie urwac


def report(rows):
    print('\n' + '=' * 70)
    print('A1 — ROZRZUT DOJAZDU')
    print('=' * 70)
    if len(rows) < 2:
        print(f'za malo prob ({len(rows)}) — potrzebne min. 2')
        return
    # grupowanie po zadanym punkcie
    groups = []
    for g, f, s in rows:
        for grp in groups:
            if math.hypot(g[0] - grp['g'][0], g[1] - grp['g'][1]) < GROUP_M:
                grp['items'].append((g, f, s))
                break
        else:
            groups.append({'g': g, 'items': [(g, f, s)]})

    for gi, grp in enumerate(groups, 1):
        it = grp['items']
        print(f'\nPUNKT {gi}: zadany ({grp["g"][0]:+.3f}, {grp["g"][1]:+.3f})'
              f'   prob: {len(it)}')
        exys = [math.hypot(f[0]-g[0], f[1]-g[1]) for g, f, _ in it]
        eyaws = [abs(ang_diff(f[2], g[2])) for g, f, _ in it]
        print(f'  DOKLADNOSC (wzgl. zadanego):  sr {sum(exys)/len(exys)*100:.1f} cm'
              f'   max {max(exys)*100:.1f} cm   dyaw max {max(eyaws):.1f} st')
        if len(it) < 2:
            continue
        cx = sum(f[0] for _, f, _ in it) / len(it)
        cy = sum(f[1] for _, f, _ in it) / len(it)
        dev = [math.hypot(f[0]-cx, f[1]-cy) for _, f, _ in it]
        rms = math.sqrt(sum(d*d for d in dev) / len(dev))
        pairs = [math.hypot(it[i][1][0]-it[j][1][0], it[i][1][1]-it[j][1][1])
                 for i in range(len(it)) for j in range(i+1, len(it))]
        print(f'  POWTARZALNOSC (wzgl. srodka):  RMS {rms*100:.1f} cm'
              f'   max odchylka {max(dev)*100:.1f} cm'
              f'   najdalsza para {max(pairs)*100:.1f} cm')
        print(f'  >>> {"SPELNIA" if max(dev) < 0.15 else "NIE SPELNIA"}'
              f' kryterium A1 (max odchylka < 15 cm)')
        print(f'  >>> tolerancja doku powinna byc >= {max(pairs)*100:.0f} cm'
              f' (najdalsza para), z zapasem')
    print('=' * 70, flush=True)


def main():
    rclpy.init()
    n = Spread()
    print(f'A1: czekam na {N_TARGET} prob (ten sam cel, rozne kierunki podejscia).')
    print('Wysylaj cel z Foxglove; sonda sama rozpozna zakonczenie kazdej proby.')
    print(f'Limit czasu {MAX_S:.0f} s. Ctrl-C konczy i wypisuje raport.\n', flush=True)
    end = time.time() + MAX_S
    try:
        while time.time() < end and rclpy.ok() and len(n.rows) < N_TARGET:
            rclpy.spin_once(n, timeout_sec=0.2)
            n.maybe_close()
    except KeyboardInterrupt:
        pass
    report(n.rows)


if __name__ == '__main__':
    main()
