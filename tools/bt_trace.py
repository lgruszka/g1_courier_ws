#!/usr/bin/env python3
"""Diagnostyka drzewa BT nav2: dlaczego cel jest przerywany bez recovery.

Nasluchuje /behavior_tree_log (nav2 publikuje tam KAZDE przejscie statusu
wezla drzewa) i wysyla cel NIEMOZLIWY do zaplanowania (poza mapa) => planner
pada na pierwszym ticku, wiec robot NIE IDZIE (gait_manager moze na ~1 s
wlaczyc stepowanie, bo reaguje na /goal_pose).

Szukamy wezla, ktory zwraca FAILURE zamiast oddac sterowanie do
RecoveryFallback (Spin/Wait/BackUp) — te behaviory sa sprawne (zweryfikowane
osobno przez /spin), wiec winowajca jest w logice drzewa.
"""
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.msg import BehaviorTreeLog
from rclpy.node import Node

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0
# Cel daleko poza mapa => NavFn nie ma jak zaplanowac, zero ruchu robota.
GOAL_X = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0
GOAL_Y = float(sys.argv[3]) if len(sys.argv) > 3 else 60.0


class Trace(Node):
    def __init__(self):
        super().__init__('bt_trace')
        self.rows = []
        self.create_subscription(BehaviorTreeLog, '/behavior_tree_log',
                                self.cb, 10)
        self.pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.t0 = time.time()
        self.sent = False
        self.create_timer(0.25, self.tick)

    def cb(self, msg):
        for e in msg.event_log:
            self.rows.append((time.time() - self.t0, e.node_name,
                              e.previous_status, e.current_status))

    def tick(self):
        if self.sent:
            return
        if time.time() - self.t0 < 1.5:
            return          # daj chwile na subskrypcje /behavior_tree_log
        g = PoseStamped()
        g.header.frame_id = 'map'
        g.header.stamp = self.get_clock().now().to_msg()
        g.pose.position.x = GOAL_X
        g.pose.position.y = GOAL_Y
        g.pose.orientation.w = 1.0
        self.pub.publish(g)
        self.sent = True
        print(f'--> cel wyslany: ({GOAL_X}, {GOAL_Y}) w ramce map '
              f'(poza mapa => planner pada od razu)\n', flush=True)


def main():
    rclpy.init()
    n = Trace()
    print(f'nasluchuje /behavior_tree_log przez {DUR:.0f} s...\n', flush=True)
    end = time.time() + DUR
    while time.time() < end and rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.2)

    print('=' * 72)
    print('PRZEJSCIA STATUSOW W DRZEWIE (czas od startu)')
    print('=' * 72)
    if not n.rows:
        print('BRAK ZDARZEN — bt_navigator nie publikuje /behavior_tree_log')
        print('(albo cel nie dotarl; sprawdz czy /goal_pose ma subskrybenta)')
    for t, name, prev, cur in n.rows:
        mark = '  <== FAILURE' if cur == 'FAILURE' else ''
        print(f'{t:7.3f}s  {name:<28} {prev:>8} -> {cur:<8}{mark}')
    print('=' * 72)
    fails = [r for r in n.rows if r[3] == 'FAILURE']
    if fails:
        print('Wezly, ktore zwrocily FAILURE (kolejnosc = propagacja w gore):')
        for t, name, _, _ in fails:
            print(f'   {t:7.3f}s  {name}')
    ran_recovery = [r for r in n.rows
                    if r[1] in ('RecoveryActions', 'ClearingActions', 'Spin',
                                'Wait', 'BackUp', 'RecoveryFallback')]
    print()
    print('Czy drzewo siegnelo po recovery: '
          + ('TAK' if ran_recovery else 'NIE — poddalo sie wczesniej'))
    print('=' * 72, flush=True)


if __name__ == '__main__':
    main()
