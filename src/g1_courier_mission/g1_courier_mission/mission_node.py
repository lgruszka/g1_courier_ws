"""Behaviour-tree mission node for the two-table courier task.

Tree:

    root (Sequence, infinite loop via Decorator)
      |- pickup_phase (Sequence)
      |    |- SetCarry(off)
      |    |- NavigateTo(table_A.predock)
      |    |- DockTo(A, MODE_APRILTAG)         # tight tolerance for pick
      |    |- Pick
      |    |- SetCarry(on)
      |
      |- transfer_phase (Sequence)
      |    |- NavigateTo(table_B.predock)
      |    |- DockTo(B, MODE_LIDAR_LINE)       # camera occluded by box
      |    |- Place
      |    |- SetCarry(off)
      |    |- RetreatBy(0.5 m)
      |
      |- swap_tables                           # next cycle goes the other way
"""
from __future__ import annotations

import os
import sys
from typing import Dict

import py_trees
import py_trees_ros
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from g1_courier_msgs.action import DockToTable
from g1_courier_msgs.msg import MissionStatus

from .behaviors import (
    DockTo, NavigateTo, Pick, Place, RetreatBy, SetCarry, TableConfig,
)


class _MissionStatusPublisher:
    """Per-tick post-handler: detects cycle completion, publishes MissionStatus,
    and shuts down rclpy when `max_cycles > 0` and the limit is reached."""

    def __init__(self, node: Node, max_cycles: int) -> None:
        self._node = node
        self._max_cycles = int(max_cycles)
        self._cycle_count = 0
        self._was_success = False
        self._stopped = False
        self._started = node.get_clock().now().to_msg()
        self._pub = node.create_publisher(MissionStatus, '/mission_status', 10)
        # Read-only blackboard view of carry state (written by SetCarry).
        self._bb = py_trees.blackboard.Client(name='mission_status_publisher')
        self._bb.register_key(key='box_held', access=py_trees.common.Access.READ)

    def post_tick(self, tree: py_trees_ros.trees.BehaviourTree) -> None:
        if self._stopped:
            return
        root = tree.root
        # Cycle completion: rising edge from non-SUCCESS to SUCCESS at root.
        if root.status == py_trees.common.Status.SUCCESS:
            if not self._was_success:
                self._cycle_count += 1
                self._node.get_logger().info(f'cycle {self._cycle_count} complete')
                if 0 < self._max_cycles <= self._cycle_count:
                    self._node.get_logger().info(
                        f'max_cycles={self._max_cycles} reached — shutting down')
                    self._stopped = True
                    try:
                        tree.interrupt()
                    except Exception:
                        pass
                    rclpy.shutdown()
                    return
            self._was_success = True
        else:
            self._was_success = False

        # Find the currently RUNNING leaf (deepest behaviour without children).
        running: py_trees.behaviour.Behaviour | None = None
        for b in root.iterate():
            if b.status == py_trees.common.Status.RUNNING and not b.children:
                running = b
                break

        msg = MissionStatus()
        msg.current_state = running.name if running else root.status.name
        msg.cycle_count = self._cycle_count
        try:
            msg.box_held = bool(self._bb.box_held)
        except KeyError:
            # SetCarry has not been ticked yet → no box.
            msg.box_held = False
        msg.started_at = self._started
        if running:
            for cand in ('table_a', 'table_b'):
                if cand in running.name:
                    msg.current_target = cand
                    break
        self._pub.publish(msg)


def _load_waypoints(node: Node) -> Dict[str, TableConfig]:
    share = get_package_share_directory('g1_courier_mission')
    path = os.path.join(share, 'config', 'waypoints.yaml')
    with open(path, 'r') as f:
        data = yaml.safe_load(f)
    out: Dict[str, TableConfig] = {}
    for name, raw in data.get('tables', {}).items():
        mode = {
            'apriltag': DockToTable.Goal.MODE_APRILTAG,
            'lidar_line': DockToTable.Goal.MODE_LIDAR_LINE,
            'amcl_only': DockToTable.Goal.MODE_AMCL_ONLY,
        }[raw['dock_mode']]
        out[name] = TableConfig(
            name=name,
            apriltag_id=int(raw['apriltag_id']),
            predock_x=float(raw['predock_x']),
            predock_y=float(raw['predock_y']),
            predock_yaw=float(raw['predock_yaw']),
            dock_mode=mode,
            final_xy_tol_m=float(raw.get('final_xy_tol_m', 0.03)),
            final_yaw_tol_rad=float(raw.get('final_yaw_tol_rad', 0.05)),
        )
    node.get_logger().info(f'Loaded waypoints: {list(out.keys())}')
    return out


def _phase_pickup(node: Node, table: TableConfig) -> py_trees.composites.Sequence:
    seq = py_trees.composites.Sequence(name=f'pickup_at_{table.name}', memory=True)
    seq.add_children([
        SetCarry('carry_off_for_pick', node, carrying=False),
        NavigateTo(
            f'navigate_to_{table.name}', node,
            frame_id='map', x=table.predock_x, y=table.predock_y, yaw=table.predock_yaw,
            waypoint_name=f'{table.name}_predock',
        ),
        DockTo(
            f'dock_to_{table.name}', node,
            mode=table.dock_mode, tag_id=table.apriltag_id,
            xy_tol_m=table.final_xy_tol_m, yaw_tol_rad=table.final_yaw_tol_rad,
        ),
        Pick(f'pick_at_{table.name}', node),
        SetCarry('carry_on_after_pick', node, carrying=True),
    ])
    return seq


# Side via-point used by both transfer phases (A->B and B->A). Without a real
# planner the straight line between table_a and table_b cuts through table_a's
# body — robot collides / gets stuck. Detour through y=+0.8 m: that's 0.2 m
# off table_a's lateral half-extent (0.6 m) so the welded-pelvis robot slides
# cleanly past on the +Y side. Loose tolerances (15 cm / ~11°) because the
# via is just a transit point, not a precise pose target.
_TRANSIT_VIA_X = 1.50
_TRANSIT_VIA_Y = 0.80
_TRANSIT_VIA_YAW = 0.0
_TRANSIT_XY_TOL = 0.15
_TRANSIT_YAW_TOL = 0.20


def _phase_transfer(node: Node, table: TableConfig) -> py_trees.composites.Sequence:
    seq = py_trees.composites.Sequence(name=f'transfer_to_{table.name}', memory=True)
    seq.add_children([
        NavigateTo(
            f'transit_via_to_{table.name}', node,
            frame_id='map',
            x=_TRANSIT_VIA_X, y=_TRANSIT_VIA_Y, yaw=_TRANSIT_VIA_YAW,
            waypoint_name='transit_via',
            xy_tolerance_m=_TRANSIT_XY_TOL,
            yaw_tolerance_rad=_TRANSIT_YAW_TOL,
        ),
        NavigateTo(
            f'navigate_to_{table.name}', node,
            frame_id='map', x=table.predock_x, y=table.predock_y, yaw=table.predock_yaw,
            waypoint_name=f'{table.name}_predock',
        ),
        DockTo(
            f'dock_to_{table.name}', node,
            mode=table.dock_mode, tag_id=table.apriltag_id,
            xy_tol_m=table.final_xy_tol_m, yaw_tol_rad=table.final_yaw_tol_rad,
        ),
        Place(f'place_at_{table.name}', node),
        SetCarry('carry_off_after_place', node, carrying=False),
        RetreatBy('retreat_after_place', node, distance_m=0.5),
    ])
    return seq


def build_tree(node: Node, waypoints: Dict[str, TableConfig]) -> py_trees.composites.Sequence:
    """Build a single A->B->A cycle. Run twice for a forward+return mission."""
    table_a = waypoints['table_a']
    table_b = waypoints['table_b']
    cycle = py_trees.composites.Sequence(name='courier_cycle', memory=True)
    cycle.add_children([
        # forward leg
        _phase_pickup(node, table_a),
        _phase_transfer(node, table_b),
        # return leg
        _phase_pickup(node, table_b),
        _phase_transfer(node, table_a),
    ])
    return cycle


def main(args=None) -> None:
    rclpy.init(args=args)
    node = rclpy.create_node('mission_node')
    node.declare_parameter('tick_period_ms', 500)
    node.declare_parameter('max_cycles', 0)
    tick_period_ms = max(50, int(node.get_parameter('tick_period_ms').value))
    max_cycles = int(node.get_parameter('max_cycles').value)

    try:
        waypoints = _load_waypoints(node)
    except Exception as exc:
        node.get_logger().error(f'Failed to load waypoints: {exc}')
        sys.exit(1)

    root = build_tree(node, waypoints)
    tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=False)

    try:
        tree.setup(node=node, timeout=15.0)
    except py_trees_ros.exceptions.TimedOutError as exc:
        node.get_logger().error(f'BT setup timed out: {exc}')
        sys.exit(1)

    status_pub = _MissionStatusPublisher(node, max_cycles=max_cycles)
    tree.add_post_tick_handler(status_pub.post_tick)

    tree.tick_tock(period_ms=tick_period_ms)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            tree.shutdown()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
