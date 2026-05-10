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

from geometry_msgs.msg import PoseStamped

from g1_courier_msgs.action import DockToTable
from g1_courier_msgs.msg import MissionStatus

from .behaviors import (
    DockTo, NavigateTo, Pause, Pick, Place, RetreatBy, SetCarry, TableConfig,
)


# Sleep between mission stages so the robot visibly stops at each transition
# (nav → dock → pick → carry, etc.). Plus emits `[STAGE PAUSE]` log line.
STAGE_PAUSE_S = 0.2


# AprilTag id stuck on the box front face (added in Faza 1.5+ scene XML
# update). pickup uses APRILTAG dock to this tag for grasp alignment.
PARCEL_TAG_ID = 10
# target_distance is along camera's optical axis (depth in cam frame), not
# horizontal world distance. With head_cam pitched 42° down + cam world Z=1.20
# vs box tag10 Z=0.975, depth-to-tag has both horizontal and vertical
# components. Dropping to 0.17 puts pelvis too far forward (wrist 5cm past
# box center) AND places tag at -41° below cam axis (outside ±30° FoV).
# 0.25 puts pelvis at ~0.87 → wrist at ~1.15 (front quarter of box, palm-
# press grips OK) and tag at -18° (well within FoV for tracking during servo).
PARCEL_DOCK_DISTANCE_M = 0.50


def _tag_relative_pose(z: float) -> PoseStamped:
    p = PoseStamped()
    p.header.frame_id = 'tag'
    p.pose.position.z = float(z)
    p.pose.orientation.w = 1.0
    return p


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
    """Pickup at table_X — head_cam unoccluded:

    1. nav2 brings robot roughly to predock (~0.60 m before table edge)
    2. APRILTAG dock to box tag10 — precise grasp alignment via the box
       itself (the table has NO tag — pickup pose is box-relative)
    3. Pick sequence — palm-press friction holds the box
    """
    seq = py_trees.composites.Sequence(name=f'pickup_at_{table.name}', memory=True)
    seq.add_children([
        SetCarry('carry_off_for_pick', node, carrying=False),
        NavigateTo(
            f'navigate_to_{table.name}', node,
            frame_id='map', x=table.predock_x, y=table.predock_y, yaw=table.predock_yaw,
            waypoint_name=f'{table.name}_predock',
        ),
        Pause(f'pause_after_nav_{table.name}', node, duration_s=STAGE_PAUSE_S),
        DockTo(
            f'dock_to_parcel_{table.name}', node,
            mode=DockToTable.Goal.MODE_APRILTAG,
            tag_id=PARCEL_TAG_ID,
            target_pose=_tag_relative_pose(z=PARCEL_DOCK_DISTANCE_M),
            # Loose tolerance — detection→cmd latency causes ~10 cm overshoot
            # at convergence. Plus PnP returns garbage when tag corners crop
            # at FoV edges (close approach). Tolerance 0.10/0.15 lets dock
            # declare done before tag exits FoV.
            xy_tol_m=0.10, yaw_tol_rad=0.15,
        ),
        Pause(f'pause_before_pick_{table.name}', node, duration_s=STAGE_PAUSE_S),
        Pick(f'pick_at_{table.name}', node),
        Pause(f'pause_after_pick_{table.name}', node, duration_s=STAGE_PAUSE_S),
        SetCarry('carry_on_after_pick', node, carrying=True),
        # Retreat 0.5 m back from table — after box-relative dock + pick,
        # robot pelvis is ~0.10 m from table front edge (well inside the
        # global_costmap inflation_radius=0.40). Without this retreat the
        # next NavigateTo (transfer to other table) fails: planner returns
        # "Failed to create plan" because start pose is in lethal-cost zone.
        RetreatBy(f'retreat_after_pick_{table.name}', node, distance_m=0.5),
    ])
    return seq


def _phase_transfer(node: Node, table: TableConfig) -> py_trees.composites.Sequence:
    """Transfer to table_X with box — head_cam occluded by carried box,
    so use LIDAR_LINE (RANSAC line fit on pelvis-mounted 360° lidar — box-
    independent). nav2 plans smooth detour around tables; place sequence
    lowers box onto table, palm-press releases via friction physics."""
    seq = py_trees.composites.Sequence(name=f'transfer_to_{table.name}', memory=True)
    seq.add_children([
        NavigateTo(
            f'navigate_to_{table.name}', node,
            frame_id='map', x=table.predock_x, y=table.predock_y, yaw=table.predock_yaw,
            waypoint_name=f'{table.name}_predock',
        ),
        Pause(f'pause_after_nav_to_{table.name}', node, duration_s=STAGE_PAUSE_S),
        DockTo(
            f'dock_to_{table.name}', node,
            mode=DockToTable.Goal.MODE_LIDAR_LINE,
            tag_id=-1,  # unused for LIDAR_LINE
            xy_tol_m=table.final_xy_tol_m, yaw_tol_rad=table.final_yaw_tol_rad,
        ),
        Pause(f'pause_before_place_{table.name}', node, duration_s=STAGE_PAUSE_S),
        Place(f'place_at_{table.name}', node),
        Pause(f'pause_after_place_{table.name}', node, duration_s=STAGE_PAUSE_S),
        SetCarry('carry_off_after_place', node, carrying=False),
        # 1.0 m retreat (was 0.5) — after place robot pelvis ~0.93 m from table
        # front; with inflation 0.35 around table, planner needs ≥0.40 m gap to
        # plan next cycle. 1.0 m retreat puts robot at ~−0.07 m (well clear)
        # with box still in head_cam FoV for next dock approach.
        RetreatBy('retreat_after_place', node, distance_m=1.0),
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
