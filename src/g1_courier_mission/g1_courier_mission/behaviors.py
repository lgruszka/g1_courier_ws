"""Reusable py_trees behaviours that wrap the project's actions and services.

These are intentionally thin: each behaviour calls one action/service and
returns SUCCESS / FAILURE / RUNNING. Mission logic lives in the tree built in
mission_node.py.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import py_trees
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Quaternion

from g1_courier_msgs.action import (
    DockToTable, NavigateToPose, PickBox, PlaceBox, Retreat,
)
from g1_courier_msgs.srv import SetCarryMode, SetFreeze


# ---------- helpers ----------

def yaw_to_quat(yaw_rad: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw_rad * 0.5)
    q.w = math.cos(yaw_rad * 0.5)
    return q


def make_pose_stamped(frame_id: str, x: float, y: float, yaw_rad: float) -> PoseStamped:
    p = PoseStamped()
    p.header.frame_id = frame_id
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation = yaw_to_quat(yaw_rad)
    return p


# ---------- generic action behavior ----------

class _ActionBehaviour(py_trees.behaviour.Behaviour):
    """Base: send goal in initialise(), tick by polling result_future."""

    action_type = None
    action_name = ''

    def __init__(self, name: str, node: Node, build_goal):
        super().__init__(name)
        self._node = node
        self._build_goal = build_goal
        self._client: Optional[ActionClient] = None
        self._goal_handle = None
        self._result_future = None

    def setup(self, **kwargs) -> None:
        self._client = ActionClient(self._node, self.action_type, self.action_name)

    def initialise(self) -> None:
        self._goal_handle = None
        self._result_future = None
        if not self._client.wait_for_server(timeout_sec=1.0):
            return
        goal = self._build_goal()
        send_future = self._client.send_goal_async(goal)
        send_future.add_done_callback(self._on_send_done)

    def _on_send_done(self, future) -> None:
        handle = future.result()
        if handle is None or not handle.accepted:
            self._goal_handle = None
            return
        self._goal_handle = handle
        self._result_future = handle.get_result_async()

    def update(self) -> py_trees.common.Status:
        if self._client is None or not self._client.server_is_ready():
            return py_trees.common.Status.FAILURE
        if self._goal_handle is None and self._result_future is None:
            return py_trees.common.Status.RUNNING
        if self._result_future is None:
            return py_trees.common.Status.RUNNING
        if not self._result_future.done():
            return py_trees.common.Status.RUNNING
        outcome = self._result_future.result()
        ok = bool(getattr(outcome.result, 'success', False))
        msg = getattr(outcome.result, 'message', '')
        self._node.get_logger().info(f'{self.name}: {"OK" if ok else "FAIL"} - {msg}')
        return py_trees.common.Status.SUCCESS if ok else py_trees.common.Status.FAILURE

    def terminate(self, new_status: py_trees.common.Status) -> None:
        if (new_status == py_trees.common.Status.INVALID
                and self._goal_handle is not None):
            self._goal_handle.cancel_goal_async()


class NavigateTo(_ActionBehaviour):
    action_type = NavigateToPose
    action_name = '/courier/navigate_to_pose'

    def __init__(self, name: str, node: Node, *, frame_id: str, x: float, y: float, yaw: float,
                 waypoint_name: str = '', timeout_s: float = 60.0,
                 xy_tolerance_m: float = 0.0, yaw_tolerance_rad: float = 0.0):
        def build():
            g = NavigateToPose.Goal()
            g.target_pose = make_pose_stamped(frame_id, x, y, yaw)
            g.waypoint_name = waypoint_name or name
            g.timeout_s = float(timeout_s)
            # 0 means "use the nav node's default tolerance" (per the .action
            # contract). Pass-through values allow loose via-points.
            g.xy_tolerance_m = float(xy_tolerance_m)
            g.yaw_tolerance_rad = float(yaw_tolerance_rad)
            return g
        super().__init__(name, node, build)


class DockTo(_ActionBehaviour):
    action_type = DockToTable
    action_name = '/dock_to_table'

    def __init__(self, name: str, node: Node, *, mode: int, tag_id: int = -1,
                 target_pose: Optional[PoseStamped] = None,
                 xy_tol_m: float = 0.03, yaw_tol_rad: float = 0.05, timeout_s: float = 30.0):
        def build():
            g = DockToTable.Goal()
            g.mode = int(mode)
            g.apriltag_id = int(tag_id)
            g.target_pose = target_pose or PoseStamped()
            g.xy_tolerance_m = float(xy_tol_m)
            g.yaw_tolerance_rad = float(yaw_tol_rad)
            g.timeout_s = float(timeout_s)
            return g
        super().__init__(name, node, build)


class Pick(_ActionBehaviour):
    action_type = PickBox
    action_name = '/pick_box'

    def __init__(self, name: str, node: Node, *, box_pose: Optional[PoseStamped] = None,
                 timeout_s: float = 30.0):
        def build():
            g = PickBox.Goal()
            g.box_pose = box_pose or PoseStamped()
            g.sequence_name = 'pick_box'
            g.timeout_s = float(timeout_s)
            return g
        super().__init__(name, node, build)


class Place(_ActionBehaviour):
    action_type = PlaceBox
    action_name = '/place_box'

    def __init__(self, name: str, node: Node, *, target_pose: Optional[PoseStamped] = None,
                 timeout_s: float = 30.0):
        def build():
            g = PlaceBox.Goal()
            g.target_pose = target_pose or PoseStamped()
            g.sequence_name = 'place_box'
            g.timeout_s = float(timeout_s)
            return g
        super().__init__(name, node, build)


class RetreatBy(_ActionBehaviour):
    action_type = Retreat
    action_name = '/retreat'

    def __init__(self, name: str, node: Node, *, distance_m: float, speed_mps: float = 0.15,
                 timeout_s: float = 10.0):
        def build():
            g = Retreat.Goal()
            g.distance_m = float(distance_m)
            g.speed_mps = float(speed_mps)
            g.timeout_s = float(timeout_s)
            return g
        super().__init__(name, node, build)


# ---------- service behaviors ----------

class SetCarry(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, node: Node, *, carrying: bool):
        super().__init__(name)
        self._node = node
        self._carrying = bool(carrying)
        self._client = node.create_client(SetCarryMode, '/safety/set_carry_mode')
        self._future = None
        # Mirror the carry state on the blackboard so MissionStatus can read it.
        self._bb = self.attach_blackboard_client(name=name)
        self._bb.register_key(key='box_held', access=py_trees.common.Access.WRITE)

    def initialise(self) -> None:
        if not self._client.wait_for_service(timeout_sec=1.0):
            self._future = None
            return
        req = SetCarryMode.Request()
        req.carrying = self._carrying
        self._future = self._client.call_async(req)

    def update(self) -> py_trees.common.Status:
        if self._future is None:
            return py_trees.common.Status.FAILURE
        if not self._future.done():
            return py_trees.common.Status.RUNNING
        # Service ack received — record the new state on blackboard.
        self._bb.box_held = self._carrying
        return py_trees.common.Status.SUCCESS


class SetFreezeMode(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, node: Node, *, freeze: bool):
        super().__init__(name)
        self._node = node
        self._freeze = bool(freeze)
        self._client = node.create_client(SetFreeze, '/safety/set_freeze')
        self._future = None

    def initialise(self) -> None:
        if not self._client.wait_for_service(timeout_sec=1.0):
            self._future = None
            return
        req = SetFreeze.Request()
        req.freeze = self._freeze
        self._future = self._client.call_async(req)

    def update(self) -> py_trees.common.Status:
        if self._future is None:
            return py_trees.common.Status.FAILURE
        if not self._future.done():
            return py_trees.common.Status.RUNNING
        return py_trees.common.Status.SUCCESS


# ---------- sequence helper ----------

@dataclass
class TableConfig:
    name: str
    apriltag_id: int
    predock_x: float
    predock_y: float
    predock_yaw: float
    dock_mode: int            # DockToTable.Goal.MODE_*
    final_xy_tol_m: float = 0.03
    final_yaw_tol_rad: float = 0.05
