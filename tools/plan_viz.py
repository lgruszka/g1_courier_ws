#!/usr/bin/env python3
"""nav2 plan + AMCL pose + global_costmap inflation visualizer.

Subscribes /map (transient_local), /global_costmap/costmap, /plan, /amcl_pose.
Renders map+inflation+plan+pose to /tmp/nav_plan.png on every plan update.

Run alongside phase1_full / real.launch and refresh image to see what
nav2 routed (live, with auto-reload viewer like feh):

    python3 tools/plan_viz.py &
    feh --reload 1 /tmp/nav_plan.png
"""
from __future__ import annotations

import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseWithCovarianceStamped


class PlanViz(Node):
    def __init__(self) -> None:
        super().__init__('plan_viz')
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 self._on_costmap, map_qos)
        self.create_subscription(Path, '/plan', self._on_plan, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self._on_pose, 10)
        self.map: OccupancyGrid | None = None
        self.costmap: OccupancyGrid | None = None
        self.plan: Path | None = None
        self.pose: PoseWithCovarianceStamped | None = None
        self.dirty = False
        self.create_timer(0.5, self._tick)
        self.get_logger().info(
            'plan_viz waiting for /map, /global_costmap/costmap, /plan, /amcl_pose')

    def _on_map(self, m): self.map = m; self.dirty = True
    def _on_costmap(self, m): self.costmap = m; self.dirty = True
    def _on_plan(self, p): self.plan = p; self.dirty = True
    def _on_pose(self, p): self.pose = p; self.dirty = True

    def _tick(self) -> None:
        if not self.dirty or self.map is None:
            return
        self.dirty = False

        plt.figure(figsize=(12, 8))

        # Static map (gray walls)
        m = self.map
        H, W = m.info.height, m.info.width
        res = m.info.resolution
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        arr = np.array(m.data, dtype=np.int8).reshape(H, W)
        img = np.full((H, W), 0.85)
        img[arr == 0] = 1.0
        img[arr == 100] = 0.0
        plt.imshow(img, cmap='gray', origin='lower',
                   extent=[ox, ox + W * res, oy, oy + H * res], vmin=0, vmax=1)

        # Inflation overlay (orange halo around obstacles)
        if self.costmap is not None:
            cm = self.costmap
            cH, cW = cm.info.height, cm.info.width
            cres = cm.info.resolution
            cox, coy = cm.info.origin.position.x, cm.info.origin.position.y
            carr = np.array(cm.data, dtype=np.int8).reshape(cH, cW)
            overlay = np.zeros((cH, cW, 4))
            inflated = (carr > 0) & (carr < 100)
            lethal = (carr >= 100)
            overlay[inflated] = [1.0, 0.5, 0.0, 0.35]
            overlay[lethal] = [1.0, 0.0, 0.0, 0.6]
            plt.imshow(overlay, origin='lower',
                       extent=[cox, cox + cW * cres, coy, coy + cH * cres])

        n_plan = 0
        if self.plan is not None and self.plan.poses:
            xs = [p.pose.position.x for p in self.plan.poses]
            ys = [p.pose.position.y for p in self.plan.poses]
            plt.plot(xs, ys, 'r-', lw=2, label=f'nav2 plan ({len(xs)} pts)')
            plt.plot(xs[0], ys[0], 'ro', markersize=8)
            plt.plot(xs[-1], ys[-1], 'rx', markersize=12, mew=3)
            n_plan = len(xs)

        if self.pose is not None:
            x = self.pose.pose.pose.position.x
            y = self.pose.pose.pose.position.y
            qz = self.pose.pose.pose.orientation.z
            qw = self.pose.pose.pose.orientation.w
            yaw = 2.0 * math.atan2(qz, qw)
            plt.plot(x, y, 'b*', markersize=18, label='AMCL pose')
            plt.arrow(x, y, 0.3 * math.cos(yaw), 0.3 * math.sin(yaw),
                      head_width=0.08, fc='blue', ec='blue')

        plt.legend(loc='upper left')
        plt.title(f'nav2 plan: {n_plan} waypoints')
        plt.gca().set_aspect('equal')
        plt.grid(True, alpha=0.3)
        plt.xlabel('x [m]')
        plt.ylabel('y [m]')
        plt.savefig('/tmp/nav_plan.png', dpi=90, bbox_inches='tight')
        plt.close()


def main() -> None:
    rclpy.init()
    node = PlanViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
