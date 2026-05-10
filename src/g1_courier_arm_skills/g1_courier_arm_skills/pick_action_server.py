"""Action server for PickBox."""
from __future__ import annotations

import threading
import sys

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from g1_courier_msgs.action import PickBox

from ._ros_glue import ArmRosBundle, UNITREE_HG_OK, UNITREE_HG_ERR
from .arm_controller import ArmControllerConfig, ArmSkillAborted


class PickActionServer(Node):
    def __init__(self) -> None:
        super().__init__('pick_action_server')

        self.declare_parameter('arm_sdk_topic', '/arm_sdk')
        self.declare_parameter('lowstate_topic', '/lowstate')
        self.declare_parameter('grasp_tau_threshold_nm', 1.5)
        self.declare_parameter('control_dt_s', 0.02)
        self.declare_parameter('kp', 80.0)
        self.declare_parameter('kd', 2.0)
        # Strict on real robot, can be turned off in MuJoCo where tau_est
        # depends on whatever pose the fallen-over robot is currently in
        # rather than on grasping a real box.
        self.declare_parameter('require_grasp_verified', True)
        # Sim-only — see ArmControllerConfig.kinematic_mode docstring.
        self.declare_parameter('kinematic_mode', False)

        cfg = ArmControllerConfig(
            control_dt_s=float(self.get_parameter('control_dt_s').value),
            kp=float(self.get_parameter('kp').value),
            kd=float(self.get_parameter('kd').value),
            kinematic_mode=bool(self.get_parameter('kinematic_mode').value),
        )
        self._bundle = ArmRosBundle(
            self,
            arm_topic=str(self.get_parameter('arm_sdk_topic').value),
            lowstate_topic=str(self.get_parameter('lowstate_topic').value),
            grasp_threshold_nm=float(self.get_parameter('grasp_tau_threshold_nm').value),
            controller_config=cfg,
        )

        self._busy_lock = threading.Lock()
        self._action_server = ActionServer(
            self,
            PickBox,
            '/pick_box',
            execute_callback=self._execute,
            goal_callback=self._goal,
            cancel_callback=self._cancel,
        )
        self.get_logger().info('pick_action_server ready on /pick_box')

    def _goal(self, _request) -> GoalResponse:
        # Busy check happens in _execute via acquire(blocking=False); see ARCH §11.
        return GoalResponse.ACCEPT

    def _cancel(self, _goal_handle) -> CancelResponse:
        self._bundle.controller.stop()
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle: ServerGoalHandle) -> PickBox.Result:
        result = PickBox.Result()
        if not self._busy_lock.acquire(blocking=False):
            goal_handle.abort()
            result.success = False
            result.message = 'already busy'
            return result

        request = goal_handle.request
        sequence_name = request.sequence_name or 'pick_box'
        # box_pose may be empty (no offset) or filled by a preceding dock action.
        offset = None
        if request.box_pose.header.frame_id:
            p = request.box_pose.pose.position
            offset = (float(p.x), float(p.y), float(p.z))

        def on_phase(label: str, progress: float) -> None:
            fb = PickBox.Feedback()
            fb.phase = label
            fb.progress = float(progress)
            goal_handle.publish_feedback(fb)

        try:
            # Tell any other arm server (place) to release its hold thread
            # before we start moving — otherwise both publishers fight on
            # /lowcmd and arms ping-pong between two poses.
            self._bundle.announce_take_control()
            # Capture baseline before any motion - arms are still in resting pose.
            self._bundle.verifier.capture_baseline()
            self._bundle.controller.run_sequence(
                sequence_name,
                on_phase=on_phase,
                pose_offset_xyz=offset,
            )
            grasp_ok = self._bundle.verifier.verify_grasp()
            result.grasp_verified = bool(grasp_ok)
            require = bool(self.get_parameter('require_grasp_verified').value)
            accept = grasp_ok or not require
            result.success = bool(accept)
            if grasp_ok:
                result.message = 'pick complete'
            elif not require:
                result.message = 'pick complete (grasp verification disabled)'
            else:
                result.message = 'pick complete but grasp not verified'
            if accept:
                goal_handle.succeed()
            else:
                goal_handle.abort()
        except ArmSkillAborted as exc:
            result.success = False
            result.message = f'aborted: {exc}'
            goal_handle.abort()
        except Exception as exc:
            self.get_logger().error(f'pick failed: {exc}')
            result.success = False
            result.message = f'error: {exc}'
            goal_handle.abort()
        finally:
            self._busy_lock.release()
        return result


def main(args=None) -> None:
    if not UNITREE_HG_OK:
        sys.stderr.write(
            f'pick_action_server requires unitree_hg messages. Import error: {UNITREE_HG_ERR}\n'
        )
        return
    rclpy.init(args=args)
    node = PickActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
