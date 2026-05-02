"""Phase 1 smoke launch — mission BT with REAL pick/place against MuJoCo.

PREREQUISITE: unitree_mujoco must be running on a separate host that can
reach this one over DDS. In our setup that is the macOS host on the
Parallels Shared Network (10.211.55.2 ↔ 10.211.55.11), publishing /lowstate
and subscribing /lowcmd. See memory/project_macos_mujoco_bridge.md.

Sim-only knobs versus the real-robot path (courier_full.launch.py):
  - arm_sdk_topic:=/lowcmd  (real default: /arm_sdk via firmware)
  - require_grasp_verified:=False, require_release_verified:=False
    (real τ_est detection is meaningful; MuJoCo τ_est here mirrors the
    welded-pelvis pose configuration, not actual parcel weight)
  - kinematic_mode:=True   (real default False = torque PD on motors;
    in sim our patched mac bridge interprets motor_cmd[i].mode==99 as
    "snap data.qpos[i] directly", giving fluid arm motion equivalent
    to g1_logistics_demo's kinematic_mode = True)
  - control_dt_s:=0.005    (200 Hz publish; real default 0.02 = 50 Hz)

Differences from phase0:
  - sim_lowstate_publisher_node REMOVED (mac MuJoCo bridge supplies /lowstate)
  - fake_pick / fake_place REPLACED by REAL action servers with sim-only
    parameters listed above
  - retreat is REAL retreat_action_server
  - navigate / dock stay fake — Faza 1.1 (AprilTag in scene + camera)
    and Faza 1.5 (nav2 + AMCL on a real LiDAR map) replace these.

Expected: full A↔B cycle in ~70 s, smooth pick/place visible in the mac
MuJoCo viewer, BT auto-restarts cycles via tick_tock.
"""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


# Fakes for nav/dock/retreat — short durations so the cycle isn't dominated
# by waiting on phases that aren't the focus of this test.
FAKE_DURATION_S = 1.0


def _fake(executable: str) -> Node:
    return Node(
        package='g1_courier_sim',
        executable=executable,
        name=executable,
        parameters=[{'duration_s': FAKE_DURATION_S, 'succeed': True}],
    )


def generate_launch_description() -> LaunchDescription:
    safety_share = get_package_share_directory('g1_courier_safety')

    return LaunchDescription([
        # Sim base only — kinematic TF + /odom for nav-related behaviors.
        # /lowstate intentionally NOT published here; comes from MuJoCo bridge.
        Node(package='g1_courier_sim', executable='sim_cmd_vel_bridge_node',
             name='sim_cmd_vel_bridge_node'),

        # Real arbiter so /safety/set_carry_mode service exists.
        Node(package='g1_courier_safety', executable='cmd_vel_arbiter',
             name='cmd_vel_arbiter',
             parameters=[os.path.join(safety_share, 'config', 'safety.yaml')]),

        # NOTE: no walking controller. Locomotion in sim is replaced by a
        # pelvis weld pin in mac-side scene.xml (see PELVIS_WELD_PATCH.md).
        # Real walking (Faza 1.3) is a separate sub-phase requiring either
        # custom RL retraining with arm-force disturbance, or whole-body MPC.

        # Real arm action servers, talking to the MuJoCo bridge on /lowcmd.
        Node(package='g1_courier_arm_skills', executable='pick_action_server',
             name='pick_action_server',
             parameters=[{
                 'arm_sdk_topic': '/lowcmd',
                 'lowstate_topic': '/lowstate',
                 'require_grasp_verified': False,
                 # Sim-only: bump publish rate to 200 Hz so the kinematic
                 # writes (mode=99 in motor_cmd) keep the qpos stream
                 # tight under the smoothstep trajectory.
                 'control_dt_s': 0.005,
                 # Sim-only escape hatch: tells the patched mac bridge to
                 # set data.qpos[arm_joints] directly instead of running
                 # PD via data.ctrl. Mirrors g1_logistics_demo's
                 # kinematic_mode = True. Keep False on the real robot.
                 'kinematic_mode': True,
             }]),
        Node(package='g1_courier_arm_skills', executable='place_action_server',
             name='place_action_server',
             parameters=[{
                 'arm_sdk_topic': '/lowcmd',
                 'lowstate_topic': '/lowstate',
                 'require_release_verified': False,
                 # Sim-only — see pick_action_server above for rationale.
                 'control_dt_s': 0.005,
                 'kinematic_mode': True,
             }]),

        # Fake nav and dock (no nav2 / AprilTag / line-fit yet).
        _fake('fake_navigate_proxy'),
        _fake('fake_dock_action_server'),

        # Real retreat — open-loop backward drive on /cmd_vel_retreat.
        # Arbiter routes it to /cmd_vel; sim_cmd_vel_bridge integrates pose.
        Node(package='g1_courier_mission', executable='retreat_action_server',
             name='retreat_action_server'),

        # Mission BT — delayed so all action servers are discovered first.
        TimerAction(period=3.0, actions=[
            Node(package='g1_courier_mission', executable='mission_node',
                 name='mission_node'),
        ]),
    ])
