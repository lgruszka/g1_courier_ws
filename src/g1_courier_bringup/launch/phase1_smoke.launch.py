"""Phase 1 smoke launch — mission BT with REAL pick/place against MuJoCo.

PREREQUISITE: unitree_mujoco must be running on a separate host that can
reach this one over DDS. In our setup that is the macOS host on the
Parallels Shared Network (10.211.55.2 ↔ 10.211.55.11), publishing /lowstate
and subscribing /lowcmd. See memory/project_macos_mujoco_bridge.md.

Differences from phase0:
  - sim_lowstate_publisher_node REMOVED (the MuJoCo bridge supplies /lowstate)
  - fake_pick_action_server REPLACED by REAL pick_action_server with
      arm_sdk_topic:=/lowcmd  (mac MuJoCo subscribes there, not /arm_sdk)
      require_grasp_verified:=false  (tau_est is unreliable in MuJoCo
      because the unbalanced bipedal robot falls over and gravity acts
      sideways on the arms — see CLAUDE.md "Najczęstsze problemy")
  - fake_place_action_server REPLACED analogously, require_release_verified:=false
  - retreat is REAL retreat_action_server (drives /cmd_vel_retreat ->
    arbiter -> /cmd_vel -> sim_cmd_vel_bridge integrates pose backward)
  - navigate / dock stay fake (no nav2/AprilTag yet)

Expected: same A→B→A cycle as phase0, but each pickup_at_X / transfer_to_X
takes longer because the arm sequence runs for real (~14s pick, ~9s place
visible in MuJoCo viewer on macOS).
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

        # NOTE: walking_controller_node is NOT launched here. The Phase 1.3
        # alpha attempt (per-joint PD with no gravity feedforward) was tested
        # and proved insufficient — the robot oscillates and falls. The
        # `g1_courier_locomotion` package keeps the node as a starting point
        # for variant β (RL policy from unitree_rl_gym). For now arm action
        # servers publish straight to /lowcmd and the robot stays lying down
        # in MuJoCo while arms are exercised.
        # Real arm action servers, talking to the MuJoCo bridge on /lowcmd.
        Node(package='g1_courier_arm_skills', executable='pick_action_server',
             name='pick_action_server',
             parameters=[{
                 'arm_sdk_topic': '/lowcmd',
                 'lowstate_topic': '/lowstate',
                 'require_grasp_verified': False,
             }]),
        Node(package='g1_courier_arm_skills', executable='place_action_server',
             name='place_action_server',
             parameters=[{
                 'arm_sdk_topic': '/lowcmd',
                 'lowstate_topic': '/lowstate',
                 'require_release_verified': False,
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
