"""Phase 1 full launch — phase1_smoke + nav2 stack with AMCL on saved map.

Difference vs phase1_smoke.launch.py:
  - kinematic_nav_node REPLACED by nav2_navigate_proxy adapter that
    forwards /courier/navigate_to_pose goals to nav2's /navigate_to_pose.
    Mission BT and behaviors.NavigateTo stay unchanged — same contract.
  - sim_cmd_vel_bridge_node runs WITHOUT static map->odom (publish_map_to_odom: False);
    AMCL publishes that transform.
  - Adds nav2_bringup launch (map_server, AMCL, planner_server,
    controller_server, bt_navigator, behavior_server, lifecycle_manager)
    loading ~/maps/lab.{pgm,yaml} from Faza 1.4.

PREREQUISITES:
  1. unitree_mujoco running on mac (rt/scan, rt/lowstate, rt/detections,
     rt/lowcmd via DDS bridge — Fazy 1.0..1.2).
  2. ~/maps/lab.yaml + ~/maps/lab.pgm (built in Faza 1.4 with
     mapping.launch.py + teleop_twist_keyboard).

Usage:
    ros2 launch g1_courier_bringup phase1_full.launch.py
    # optional: ros2 launch ... phase1_full.launch.py map:=$HOME/maps/other.yaml
"""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    safety_share = get_package_share_directory('g1_courier_safety')
    docking_share = get_package_share_directory('g1_courier_docking')
    bringup_share = get_package_share_directory('g1_courier_bringup')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    nav2_params = os.path.join(bringup_share, 'config', 'nav2_params.yaml')
    default_map = os.path.join(os.path.expanduser('~'), 'maps', 'lab.yaml')

    map_arg = DeclareLaunchArgument(
        'map', default_value=default_map,
        description='Path to map yaml (built in Faza 1.4)',
    )

    # nav2 stack via the official bringup_launch — handles map_server, AMCL,
    # planner_server, controller_server, bt_navigator, behavior_server,
    # smoother_server, velocity_smoother, and the lifecycle manager that
    # configures + activates them all in order.
    # nav2 stack — bringup_launch.py in jazzy doesn't expose toggles for
    # collision_monitor/route/docking_server; they're always launched. We
    # supply minimal stub configs in nav2_params.yaml that let them pass
    # the lifecycle configure step but keep them out of the cmd_vel path.
    # Our own dock_action_server (g1_courier_docking) replaces opennav_docking.
    nav2_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py'),
        ]),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': 'false',
            'params_file': nav2_params,
            'autostart': 'true',
        }.items(),
    )

    return LaunchDescription([
        map_arg,

        # Kinematic /odom integrator — publish_map_to_odom False because AMCL
        # will own that transform.
        Node(package='g1_courier_sim', executable='sim_cmd_vel_bridge_node',
             name='sim_cmd_vel_bridge_node',
             parameters=[{'publish_map_to_odom': False}]),

        # Arbiter merges nav2 controller_server (/cmd_vel_nav, jazzy
        # nav2_bringup remaps cmd_vel→cmd_vel_nav for controller_server,
        # bt_navigator etc.) with dock (/cmd_vel_dock) and retreat
        # (/cmd_vel_retreat). Output: /cmd_vel → mac MuJoCo kinematic mocap.
        Node(package='g1_courier_safety', executable='cmd_vel_arbiter',
             name='cmd_vel_arbiter',
             parameters=[
                 os.path.join(safety_share, 'config', 'safety.yaml'),
             ]),

        # Real arm action servers (same sim-only knobs as phase1_smoke).
        Node(package='g1_courier_arm_skills', executable='pick_action_server',
             name='pick_action_server',
             parameters=[{
                 'arm_sdk_topic': '/lowcmd',
                 'lowstate_topic': '/lowstate',
                 'require_grasp_verified': False,
                 'control_dt_s': 0.005,
                 'kinematic_mode': True,
             }]),
        Node(package='g1_courier_arm_skills', executable='place_action_server',
             name='place_action_server',
             parameters=[{
                 'arm_sdk_topic': '/lowcmd',
                 'lowstate_topic': '/lowstate',
                 'require_release_verified': False,
                 'control_dt_s': 0.005,
                 'kinematic_mode': True,
             }]),

        # nav2 adapter — accepts /courier/navigate_to_pose, forwards to
        # nav2's /navigate_to_pose. Mission BT contract preserved.
        Node(package='g1_courier_sim', executable='nav2_navigate_proxy',
             name='nav2_navigate_proxy'),

        # TF chain so AMCL/nav2/dock_action_server can locate the lidar.
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='base_link_to_pelvis_tf',
             arguments=['0', '0', '0',
                        '0', '0', '0', '1',
                        'base_link', 'pelvis']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='lidar_static_tf',
             arguments=['0', '0', '-0.393',
                        '0', '0', '0', '1',
                        'pelvis', 'lidar_link']),

        # Dock + retreat publish to dedicated topics; arbiter routes to /cmd_vel.
        Node(package='g1_courier_docking', executable='dock_action_server',
             name='dock_action_server',
             parameters=[
                 os.path.join(docking_share, 'config', 'docking.yaml'),
             ]),

        Node(package='g1_courier_mission', executable='retreat_action_server',
             name='retreat_action_server'),

        # nav2 stack (delay 1 s so the TF tree is ready before AMCL starts).
        TimerAction(period=1.0, actions=[nav2_stack]),

        # Mission BT — longer delay (5 s) so nav2 lifecycle finishes
        # configure + activate before mission BT starts firing goals.
        TimerAction(period=5.0, actions=[
            Node(package='g1_courier_mission', executable='mission_node',
                 name='mission_node'),
        ]),
    ])
