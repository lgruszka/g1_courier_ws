"""Launches the full courier stack:

- pointcloud_to_laserscan (Mid360 -> /scan)
- nav2 bringup with AMCL on a known map
- apriltag_ros detector
- cmd_vel arbiter
- dock action server
- pick / place action servers
- navigate_proxy + retreat action server
- mission_node (BT)

The unitree bridges (`unitree_cmd_vel_bridge_node`, sport API consumer of /cmd_vel)
are NOT launched here - bring them up from the existing j2s-light_tracking
package or vendor them in later.
"""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup = get_package_share_directory('g1_courier_bringup')
    arm_share = get_package_share_directory('g1_courier_arm_skills')
    dock_share = get_package_share_directory('g1_courier_docking')
    mission_share = get_package_share_directory('g1_courier_mission')
    safety_share = get_package_share_directory('g1_courier_safety')

    nav2_params = LaunchConfiguration('nav2_params')
    map_yaml = LaunchConfiguration('map')

    return LaunchDescription([
        DeclareLaunchArgument('nav2_params',
            default_value=os.path.join(bringup, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('map',
            default_value=os.path.join(bringup, 'maps', 'lab.yaml'),
            description='Saved 2D map for AMCL.'),

        # Sensors -> 2D scan.
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            parameters=[os.path.join(bringup, 'config', 'pointcloud_to_laserscan.yaml')],
            remappings=[('cloud_in', '/livox/lidar'), ('scan', '/scan')],
        ),

        # AprilTag detector.
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            parameters=[os.path.join(bringup, 'config', 'apriltag.yaml')],
            remappings=[
                ('image_rect', '/camera/color/image_raw'),
                ('camera_info', '/camera/color/camera_info'),
            ],
        ),

        # Nav2 (planner, controller, BT navigator, AMCL).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': map_yaml,
                'use_sim_time': 'false',
                'params_file': nav2_params,
            }.items(),
        ),

        # Safety / arbitration.
        Node(
            package='g1_courier_safety',
            executable='cmd_vel_arbiter',
            name='cmd_vel_arbiter',
            parameters=[os.path.join(safety_share, 'config', 'safety.yaml')],
        ),

        # Docking action server.
        Node(
            package='g1_courier_docking',
            executable='dock_action_server',
            name='dock_action_server',
            parameters=[os.path.join(dock_share, 'config', 'docking.yaml')],
        ),

        # Arm skill action servers.
        GroupAction([
            Node(package='g1_courier_arm_skills', executable='pick_action_server',
                 name='pick_action_server',
                 parameters=[os.path.join(arm_share, 'config', 'arm_skills.yaml')]),
            Node(package='g1_courier_arm_skills', executable='place_action_server',
                 name='place_action_server',
                 parameters=[os.path.join(arm_share, 'config', 'arm_skills.yaml')]),
        ]),

        # Navigation proxy (wraps nav2 NavigateToPose) + retreat helper.
        Node(package='g1_courier_mission', executable='navigate_proxy', name='navigate_proxy'),
        Node(package='g1_courier_mission', executable='retreat_action_server',
             name='retreat_action_server',
             parameters=[os.path.join(mission_share, 'config', 'mission.yaml')]),

        # Mission orchestrator (Behavior Tree).
        Node(package='g1_courier_mission', executable='mission_node', name='mission_node',
             parameters=[os.path.join(mission_share, 'config', 'mission.yaml')]),
    ])
