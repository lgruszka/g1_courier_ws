"""Real-robot full courier stack.

- cropbox 3D (pcl_ros, wycina paczkę z chmury) -> pointcloud_to_laserscan (Mid-360 -> /scan)
- nav2 bringup with AMCL on a known map
- apriltag_ros detector (RealSense D435i)
- cmd_vel arbiter
- dock action server
- pick / place action servers
- navigate_proxy + retreat action server
- mission_node (BT)

This launch assumes the Unitree firmware-side bridges are already running:
  - sport API consumer of /cmd_vel (firmware side, proprietary)
  - /lowstate publisher and /arm_sdk subscriber (firmware DDS bridge)
  - Livox PointCloud2 published by Unitree firmware (default
    /utlidar/cloud_livox_mid360; override via launch arg cloud_topic)
  - RealSense D435i driver publishing /camera/camera/color/image_raw + /camera/camera/color/camera_info

Sensors + TF: wspolny sensors_tf.launch.py (drzewo wg REP-120:
odom->base_footprint [relay, flatten] -> base_link [spaw] -> lidar/URDF;
tor skanu z cropboxem paczki, respawn i watchdogiem). Montaz lidaru
per-robot: argi lidar_roll/lidar_pitch/lidar_z/footprint_z (pomiar RANSAC
plaszczyzny podlogi; defaulty = montaz nominalny).

Sim equivalent: `g1_courier_sim/launch/sim_bridge.launch.py` (separate branch
`courier-sim`).
"""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def generate_launch_description() -> LaunchDescription:
    bringup = get_package_share_directory('g1_courier_bringup')
    arm_share = get_package_share_directory('g1_courier_arm_skills')
    dock_share = get_package_share_directory('g1_courier_docking')
    mission_share = get_package_share_directory('g1_courier_mission')
    safety_share = get_package_share_directory('g1_courier_safety')

    nav2_params = LaunchConfiguration('nav2_params')
    map_yaml = LaunchConfiguration('map')

    default_urdf = os.path.join(
        get_package_share_directory('g1_description'),
        'urdf', 'g1_29dof.urdf',
    )

    odom_topic = LaunchConfiguration('odom_topic')
    odom_frame = LaunchConfiguration('odom_frame')
    base_frame = LaunchConfiguration('base_frame')

    return LaunchDescription([
        DeclareLaunchArgument('nav2_params',
            default_value=os.path.join(bringup, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('map',
            default_value=os.path.join(bringup, 'maps', 'lab.yaml'),
            description='Saved 2D map for AMCL.'),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/dog_odom',
            description='Real odometry topic used as source for odom->base TF relay.',
        ),
        DeclareLaunchArgument(
            'odom_frame',
            default_value='odom',
            description='Global odometry frame name for localization and nav.',
        ),
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_footprint',
            description='Child frame TF relaya (REP-120: base_footprint '
                        'plasko na podlodze; spaw base_footprint->base_link '
                        'w sensors_tf). NIE ustawiaj base_link — konflikt '
                        'ze spawem (dwoch rodzicow w drzewie TF).',
        ),
        DeclareLaunchArgument(
            'publish_odom_tf',
            default_value='true',
            description='Publish odom->base_link TF from odometry topic.',
        ),
        DeclareLaunchArgument('urdf_path', default_value=default_urdf,
            description='Absolute path to G1 URDF. Default uses the vendored '
                        'g1_description package; override to point elsewhere.'),
        DeclareLaunchArgument('enable_robot_model', default_value='true',
            description='Start robot_state_publisher + lowstate_to_joint_states '
                        'for TF and RViz RobotModel. Set false for headless/minimal runs.'),
        DeclareLaunchArgument('cloud_topic', default_value='/livox/lidar',
            description='PointCloud2 source topic (Unitree firmware default).'),
        DeclareLaunchArgument(
            'pointcloud_start_delay',
            default_value='10.0',
            description='Delay (s) before starting pointcloud_to_laserscan.',
        ),
        DeclareLaunchArgument('lidar_frame_id', default_value='livox_frame',
            description='frame_id stamped by Unitree firmware on lidar messages.'),
        DeclareLaunchArgument('enable_mission', default_value='true',
            description='Start mission_node (BT). Set false for nav-only smoke '
                        'tests where you send manual goals from RViz.'),
        DeclareLaunchArgument('filter_parcel', default_value='true',
            description='Wytnij bryłę niesionej paczki z chmury 3D cropboxem '
                        '(pcl_ros) PRZED pointcloud_to_laserscan, by nie psuła '
                        'AMCL — wiązki za paczką pokazują ścianę widzianą ponad '
                        'nią. false => p2l czyta chmurę wprost, bez filtra.'),
        DeclareLaunchArgument(
            'nav2_start_delay',
            default_value='1.0',
            description='Delay (s) before starting Nav2 so odom->base TF and first sensor data are available.',
        ),
        DeclareLaunchArgument('enable_camera', default_value='false',
            description='Odpal realsense2_camera (RealSense D435i RGB+depth). '
                        'Default false.'),
                        
        # Wspolny tor sensoryczny + TF (sensors_tf.launch.py): model robota,
        # spawy base_footprint/pelvis, TF lidaru (montaz per-robot argami
        # lidar_roll/lidar_pitch/lidar_z/footprint_z), odom_tf_relay
        # (flatten, throttle 50 Hz), tor skanu z cropboxem paczki,
        # respawnem i scan_watchdogiem — historia i uzasadnienia w
        # sensors_tf.launch.py (m.in. zaciecia p2l na realnym G1).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup, 'launch', 'sensors_tf.launch.py')),
            launch_arguments={
                'cloud_topic': LaunchConfiguration('cloud_topic'),
                'filter_parcel': LaunchConfiguration('filter_parcel'),
                'pointcloud_start_delay':
                    LaunchConfiguration('pointcloud_start_delay'),
                'odom_topic': odom_topic,
                'odom_frame': odom_frame,
                'base_frame': base_frame,
                'publish_odom_tf': LaunchConfiguration('publish_odom_tf'),
                'odom_use_msg_stamp': 'false',
                'enable_robot_model':
                    LaunchConfiguration('enable_robot_model'),
                'urdf_path': LaunchConfiguration('urdf_path'),
                'lidar_frame_id': LaunchConfiguration('lidar_frame_id'),
            }.items(),
        ),

        # AprilTag detector.
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            parameters=[os.path.join(bringup, 'config', 'apriltag.yaml')],
            remappings=[
                ('image_rect', '/camera/camera/color/image_raw'),
                ('camera_info', '/camera/camera/color/camera_info'),
            ],
        ),
        # RealSense D435i. Match the packaged rs_launch.py defaults:
        # namespace=camera + name=camera -> /camera/camera/color/...
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            namespace='camera',
            name='camera',
            output='screen',
            parameters=[{
                'camera_namespace': 'camera',
                'camera_name': 'camera',
                'enable_depth': True,
                'enable_color': True,
                'enable_infra1': False,
                'enable_infra2': False,
                'enable_sync': True,
                'align_depth.enable': True,
                'rgb_camera.color_profile': '640,480,30',
                'depth_module.depth_profile': '640,480,30',
            }],
            arguments=['--ros-args', '--log-level', 'info'],
            condition=IfCondition(LaunchConfiguration('enable_camera')),
        ),

        # Nav2 (planner, controller, BT navigator, AMCL). SetRemap forces
        # nav2 to publish on /cmd_vel_nav so cmd_vel_arbiter can merge it
        # with dock + retreat. Required on Humble (where nav2 publishes
        # on /cmd_vel by default); no-op on Jazzy (nav2_bringup already
        # remaps internally there).
        TimerAction(
            period=LaunchConfiguration('nav2_start_delay'),
            actions=[
                GroupAction([
                    SetRemap(src='/cmd_vel', dst='/cmd_vel_nav'),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(os.path.join(
                            get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
                        launch_arguments={
                            'map': map_yaml,
                            'use_sim_time': 'false',
                            'params_file': nav2_params,
                        }.items(),
                    ),
                ]),
            ],
        ),

        # Safety / arbitration.
        Node(
            package='g1_courier_safety',
            executable='cmd_vel_arbiter',
            name='cmd_vel_arbiter',
            parameters=[os.path.join(safety_share, 'config', 'safety.yaml')],
        ),

        # /cmd_vel → Unitree sport API Request (firmware bridge).
        # Vendored 1:1 z j2s light_tracking_lts (prod-tested).
        Node(
            package='g1_courier_safety',
            executable='unitree_cmd_vel_bridge_node',
            name='unitree_cmd_vel_bridge_node',
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

        # Mission orchestrator (Behavior Tree). Gated by enable_mission so
        # nav-only smoke tests can skip auto-goal injection and let the
        # operator drive from RViz 2D Goal Pose instead.
        Node(package='g1_courier_mission', executable='mission_node', name='mission_node',
             parameters=[os.path.join(mission_share, 'config', 'mission.yaml')],
             condition=IfCondition(LaunchConfiguration('enable_mission'))),
    ])
