"""Real-robot full courier stack.

- pointcloud_to_laserscan (Livox Mid-360 -> /scan)
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
    /utlidar/cloud_livox_360mid; override via launch arg cloud_topic)
  - RealSense D435i driver publishing /camera/color/image_raw + /camera/color/camera_info

This launch wires the missing TF pieces that the firmware does not
provide:
  - robot_state_publisher loads G1 URDF -> base_link → body links TF
  - lowstate_to_joint_states converts /lowstate -> /joint_states so RSP
    has live angles
  - static_transform_publisher base_link -> lidar_frame (override via
    lidar_xyz / lidar_frame_id launch args; defaults assume Mid-360 on head)

Sim equivalent: `g1_courier_sim/launch/sim_bridge.launch.py` (separate branch
`courier-sim`).
"""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue


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

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('urdf_path')]),
        value_type=str,
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
            default_value='base_link',
            description='Robot base frame name used by nav and TF relay.',
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
        DeclareLaunchArgument('cloud_topic', default_value='/utlidar/cloud_livox_360mid',
            description='PointCloud2 source topic (Unitree firmware default).'),
        DeclareLaunchArgument('lidar_frame_id', default_value='utlidar_lidar',
            description='frame_id stamped by Unitree firmware on lidar messages.'),

        # robot_state_publisher: TF from base_link to every URDF link.
        # Gated on enable_robot_model — set false if URDF missing.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            condition=IfCondition(LaunchConfiguration('enable_robot_model')),
        ),

        # /lowstate -> /joint_states adapter so robot_state_publisher
        # has live joint angles to compute TF.
        Node(
            package='g1_courier_safety',
            executable='lowstate_to_joint_states',
            name='lowstate_to_joint_states',
            condition=IfCondition(LaunchConfiguration('enable_robot_model')),
        ),

        # Static TF base_link -> lidar frame. Unitree firmware does NOT
        # publish this. Default assumes Mid-360 on G1 head ~1.45 m above
        # pelvis. MEASURE PHYSICALLY and override if wrong.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_lidar',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '1.45',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                '--frame-id', 'base_link',
                '--child-frame-id', LaunchConfiguration('lidar_frame_id'),
            ],
        ),

        # Sensors -> 2D scan. Bumped input_queue_size from default 10 —
        # Livox publishes large clouds; default queue overruns under TF
        # lookup latency.
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            parameters=[
                os.path.join(bringup, 'config', 'pointcloud_to_laserscan.yaml'),
                {'input_queue_size': 50},
                {'target_frame': base_frame},
            ],
            remappings=[('cloud_in', LaunchConfiguration('cloud_topic')),
                        ('scan', '/scan')],
        ),

        Node(
            package='g1_courier_bringup',
            executable='odom_tf_relay',
            name='odom_tf_relay',
            condition=IfCondition(LaunchConfiguration('publish_odom_tf')),
            parameters=[{
                'odom_topic': odom_topic,
                'odom_frame': odom_frame,
                'base_frame': base_frame,
                'use_msg_frame_ids': False,
                'use_msg_stamp': True,
            }],
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

        # Nav2 (planner, controller, BT navigator, AMCL). SetRemap forces
        # nav2 to publish on /cmd_vel_nav so cmd_vel_arbiter can merge it
        # with dock + retreat. Required on Humble (where nav2 publishes
        # on /cmd_vel by default); no-op on Jazzy (nav2_bringup already
        # remaps internally there).
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
