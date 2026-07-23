"""Real-robot SLAM mapping run.

Drive the robot manually (e.g. via teleop_twist_keyboard) while slam_toolbox
builds a map from the Livox 2D-slice scan. Save the resulting map with:

  ros2 run nav2_map_server map_saver_cli -f ~/maps/lab

Then point real.launch.py 'map' arg at the saved YAML.

This launch DOES NOT bring up nav2, AMCL, or the mission stack — it is a
mapping-only run. Locomotion comes from the Unitree firmware sport API
consuming /cmd_vel directly (assumed already running).

TF tree wiring:
  - odom -> base_link        published by Unitree firmware
  - base_link -> body links  published by robot_state_publisher (URDF)
  - base_link -> lidar_frame published by static_transform_publisher below
  - /joint_states            published by lowstate_to_joint_states adapter
                              (converts /lowstate motor_state[i].q)

Override defaults via launch args:
  ros2 launch g1_courier_bringup mapping_real.launch.py \\
      cloud_topic:=/livox/lidar \\
      lidar_frame_id:=livox_frame \\
      urdf_path:=$HOME/path/to/g1.urdf

On Jazzy slam_toolbox 2.8.x is a LIFECYCLE node (on Humble it was a plain
rclcpp node): without activation it stays UNCONFIGURED — no /scan
subscription, no /map, only 4 params. The lifecycle_manager below
configures+activates it automatically.

Debug view: mapping_debug.rviz (rviz:=true here, or standalone
`rviz2 -d .../rviz/mapping_debug.rviz` on a machine that sees the topics).
"""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    bringup = get_package_share_directory('g1_courier_bringup')

    default_urdf = os.path.join(
        get_package_share_directory('g1_description'),
        'urdf', 'g1_29dof.urdf',
    )

    urdf_arg = DeclareLaunchArgument(
        'urdf_path', default_value=default_urdf,
        description='Absolute path to G1 URDF. Default uses the vendored '
                    'g1_description package; override to point elsewhere.',
    )
    enable_robot_model_arg = DeclareLaunchArgument(
        'enable_robot_model', default_value='true',
        description='Start robot_state_publisher + lowstate_to_joint_states for '
                    'TF and RViz RobotModel display. Set false to skip when URDF '
                    'is missing or for headless/minimal runs (slam still works '
                    'without RSP — only model viz disappears).',
    )
    cloud_topic_arg = DeclareLaunchArgument(
        'cloud_topic', default_value='/livox/lidar',
        description='PointCloud2 source topic (Unitree firmware default).',
    )
    lidar_frame_arg = DeclareLaunchArgument(
        'lidar_frame_id', default_value='livox_frame',
        description='frame_id stamped on lidar messages by Unitree firmware. '
                    'Verify with: ros2 topic echo <cloud_topic> --field header.frame_id --once',
    )

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('urdf_path')]),
        value_type=str,
    )

    cloud_topic = LaunchConfiguration('cloud_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    odom_frame = LaunchConfiguration('odom_frame')
    base_frame = LaunchConfiguration('base_frame')

    enable_robot_model = IfCondition(LaunchConfiguration('enable_robot_model'))

    return LaunchDescription([
        urdf_arg,
        enable_robot_model_arg,
        cloud_topic_arg,
        lidar_frame_arg,

        # robot_state_publisher: TF from base_link to every URDF link.
        # Gated on enable_robot_model — set false if URDF missing.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            condition=enable_robot_model,
        ),

        # /lowstate -> /joint_states adapter so robot_state_publisher
        # has live joint angles to compute TF.
        Node(
            package='g1_courier_safety',
            executable='lowstate_to_joint_states',
            name='lowstate_to_joint_states',
            condition=enable_robot_model,
        ),

        # Montaz PER-ROBOT (przekazuj argami; defaulty = montaz nominalny):
        # zmierz na swoim egzemplarzu RANSAC-iem plaszczyzny podlogi z chmury
        # (fituje normalna podlogi -> roll/pitch lidaru i wysokosc bazy).
        # Sam roll=pi przy realnym pitchu montazu daje pas ciecia skosny
        # (~0.7 m/10 m na 4 st) i rozmyte sciany na mapie.
        DeclareLaunchArgument(
            'footprint_z', default_value='0.75',
            description='Wysokosc base_link (pelvis) nad podloga [m].'),
        DeclareLaunchArgument(
            'lidar_z', default_value='0.5',
            description='Wysokosc lidaru nad base_link [m].'),
        DeclareLaunchArgument(
            'lidar_roll', default_value='3.14159',
            description='Roll montazu lidaru [rad] (do gory nogami = pi).'),
        DeclareLaunchArgument(
            'lidar_pitch', default_value='0.0',
            description='Pitch montazu lidaru [rad] (zmierz RANSAC).'),

        # Spaw base_footprint -> base_link (REP-120: baza nawigacji 2D
        # plasko na podlodze; mapa renderuje sie pod stopami, nie w pelvis).
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_footprint',
            arguments=[
                '--x', '0.0', '--y', '0.0',
                '--z', LaunchConfiguration('footprint_z'),
                '--frame-id', 'base_footprint',
                '--child-frame-id', 'base_link',
            ],
        ),

        # Static TF base_link -> lidar frame (Unitree firmware NIE publikuje).
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_lidar',
            arguments=[
                '--x', '0.0', '--y', '0.0',
                '--z', LaunchConfiguration('lidar_z'),
                '--roll', LaunchConfiguration('lidar_roll'),
                '--pitch', LaunchConfiguration('lidar_pitch'),
                '--yaw', '0.0',
                '--frame-id', 'base_link',
                '--child-frame-id', LaunchConfiguration('lidar_frame_id'),
            ],
        ),

        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan',
            description='Output LaserScan topic used by slam_toolbox.',
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/dog_odom',
            description='Real odometry topic used as source for odom->base TF relay.',
        ),
        DeclareLaunchArgument(
            'odom_frame',
            default_value='odom',
            description='Global odometry frame name for slam_toolbox.',
        ),
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_footprint',
            description='Robot base frame for slam_toolbox and odom TF relay. '
                        'base_footprint (REP-120) lezy plasko na podlodze; '
                        'spaw base_footprint->base_link (pelvis) nizej.',
        ),
        DeclareLaunchArgument(
            'publish_odom_tf',
            default_value='true',
            description='Publish odom->base_link TF from odometry topic.',
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
                'max_rate': 50.0,
                # Firmware daje poze pelvis (z~0.75, przechyly) — baza 2D
                # ma lezec plasko na podlodze (REP-120).
                'flatten': True,
            }],
        ),

        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            parameters=[
                os.path.join(bringup, 'config', 'pointcloud_to_laserscan.yaml'),
                {'queue_size': 50},
            ],
            remappings=[('cloud_in', LaunchConfiguration('cloud_topic')),
                        ('scan', '/scan')],
        ),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                os.path.join(bringup, 'config', 'slam_toolbox_mapping.yaml'),
                {
                    'scan_topic': scan_topic,
                    'odom_frame': odom_frame,
                    'base_frame': base_frame,
                },
            ],
        ),

        # KRYTYCZNE na Jazzy: slam_toolbox 2.8.x to wezel LIFECYCLE — bez
        # aktywacji startuje UNCONFIGURED (nie subskrybuje /scan, brak /map,
        # tylko 4 paramy). lifecycle_manager z autostart robi
        # configure+activate; bond_timeout=0 bo slam_toolbox nie bonduje.
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['slam_toolbox'],
                'bond_timeout': 0.0,
            }],
        ),

        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Start RViz with the mapping_debug preset (only on a '
                        'machine with a display; headless container: keep false '
                        'and run rviz2 -d .../mapping_debug.rviz elsewhere).',
        ),
        DeclareLaunchArgument(
            'foxglove', default_value='false',
            description='Start foxglove_bridge (WebSocket :8765) for offboard '
                        'viz. Requires ros-jazzy-foxglove-bridge in the image.',
        ),
        # topic_whitelist konieczny: nieznane typy unitree_hg z firmware
        # (np. /lf/sportmodestate) wywalaja bridge (bad_optional_access).
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[{
                'port': 8765,
                'address': '0.0.0.0',
                'topic_whitelist': [
                    '^/map$', '^/scan$', '^/tf$', '^/tf_static$',
                    '^/robot_description$', '^/joint_states$',
                    '^/utlidar/cloud_livox_mid360$', '^/livox/lidar$',
                    '^/dog_odom$', '^/odom$',
                    '^/initialpose$', '^/goal_pose$', '^/clicked_point$',
                    '^/rosout$',
                ],
            }],
            condition=IfCondition(LaunchConfiguration('foxglove')),
        ),
        # Rebroadcast /tf_static co 3 s: foxglove_bridge gubi latched statyki
        # przy (re)connectach klienta -> model w Studio sie rozsypuje.
        Node(
            package='g1_courier_bringup',
            executable='tf_static_republisher',
            name='tf_static_republisher',
            condition=IfCondition(LaunchConfiguration('foxglove')),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_mapping',
            arguments=['-d', os.path.join(bringup, 'rviz', 'mapping_debug.rviz')],
            additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'},
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
