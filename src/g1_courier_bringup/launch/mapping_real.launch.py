"""Real-robot SLAM mapping run.

Drive the robot manually while slam_toolbox builds a map from the Livox
2D-slice scan. Save the resulting map with:

  ros2 run nav2_map_server map_saver_cli -f ~/maps/lab

Then point real.launch.py / localization_real.launch.py 'map' arg at the
saved YAML.

This launch DOES NOT bring up nav2, AMCL, or the mission stack — it is a
mapping-only run. Locomotion comes from the Unitree firmware sport API
consuming /cmd_vel directly (assumed already running) or the remote.

Sensors + TF: wspolny sensors_tf.launch.py (statyczne TF wg REP-120,
odom_tf_relay z flatten, p2l, model robota). Montaz lidaru per-robot
przekazuj argami lidar_roll/lidar_pitch/lidar_z/footprint_z (pomiar:
RANSAC plaszczyzny podlogi z chmury).

On Jazzy slam_toolbox 2.8.x is a LIFECYCLE node (on Humble it was a plain
rclcpp node): without activation it stays UNCONFIGURED — no /scan
subscription, no /map, only 4 params. The lifecycle_manager below
configures+activates it automatically.

Debug view: mapping_debug.rviz (rviz:=true here, or standalone
`rviz2 -d .../rviz/mapping_debug.rviz` on a machine that sees the topics)
albo foxglove:=true (WebSocket :8765).

Override defaults via launch args:
  ros2 launch g1_courier_bringup mapping_real.launch.py \\
      cloud_topic:=/livox/lidar lidar_frame_id:=livox_frame \\
      lidar_roll:=<rad> lidar_pitch:=<rad> footprint_z:=<m>
"""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup = get_package_share_directory('g1_courier_bringup')

    return LaunchDescription([
        DeclareLaunchArgument(
            'cloud_topic', default_value='/livox/lidar',
            description='PointCloud2 source topic (Unitree firmware default).'),
        DeclareLaunchArgument(
            'odom_topic', default_value='/dog_odom',
            description='Odometry topic for odom->base TF relay.'),
        DeclareLaunchArgument(
            'scan_topic', default_value='/scan',
            description='Output LaserScan topic used by slam_toolbox.'),
        DeclareLaunchArgument(
            'odom_frame', default_value='odom',
            description='Global odometry frame name for slam_toolbox.'),
        DeclareLaunchArgument(
            'base_frame', default_value='base_footprint',
            description='Robot base frame for slam_toolbox and odom TF relay '
                        '(REP-120: base_footprint lezy plasko na podlodze).'),
        DeclareLaunchArgument(
            'enable_robot_model', default_value='true',
            description='Model URDF (RSP + lowstate adapter). false gdy brak '
                        'URDF / headless (slam dziala bez RSP).'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Start RViz with the mapping_debug preset (only on a '
                        'machine with a display).'),
        DeclareLaunchArgument(
            'foxglove', default_value='false',
            description='foxglove_bridge (WebSocket :8765) + rebroadcast '
                        '/tf_static. Wymaga ros-jazzy-foxglove-bridge.'),

        # Wspolny tor sensoryczny + TF (montaz per-robot: argi lidar_* /
        # footprint_z przechodza przez wspolna przestrzen argumentow).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup, 'launch', 'sensors_tf.launch.py')),
            launch_arguments={
                'cloud_topic': LaunchConfiguration('cloud_topic'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'odom_frame': LaunchConfiguration('odom_frame'),
                'base_frame': LaunchConfiguration('base_frame'),
                'enable_robot_model': LaunchConfiguration('enable_robot_model'),
            }.items(),
        ),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                os.path.join(bringup, 'config', 'slam_toolbox_mapping.yaml'),
                {
                    'scan_topic': LaunchConfiguration('scan_topic'),
                    'odom_frame': LaunchConfiguration('odom_frame'),
                    'base_frame': LaunchConfiguration('base_frame'),
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
