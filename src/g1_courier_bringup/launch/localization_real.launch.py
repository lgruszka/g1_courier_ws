"""Localization-only sanity run: AMCL vs saved map, BEZ toru ruchu.

Faza testow przed nawigacja (docs/amcl_sanity_test.md): robot jezdzi z PILOTA,
a my patrzymy czy AMCL trzyma poze na mapie (scan lezy na scianach w RViz/
Foxglove).

Celowo NIE startuje: planner/controller/bt_navigator, cmd_vel_arbiter,
unitree_cmd_vel_bridge, mission, docking, arm skills — nie istnieje zadna
sciezka nav->robot, wiec przypadkowy goal z wizualizacji nie ruszy robota.

Sklad: wspolny tor sensoryczny+TF (sensors_tf.launch.py: statyczne TF,
odom_tf_relay, p2l, model robota) + map_server + amcl + lifecycle autostart
(JAWNE nody, nie include nav2_bringup/localization_launch.py — tam
forwarding argumentu `map` do yaml_filename nie zadzialal: map_server
wstawal z pustym yaml_filename i AMCL wisial na "Waiting for map").

Kontener na Jetsonie (montaz lidaru per-robot — pomiar RANSAC podlogi):
  ros2 launch g1_courier_bringup localization_real.launch.py \\
      map:=/ws/maps/lab2.yaml \\
      cloud_topic:=/utlidar/cloud_livox_mid360 odom_topic:=/dog_odom \\
      lidar_roll:=<rad> lidar_pitch:=<rad> footprint_z:=<m>

Poza startowa: AMCL wstaje z set_initial_pose (origin mapy) — ustaw realna
poze narzedziem "2D Pose Estimate"/"Pose estimate" (topik /initialpose).
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
            'map', default_value=os.path.join(bringup, 'maps', 'lab.yaml'),
            description='Saved 2D map yaml for AMCL.'),
        DeclareLaunchArgument(
            'nav2_params',
            default_value=os.path.join(bringup, 'config', 'nav2_params.yaml'),
            description='nav2 params (amcl + map_server sections used).'),
        DeclareLaunchArgument(
            'cloud_topic', default_value='/livox/lidar',
            description='PointCloud2 source topic.'),
        DeclareLaunchArgument(
            'odom_topic', default_value='/dog_odom',
            description='Odometry topic for odom->base TF relay.'),
        DeclareLaunchArgument(
            'enable_robot_model', default_value='false',
            description='Model URDF (RSP + lowstate adapter) — viz.'),
        DeclareLaunchArgument(
            'foxglove', default_value='false',
            description='foxglove_bridge (WebSocket :8765) + rebroadcast '
                        '/tf_static. Wymaga ros-jazzy-foxglove-bridge.'),

        # Wspolny tor sensoryczny + TF. Montaz per-robot przekazuj argami
        # (footprint_z / lidar_z / lidar_roll / lidar_pitch — przechodza
        # do include przez wspolna przestrzen argumentow launcha).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup, 'launch', 'sensors_tf.launch.py')),
            launch_arguments={
                'cloud_topic': LaunchConfiguration('cloud_topic'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'enable_robot_model': LaunchConfiguration('enable_robot_model'),
            }.items(),
        ),

        # Wizualizacja offboard bez DDS przez WiFi: WebSocket :8765.
        # topic_whitelist KONIECZNY: robot rozglasza topiki unitree_hg,
        # ktorych definicji nie mamy — bridge pada na bad_optional_access.
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
                    '^/amcl_pose$', '^/particle_cloud$',
                    '^/utlidar/cloud_livox_mid360$', '^/livox/lidar$',
                    '^/dog_odom$', '^/odom$',
                    '^/initialpose$', '^/goal_pose$', '^/clicked_point$',
                    '^/rosout$', '^/diagnostics$',
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

        # map_server + amcl + lifecycle_manager_localization (autostart).
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                LaunchConfiguration('nav2_params'),
                {'yaml_filename': LaunchConfiguration('map')},
            ],
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                LaunchConfiguration('nav2_params'),
                # Baza 2D na podlodze (spaw w sensors_tf), nie pelvis z yaml.
                {'base_frame_id': 'base_footprint'},
            ],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['map_server', 'amcl'],
            }],
        ),
    ])
