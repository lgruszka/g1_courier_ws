"""Localization-only sanity run: AMCL vs saved map, BEZ toru ruchu.

Faza testow przed nawigacja (docs/amcl_sanity_test.md): robot jezdzi z PILOTA,
a my patrzymy czy AMCL trzyma poze na mapie (scan lezy na scianach w RViz).

Celowo NIE startuje: planner/controller/bt_navigator, cmd_vel_arbiter,
unitree_cmd_vel_bridge, mission, docking, arm skills — nie istnieje zadna
sciezka nav->robot, wiec przypadkowy goal z RViz nie ruszy robota.

Sklad: sensory jak w mapping_real (static TF lidaru, odom_tf_relay, p2l)
+ map_server + amcl + lifecycle autostart (JAWNE nody, nie include
nav2_bringup/localization_launch.py — tam forwarding argumentu `map` do
yaml_filename nie zadzialal: map_server wstal z pustym yaml_filename
i AMCL wisial na "Waiting for map"; jawny parametr jest deterministyczny).

Kontener na Jetsonie:
  ros2 launch g1_courier_bringup localization_real.launch.py \\
      map:=/ws/maps/lab2.yaml \\
      cloud_topic:=/utlidar/cloud_livox_mid360 odom_topic:=/dog_odom

Poza startowa: AMCL wstaje z set_initial_pose (origin mapy) — ustaw realna
poze narzedziem "2D Pose Estimate" w RViz (mapping_debug.rviz; /initialpose
przechodzi z VM przez most TCP).
"""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    bringup = get_package_share_directory('g1_courier_bringup')

    default_urdf = os.path.join(
        get_package_share_directory('g1_description'),
        'urdf', 'g1_29dof.urdf',
    )
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('urdf_path')]),
        value_type=str,
    )
    enable_robot_model = IfCondition(LaunchConfiguration('enable_robot_model'))

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
            'lidar_frame_id', default_value='livox_frame',
            description='frame_id stamped on lidar messages.'),
        DeclareLaunchArgument(
            'odom_topic', default_value='/dog_odom',
            description='Odometry topic for odom->base_link TF relay.'),
        DeclareLaunchArgument(
            'publish_odom_tf', default_value='true',
            description='Publish odom->base_link TF from odometry topic.'),
        DeclareLaunchArgument(
            'urdf_path', default_value=default_urdf,
            description='Absolute path to G1 URDF (robot model viz).'),
        DeclareLaunchArgument(
            'enable_robot_model', default_value='false',
            description='Start robot_state_publisher + lowstate_to_joint_states '
                        '(live URDF model in RViz/Foxglove). Needs /lowstate.'),
        DeclareLaunchArgument(
            'foxglove', default_value='false',
            description='Start foxglove_bridge (WebSocket :8765) for offboard '
                        'viz. Requires ros-jazzy-foxglove-bridge in the image.'),

        # Model robota: RSP + adapter /lowstate->/joint_states + spaw
        # base_link->pelvis (root URDF to pelvis; identity jak w real.launch).
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            condition=enable_robot_model,
        ),
        Node(
            package='g1_courier_safety',
            executable='lowstate_to_joint_states',
            name='lowstate_to_joint_states',
            condition=enable_robot_model,
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_pelvis_tf',
            arguments=['0', '0', '0', '0', '0', '0', '1',
                       'base_link', 'pelvis'],
            condition=enable_robot_model,
        ),

        # Wizualizacja offboard bez DDS przez WiFi: WebSocket :8765.
        # topic_whitelist KONIECZNY: robot rozglasza topiki unitree_hg,
        # ktorych definicji nie mamy (np. /lf/sportmodestate SportModeState)
        # — bridge przy budowie schematu pada na bad_optional_access.
        # Whitelist = tylko nasz tor viz + brak smieci po WiFi.
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

        # Montaz PER-ROBOT (przekazuj argami; defaulty = montaz nominalny):
        # zmierz na swoim egzemplarzu RANSAC-iem plaszczyzny podlogi z chmury.
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

        # Spaw base_footprint -> base_link (REP-120): AMCL sadza
        # base_footprint na plaszczyznie mapy, mapa pod stopami robota.
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

        # Static TF base_link -> lidar (firmware NIE publikuje).
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

        Node(
            package='g1_courier_bringup',
            executable='odom_tf_relay',
            name='odom_tf_relay',
            condition=IfCondition(LaunchConfiguration('publish_odom_tf')),
            parameters=[{
                'odom_topic': LaunchConfiguration('odom_topic'),
                'odom_frame': 'odom',
                # REP-120: baza nawigacji plasko na podlodze (firmware daje
                # poze pelvis ~0.75 m z przechylami -> flatten zeruje z/rp).
                'base_frame': 'base_footprint',
                'flatten': True,
                'use_msg_frame_ids': False,
                'use_msg_stamp': True,
                'max_rate': 50.0,
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
                # Baza 2D na podlodze (spaw wyzej), nie pelvis z yaml.
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
