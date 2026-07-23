"""Wspolny tor sensoryczny + TF dla real/mapping/localization.

Jedno zrodlo prawdy dla:
  - drzewa TF (REP-105/120):
      odom -> base_footprint          odom_tf_relay (flatten: x/y/yaw z /dog_odom)
      base_footprint -> base_link     static, footprint_z (pelvis nad podloga)
      base_link -> pelvis             static identity (root URDF to pelvis)
      base_link -> <lidar_frame_id>   static, montaz per-robot (lidar_* argi)
      base_link -> linki URDF         robot_state_publisher + lowstate adapter
  - toru skanu: chmura -> [parcel_cropbox] -> pointcloud_to_laserscan -> /scan
    (+ scan_watchdog; respawn na wezlach toru — znane zaciecia p2l na G1).

Montaz lidaru i wysokosc bazy sa PER-ROBOT: zmierz RANSAC-iem plaszczyzny
podlogi z chmury (fit normalnej -> roll/pitch i wysokosc) i podaj argami;
defaulty = montaz nominalny (lidar do gory nogami, roll=pi).

Uzycie (z launcha nadrzednego):
    IncludeLaunchDescription(... 'sensors_tf.launch.py',
        launch_arguments={'cloud_topic': ..., 'filter_parcel': ..., ...})
"""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
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
        # --- argi: model robota ---
        DeclareLaunchArgument('urdf_path', default_value=default_urdf,
            description='Absolute path to G1 URDF.'),
        DeclareLaunchArgument('enable_robot_model', default_value='true',
            description='RSP + lowstate_to_joint_states + spaw base_link->pelvis.'),

        # --- argi: odometria/TF ---
        DeclareLaunchArgument('odom_topic', default_value='/dog_odom',
            description='Odometry topic for odom->base TF relay.'),
        DeclareLaunchArgument('odom_frame', default_value='odom',
            description='Odometry frame name.'),
        DeclareLaunchArgument('base_frame', default_value='base_footprint',
            description='Child frame publikowany przez relay. Z domyslnym '
                        'spawem base_footprint->base_link NIE ustawiaj tu '
                        'base_link (dwoch rodzicow = zepsute drzewo TF).'),
        DeclareLaunchArgument('publish_odom_tf', default_value='true',
            description='Publish odom->base TF from odometry topic.'),
        DeclareLaunchArgument('odom_use_msg_stamp', default_value='true',
            description='Stampuj TF czasem wiadomosci odom (false = now()).'),
        DeclareLaunchArgument('odom_max_rate', default_value='50.0',
            description='Throttle TF z odometrii [Hz] (0 = 1:1; /dog_odom '
                        'na G1 to ~950 Hz — patrz odom_tf_relay).'),
        DeclareLaunchArgument('odom_trans_scale', default_value='1.0',
            description='Korekta skali translacji odometrii PER-ROBOT '
                        '(1.0 = bez zmian; biped G1 potrafi niedoszacowywac '
                        'dystans — zmierz narzedziem verdict, ustaw 1/ratio).'),

        # --- argi: montaz PER-ROBOT (pomiar RANSAC podlogi) ---
        DeclareLaunchArgument('footprint_z', default_value='0.75',
            description='Wysokosc base_link (pelvis) nad podloga [m].'),
        DeclareLaunchArgument('lidar_z', default_value='0.5',
            description='Wysokosc lidaru nad base_link [m].'),
        DeclareLaunchArgument('lidar_roll', default_value='3.14159',
            description='Roll montazu lidaru [rad] (do gory nogami = pi).'),
        DeclareLaunchArgument('lidar_pitch', default_value='0.0',
            description='Pitch montazu lidaru [rad] (zmierz RANSAC).'),
        DeclareLaunchArgument('lidar_frame_id', default_value='livox_frame',
            description='frame_id stamped on lidar messages.'),

        # --- argi: tor skanu ---
        DeclareLaunchArgument('cloud_topic', default_value='/livox/lidar',
            description='PointCloud2 source topic.'),
        DeclareLaunchArgument('filter_parcel', default_value='false',
            description='Wytnij bryle niesionej paczki cropboxem 3D PRZED '
                        'p2l (pelny stack courier: true).'),
        DeclareLaunchArgument('pointcloud_start_delay', default_value='0.0',
            description='Opoznienie startu toru skanu [s].'),
        DeclareLaunchArgument('enable_scan_watchdog', default_value='true',
            description='Watchdog ciszy na /scan (restart toru przez respawn).'),

        # --- model robota ---
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
        # Root URDF to pelvis; stack uzywa base_link — spaw identity.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_pelvis_tf',
            arguments=['0', '0', '0', '0', '0', '0', '1',
                       'base_link', 'pelvis'],
            condition=enable_robot_model,
        ),

        # --- TF bazy i lidaru ---
        # REP-120: baza nawigacji 2D plasko na podlodze; mapa pod stopami.
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
        # Firmware NIE publikuje TF lidaru. Sam roll=pi przy realnym pitchu
        # montazu daje skosny pas ciecia (~0.7 m/10 m na 4 st) i rozmyte
        # sciany na mapie — dlatego orientacja jest argiem per-robot.
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
                'odom_frame': LaunchConfiguration('odom_frame'),
                'base_frame': LaunchConfiguration('base_frame'),
                'use_msg_frame_ids': False,
                'use_msg_stamp': LaunchConfiguration('odom_use_msg_stamp'),
                # Firmware daje poze PELVIS (z~0.75, przechyly tulowia) —
                # do bazy 2D rzutujemy na podloge (x/y/yaw).
                'flatten': True,
                'max_rate': LaunchConfiguration('odom_max_rate'),
                'trans_scale': LaunchConfiguration('odom_trans_scale'),
            }],
        ),

        # --- tor skanu (respawn + watchdog: znane zaciecia p2l na G1) ---
        TimerAction(
            period=LaunchConfiguration('pointcloud_start_delay'),
            actions=[
                # Cropbox 3D: chmura -> /livox/lidar_filtered (bez bryly
                # paczki) — p2l za paczka bierze sciane widziana PONAD nia.
                Node(
                    package='g1_courier_bringup',
                    executable='parcel_cropbox',
                    name='parcel_cropbox',
                    parameters=[os.path.join(bringup, 'config',
                                             'parcel_cropbox.yaml')],
                    remappings=[('input', LaunchConfiguration('cloud_topic')),
                                ('output', '/livox/lidar_filtered')],
                    respawn=True, respawn_delay=2.0,
                    condition=IfCondition(LaunchConfiguration('filter_parcel')),
                ),
                Node(
                    package='pointcloud_to_laserscan',
                    executable='pointcloud_to_laserscan_node',
                    name='pointcloud_to_laserscan',
                    parameters=[
                        os.path.join(bringup, 'config',
                                     'pointcloud_to_laserscan.yaml'),
                        {'queue_size': 50},
                    ],
                    remappings=[('cloud_in', '/livox/lidar_filtered'),
                                ('scan', '/scan')],
                    respawn=True, respawn_delay=2.0,
                    condition=IfCondition(LaunchConfiguration('filter_parcel')),
                ),
                Node(
                    package='pointcloud_to_laserscan',
                    executable='pointcloud_to_laserscan_node',
                    name='pointcloud_to_laserscan',
                    parameters=[
                        os.path.join(bringup, 'config',
                                     'pointcloud_to_laserscan.yaml'),
                        {'queue_size': 50},
                    ],
                    remappings=[('cloud_in', LaunchConfiguration('cloud_topic')),
                                ('scan', '/scan')],
                    respawn=True, respawn_delay=2.0,
                    condition=UnlessCondition(LaunchConfiguration('filter_parcel')),
                ),
                Node(
                    package='g1_courier_bringup',
                    executable='scan_watchdog',
                    name='scan_watchdog',
                    respawn=True, respawn_delay=2.0,
                    condition=IfCondition(LaunchConfiguration('enable_scan_watchdog')),
                ),
            ],
        ),
    ])
