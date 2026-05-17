"""View-only launch — wizualizacja G1 + LiDAR + TF w RViz.

Bez nav2, bez AMCL, bez mission BT — sama wizualizacja realnego
robota. Użyteczne do:
  - sprawdzenia czy URDF wczytuje się poprawnie
  - obserwacji live joint angles (czy /lowstate dochodzi z onboard PC)
  - podglądu skanu Mid-360 i chmury punktów
  - debugowania TF tree przed odpaleniem real.launch.py

Zakłada że robot publikuje przez DDS:
  - /lowstate (Unitree firmware, ~500 Hz)
  - /livox/lidar (livox_ros_driver2, ~10 Hz) lub override przez cloud_topic

TF tree po starcie:
  base_link → pelvis → torso → head → ... (z URDF, via robot_state_publisher)
  base_link → <lidar_frame_id> (static TF, default livox_frame @ z=1.45)

Fixed Frame w RViz: `base_link` (robot jest "kotwicą", świat się rusza).

Uruchomienie:
  # sam model robota + RViz (default — bez LiDAR-a, bez kamery):
  ros2 launch g1_courier_bringup robot_view.launch.py
  # plus LiDAR (Mid-360 → /scan, static TF, display LaserScan w RViz):
  ros2 launch g1_courier_bringup robot_view.launch.py enable_lidar:=true
  # plus kamera (D435i → /camera/image_raw + /camera/depth/...):
  ros2 launch g1_courier_bringup robot_view.launch.py enable_camera:=true
  # plus odom (robot porusza się po RViz zamiast dreptać w miejscu;
  # wymaga /dog_odom z firmware Unitree):
  ros2 launch g1_courier_bringup robot_view.launch.py enable_odom:=true
  # wszystko naraz:
  ros2 launch g1_courier_bringup robot_view.launch.py \
    enable_lidar:=true enable_camera:=true enable_odom:=true
  # headless (bez RViz):
  ros2 launch g1_courier_bringup robot_view.launch.py enable_rviz:=false
"""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    AndSubstitution, Command, LaunchConfiguration, NotSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    bringup = get_package_share_directory('g1_courier_bringup')
    default_urdf = os.path.join(
        get_package_share_directory('g1_description'),
        'urdf', 'g1_29dof.urdf',
    )
    default_rviz = os.path.join(bringup, 'rviz', 'robot_view.rviz')

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('urdf_path')]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument('urdf_path', default_value=default_urdf,
            description='Absolute path to G1 URDF (vendored g1_description).'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz,
            description='RViz config — domyślny robot_view.rviz w bringup/rviz/.'),
        DeclareLaunchArgument('enable_rviz', default_value='true',
            description='Odpal RViz. False = headless (tylko TF + joint_states).'),
        DeclareLaunchArgument('enable_lidar', default_value='false',
            description='Odpal pointcloud_to_laserscan + static TF base_link→lidar. '
                        'Default false — sam model robota. Set true żeby zobaczyć '
                        '/scan w RViz.'),
        DeclareLaunchArgument('enable_camera', default_value='false',
            description='Odpal d435i_node (RealSense D435i RGB+depth) plus pokaż '
                        '/camera/image_raw w RViz Image display. Default false.'),
        DeclareLaunchArgument('enable_odom', default_value='false',
            description='Odpal odom_tf_relay (/dog_odom → TF odom→base_link) plus '
                        'zmień Fixed Frame w RViz na `odom`. Wtedy robot porusza '
                        'się po przestrzeni RViz zamiast dreptać w miejscu. '
                        'Default false — robot jako kotwica w (0,0,0).'),
        DeclareLaunchArgument('odom_topic', default_value='/dog_odom',
            description='Topic z nav_msgs/Odometry — firmware odom. Override '
                        'np. na /Odometry jeśli używasz FAST-LIO.'),
        DeclareLaunchArgument('cloud_topic', default_value='/livox/lidar',
            description='PointCloud2 source topic (livox_ros_driver2 default).'),
        DeclareLaunchArgument('lidar_frame_id', default_value='livox_frame',
            description='frame_id na cloud_topic — używane przez static TF.'),

        # robot_state_publisher: TF od base_link do wszystkich linków URDF
        # plus publikacja /robot_description (RViz RobotModel display).
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),

        # /lowstate (Unitree firmware) → /joint_states. RSP używa angles
        # do policzenia transform body links.
        Node(
            package='g1_courier_safety',
            executable='lowstate_to_joint_states',
            name='lowstate_to_joint_states',
        ),

        # Static TF base_link → pelvis (identity). URDF root to `pelvis`,
        # ale Fixed Frame w RViz to `base_link` — bez tego mostka RViz
        # nie potrafi rozwinąć drzewa URDF (każdy link rzuca "No transform
        # from <link> to base_link"). Identity bo na G1 base_link ≡ pelvis.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_pelvis_tf',
            arguments=['0', '0', '0', '0', '0', '0', '1',
                       'base_link', 'pelvis'],
        ),

        # Static TF base_link → lidar frame. Always-on — opisuje fizyczny
        # montaż Mid-360 na głowie (~1.45 m nad pelvis), niezależnie od
        # tego czy uruchamiamy pointcloud_to_laserscan. MEASURE FIZYCZNIE
        # i override przez lidar_frame_id + static_tf params jeśli mount
        # inny.
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

        # PointCloud2 → 2D LaserScan dla RViz (opcjonalne — 3D PointCloud2
        # display też zadziała, ale 2D skan jest lżejszy).
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
            condition=IfCondition(LaunchConfiguration('enable_lidar')),
        ),

        # RealSense D435i — kolor + depth. Domyślnie wyłączone, włącz
        # `enable_camera:=true`. Wymaga pyrealsense2 + opencv-python.
        Node(
            package='g1_courier_bringup',
            executable='d435i_node',
            name='d435i_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_camera')),
        ),

        # odom_tf_relay — czyta nav_msgs/Odometry z firmware i publikuje
        # TF odom→base_link. Bez tego robot stoi w (0,0,0) bo nic nie
        # aktualizuje pozycji base_link względem zewnętrznego frame'u.
        Node(
            package='g1_courier_bringup',
            executable='odom_tf_relay',
            name='odom_tf_relay',
            parameters=[{
                'odom_topic': LaunchConfiguration('odom_topic'),
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'use_msg_frame_ids': False,
                'use_msg_stamp': True,
            }],
            condition=IfCondition(LaunchConfiguration('enable_odom')),
        ),

        # RViz — dwa warianty zależnie od enable_odom:
        # - bez odom (default): Fixed Frame z preset (base_link), robot kotwica
        # - z odom: -f odom override Fixed Frame, robot porusza się po RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_robot_view',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            output='screen',
            condition=IfCondition(AndSubstitution(
                LaunchConfiguration('enable_rviz'),
                NotSubstitution(LaunchConfiguration('enable_odom')),
            )),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_robot_view',
            arguments=['-d', LaunchConfiguration('rviz_config'), '-f', 'odom'],
            output='screen',
            condition=IfCondition(AndSubstitution(
                LaunchConfiguration('enable_rviz'),
                LaunchConfiguration('enable_odom'),
            )),
        ),
    ])
