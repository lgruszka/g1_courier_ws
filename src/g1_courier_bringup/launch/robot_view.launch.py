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
  ros2 launch g1_courier_bringup robot_view.launch.py
  # bez RViz (headless):
  ros2 launch g1_courier_bringup robot_view.launch.py enable_rviz:=false
  # bez LiDAR (sam model + TF):
  ros2 launch g1_courier_bringup robot_view.launch.py enable_lidar:=false
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
        DeclareLaunchArgument('enable_lidar', default_value='true',
            description='Odpal pointcloud_to_laserscan + static TF base_link→lidar.'),
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

        # Static TF base_link → lidar frame. Mid-360 ~1.45 m nad pelvis
        # (głowa). MEASURE FIZYCZNIE i override przez lidar_frame_id +
        # static_tf params jeśli twój mount jest inny.
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
            condition=IfCondition(LaunchConfiguration('enable_lidar')),
        ),

        # PointCloud2 → 2D LaserScan dla RViz (opcjonalne — 3D PointCloud2
        # display też zadziała, ale 2D skan jest lżejszy).
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            parameters=[
                os.path.join(bringup, 'config', 'pointcloud_to_laserscan.yaml'),
                {'input_queue_size': 50},
                {'target_frame': 'base_link'},
            ],
            remappings=[('cloud_in', LaunchConfiguration('cloud_topic')),
                        ('scan', '/scan')],
            condition=IfCondition(LaunchConfiguration('enable_lidar')),
        ),

        # RViz z presetem robot_view.rviz.
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_robot_view',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_rviz')),
        ),
    ])
