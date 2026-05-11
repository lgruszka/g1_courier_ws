"""Standalone FAST-LIO2 mapping dla G1 + Mid-360.

Niezależny od mapping_real.launch.py — odpalaj **zamiast** niego (nie
jednocześnie) gdy chcesz porównać mapowanie 2D slam_toolbox vs 3D FAST-LIO.

Pipeline:
  /livox/lidar + /livox/imu  (Livox driver, hardware-synced)
                ↓
         fast_lio (LIO + iEKF, 3D)
                ↓
  /Odometry  (PoseStamped, low-drift, 6-DoF)
  /path      (Path, full trajectory)
  /cloud_registered  (PointCloud2, accumulated 3D map)
                ↓
  ros2 service call /map_save std_srvs/srv/Trigger
                ↓
         g1_map.pcd  (zapisany w cwd)

Wymagane (clone do src/, build osobno):
  git clone https://github.com/Ericsii/fast_lio_ros2 src/fast_lio_ros2
  git clone https://github.com/Livox-SDK/livox_ros_driver2 src/livox_ros_driver2
  colcon build --packages-select livox_interfaces livox_ros_driver2 fast_lio
  source install/setup.bash

Plus assumption: Unitree firmware publikuje chmurę na /livox/lidar +
IMU na /livox/imu. Jeśli twoje topiki to /utlidar/cloud_livox_360mid +
/utlidar/imu, dodaj remap przez launch arg `lidar_topic` / `imu_topic`
(default w configu g1_mid360.yaml — zmień tam albo użyj remap node'a).

Uruchomienie:
  ros2 launch g1_courier_fastlio fastlio_mapping.launch.py
  # Robot teleop'em ~0.1 m/s — wolno, żeby FAST-LIO nie traci track.
  # Gdy mapa wygląda kompletnie:
  ros2 service call /map_save std_srvs/srv/Trigger

Plus dokumentacja krok po kroku: docs/fast_lio_setup.md.
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
    pkg_share = get_package_share_directory('g1_courier_fastlio')
    default_config = os.path.join(pkg_share, 'config', 'g1_mid360.yaml')

    config_arg = DeclareLaunchArgument(
        'config_file', default_value=default_config,
        description='Path to FAST-LIO config (default: g1_mid360.yaml — pre-tuned dla G1).',
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Czy odpalić RViz z presetem FAST-LIO.',
    )

    return LaunchDescription([
        config_arg,
        rviz_arg,

        # FAST-LIO mapping node. Pakiet `fast_lio` jest dostarczony przez
        # external clone fast_lio_ros2 (patrz docs/fast_lio_setup.md).
        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            name='laserMapping',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
        ),

        # RViz z presetem FAST-LIO (jeśli istnieje w fast_lio_ros2 share).
        # Domyślny RViz config fast_lio pokazuje /cloud_registered + /path +
        # /Odometry. Plus przełącz Fixed Frame na "camera_init" (frame FAST-LIO).
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_fastlio',
            arguments=['-d', os.path.join(
                get_package_share_directory('fast_lio'),
                'rviz_cfg', 'loam_livox.rviz'),
            ],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen',
        ),
    ])
