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
         scans.pcd  (zapisany w cwd lub ~/.ros)

Wymagane (clone do src/, build osobno) — pełna instrukcja:
docs/fast_lio_setup.md. Streszczenie:
  git clone https://github.com/Livox-SDK/Livox-SDK2 ~/
  cd ./Livox-SDK2/
  mkdir build
  cd build
  cmake .. && make -j
  sudo make install
  sudo ldconfig

  git clone --recursive https://github.com/Ericsii/FAST_LIO_ROS2 src/FAST_LIO_ROS2
  git clone https://github.com/Livox-SDK/livox_ros_driver2 src/livox_ros_driver2
  sudo apt install ros-humble-pcl-ros ros-humble-pcl-conversions ros-humble-pcl-msgs
  cd src/livox_ros_driver2 && ./build.sh humble && cd ../..
  rosdep install --from-paths src --ignore-src -y
  colcon build --packages-select fast_lio livox_interfaces g1_courier_fastlio
  source install/setup.bash

Plus assumption: Livox driver publikuje na /livox/lidar + /livox/imu.
Jeśli twoje topiki to /utlidar/cloud_livox_360mid + /utlidar/imu,
edytuj `config/g1_mid360.yaml` (klucz `common.lid_topic` / `imu_topic`).

Uruchomienie lidaru z drivera
ros2 launch livox_ros_driver2 msg_MID360_launch.py
Nalezy zedytowac wczesniej MID360_config.json i ustawic IP lidaru

Uruchomienie:
  ros2 launch g1_courier_fastlio fastlio_mapping.launch.py
  # Robot teleop'em ~0.1 m/s — wolno, żeby FAST-LIO nie traci track.
  # Gdy mapa wygląda kompletnie:
  ros2 service call /map_save std_srvs/srv/Trigger
"""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('g1_courier_fastlio')
    default_config = os.path.join(pkg_share, 'config', 'g1_mid360.yaml')

    # Default RViz preset z fast_lio (Ericsii ROS2 port).
    # Jeśli pakiet zainstalowany, użyj jego presetu; inaczej fallback do
    # gołego rviz2 (Fixed Frame ustaw ręcznie na "camera_init").
    try:
        fast_lio_share = get_package_share_directory('fast_lio')
        default_rviz = os.path.join(fast_lio_share, 'rviz', 'fastlio.rviz')
    except Exception:
        default_rviz = ''

    config_arg = DeclareLaunchArgument(
        'config_file', default_value=default_config,
        description='Path to FAST-LIO config (default: g1_mid360.yaml — pre-tuned dla G1).',
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Czy odpalić RViz z presetem FAST-LIO.',
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value=default_rviz,
        description='RViz config file (default: fast_lio/rviz/fastlio.rviz).',
    )

    rviz_args = ['-d', LaunchConfiguration('rviz_config')] if default_rviz else []

    return LaunchDescription([
        config_arg,
        rviz_arg,
        rviz_config_arg,

        # FAST-LIO mapping node. Pakiet `fast_lio` jest dostarczony przez
        # external clone FAST_LIO_ROS2 (patrz docs/fast_lio_setup.md).
        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            name='laserMapping',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
        ),

        # RViz z presetem FAST-LIO. Default config (fastlio.rviz) pokazuje
        # /cloud_registered + /path + /Odometry z Fixed Frame "camera_init".
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_fastlio',
            arguments=rviz_args,
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen',
        ),
    ])
