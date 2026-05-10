"""Real-robot SLAM mapping run.

Drive the robot manually (e.g. via teleop_twist_keyboard) while slam_toolbox
builds a map from the Livox 2D-slice scan. Save the resulting map with:

  ros2 run nav2_map_server map_saver_cli -f ~/maps/lab

Then point real.launch.py 'map' arg at the saved YAML.

This launch DOES NOT bring up nav2, AMCL, or the mission stack — it is a
mapping-only run. Locomotion comes from the Unitree firmware sport API
consuming /cmd_vel directly (assumed already running).
"""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup = get_package_share_directory('g1_courier_bringup')

    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            parameters=[os.path.join(bringup, 'config', 'pointcloud_to_laserscan.yaml')],
            remappings=[('cloud_in', '/livox/lidar'), ('scan', '/scan')],
        ),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[os.path.join(bringup, 'config', 'slam_toolbox_mapping.yaml')],
        ),

        # Lifecycle manager so slam_toolbox auto-activates.
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            parameters=[{'autostart': True, 'node_names': ['slam_toolbox']}],
        ),
    ])
