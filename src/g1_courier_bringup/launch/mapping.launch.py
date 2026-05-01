"""Launches sensors + slam_toolbox in online_async mapping mode.

Drive the robot manually through the area, then save the map with
  ros2 run nav2_map_server map_saver_cli -f ~/maps/lab
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
    ])
