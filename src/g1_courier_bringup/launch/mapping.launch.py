"""Mapping launch — slam_toolbox builds a map of the mac MuJoCo scene
while the user manually teleop-drives the welded robot via /cmd_vel.

PREREQUISITE: unitree_mujoco running on mac (publishes rt/scan from the
mj_ray() lidar — Faza 1.2). Mac kinematic mocap movement integrates
/cmd_vel into pelvis_anchor.mocap_pos, so the robot physically slides
through the scene without a real walking controller (Faza 1.3 deferred).

Linux side (this launch):
  - sim_cmd_vel_bridge_node integrates /cmd_vel into /odom and publishes
    odom -> base_link TF. WITHOUT static map -> odom (slam_toolbox owns it).
  - static_transform_publisher base_link -> pelvis (welded robot, identity)
    plus pelvis -> lidar_link (offset 0,0,-0.393).
  - slam_toolbox 'mapping' mode subscribes /scan, publishes /map and
    map -> odom TF.

Workflow:
  1. ros2 launch g1_courier_bringup mapping.launch.py
  2. (terminal 2) ros2 run teleop_twist_keyboard teleop_twist_keyboard
     drive the robot around: i/j/k/l/u/o/m/, keys
  3. (optional) rviz2 — add Map (/map), TF, LaserScan (/scan) displays
  4. When map looks good:
        ros2 run nav2_map_server map_saver_cli -f ~/maps/lab
     Saves ~/maps/lab.yaml + lab.pgm. Faza 1.5 (AMCL+nav2) will load it.
"""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup = get_package_share_directory('g1_courier_bringup')
    slam_params = os.path.join(bringup, 'config', 'slam_toolbox_mapping.yaml')

    return LaunchDescription([
        # Kinematic integrator — publishes /odom + odom->base_link TF.
        # WITHOUT static map->odom (slam_toolbox publishes that one).
        Node(package='g1_courier_sim', executable='sim_cmd_vel_bridge_node',
             name='sim_cmd_vel_bridge_node',
             parameters=[{'publish_map_to_odom': False}]),

        # base_link -> pelvis (welded robot stays upright; identity TF lets
        # slam_toolbox find lidar via base_link, since rt/scan is
        # frame_id=lidar_link and slam wants base_frame=base_link).
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='base_link_to_pelvis_tf',
             arguments=['0', '0', '0',
                        '0', '0', '0', '1',
                        'base_link', 'pelvis']),

        # pelvis -> lidar_link — same offset as in phase1_smoke.launch.py.
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='lidar_static_tf',
             arguments=['0', '0', '-0.393',
                        '0', '0', '0', '1',
                        'pelvis', 'lidar_link']),

        # slam_toolbox in async mapping mode. Publishes /map and map->odom.
        # In Jazzy this is a lifecycle node — won't subscribe to /scan or
        # publish /map until it's transitioned to 'active'. The
        # nav2_lifecycle_manager below auto-configures + activates it on
        # startup; without it the node sits in 'unconfigured' state.
        Node(package='slam_toolbox', executable='async_slam_toolbox_node',
             name='slam_toolbox',
             parameters=[slam_params],
             output='screen'),
        Node(package='nav2_lifecycle_manager',
             executable='lifecycle_manager',
             name='lifecycle_manager_slam',
             output='screen',
             parameters=[{
                 'autostart': True,
                 'bond_timeout': 0.0,   # slam_toolbox doesn't use bonds
                 'node_names': ['slam_toolbox'],
             }]),
    ])
