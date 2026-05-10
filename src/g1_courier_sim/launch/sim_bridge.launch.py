"""Launch the courier MuJoCo bridge (Ubuntu native sim — Option C).

Runs the MuJoCo viewer + simulation thread + DDS publishers/subscribers.
Equivalent to running mac unitree_mujoco standalone, but native on Linux.

Usage:
    ros2 launch g1_courier_sim sim_bridge.launch.py

Then in another terminal start the mission stack:
    ros2 launch g1_courier_bringup phase1_full.launch.py

Prerequisites (one-time install):
    pip install mujoco unitree_sdk2py pupil-apriltags pygame opencv-python
    # Plus mesh STL files — see docs/sim_setup_ubuntu.md.

Optional env vars:
    G1_COURIER_SCENE   override path to scene XML (default: bundled scene_courier.xml)
    G1_COURIER_INTERFACE   DDS network interface (default: 'lo')
    ROS_DOMAIN_ID      DDS domain (default: 0)
"""
from __future__ import annotations

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='g1_courier_sim',
            executable='sim_bridge_node',
            name='sim_bridge',
            output='screen',
            emulate_tty=True,
        ),
    ])
