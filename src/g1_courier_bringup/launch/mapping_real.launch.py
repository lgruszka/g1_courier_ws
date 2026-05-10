"""Real-robot SLAM mapping run.

Drive the robot manually (e.g. via teleop_twist_keyboard) while slam_toolbox
builds a map from the Livox 2D-slice scan. Save the resulting map with:

  ros2 run nav2_map_server map_saver_cli -f ~/maps/lab

Then point real.launch.py 'map' arg at the saved YAML.

This launch DOES NOT bring up nav2, AMCL, or the mission stack — it is a
mapping-only run. Locomotion comes from the Unitree firmware sport API
consuming /cmd_vel directly (assumed already running).

TF tree wiring:
  - odom -> base_link        published by Unitree firmware
  - base_link -> body links  published by robot_state_publisher (URDF)
  - base_link -> lidar_frame published by static_transform_publisher below
  - /joint_states            published by lowstate_to_joint_states adapter
                              (converts /lowstate motor_state[i].q)

Override defaults via launch args:
  ros2 launch g1_courier_bringup mapping_real.launch.py \\
      cloud_topic:=/livox/lidar \\
      lidar_frame_id:=livox_frame \\
      urdf_path:=$HOME/path/to/g1.urdf
"""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    bringup = get_package_share_directory('g1_courier_bringup')

    default_urdf = os.path.expanduser(
        '~/g1_courier_ws/src/unitree_ros/robots/g1_description/g1_29dof.urdf'
    )

    urdf_arg = DeclareLaunchArgument(
        'urdf_path', default_value=default_urdf,
        description='Absolute path to G1 URDF (from unitree_ros/robots/g1_description).',
    )
    cloud_topic_arg = DeclareLaunchArgument(
        'cloud_topic', default_value='/utlidar/cloud_livox_360mid',
        description='PointCloud2 source topic (Unitree firmware default).',
    )
    lidar_frame_arg = DeclareLaunchArgument(
        'lidar_frame_id', default_value='utlidar_lidar',
        description='frame_id stamped on lidar messages by Unitree firmware. '
                    'Verify with: ros2 topic echo <cloud_topic> --field header.frame_id --once',
    )

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('urdf_path')]),
        value_type=str,
    )

    return LaunchDescription([
        urdf_arg,
        cloud_topic_arg,
        lidar_frame_arg,

        # robot_state_publisher: TF from base_link to every URDF link.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),

        # /lowstate -> /joint_states adapter so robot_state_publisher
        # has live joint angles to compute TF.
        Node(
            package='g1_courier_safety',
            executable='lowstate_to_joint_states',
            name='lowstate_to_joint_states',
        ),

        # Static TF base_link -> lidar frame. Unitree firmware does NOT
        # publish this. Default values assume Mid-360 mounted on G1 head
        # ~1.45 m above pelvis. MEASURE PHYSICALLY and override if wrong:
        #   ros2 run tf2_ros static_transform_publisher
        #     --x 0 --y 0 --z 1.45 --frame-id base_link --child-frame-id <frame>
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

        # PointCloud2 -> 2D LaserScan for slam_toolbox.
        # input_queue_size bumped from default 10 — Livox publishes ~10 Hz
        # large clouds and TF lookup latency can fill the default queue.
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            parameters=[
                os.path.join(bringup, 'config', 'pointcloud_to_laserscan.yaml'),
                {'input_queue_size': 50},
            ],
            remappings=[('cloud_in', LaunchConfiguration('cloud_topic')),
                        ('scan', '/scan')],
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
