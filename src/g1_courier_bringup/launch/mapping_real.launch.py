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

On ROS2 Humble, async_slam_toolbox_node is a regular rclcpp node, not a
lifecycle node, so it must not be managed by nav2_lifecycle_manager.
"""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    bringup = get_package_share_directory('g1_courier_bringup')

    default_urdf = os.path.join(
        get_package_share_directory('g1_description'),
        'urdf', 'g1_29dof.urdf',
    )

    urdf_arg = DeclareLaunchArgument(
        'urdf_path', default_value=default_urdf,
        description='Absolute path to G1 URDF. Default uses the vendored '
                    'g1_description package; override to point elsewhere.',
    )
    enable_robot_model_arg = DeclareLaunchArgument(
        'enable_robot_model', default_value='true',
        description='Start robot_state_publisher + lowstate_to_joint_states for '
                    'TF and RViz RobotModel display. Set false to skip when URDF '
                    'is missing or for headless/minimal runs (slam still works '
                    'without RSP — only model viz disappears).',
    )
    cloud_topic_arg = DeclareLaunchArgument(
        'cloud_topic', default_value='/livox/lidar',
        description='PointCloud2 source topic (Unitree firmware default).',
    )
    lidar_frame_arg = DeclareLaunchArgument(
        'lidar_frame_id', default_value='livox_frame',
        description='frame_id stamped on lidar messages by Unitree firmware. '
                    'Verify with: ros2 topic echo <cloud_topic> --field header.frame_id --once',
    )

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('urdf_path')]),
        value_type=str,
    )

    cloud_topic = LaunchConfiguration('cloud_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    odom_frame = LaunchConfiguration('odom_frame')
    base_frame = LaunchConfiguration('base_frame')
    lidar_frame = LaunchConfiguration('lidar_frame')

    enable_robot_model = IfCondition(LaunchConfiguration('enable_robot_model'))

    return LaunchDescription([
        urdf_arg,
        enable_robot_model_arg,
        cloud_topic_arg,
        lidar_frame_arg,

        # robot_state_publisher: TF from base_link to every URDF link.
        # Gated on enable_robot_model — set false if URDF missing.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            condition=enable_robot_model,
        ),

        # /lowstate -> /joint_states adapter so robot_state_publisher
        # has live joint angles to compute TF.
        Node(
            package='g1_courier_safety',
            executable='lowstate_to_joint_states',
            name='lowstate_to_joint_states',
            condition=enable_robot_model,
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
                '--x', '0.0', '--y', '0.0', '--z', '0.5',
                '--roll', '3.14159', '--pitch', '0.0', '--yaw', '0.0',
                '--frame-id', 'base_link',
                '--child-frame-id', LaunchConfiguration('lidar_frame_id'),
            ],
        ),

        # PointCloud2 -> 2D LaserScan for slam_toolbox.
        # input_queue_size bumped from default 10 — Livox publishes ~10 Hz
        # large clouds and TF lookup latency can fill the default queue.
        DeclareLaunchArgument(
            'cloud_topic',
            default_value='/utlidar/cloud_livox_mid360',
            description='Input PointCloud2 topic from Livox driver.',
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan',
            description='Output LaserScan topic used by slam_toolbox.',
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/dog_odom',
            description='Real odometry topic used as source for odom->base TF relay.',
        ),
        DeclareLaunchArgument(
            'odom_frame',
            default_value='odom',
            description='Global odometry frame name for slam_toolbox.',
        ),
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_link',
            description='Robot base frame for scan projection and slam_toolbox.',
        ),
        DeclareLaunchArgument(
            'lidar_frame',
            default_value='livox_frame',
            description='Frame id of incoming Livox point clouds.',
        ),
        DeclareLaunchArgument(
            'publish_lidar_static_tf',
            default_value='true',
            description='Publish base_frame->lidar_frame static TF fallback for setups with missing driver TF.',
        ),
        DeclareLaunchArgument('lidar_x', default_value='0.0'),
        DeclareLaunchArgument('lidar_y', default_value='0.0'),
        DeclareLaunchArgument('lidar_z', default_value='0.0'),
        DeclareLaunchArgument('lidar_roll', default_value='0.0'),
        DeclareLaunchArgument('lidar_pitch', default_value='0.0'),
        DeclareLaunchArgument('lidar_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'publish_odom_tf',
            default_value='true',
            description='Publish odom->base_link TF from odometry topic.',
        ),

        # Fallback static TF to avoid message filter backlog when the runtime
        # environment does not provide livox_frame -> base_link.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_lidar_static_tf',
            condition=IfCondition(LaunchConfiguration('publish_lidar_static_tf')),
            arguments=[
                '--x', LaunchConfiguration('lidar_x'),
                '--y', LaunchConfiguration('lidar_y'),
                '--z', LaunchConfiguration('lidar_z'),
                '--yaw', LaunchConfiguration('lidar_yaw'),
                '--pitch', LaunchConfiguration('lidar_pitch'),
                '--roll', LaunchConfiguration('lidar_roll'),
                '--frame-id', base_frame,
                '--child-frame-id', lidar_frame,
            ],
        ),

        Node(
            package='g1_courier_bringup',
            executable='odom_tf_relay',
            name='odom_tf_relay',
            condition=IfCondition(LaunchConfiguration('publish_odom_tf')),
            parameters=[{
                'odom_topic': odom_topic,
                'odom_frame': odom_frame,
                'base_frame': base_frame,
                'use_msg_frame_ids': False,
                'use_msg_stamp': True,
            }],
        ),

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
        ),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                os.path.join(bringup, 'config', 'slam_toolbox_mapping.yaml'),
                {
                    'scan_topic': scan_topic,
                    'odom_frame': odom_frame,
                    'base_frame': base_frame,
                },
            ],
        ),
    ])
