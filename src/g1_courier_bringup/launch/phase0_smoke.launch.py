"""Phase 0 smoke launch — exercise the mission BT against fake action servers.

Brings up:
  - sim_cmd_vel_bridge_node       (kinematic /cmd_vel -> TF + /odom)
  - sim_lowstate_publisher_node   (idle /lowstate so arm-related code wakes up)
  - cmd_vel_arbiter               (REAL — provides /safety/set_carry_mode)
  - fake_{navigate,dock,pick,place,retreat}_action_server
  - mission_node (delayed 3 s so action servers are ready first)

Expected: BT logs `pickup_at_table_a -> transfer_to_table_b -> pickup_at_table_b
-> transfer_to_table_a` with each fake reporting "fake done", then idles
(no loop decorator yet).
"""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


# How long each fake action sleeps before returning success. Short = fast cycle.
FAKE_DURATION_S = 1.0


def _fake(executable: str) -> Node:
    return Node(
        package='g1_courier_sim',
        executable=executable,
        name=executable,
        parameters=[{'duration_s': FAKE_DURATION_S, 'succeed': True}],
    )


def generate_launch_description() -> LaunchDescription:
    safety_share = get_package_share_directory('g1_courier_safety')

    return LaunchDescription([
        # Sim base + sensors fixture.
        Node(package='g1_courier_sim', executable='sim_cmd_vel_bridge_node',
             name='sim_cmd_vel_bridge_node'),
        Node(package='g1_courier_sim', executable='sim_lowstate_publisher_node',
             name='sim_lowstate_publisher_node'),

        # Real arbiter so /safety/set_carry_mode service exists.
        Node(package='g1_courier_safety', executable='cmd_vel_arbiter',
             name='cmd_vel_arbiter',
             parameters=[os.path.join(safety_share, 'config', 'safety.yaml')]),

        # Fake action servers (one process each, single-purpose).
        _fake('fake_navigate_proxy'),
        _fake('fake_dock_action_server'),
        _fake('fake_pick_action_server'),
        _fake('fake_place_action_server'),
        _fake('fake_retreat_action_server'),

        # Mission BT — delayed so all action servers are discovered first.
        # tree.setup() is fast (only creates clients), but the first tick calls
        # wait_for_server(1.0) on each behaviour; missing a server -> FAILURE.
        TimerAction(period=3.0, actions=[
            Node(package='g1_courier_mission', executable='mission_node',
                 name='mission_node'),
        ]),
    ])
