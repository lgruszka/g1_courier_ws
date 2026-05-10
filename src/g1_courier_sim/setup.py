import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'g1_courier_sim'


def _data_tree(local_dir, dest_dir):
    """Recursively map files under local_dir → install dest_dir, preserving
    structure. Used for sim_bridge/assets/ and sim_bridge/idl_local/ which
    contain non-Python data (XML, PNG, IDL bindings)."""
    out = []
    for root, _dirs, files in os.walk(local_dir):
        if not files:
            continue
        rel = os.path.relpath(root, local_dir)
        target = os.path.join(dest_dir, rel) if rel != '.' else dest_dir
        out.append((target, [os.path.join(root, f) for f in files]))
    return out


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ] + _data_tree(
        os.path.join('g1_courier_sim', 'sim_bridge', 'assets'),
        os.path.join('share', package_name, 'sim_bridge', 'assets'),
    ) + _data_tree(
        os.path.join('g1_courier_sim', 'sim_bridge', 'idl_local'),
        os.path.join('share', package_name, 'sim_bridge', 'idl_local'),
    ),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lukasz Gruszka',
    maintainer_email='lukasz.gruszka90@gmail.com',
    description='Sim-only nodes for G1 Courier (kinematic /odom integrator + MuJoCo bridge).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_cmd_vel_bridge_node = g1_courier_sim.sim_cmd_vel_bridge_node:main',
            'sim_lowstate_publisher_node = g1_courier_sim.sim_lowstate_publisher_node:main',
            'sim_lidar_publisher_node = g1_courier_sim.sim_lidar_publisher_node:main',
            'kinematic_nav_node = g1_courier_sim.kinematic_nav_node:main',
            'nav2_navigate_proxy = g1_courier_sim.nav2_navigate_proxy:main',
            'fake_navigate_proxy = g1_courier_sim.fake_action_servers:main_navigate',
            'fake_dock_action_server = g1_courier_sim.fake_action_servers:main_dock',
            'fake_pick_action_server = g1_courier_sim.fake_action_servers:main_pick',
            'fake_place_action_server = g1_courier_sim.fake_action_servers:main_place',
            'fake_retreat_action_server = g1_courier_sim.fake_action_servers:main_retreat',
            # Linux-native MuJoCo bridge (Ubuntu sim path — Option C).
            # Requires: pip install mujoco unitree_sdk2py pupil-apriltags pygame opencv-python.
            'sim_bridge_node = g1_courier_sim.sim_bridge.sim_main:main',
        ],
    },
)
