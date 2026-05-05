from setuptools import find_packages, setup

package_name = 'g1_courier_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lukasz Gruszka',
    maintainer_email='lukasz.gruszka90@gmail.com',
    description='Sim-only nodes (Phase 0 kinematic cmd_vel bridge).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_cmd_vel_bridge_node = g1_courier_sim.sim_cmd_vel_bridge_node:main',
            'sim_lowstate_publisher_node = g1_courier_sim.sim_lowstate_publisher_node:main',
            'sim_lidar_publisher_node = g1_courier_sim.sim_lidar_publisher_node:main',
            'kinematic_nav_node = g1_courier_sim.kinematic_nav_node:main',
            'fake_navigate_proxy = g1_courier_sim.fake_action_servers:main_navigate',
            'fake_dock_action_server = g1_courier_sim.fake_action_servers:main_dock',
            'fake_pick_action_server = g1_courier_sim.fake_action_servers:main_pick',
            'fake_place_action_server = g1_courier_sim.fake_action_servers:main_place',
            'fake_retreat_action_server = g1_courier_sim.fake_action_servers:main_retreat',
        ],
    },
)
