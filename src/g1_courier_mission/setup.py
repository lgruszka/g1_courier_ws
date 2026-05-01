from setuptools import find_packages, setup

package_name = 'g1_courier_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config',
            ['config/mission.yaml', 'config/waypoints.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lukasz Gruszka',
    maintainer_email='lukasz.gruszka90@gmail.com',
    description='Mission orchestrator (behavior tree).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_node = g1_courier_mission.mission_node:main',
            'navigate_proxy = g1_courier_mission.navigate_proxy:main',
            'retreat_action_server = g1_courier_mission.retreat_action_server:main',
        ],
    },
)
