import os
from glob import glob
from setuptools import setup

package_name = 'g1_courier_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*')),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lukasz Gruszka',
    maintainer_email='lukasz.gruszka90@gmail.com',
    description='Bringup for the G1 courier stack.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'odom_tf_relay = g1_courier_bringup.odom_tf_relay:main',
            'd435i_node = g1_courier_bringup.d435i_node:main',
        ],
    },
)
