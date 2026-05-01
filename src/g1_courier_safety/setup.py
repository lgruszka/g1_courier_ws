from setuptools import find_packages, setup

package_name = 'g1_courier_safety'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/safety.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lukasz Gruszka',
    maintainer_email='lukasz.gruszka90@gmail.com',
    description='cmd_vel arbiter and safety layer.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_arbiter = g1_courier_safety.cmd_vel_arbiter:main',
        ],
    },
)
