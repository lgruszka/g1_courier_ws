from setuptools import find_packages, setup

package_name = 'g1_courier_locomotion'

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
    description='Locomotion controllers for G1 (Phase 1.3 alpha: standing PD).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'walking_controller_node = g1_courier_locomotion.walking_controller_node:main',
        ],
    },
)
