from setuptools import find_packages, setup

package_name = 'g1_courier_arm_skills'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/arm_skills.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lukasz Gruszka',
    maintainer_email='lukasz.gruszka90@gmail.com',
    description='Parametric arm controller and PickBox/PlaceBox action servers.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pick_action_server = g1_courier_arm_skills.pick_action_server:main',
            'place_action_server = g1_courier_arm_skills.place_action_server:main',
        ],
    },
)
