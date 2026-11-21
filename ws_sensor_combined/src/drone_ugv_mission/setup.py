from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'drone_ugv_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Required for ament to find the package
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install all launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='iheb',
    maintainer_email='iheb@todo.com',
    description='Drone detects person, UGV navigates to them via Nav2.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ros2 run drone_ugv_mission drone_vision
            'drone_vision   = drone_ugv_mission.drone_advanced_vision:main',
            # ros2 run drone_ugv_mission person_locator
            'person_locator = drone_ugv_mission.person_locator_node:main',
            # ros2 run drone_ugv_mission robot_mission
            'robot_mission  = drone_ugv_mission.robot_mission_node:main',
        ],
    },
)
