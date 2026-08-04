import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'arm_kinematics_planner'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name] if os.path.exists('resource/' + package_name) else []),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nirina',
    maintainer_email='nirina@todo.todo',
    description='ROS 2 6-DOF Robotic Arm Kinematics & Path Planning Package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'kinematics_planner_node = arm_kinematics_planner.kinematics_planner_node:main',
        ],
    },
)
