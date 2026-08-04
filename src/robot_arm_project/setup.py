from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'robot_arm_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
        (
            os.path.join('share', package_name, 'urdf'),
            glob('urdf/*'),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*'),
        ),
        (
            os.path.join('share', package_name, 'rviz'),
            glob('rviz/*'),
        ),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nirina',
    maintainer_email='nirina@todo.todo',
    description='6-DOF Robot Arm Project',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [],
    },
)