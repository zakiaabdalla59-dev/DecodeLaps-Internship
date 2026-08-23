"""
DecodeLabs Project 4: Gazebo & RViz Virtual Commissioning Launch File
Author: Zakia Abdalla
"""

import os
import sys
import subprocess

def launch_simulation():
    pkg_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    script_path = os.path.join(pkg_dir, 'scripts', 'run_virtual_commissioning.py')

    print("=" * 75)
    print("ROS 2 Launching: Gazebo & RViz Virtual Commissioning Suite")
    print("Package: conveyor_sorting_plc (Task-4-ZakiaAbdalla)")
    print("=" * 75)

    res = subprocess.run([sys.executable, script_path], check=True)
    return res.returncode

try:
    from launch import LaunchDescription
    from launch.actions import ExecuteProcess, LogInfo

    def generate_launch_description():
        pkg_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
        script_path = os.path.join(pkg_dir, 'scripts', 'run_virtual_commissioning.py')
        return LaunchDescription([
            LogInfo(msg="Launching Gazebo & RViz Virtual Commissioning Suite..."),
            ExecuteProcess(cmd=[sys.executable, script_path], output='screen')
        ])
except ImportError:
    pass

if __name__ == '__main__':
    launch_simulation()
