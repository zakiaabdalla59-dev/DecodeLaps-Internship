#!/usr/bin/env python3
"""
DecodeLabs Project 4: Gazebo & RViz Virtual Commissioning ROS 2 Launch Description
Launches real Gazebo 3D simulation, RViz2 diagnostic GUI window, and ROS 2 PLC execution node.
Author: Zakia Abdalla
"""

import os
import sys
import subprocess

def main():
    base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    world_file = os.path.join(base_dir, 'worlds', 'conveyor_sorting_world.sdf')
    rviz_file = os.path.join(base_dir, 'config', 'conveyor_diagnostics.rviz')
    script_path = os.path.join(base_dir, 'scripts', 'run_virtual_commissioning.py')

    print("=" * 80)
    print("LAUNCHING REAL GAZEBO 3D & RVIZ2 VIRTUAL COMMISSIONING SUITE")
    print(f"Package Path: {base_dir}")
    print(f"World SDF:    {world_file}")
    print(f"RViz Config:  {rviz_file}")
    print("=" * 80)

    # 1. Launch real Gazebo Sim 3D Environment (gz sim) in background
    gz_cmd = f"source /opt/ros/lyrical/setup.bash && gz sim -r {world_file}"
    print("\n[1/3] Starting Gazebo 3D Simulation Window (gz sim)...")
    gz_proc = subprocess.Popen(["bash", "-c", gz_cmd])

    # 2. Launch real RViz2 Diagnostic Panel in background
    rviz_cmd = f"source /opt/ros/lyrical/setup.bash && rviz2 -d {rviz_file}"
    print("[2/3] Starting RViz2 Diagnostic Panel (rviz2)...")
    rviz_proc = subprocess.Popen(["bash", "-c", rviz_cmd])

    # 3. Launch ROS 2 / PLC Logic Execution Node in foreground terminal
    print("[3/3] Starting ROS 2 PLC Logic Execution Node...")
    plc_proc = subprocess.run([sys.executable, script_path])

    print("\nSimulation GUI windows are live on screen. Keeping processes active...")
    try:
        gz_proc.wait()
        rviz_proc.wait()
    except KeyboardInterrupt:
        gz_proc.terminate()
        rviz_proc.terminate()

if __name__ == '__main__':
    main()
