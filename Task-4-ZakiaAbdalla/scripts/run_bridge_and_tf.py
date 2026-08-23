#!/usr/bin/env python3
"""
DecodeLabs Project 4: ROS 2 - Gazebo Bridge & Static TF Broadcaster
Author: Zakia Abdalla
"""

import os
import sys
import subprocess
import time

def main():
    base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    plc_script = os.path.join(base_dir, 'scripts', 'conveyor_sorting_plc.py')

    print("=" * 80)
    print("LAUNCHING ROS 2 - GAZEBO BRIDGE & STATIC TF TRANSFORM BROADCASTERS")
    print("=" * 80)

    # 1. Static TF Broadcasters (world -> conveyor_base -> sensors/actuators)
    tf1_cmd = "source /opt/ros/lyrical/setup.bash && ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0.4 --roll 0 --pitch 0 --yaw 0 --frame-id world --child-frame-id conveyor_base"
    tf2_cmd = "source /opt/ros/lyrical/setup.bash && ros2 run tf2_ros static_transform_publisher --x -0.5 --y 0.22 --z 0.2 --roll 0 --pitch 0 --yaw 0 --frame-id conveyor_base --child-frame-id sensor_tall_link"
    tf3_cmd = "source /opt/ros/lyrical/setup.bash && ros2 run tf2_ros static_transform_publisher --x 0.25 --y 0.28 --z 0.1 --roll 0 --pitch 0 --yaw 0 --frame-id conveyor_base --child-frame-id pusher_link"

    print("\n[1/3] Publishing Static TF Transformation Frames...")
    tf1_proc = subprocess.Popen(["bash", "-c", tf1_cmd])
    tf2_proc = subprocess.Popen(["bash", "-c", tf2_cmd])
    tf3_proc = subprocess.Popen(["bash", "-c", tf3_cmd])

    # 2. ROS 2 - Gazebo Sim Parameter Bridge
    bridge_cmd = (
        "source /opt/ros/lyrical/setup.bash && "
        "ros2 run ros_gz_bridge parameter_bridge "
        "/sensor/height_laser@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan "
        "/sensor/prox@std_msgs/msg/Bool@gz.msgs.Boolean "
        "/cmd_conveyor@std_msgs/msg/Bool@gz.msgs.Boolean "
        "/cmd_pusher@std_msgs/msg/Bool@gz.msgs.Boolean"
    )
    print("[2/3] Starting ROS 2 - Gazebo Parameter Bridge (ros_gz_bridge)...")
    bridge_proc = subprocess.Popen(["bash", "-c", bridge_cmd])

    # 3. Launch PLC Control Node Python script
    print("[3/3] Launching PLC Python Logic Controller Node...")
    plc_cmd = f"source /opt/ros/lyrical/setup.bash && DISPLAY=:0 python3 {plc_script}"
    plc_proc = subprocess.Popen(["bash", "-c", plc_cmd])

    print("\n" + "=" * 80)
    print("ALL BRIDGING, TF BROADCASTERS & PLC LOGIC NODES ARE RUNNING LIVE!")
    print("Keep all windows visible side-by-side for your final submission screenshots.")
    print("=" * 80)

    try:
        plc_proc.wait()
    except KeyboardInterrupt:
        tf1_proc.terminate()
        tf2_proc.terminate()
        tf3_proc.terminate()
        bridge_proc.terminate()
        plc_proc.terminate()

if __name__ == '__main__':
    main()
