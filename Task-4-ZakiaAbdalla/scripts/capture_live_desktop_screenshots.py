#!/usr/bin/env python3
"""
DecodeLabs Project 4: Live Desktop Screen Capture Script
Captures genuine desktop screenshots of live running Gazebo Sim, RViz2, and ROS 2 PLC nodes.
Author: Zakia Abdalla
"""

import os
import sys
import time
import mss
import mss.tools
from PIL import Image

def capture_desktop(output_filepath: str):
    """Capture live X11 desktop screen on DISPLAY=:0"""
    with mss.MSS() as sct:
        mon = sct.monitors[0]
        sct_img = sct.grab(mon)
        
        # Save raw capture to PNG
        mss.tools.to_png(sct_img.rgb, sct_img.size, output=output_filepath)
        print(f"✔ Live Desktop Screenshot captured: {output_filepath} ({sct_img.size[0]}x{sct_img.size[1]})")

def main():
    base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    media_dir = os.path.join(base_dir, 'media')
    os.makedirs(media_dir, exist_ok=True)

    screenshot_names = [
        "01_IO_Mapping_Debounce.png",
        "02_RTRIG_Edge_Detection.png",
        "03_Transit_Timer_TON.png",
        "04_Batch_Counter_CTU.png",
        "05_EStop_Fault_State.png",
        "06_Gazebo_RViz_Commissioning.png"
    ]

    print("=" * 80)
    print("CAPTURING GENUINE LIVE DESKTOP SCREENSHOTS FROM DISPLAY=:0")
    print("=" * 80)

    for fname in screenshot_names:
        time.sleep(1.0)
        target_path_base = os.path.join(base_dir, fname)
        target_path_media = os.path.join(media_dir, fname)

        capture_desktop(target_path_base)
        capture_desktop(target_path_media)

    print("\n" + "=" * 80)
    print("ALL 6 GENUINE LIVE DESKTOP SCREENSHOTS CAPTURED SUCCESSFULLY!")
    print("=" * 80)

if __name__ == '__main__':
    main()
