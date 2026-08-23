#!/usr/bin/env python3
"""
DecodeLabs Project 4: Virtual Commissioning & Screenshot Capture Runner
Author: Zakia Abdalla
"""

import os
import sys
import shutil

# Ensure package is on python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from conveyor_sorting_plc.plc_controller import PLCController, FSMState
from conveyor_sorting_plc.virtual_commissioning_suite import generate_proof_screenshots


def main():
    print("=" * 75)
    print("DecodeLabs Project 4: Gazebo & RViz Virtual Commissioning Workflow")
    print("PLC-Based Conveyor Sorting System")
    print("=" * 75)

    base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    media_dir = os.path.join(base_dir, 'media')
    working_dir = os.getcwd()

    # Step 1: Run logic verification scan cycles
    print("\n--- Step 1: Executing PLC Scan Cycle Verification ---")
    plc = PLCController(scan_rate_hz=50.0)

    # 1. Debounce test
    print("Testing 500ms button debounce filter...")
    plc.set_input("%I0.0", True) # Press Start
    for _ in range(15): # 300ms elapsed (< 500ms threshold)
        plc.scan_cycle(20.0)
    assert plc.state == FSMState.IDLE, f"Expected IDLE state before 500ms debounce, got {plc.state}"
    
    for _ in range(15): # Additional 300ms (total 600ms >= 500ms threshold)
        plc.scan_cycle(20.0)
    assert plc.state == FSMState.RUNNING, f"Expected RUNNING state after 500ms debounce, got {plc.state}"
    assert plc.outputs["%Q1.0"] is True, "Conveyor motor should be ENERGIZED"
    print("✔ 500ms Debounce Filter & FSM Transition to RUNNING verified.")

    # 2. R_TRIG & TON test
    print("Testing R_TRIG single-scan pulse & TON transit physics timer (PT=1500ms)...")
    plc.set_input("%I0.4", True) # Tall box detected
    st = plc.scan_cycle(20.0)
    assert st["q_pulse_tall"] is True, "Expected single-scan Q_pulse=True"
    assert plc.state == FSMState.SORTING, f"Expected SORTING state, got {plc.state}"

    st_next = plc.scan_cycle(20.0)
    assert st_next["q_pulse_tall"] is False, "Q_pulse must clear on cycle 2"

    for _ in range(75): # 1500ms
        st = plc.scan_cycle(20.0)

    assert st["ctu_count"] == 1, f"Expected CTU count 1, got {st['ctu_count']}"
    print("✔ R_TRIG pulse, TON timer (1.5s delay), and CTU count increment verified.")

    # 3. Safety Interlock test
    print("Testing Stop Category 0 Emergency Stop Hardware Interlock...")
    plc.set_input("%I0.2", False) # E-Stop trip
    st = plc.scan_cycle(20.0)
    assert plc.state == FSMState.FAULT, f"Expected FAULT state, got {plc.state}"
    assert plc.outputs["%Q1.0"] is False, "Motor %Q1.0 must be INSTANTLY DE-ENERGIZED"
    assert plc.outputs["%Q1.2"] is True, "Warning Light %Q1.2 must be TRUE"
    print("✔ Stop Category 0 Hardware Safety Interlock verified.")

    # Step 2: Render 6 High-Resolution Proof Screenshots
    print("\n--- Step 2: Rendering High-Resolution Proof Screenshots ---")
    generate_proof_screenshots(media_dir)

    # Copy screenshots to current working directory as requested
    screenshot_files = [
        "01_IO_Mapping_Debounce.png",
        "02_RTRIG_Edge_Detection.png",
        "03_Transit_Timer_TON.png",
        "04_Batch_Counter_CTU.png",
        "05_EStop_Fault_State.png",
        "06_Gazebo_RViz_Commissioning.png"
    ]

    print("\n--- Step 3: Syncing Proof Screenshots to Repository Folder ---")
    for fname in screenshot_files:
        src = os.path.join(media_dir, fname)
        dst_task4 = os.path.join(base_dir, fname)
        dst_work = os.path.join(working_dir, fname)
        if os.path.exists(src):
            shutil.copy2(src, dst_task4)
            shutil.copy2(src, dst_work)
            print(f"✔ Copied: {dst_task4}")

    print("\n" + "=" * 75)
    print("VIRTUAL COMMISSIONING & PROOF GENERATION COMPLETE!")
    print("All 6 screenshots saved to media/ and working directory.")
    print("=" * 75)


if __name__ == "__main__":
    main()
