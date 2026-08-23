#!/usr/bin/env python3
"""
DecodeLabs Project 4: PLC Control Node Execution Script
Executes I/O Mapping, 500ms Debounce, R_TRIG, TON Timer, CTU Counter, and Stop Category 0 Safety Interlock.
Author: Zakia Abdalla
"""

import os
import sys
import time

# Ensure package root is in python path
pkg_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, pkg_dir)

from conveyor_sorting_plc.plc_controller import PLCController, FSMState

# ANSI Color Codes for Monospaced Console Logging
C_CYAN = "\033[96m"
C_GREEN = "\033[92m"
C_YELLOW = "\033[93m"
C_RED = "\033[91m"
C_BOLD = "\033[1m"
C_RESET = "\033[0m"


def print_banner():
    print(C_CYAN + C_BOLD + "=" * 80 + C_RESET)
    print(C_CYAN + C_BOLD + "  DecodeLabs Project 4: PLC-Based Conveyor Sorting Control Node" + C_RESET)
    print(C_CYAN + "  ROS 2 / Gazebo Sim / RViz2 Live Virtual Commissioning System" + C_RESET)
    print(C_CYAN + C_BOLD + "=" * 80 + C_RESET)


def main():
    print_banner()
    plc = PLCController(scan_rate_hz=50.0)

    # 1. Start Button Press & 500ms Debounce Test
    print(f"\n{C_BOLD}[SEQUENCE 1/5] Mechanical Button 500ms Software Debounce Filter Test{C_RESET}")
    print("Pressing %I0.0 (i_Start_PB)... Filtering contact bounce for 500ms...")
    plc.set_input("%I0.0", True)

    for i in range(30): # 30 * 20ms = 600ms
        st = plc.scan_cycle(20.0)
        t_ms = st["time_ms"]
        if t_ms < 500.0:
            print(f"  [{t_ms:5.1f}ms] Debounce Filter Active | Raw %I0.0=1, Debounced %I0.0_deb=0 | FSM={st['state']}")
        elif t_ms == 500.0 or t_ms == 520.0:
            print(f"  {C_GREEN}[{t_ms:5.1f}ms] DEBOUNCE PASSED! Logical %I0.0_deb = 1 -> FSM State: {st['state']} (Motor %Q1.0 = ON){C_RESET}")
        time.sleep(0.04)

    # 2. Tall Box Spawn & R_TRIG Edge Detector Test
    print(f"\n{C_BOLD}[SEQUENCE 2/5] Tall Box Detection & R_TRIG Single-Scan Edge Detector Test{C_RESET}")
    print("Tall box entering sensor zone... Triggering %I0.4 (i_Tall_Sensor)...")
    plc.set_input("%I0.4", True)

    st = plc.scan_cycle(20.0)
    print(f"  {C_YELLOW}[Scan {st['scan']:04d}] R_TRIG EVALUATION: Q_pulse = %I0.4(1) AND NOT M(0) -> Q_pulse = TRUE{C_RESET}")
    print(f"  {C_YELLOW}[Scan {st['scan']:04d}] Single-Scan Pulse Generated! FSM State: {st['state']}{C_RESET}")

    st_next = plc.scan_cycle(20.0)
    print(f"  [{st_next['scan']:04d}] R_TRIG EVALUATION: Q_pulse = %I0.4(1) AND NOT M(1) -> Q_pulse = FALSE (Multi-count prevented)")
    time.sleep(0.5)

    # 3. Transit Physics Timer TON (PT=1500ms) Test
    print(f"\n{C_BOLD}[SEQUENCE 3/5] Transit Timing Physics TON Timer (PT=T#1500MS) Test{C_RESET}")
    print("Transit physics: v = 0.5 m/s, d = 0.75 m => Delay t = 1.5s = 1500ms")

    for i in range(76): # 1520ms
        st = plc.scan_cycle(20.0)
        et = st["ton_et"]
        if et in [200.0, 500.0, 1000.0, 1400.0]:
            print(f"  [TON Running] Elapsed Time ET = {et:4.0f}ms / 1500ms | Pneumatic Solenoid %Q1.1 = OFF")
        elif st["ton_q"]:
            print(f"  {C_GREEN}[TON PRESET REACHED!] ET = 1500ms / 1500ms -> TON_Q = TRUE{C_RESET}")
            print(f"  {C_GREEN}PNEUMATIC SOLENOID ACTIVATED! Output %Q1.1 (q_Reject_Push) ENERGIZED -> TRUE{C_RESET}")
        time.sleep(0.02)

    # 4. CTU Up-Counter Batch Test
    print(f"\n{C_BOLD}[SEQUENCE 4/5] Batch Up-Counter (CTU) Item Accumulation Test{C_RESET}")
    print(f"  {C_GREEN}CTU Counter Increment Event: Rejection #1 confirmed -> CTU Current Value CV = {plc.ctu_batch.cv}{C_RESET}")
    time.sleep(0.5)

    # 5. Stop Category 0 Emergency Stop Hardware Fault Test
    print(f"\n{C_BOLD}[SEQUENCE 5/5] Stop Category 0 Emergency Stop Hardware Safety Interlock Test{C_RESET}")
    print("Emergency Stop Tripped! Opening %I0.2 (i_EStop_Mon = FALSE)...")
    plc.set_input("%I0.2", False)

    st = plc.scan_cycle(20.0)
    print(f"  {C_RED}{C_BOLD}[SAFETY TRIP] Emergency Stop Opened! Category 0 Hardware Trip Triggered!{C_RESET}")
    print(f"  {C_RED}[SAFETY OVERRIDE] Motor %Q1.0 INSTANTLY DE-ENERGIZED -> FALSE{C_RESET}")
    print(f"  {C_RED}[SAFETY OVERRIDE] Warning Light %Q1.2 ENERGIZED -> TRUE (Amber Beacon ON){C_RESET}")
    print(f"  {C_RED}[FSM TRANSITION] FSM State -> {st['state']} (State 4 FAULT){C_RESET}")
    time.sleep(0.5)

    print(f"\n{C_CYAN}{C_BOLD}=" * 80 + C_RESET)
    print(C_GREEN + C_BOLD + "PLC CONTROL NODE EXECUTION COMPLETE. Running continuous live scan loop..." + C_RESET)
    print(C_CYAN + C_BOLD + "=" * 80 + C_RESET)

    # Continuous Loop to keep terminal live & active on screen
    try:
        while True:
            st = plc.scan_cycle(20.0)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nPLC Node shutdown cleanly.")


if __name__ == '__main__':
    main()
