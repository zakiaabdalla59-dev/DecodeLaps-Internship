"""
DecodeLabs Project 4: PLC-Based Conveyor Sorting System
PLC Controller & IEC 61131-3 Standard Function Blocks
Author: Zakia Abdalla
"""

import time
from enum import IntEnum
from typing import Dict, Any, Tuple, List


class FSMState(IntEnum):
    """Finite State Machine States - Enforced Mutual Exclusion"""
    IDLE = 1
    RUNNING = 2
    SORTING = 3
    FAULT = 4


class DebounceFilter:
    """
    Software Debounce Filter for Mechanical Contacts
    Requires a state to be sustained for >= debounce_ms before updating output.
    """
    def __init__(self, debounce_ms: float = 500.0, initial_state: bool = False):
        self.debounce_ms = debounce_ms
        self.stable_state = initial_state
        self.candidate_state = initial_state
        self.transition_start_time = 0.0

    def update(self, raw_input: bool, current_time_ms: float) -> bool:
        if raw_input != self.candidate_state:
            self.candidate_state = raw_input
            self.transition_start_time = current_time_ms

        if raw_input != self.stable_state:
            elapsed = current_time_ms - self.transition_start_time
            if elapsed >= self.debounce_ms:
                self.stable_state = raw_input

        return self.stable_state


class RTRIG:
    """
    Rising Edge Detector (R_TRIG)
    Q_pulse = Input_current AND NOT Input_previous
    Generates a True pulse for exactly ONE scan cycle on a 0 -> 1 transition.
    """
    def __init__(self):
        self.previous_state = False

    def update(self, clk: bool) -> bool:
        q_pulse = clk and not self.previous_state
        self.previous_state = clk
        return q_pulse


class TON:
    """
    IEC 61131-3 On-Delay Timer (TON)
    Transit Timing Physics: v = 0.5 m/s, d = 0.75 m => PT = T#1500MS
    """
    def __init__(self, pt_ms: float = 1500.0):
        self.pt_ms = pt_ms  # Preset Time in ms
        self.et_ms = 0.0    # Elapsed Time in ms
        self.q = False      # Timer Output
        self.running = False

    def update(self, in_sig: bool, dt_ms: float) -> Tuple[bool, float]:
        if not in_sig:
            self.et_ms = 0.0
            self.q = False
            self.running = False
        else:
            self.running = True
            if self.et_ms < self.pt_ms:
                self.et_ms = min(self.pt_ms, self.et_ms + dt_ms)
            if self.et_ms >= self.pt_ms:
                self.q = True
        return self.q, self.et_ms

    def reset(self):
        self.et_ms = 0.0
        self.q = False
        self.running = False


class CTU:
    """
    IEC 61131-3 Up-Counter (CTU)
    Increments item count on rising edge pulses.
    """
    def __init__(self, pv: int = 9999):
        self.pv = pv      # Preset Value
        self.cv = 0       # Current Value
        self.q = False    # Counter Done Output
        self.r_trig = RTRIG()

    def update(self, cu: bool, reset: bool = False) -> Tuple[int, bool]:
        if reset:
            self.cv = 0
            self.q = False
        else:
            pulse = self.r_trig.update(cu)
            if pulse:
                self.cv += 1
                if self.cv >= self.pv:
                    self.q = True
        return self.cv, self.q


class PLCController:
    """
    DecodeLabs Project 4 Main PLC Control Unit
    Integrates I/O Image Memory, FSM, Timers, Edge Detectors, and Safety Relay.
    """
    def __init__(self, scan_rate_hz: float = 50.0):
        self.scan_rate_hz = scan_rate_hz
        self.scan_dt_ms = 1000.0 / scan_rate_hz
        self.current_time_ms = 0.0
        self.scan_count = 0

        # Discrete Inputs Image Memory (%I0.0 - %I0.4)
        self.inputs = {
            "%I0.0": False,  # i_Start_PB (NO)
            "%I0.1": False,  # i_Prox_Sensor (NO)
            "%I0.2": True,   # i_EStop_Mon (NC, True when Healthy)
            "%I0.3": True,   # i_Stop_PB (NC, True when Unpressed)
            "%I0.4": False,  # i_Tall_Sensor (NO)
        }

        # Discrete Outputs Image Memory (%Q1.0 - %Q1.2)
        self.outputs = {
            "%Q1.0": False,  # q_Conv_Motor
            "%Q1.1": False,  # q_Reject_Push
            "%Q1.2": False,  # q_Warn_Light
        }

        # Software Debounce Filters (500ms on mechanical buttons)
        self.debounce_start = DebounceFilter(debounce_ms=500.0, initial_state=False)
        self.debounce_stop = DebounceFilter(debounce_ms=500.0, initial_state=True)
        self.debounce_estop = DebounceFilter(debounce_ms=500.0, initial_state=True)

        # Logical Debounced Input States
        self.i_Start_PB_deb = False
        self.i_Stop_PB_deb = True
        self.i_EStop_Mon_deb = True

        # Edge Detectors & Timers
        self.rtrig_tall = RTRIG()
        self.rtrig_prox = RTRIG()
        self.ton_transit = TON(pt_ms=1500.0)  # 1.5s transit delay
        self.ctu_batch = CTU(pv=9999)

        # FSM State
        self.state = FSMState.IDLE
        self.pusher_pulse_timer_ms = 0.0
        self.pusher_active_pulse = False

        # Event Log
        self.log_history: List[str] = []

    def set_input(self, tag: str, value: bool):
        """Set raw physical input channel state"""
        if tag in self.inputs:
            self.inputs[tag] = value

    def scan_cycle(self, dt_ms: float = 20.0) -> Dict[str, Any]:
        """
        Execute one full PLC scan cycle:
        1. Input Scanning & Debouncing
        2. Hardware Safety Interlock Check
        3. Edge Detection Execution
        4. FSM State Evaluation
        5. Timers & Counter Processing
        6. Output Memory Updating
        """
        self.scan_count += 1
        self.current_time_ms += dt_ms

        # 1. Input Debouncing (%I0.0, %I0.2, %I0.3)
        raw_start = self.inputs["%I0.0"]
        raw_estop = self.inputs["%I0.2"]
        raw_stop = self.inputs["%I0.3"]

        self.i_Start_PB_deb = self.debounce_start.update(raw_start, self.current_time_ms)
        self.i_Stop_PB_deb = self.debounce_stop.update(raw_stop, self.current_time_ms)
        self.i_EStop_Mon_deb = self.debounce_estop.update(raw_estop, self.current_time_ms)

        # 2. Hardware Safety Relay (Stop Category 0)
        # Immediate physical disconnect of motor power independent of software logic
        hardware_safety_healthy = self.i_EStop_Mon_deb and raw_estop

        # 3. Edge Detection on Tall Box Sensor (%I0.4)
        raw_tall = self.inputs["%I0.4"]
        q_pulse_tall = self.rtrig_tall.update(raw_tall)

        raw_prox = self.inputs["%I0.1"]
        q_pulse_prox = self.rtrig_prox.update(raw_prox)

        # 4. FSM Execution (Strict Mutual Exclusion)
        if not hardware_safety_healthy:
            # Emergency Stop Triggered -> Immediate FAULT state
            self.state = FSMState.FAULT
        else:
            if self.state == FSMState.FAULT:
                # Recover from Fault if E-Stop restored and Start pressed
                if self.i_Start_PB_deb:
                    self.state = FSMState.IDLE

            if self.state == FSMState.IDLE:
                if self.i_Start_PB_deb and self.i_Stop_PB_deb:
                    self.state = FSMState.RUNNING

            elif self.state == FSMState.RUNNING:
                if not self.i_Stop_PB_deb:
                    self.state = FSMState.IDLE
                elif q_pulse_tall:
                    self.state = FSMState.SORTING

            elif self.state == FSMState.SORTING:
                if not self.i_Stop_PB_deb:
                    self.state = FSMState.IDLE

        # 5. Timer & Output Logic Execution
        # Transit Physics Timer TON (PT = 1500ms)
        ton_enable = (self.state == FSMState.SORTING or self.ton_transit.running) and hardware_safety_healthy
        ton_q, ton_et = self.ton_transit.update(ton_enable, dt_ms)

        # Batch Counter CTU increment on rejection trigger
        if ton_q:
            self.ctu_batch.update(cu=True, reset=False)
            self.pusher_active_pulse = True
            self.pusher_pulse_timer_ms = 500.0  # 500ms extension pulse
            self.ton_transit.reset()

        if self.pusher_active_pulse:
            self.pusher_pulse_timer_ms -= dt_ms
            if self.pusher_pulse_timer_ms <= 0:
                self.pusher_active_pulse = False
                if self.state == FSMState.SORTING:
                    self.state = FSMState.RUNNING

        # 6. Physical Output Assigns (%Q Image Memory)
        if self.state == FSMState.FAULT:
            self.outputs["%Q1.0"] = False  # Motor DE-ENERGIZED (Cat 0 Safety)
            self.outputs["%Q1.1"] = False  # Pusher Retracted
            self.outputs["%Q1.2"] = True   # Warning Amber Light ON
        elif self.state == FSMState.RUNNING:
            self.outputs["%Q1.0"] = True   # Motor Active
            self.outputs["%Q1.1"] = self.pusher_active_pulse
            self.outputs["%Q1.2"] = False
        elif self.state == FSMState.SORTING:
            self.outputs["%Q1.0"] = True   # Motor Active during transit
            self.outputs["%Q1.1"] = self.pusher_active_pulse
            self.outputs["%Q1.2"] = False
        else:  # IDLE
            self.outputs["%Q1.0"] = False
            self.outputs["%Q1.1"] = False
            self.outputs["%Q1.2"] = False

        # Hardware Safety Override (Category 0 Stop)
        if not hardware_safety_healthy:
            self.outputs["%Q1.0"] = False  # Power trip override

        # Format Cycle Summary
        status = {
            "scan": self.scan_count,
            "time_ms": self.current_time_ms,
            "state": self.state.name,
            "state_val": int(self.state),
            "inputs": self.inputs.copy(),
            "outputs": self.outputs.copy(),
            "q_pulse_tall": q_pulse_tall,
            "ton_et": ton_et,
            "ton_q": ton_q,
            "ctu_count": self.ctu_batch.cv,
            "cat0_safety": hardware_safety_healthy
        }

        log_entry = (
            f"[Scan {self.scan_count:04d} | {self.current_time_ms:6.1f}ms] "
            f"FSM State={self.state.name} ({int(self.state)}) | "
            f"I:(Start=%I0.0:{int(self.inputs['%I0.0'])}, Tall=%I0.4:{int(self.inputs['%I0.4'])}, EStop=%I0.2:{int(self.inputs['%I0.2'])}) | "
            f"Q:(Motor=%Q1.0:{int(self.outputs['%Q1.0'])}, Push=%Q1.1:{int(self.outputs['%Q1.1'])}, Light=%Q1.2:{int(self.outputs['%Q1.2'])}) | "
            f"TON_ET={ton_et:4.0f}ms | CTU={self.ctu_batch.cv}"
        )
        self.log_history.append(log_entry)
        if len(self.log_history) > 100:
            self.log_history.pop(0)

        return status
