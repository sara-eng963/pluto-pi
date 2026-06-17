#!/usr/bin/env python3
"""
Interactive ESP1 tuning console for Pluto.

Publishes to : /drive_cmd    std_msgs/String
               /zero_yaw     std_msgs/Empty
Subscribes   : /drive_status std_msgs/String

Run with the full launch NOT running (navigation_node uses the same topics).

AUTO MODES
----------
  AUTO ROT  <angle_deg> [count=5]   — rotates to angle and back N times,
                                       reports error statistics per trial.
  AUTO MOVE <dist_m>    [count=3]   — moves forward and back N times,
                                       reports timing per trial.
  AUTO TRIP <dist_m>    [count=3]   — full round-trip:
                                       ROTATE 0, MOVE dist, ROTATE 180,
                                       MOVE dist, ROTATE 0 × N.

Press Ctrl-C or type ABORT during auto to stop immediately.
"""

import math
import sys
import threading
import time
from typing import Iterable, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String


# ---------------------------------------------------------------------------
# Safety limits — values outside these are rejected before sending to ESP
# ---------------------------------------------------------------------------
_LIMITS: dict[str, tuple[float, float]] = {
    "PKP":     (0.0,  20.0),
    "PKI":     (0.0,   5.0),
    "PKPF":    (0.0,  30.0),
    "HKP":     (0.0, 500.0),
    "HKI":     (0.0,  50.0),
    "HKD":     (0.0, 100.0),
    "RKP":     (0.0, 300.0),
    "RKI":     (0.0,  20.0),
    "RTOL":    (0.2,  10.0),
    "FKP":     (0.0, 3000.0),
    "FMAXRPM": (1.0,  60.0),
    # Velocity PID — firmware defaults are 7-8, so allow up to 15
    "VKP":     (0.0,  15.0),
    "VKI":     (0.0,   2.0),
    "VKD":     (0.0,   1.0),
    "VKPALL":  (0.0,  15.0),
    "VKIALL":  (0.0,   2.0),
    "VKDALL":  (0.0,   1.0),
}

# Wheel name → VKP/VKI/VKD index (from HardwareConfig.h)
_WHEEL_IDX = {"R1": 0, "R2": 1, "F1": 2, "F2": 3}

_ACK_TIMEOUT  = 8.0
_DONE_TIMEOUT = 45.0

MENU = r"""
═══════════════════════════════════════════════
  PLUTO ESP1 TEST/TUNE CONSOLE
  drive → /drive_cmd   feedback ← /drive_status
═══════════════════════════════════════════════

Motion:
  MOVE   <meters> [heading_deg]    e.g.  MOVE 0.5 0
  ROTATE <deg>                     e.g.  ROTATE 90
  STOP / RESUME / RESET / STATUS
  ZERO                              zero IMU yaw

Position PID  (PKP ~2.05  PKPF ~8.0):
  PKP <v>   PKI <v>   PKPF <v>

Heading hold during MOVE  (HKP ~260  HKI 0  HKD ~25):
  HKP <v>   HKI <v>   HKD <v>

Rotate PID  (RKP ~55  RKI ~1.5  RTOL ~0.5):
  RKP <v>   RKI <v>   RTOL <deg>

Final heading Kp — near-zone (FKP ~1200  FMAXRPM ~20):
  FKP <v>   FMAXRPM <v>

Wheel velocity PID  (wheel = R1 R2 F1 F2):
  <wheel> KP/KI/KD <v>    e.g.  R1 KP 1.2
  ALL     KP/KI/KD <v>    sets all wheels at once

  Wheel map: F1=front-right  F2=front-left
             R1=rear-right   R2=rear-left

Automatic test modes:
  AUTO ROT  <angle_deg> [count=5]   repeated rotate to angle and back
  AUTO MOVE <dist_m>    [count=3]   repeated forward + back move
  AUTO TRIP <dist_m>    [count=3]   full Manhattan round-trip

  Type ABORT (or Ctrl-C) to stop any auto sequence.

Other:
  HEADING ON|OFF   HINVERT   RINVERT
  p / e            show / hide STATUS telemetry
  help             this menu
  q                quit
""".strip()


# ---------------------------------------------------------------------------
class TestTuneNode(Node):

    def __init__(self) -> None:
        super().__init__("test_tune")

        self.drive_pub = self.create_publisher(String, "/drive_cmd",    10)
        self.zero_pub  = self.create_publisher(Empty,  "/zero_yaw",     10)
        self._sub      = self.create_subscription(
            String, "/drive_status", self._on_feedback, 50
        )

        # Synchronisation events
        self._ack_event   = threading.Event()
        self._done_event  = threading.Event()
        self._fault_event = threading.Event()
        self._last_esp    = ""
        self._last_done   = ""

        self._auto_abort  = threading.Event()
        self.show_status  = False
        self.running      = True

        # Velocity PID tracking — initialised to firmware defaults
        # (WheelVelocityController.cpp order: R1=0, R2=1, F1=2, F2=3)
        self._vkp_per_wheel: list[float] = [4.93, 7.11, 4.26, 7.3]
        self._vki_per_wheel: list[float] = [0.13, 0.10, 0.08, 0.12]
        self._vkd_per_wheel: list[float] = [0.03, 0.015, 0.03, 0.015]

        # Velocity sample collection (filled during auto modes)
        self._vel_samples: list[dict]              = []
        self._collecting_vel: bool                 = False
        self._vel_poll_stop                        = threading.Event()
        self._vel_poll_thread: Optional[threading.Thread] = None

        print(MENU)
        print()

    # ── Feedback ─────────────────────────────────────────────────────────

    def _on_feedback(self, msg: String) -> None:
        text = msg.data.strip()
        self._last_esp = text

        if text.startswith("ACK") or text.startswith("FROZEN") \
                or text.startswith("RESUMED") or text.startswith("RESET"):
            self._ack_event.set()

        if text.startswith("DONE"):
            self._last_done = text
            self._done_event.set()

        if text.startswith("FAULT") or text.startswith("ERR"):
            self._last_done = text
            self._fault_event.set()

        if text.startswith("STATUS") and self._collecting_vel:
            sample = self._parse_status_vel(text)
            if sample:
                self._vel_samples.append(sample)

        if text.startswith("STATUS") and not self.show_status:
            return
        print(f"\r\033[KESP> {text}\n> ", end="", flush=True)

    # ── Low-level send ────────────────────────────────────────────────────

    def _send(self, command: str) -> None:
        msg = String()
        msg.data = command.strip()
        self.drive_pub.publish(msg)
        print(f"→ {msg.data}")
        # Track velocity PID parameter changes so suggestions use current values
        parts = msg.data.upper().split()
        if len(parts) >= 2:
            v = self._float(parts[-1])
            if v is None:
                return
            if parts[0] == "VKPALL":
                self._vkp_per_wheel = [v] * 4
            elif parts[0] == "VKIALL":
                self._vki_per_wheel = [v] * 4
            elif parts[0] == "VKDALL":
                self._vkd_per_wheel = [v] * 4
            elif parts[0] == "VKP" and len(parts) == 3 and parts[1].isdigit():
                idx = int(parts[1])
                if 0 <= idx <= 3:
                    self._vkp_per_wheel[idx] = v
            elif parts[0] == "VKI" and len(parts) == 3 and parts[1].isdigit():
                idx = int(parts[1])
                if 0 <= idx <= 3:
                    self._vki_per_wheel[idx] = v
            elif parts[0] == "VKD" and len(parts) == 3 and parts[1].isdigit():
                idx = int(parts[1])
                if 0 <= idx <= 3:
                    self._vkd_per_wheel[idx] = v

    # ── Blocking send-and-wait (used by auto modes) ───────────────────────

    def _send_and_wait(
        self,
        command: str,
        ack_timeout: float = _ACK_TIMEOUT,
        done_timeout: float = _DONE_TIMEOUT,
    ) -> tuple[Optional[str], bool]:
        """Send command, wait for ACK then DONE/FAULT. Abort-safe."""
        if self._auto_abort.is_set():
            return None, False

        self._ack_event.clear()
        self._done_event.clear()
        self._fault_event.clear()
        self._send(command)

        # wait for ACK
        deadline = time.monotonic() + ack_timeout
        while time.monotonic() < deadline:
            if self._auto_abort.is_set():
                return None, False
            if self._ack_event.wait(timeout=0.05):
                break
        else:
            print(f"  ✗ ACK timeout: {command}")
            return None, False

        if self._auto_abort.is_set():
            return None, False

        verb = command.strip().upper().split()[0]
        if verb not in ("MOVE", "ROTATE"):
            return self._last_esp, True

        # wait for DONE
        deadline = time.monotonic() + done_timeout
        while time.monotonic() < deadline:
            if self._auto_abort.is_set():
                return None, False
            if self._fault_event.is_set():
                print(f"  ✗ FAULT: {self._last_done}")
                return self._last_done, False
            if self._done_event.wait(timeout=0.05):
                return self._last_done, True

        print(f"  ✗ DONE timeout: {command}")
        return None, False

    # ── Helpers ──────────────────────────────────────────────────────────

    @staticmethod
    def _float(text: str) -> Optional[float]:
        try:
            return float(text)
        except ValueError:
            return None

    @staticmethod
    def _check_limit(cmd: str, value: float) -> Optional[str]:
        lo, hi = _LIMITS.get(cmd, (None, None))
        if lo is None:
            return None
        if not (lo <= value <= hi):
            return f"{cmd} {value} out of safe range [{lo}, {hi}]"
        return None

    @staticmethod
    def _parse_kv(text: str) -> dict:
        result = {}
        for token in (text or "").split():
            if "=" in token:
                k, _, v = token.partition("=")
                try:
                    result[k.lower()] = float(v)
                except ValueError:
                    pass
        return result

    def _safe_reset_zero(self) -> bool:
        """RESET + ZERO before any auto sequence. Returns False if aborted."""
        print("  [prep] RESET...")
        _, ok = self._send_and_wait("RESET", done_timeout=10.0)
        if not ok:
            print("  [prep] RESET failed — aborting")
            return False
        time.sleep(0.2)
        self.zero_pub.publish(Empty())
        print("  [prep] YAW zeroed")
        time.sleep(0.4)
        return True

    # ── Velocity monitoring ───────────────────────────────────────────────

    def _start_vel_poll(self) -> None:
        self._vel_samples.clear()
        self._vel_poll_stop.clear()
        self._collecting_vel = True
        self._vel_poll_thread = threading.Thread(
            target=self._vel_poll_loop, daemon=True
        )
        self._vel_poll_thread.start()

    def _stop_vel_poll(self) -> None:
        self._vel_poll_stop.set()
        self._collecting_vel = False
        if self._vel_poll_thread is not None:
            self._vel_poll_thread.join(timeout=1.0)
            self._vel_poll_thread = None

    def _vel_poll_loop(self) -> None:
        """Background thread: publishes STATUS every 150 ms for vel data."""
        while not self._vel_poll_stop.wait(timeout=0.15):
            msg = String()
            msg.data = "STATUS"
            self.drive_pub.publish(msg)

    @staticmethod
    def _parse_csv_floats(text: str, prefix: str) -> Optional[list]:
        for token in text.split():
            if token.startswith(prefix + "="):
                try:
                    return [float(x) for x in token[len(prefix) + 1:].split(",")]
                except ValueError:
                    return None
        return None

    def _parse_status_vel(self, text: str) -> Optional[dict]:
        target   = self._parse_csv_floats(text, "targetRPM")
        measured = self._parse_csv_floats(text, "measuredRPM")
        pwm      = self._parse_csv_floats(text, "pwm")
        if target and measured and len(target) == 4 and len(measured) == 4:
            return {
                "t":       time.monotonic(),
                "target":  target,
                "measured": measured,
                "pwm":     pwm or [],
            }
        return None

    # ── Velocity analysis ─────────────────────────────────────────────────

    def _analyze_vel(self) -> dict:
        """
        Per-wheel time-domain and steady-state analysis.

        Metrics computed per wheel:
          mean_err       — mean |target - measured| over all active samples
          ss_error       — steady-state error: mean error in last 25% of active samples
          max_overshoot  — peak (measured - target) when target is positive
          overshoot_pct  — max_overshoot as % of target
          rise_time_s    — time for measured to go from 10% to 90% of target
                           at the very first activation burst (None if n/a)
          settling_time_s— time until measured stays within ±15% of target (None if n/a)
          oscillations   — number of tracking-error sign changes
        """
        samples = self._vel_samples
        ACTIVE_THR = 5.0
        BAND = 0.15           # ±15% settling band
        POLL_DT = 0.15        # approximate sample interval (s)

        active = [s for s in samples
                  if any(abs(t) > ACTIVE_THR for t in s["target"])]

        result: dict = {
            "total": len(samples), "active": len(active),
            "mean_err":       [0.0] * 4,
            "ss_error":       [0.0] * 4,
            "max_overshoot":  [0.0] * 4,
            "overshoot_pct":  [0.0] * 4,
            "rise_time_s":    [None] * 4,
            "settling_time_s":[None] * 4,
            "oscillations":   [0]   * 4,
            "mean_target":    [0.0] * 4,
            "mean_measured":  [0.0] * 4,
        }

        if not active:
            return result

        t0 = active[0]["t"]   # reference time for this burst

        for wi in range(4):
            wsamples = [s for s in active if abs(s["target"][wi]) > ACTIVE_THR]
            if not wsamples:
                continue

            tgts = [s["target"][wi]   for s in wsamples]
            meas = [s["measured"][wi] for s in wsamples]
            ts   = [s["t"]            for s in wsamples]
            errs = [abs(t - m) for t, m in zip(tgts, meas)]

            result["mean_target"][wi]   = sum(abs(t) for t in tgts) / len(tgts)
            result["mean_measured"][wi] = sum(abs(m) for m in meas) / len(meas)
            result["mean_err"][wi]      = sum(errs) / len(errs)

            # Steady-state error: last 25% of samples (robot moving at constant speed)
            tail = max(1, len(wsamples) // 4)
            ss_errs = errs[-tail:]
            result["ss_error"][wi] = sum(ss_errs) / len(ss_errs)

            # Overshoot: peak (|measured| - |target|) when both positive
            overs = [abs(m) - abs(t) for t, m in zip(tgts, meas)
                     if abs(m) > abs(t) and abs(t) > ACTIVE_THR]
            peak_over = max(overs) if overs else 0.0
            result["max_overshoot"][wi] = peak_over
            avg_tgt = result["mean_target"][wi]
            result["overshoot_pct"][wi] = (peak_over / avg_tgt * 100.0
                                           if avg_tgt > 0 else 0.0)

            # Rise time: 10% → 90% of target at first activation
            abs_tgt  = [abs(t) for t in tgts]
            abs_meas = [abs(m) for m in meas]
            peak_tgt = max(abs_tgt) if abs_tgt else 0.0
            if peak_tgt > ACTIVE_THR:
                t10 = peak_tgt * 0.10
                t90 = peak_tgt * 0.90
                idx10 = next((i for i, m in enumerate(abs_meas) if m >= t10), None)
                idx90 = next((i for i, m in enumerate(abs_meas) if m >= t90), None)
                if idx10 is not None and idx90 is not None and idx90 > idx10:
                    result["rise_time_s"][wi] = round(
                        (ts[idx90] - ts[idx10]), 2
                    )

            # Settling time: last sample outside ±15% band, measured from start
            settled = True
            last_unsettled = None
            for i, (t_, m_) in enumerate(zip(tgts, meas)):
                if abs(t_) > ACTIVE_THR:
                    if abs(m_ - t_) > BAND * abs(t_):
                        last_unsettled = i
                        settled = False
            if last_unsettled is not None and ts:
                result["settling_time_s"][wi] = round(
                    ts[last_unsettled] - t0, 2
                )
            elif settled and ts:
                result["settling_time_s"][wi] = 0.0

            # Oscillations: sign changes in tracking error
            signed = [t - m for t, m in zip(tgts, meas)]
            result["oscillations"][wi] = sum(
                1 for i in range(1, len(signed))
                if signed[i] * signed[i - 1] < 0
            )

        return result

    def _print_vel_report(self, a: dict) -> None:
        print()
        print(f"  ── VELOCITY LOOP ANALYSIS ─────────────────────────────")
        print(f"  Samples: {a['active']} active / {a['total']} total")
        if a["active"] < 3:
            print(f"  Not enough active motion data.")
            return
        names = ["R1", "R2", "F1", "F2"]

        hdr = (f"  {'Whl':>4}  {'Target':>7}  {'Meas':>7}  "
               f"{'MeanErr':>8}  {'SSErr':>6}  "
               f"{'Over%':>6}  {'Rise':>6}  {'Settle':>7}  {'Osc':>4}")
        print(f"\n{hdr}")
        print("  " + "─" * (len(hdr) - 2))

        def _fmt(v, fmt):
            return "n/a" if v is None else format(v, fmt)

        for wi, wn in enumerate(names):
            print(
                f"  {wn:>4}  "
                f"{a['mean_target'][wi]:>7.1f}  "
                f"{a['mean_measured'][wi]:>7.1f}  "
                f"{a['mean_err'][wi]:>8.2f}  "
                f"{a['ss_error'][wi]:>6.2f}  "
                f"{a['overshoot_pct'][wi]:>6.1f}  "
                f"{_fmt(a['rise_time_s'][wi], '6.2f')}  "
                f"{_fmt(a['settling_time_s'][wi], '7.2f')}  "
                f"{a['oscillations'][wi]:>4}"
            )

        avg_e  = sum(a["mean_err"]) / 4
        avg_ss = sum(a["ss_error"]) / 4
        avg_op = sum(a["overshoot_pct"]) / 4
        print(f"  {'AVG':>4}  {'':>7}  {'':>7}  "
              f"{avg_e:>8.2f}  {avg_ss:>6.2f}  {avg_op:>6.1f}")
        print(f"\n  MeanErr=mean tracking err(RPM)  SSErr=steady-state err  "
              f"Over%=peak overshoot  Rise=10→90% time(s)  Settle=settle time(s)")
        print()

    def _compute_vel_suggestions(self, a: dict) -> dict:
        """
        Per-wheel PID suggestions using time-domain + steady-state metrics.
        Each wheel analysed independently (mecanum wheels differ in load/friction).

        Rules applied per wheel:
          KP ↑  slow rise (>0.8s) and no overshoot
          KP ↓  overshoot% >25% or oscillation + long settle
          KI ↑  steady-state error >1.5 RPM after acceptable transient
          KI ↓  KI causing overshoot (ss_err good but over% high)
          KD ↑  overshoot% >15% AND oscillations >4  (derivative damping)
          KD ↓  KD already set, oscillations low (may be amplifying noise)
        """
        _WNAMES = ["R1", "R2", "F1", "F2"]

        if a["active"] < 3:
            return {"msgs": ["not enough active motion data"], "cmds": {}}

        msgs: list[str] = []
        cmds: dict[str, float] = {}
        any_suggestion = False

        for wi, wn in enumerate(_WNAMES):
            mean_err = a["mean_err"][wi]
            ss_err   = a["ss_error"][wi]
            over_pct = a["overshoot_pct"][wi]
            rise     = a["rise_time_s"][wi]
            settle   = a["settling_time_s"][wi]
            osc      = a["oscillations"][wi]

            kp = self._vkp_per_wheel[wi]
            ki = self._vki_per_wheel[wi]
            kd = self._vkd_per_wheel[wi]
            new_kp, new_ki, new_kd = kp, ki, kd
            w_msgs: list[str] = []

            # ── KP ────────────────────────────────────────────────────────
            if over_pct > 25.0 or (settle is not None and settle > 3.0 and osc > 6):
                factor = 0.78 if over_pct > 35.0 or osc > 10 else 0.87
                new_kp = round(max(kp * factor, 0.5), 2)
                w_msgs.append(f"  {wn}: ⚠ over {over_pct:.0f}% osc {osc} "
                               f"→ KP {kp:.2f}→{new_kp} (↓{100*(1-factor):.0f}%)")
            elif rise is not None and rise > 0.8 and over_pct < 10.0:
                new_kp = round(min(kp * 1.18, _LIMITS["VKP"][1]), 2)
                w_msgs.append(f"  {wn}: ⏱ rise {rise:.2f}s slow "
                               f"→ KP {kp:.2f}→{new_kp} (+18%)")
            elif mean_err > 10.0 and over_pct < 15.0:
                new_kp = round(min(kp * 1.20, _LIMITS["VKP"][1]), 2)
                w_msgs.append(f"  {wn}: ✗ err {mean_err:.1f}RPM high "
                               f"→ KP {kp:.2f}→{new_kp} (+20%)")
            elif mean_err > 5.0 and over_pct < 15.0:
                new_kp = round(min(kp * 1.10, _LIMITS["VKP"][1]), 2)
                w_msgs.append(f"  {wn}: ~ err {mean_err:.1f}RPM moderate "
                               f"→ KP {kp:.2f}→{new_kp} (+10%)")

            # ── KI ────────────────────────────────────────────────────────
            transient_ok = (rise is None or rise < 1.0) and over_pct < 15.0
            if transient_ok and ss_err > 1.5 and ki < 0.5:
                new_ki = round(min(ki + 0.03, _LIMITS["VKI"][1]), 3)
                w_msgs.append(f"  {wn}: → SS {ss_err:.2f}RPM persists "
                               f"→ KI {ki:.3f}→{new_ki}")
            elif ki > 0.0 and ss_err < 0.5 and over_pct > 10.0:
                new_ki = round(max(ki * 0.7, 0.0), 3)
                w_msgs.append(f"  {wn}: ⚠ KI may cause overshoot "
                               f"→ KI {ki:.3f}→{new_ki}")

            # ── KD ────────────────────────────────────────────────────────
            if over_pct > 15.0 and osc > 4:
                new_kd = round(min(kd + 0.015, _LIMITS["VKD"][1]), 3)
                w_msgs.append(f"  {wn}: ↓ over {over_pct:.0f}% + osc {osc} "
                               f"→ KD {kd:.3f}→{new_kd} (damp)")
            elif kd > 0.0 and osc <= 2 and over_pct < 5.0:
                new_kd = round(max(kd * 0.7, 0.0), 3)
                w_msgs.append(f"  {wn}: ~ KD may amplify noise "
                               f"→ KD {kd:.3f}→{new_kd} (reduce)")

            if not w_msgs:
                w_msgs.append(f"  {wn}: ✓ err {mean_err:.2f}  ss {ss_err:.2f}  "
                               f"rise {'n/a' if rise is None else f'{rise:.2f}s'}  "
                               f"over {over_pct:.0f}% — OK")

            msgs.extend(w_msgs)

            if round(new_kp, 4) != round(kp, 4):
                cmds[f"VKP {wi}"] = new_kp
                any_suggestion = True
            if round(new_ki, 4) != round(ki, 4):
                cmds[f"VKI {wi}"] = new_ki
                any_suggestion = True
            if round(new_kd, 4) != round(kd, 4):
                cmds[f"VKD {wi}"] = new_kd
                any_suggestion = True

        if not any_suggestion:
            msgs.append("\n  All wheels look good — no parameter changes suggested.")

        return {"msgs": msgs, "cmds": cmds}

    # ── Auto run loop (wraps auto methods with vel analysis + suggest) ─────

    def _run_auto_loop(self, run_fn, *args) -> None:
        """
        Runs run_fn(*args), then shows velocity analysis and suggestions.
        Loops until user types 'done' or 'q'.
        """
        while True:
            self._auto_abort.clear()
            self._start_vel_poll()
            run_fn(*args)
            self._stop_vel_poll()

            if self._auto_abort.is_set():
                break

            analysis    = self._analyze_vel()
            self._print_vel_report(analysis)
            suggestion  = self._compute_vel_suggestions(analysis)

            print(f"  ── SUGGESTIONS ─────────────────────────────────────")
            for m in suggestion["msgs"]:
                print(f"  {m}")
            print()

            has_cmds = bool(suggestion.get("cmds"))
            prompt = (
                "  [ok=apply+rerun  skip=rerun  done=stop  q=quit] > "
                if has_cmds else
                "  [skip=rerun  done=stop  q=quit] > "
            )
            try:
                resp = input(prompt).strip().lower()
            except (EOFError, KeyboardInterrupt):
                break

            if resp == "q":
                self.running = False
                rclpy.shutdown()
                return
            if resp == "done":
                break
            if resp == "ok" and has_cmds:
                for cmd, val in suggestion["cmds"].items():
                    self._send(f"{cmd} {val:.6g}")
                    time.sleep(0.08)
            # "ok" or "skip": continue loop (rerun with new or same params)

    # ── AUTO ROT ─────────────────────────────────────────────────────────

    def _auto_rotate(self, angle: float, count: int) -> None:
        self._auto_abort.clear()
        print(f"\n  AUTO ROT  angle=±{angle}°  count={count}  (type ABORT to stop)\n")

        if not self._safe_reset_zero():
            return

        hdr = f"  {'#':>3}  {'Dir':>7}  {'Target':>8}  {'Yaw':>8}  {'Error':>8}  {'Time':>6}  {'rtol':>5}"
        print(hdr)
        print("  " + "─" * (len(hdr) - 2))

        errors: list[float] = []
        durations: list[float] = []

        for i in range(count):
            for target in (angle, 0.0):
                if self._auto_abort.is_set():
                    print("\n  ── aborted ──")
                    return

                label = f"+{angle:.0f}°" if target != 0.0 else "0°"
                t0 = time.monotonic()
                done_text, ok = self._send_and_wait(f"ROTATE {target:.1f}")
                elapsed = time.monotonic() - t0

                if not ok:
                    print(f"  {i+1:>3}  {label:>7}  FAILED")
                    return

                kv   = self._parse_kv(done_text)
                yaw  = kv.get("yaw",  math.nan)
                err  = kv.get("err",  math.nan)
                rtol = kv.get("rtol", math.nan)

                if not math.isnan(err):
                    errors.append(abs(err))
                durations.append(elapsed)

                print(f"  {i+1:>3}  {label:>7}  {target:>8.2f}  {yaw:>8.2f}  "
                      f"{err:>+8.3f}  {elapsed:>5.1f}s  {rtol:>5.2f}")
                time.sleep(0.25)

        print()
        if errors:
            mean_t = sum(durations) / len(durations)
            print(f"  ── SUMMARY ──────────────────────────────────")
            print(f"  Samples  : {len(errors)}")
            print(f"  Mean |e| : {sum(errors)/len(errors):.3f}°")
            print(f"  Max  |e| : {max(errors):.3f}°  ← worst")
            print(f"  Min  |e| : {min(errors):.3f}°  ← best")
            print(f"  Mean t   : {mean_t:.1f}s / rotation")
            print()

    # ── AUTO MOVE ────────────────────────────────────────────────────────

    def _auto_move(self, dist: float, count: int) -> None:
        self._auto_abort.clear()
        print(f"\n  AUTO MOVE  dist=±{dist}m  count={count}  (type ABORT to stop)\n")

        if not self._safe_reset_zero():
            return

        hdr = f"  {'#':>3}  {'Dir':>5}  {'Dist':>7}  {'Time':>7}  {'Result':>8}"
        print(hdr)
        print("  " + "─" * (len(hdr) - 2))

        times_fwd: list[float] = []
        times_back: list[float] = []

        for i in range(count):
            for label, d in (("fwd", dist), ("back", -dist)):
                if self._auto_abort.is_set():
                    print("\n  ── aborted ──")
                    return

                t0 = time.monotonic()
                _, ok = self._send_and_wait(f"MOVE {d:.3f} 0.0")
                elapsed = time.monotonic() - t0

                (times_fwd if label == "fwd" else times_back).append(elapsed)
                print(f"  {i+1:>3}  {label:>5}  {d:>+7.3f}  {elapsed:>6.1f}s  {'DONE' if ok else 'FAIL':>8}")

                if not ok:
                    return
                time.sleep(0.25)

        print()
        if times_fwd:
            print(f"  ── SUMMARY ──────────────────────────────────")
            print(f"  Fwd  mean: {sum(times_fwd)/len(times_fwd):.1f}s  "
                  f"max: {max(times_fwd):.1f}s")
            print(f"  Back mean: {sum(times_back)/len(times_back):.1f}s  "
                  f"max: {max(times_back):.1f}s")
            print()

    # ── AUTO TRIP ────────────────────────────────────────────────────────

    def _auto_trip(self, dist: float, count: int) -> None:
        self._auto_abort.clear()
        print(f"\n  AUTO TRIP  dist={dist}m  count={count}  (type ABORT to stop)\n")

        if not self._safe_reset_zero():
            return

        print(f"  {'Trip':>4}  {'Step':>14}  {'Err/Time':>10}  Result")
        print("  " + "─" * 44)

        rot_errors: list[float] = []
        move_times: list[float] = []
        ok_trips = 0

        sequence = [
            ("ROTATE 0.0",              "rot→0°"),
            (f"MOVE {dist:.3f} 0.0",    "move fwd"),
            ("ROTATE 0.0",              "rot@fruit"),
            ("ROTATE 180.0",            "rot→180°"),
            (f"MOVE {dist:.3f} 180.0",  "move back"),
            ("ROTATE 0.0",              "rot→home"),
        ]

        for i in range(count):
            if self._auto_abort.is_set():
                print("\n  ── aborted ──")
                break

            trip_ok = True
            for cmd, step_name in sequence:
                if self._auto_abort.is_set():
                    trip_ok = False
                    break

                t0 = time.monotonic()
                done_text, ok = self._send_and_wait(cmd)
                elapsed = time.monotonic() - t0

                if "ROTATE" in cmd:
                    kv  = self._parse_kv(done_text)
                    err = kv.get("err", math.nan)
                    label = f"{err:>+.3f}°" if not math.isnan(err) else "n/a"
                    if not math.isnan(err):
                        rot_errors.append(abs(err))
                else:
                    label = f"{elapsed:.1f}s"
                    move_times.append(elapsed)

                print(f"  {i+1:>4}  {step_name:>14}  {label:>10}  {'OK' if ok else 'FAIL'}")

                if not ok:
                    trip_ok = False
                    break
                time.sleep(0.2)

            if trip_ok:
                ok_trips += 1
            print()

        print(f"  ── SUMMARY ──────────────────────────────────")
        print(f"  Completed : {ok_trips}/{count} trips")
        if rot_errors:
            print(f"  Rot mean |e|: {sum(rot_errors)/len(rot_errors):.3f}°  "
                  f"max: {max(rot_errors):.3f}°")
        if move_times:
            print(f"  Move mean t : {sum(move_times)/len(move_times):.1f}s  "
                  f"max: {max(move_times):.1f}s")
        print()

    # ── Command dispatch ──────────────────────────────────────────────────

    def handle_line(self, line: str) -> None:
        raw = line.strip()
        if not raw:
            return

        if raw.upper() == "ABORT":
            self._auto_abort.set()
            print("  Abort requested.")
            return

        low = raw.lower()
        if low in ("q", "quit", "exit"):
            self.running = False
            rclpy.shutdown()
            return
        if low in ("help", "h", "?"):
            print(MENU)
            return
        if low == "p":
            self.show_status = True
            print("STATUS telemetry: ON")
            return
        if low == "e":
            self.show_status = False
            print("STATUS telemetry: OFF")
            return
        if low in ("zero", "zero_yaw"):
            self.zero_pub.publish(Empty())
            print("→ /zero_yaw")
            return

        parts = raw.upper().split()
        verb  = parts[0]

        # ── AUTO modes ────────────────────────────────────────────────
        if verb == "AUTO":
            if len(parts) < 3:
                print("usage: AUTO ROT <angle> [count]  |  AUTO MOVE <dist> [count]  |  AUTO TRIP <dist> [count]")
                return
            mode = parts[1]
            val = self._float(parts[2])
            if val is None:
                print("invalid value")
                return
            cnt_default = 5 if mode == "ROT" else 3
            cnt = int(self._float(parts[3]) or cnt_default) if len(parts) >= 4 else cnt_default

            if mode == "ROT":
                if not (1.0 <= abs(val) <= 180.0):
                    print("angle must be 1–180°")
                    return
                self._run_auto_loop(self._auto_rotate, val, cnt)
            elif mode == "MOVE":
                if abs(val) > 3.0:
                    print("distance > 3 m blocked")
                    return
                self._run_auto_loop(self._auto_move, val, cnt)
            elif mode == "TRIP":
                if abs(val) > 3.0:
                    print("distance > 3 m blocked")
                    return
                self._run_auto_loop(self._auto_trip, val, cnt)
            else:
                print(f"unknown auto mode '{mode}'  (ROT / MOVE / TRIP)")
            return

        # ── Motion ────────────────────────────────────────────────────
        if verb == "MOVE":
            if len(parts) < 2:
                print("usage: MOVE <meters> [heading_deg]")
                return
            dist = self._float(parts[1])
            if dist is None or abs(dist) > 3.0:
                print("invalid or unsafe distance (max 3 m)")
                return
            heading = self._float(parts[2]) if len(parts) >= 3 else 0.0
            self._send(f"MOVE {dist:.3f} {heading:.1f}")
            return

        if verb in ("ROTATE", "STOP", "RESUME", "RESET", "STATUS",
                    "HEADING", "HINVERT", "RINVERT"):
            self._send(" ".join(parts))
            return

        # ── Scalar tune commands ──────────────────────────────────────
        if verb in ("PKP", "PKI", "PKPF", "HKP", "HKI", "HKD",
                    "RKP", "RKI", "RTOL", "FKP", "FMAXRPM"):
            if len(parts) < 2:
                print(f"usage: {verb} <value>")
                return
            val = self._float(parts[1])
            if val is None:
                print("invalid value")
                return
            err = self._check_limit(verb, val)
            if err:
                print(f"BLOCKED: {err}")
                return
            self._send(f"{verb} {val:.6g}")
            return

        # ── Wheel velocity gains ──────────────────────────────────────
        if verb in (*_WHEEL_IDX, "ALL") and len(parts) == 3 and parts[1] in ("KP", "KI", "KD"):
            gain = parts[1]
            val  = self._float(parts[2])
            if val is None:
                print("invalid value")
                return
            limit_key = f"VK{gain}ALL" if verb == "ALL" else f"VK{gain}"
            err = self._check_limit(limit_key, val)
            if err:
                print(f"BLOCKED: {err}")
                return
            if verb == "ALL":
                self._send(f"VK{gain}ALL {val:.6g}")
            else:
                self._send(f"VK{gain} {_WHEEL_IDX[verb]} {val:.6g}")
            return

        print("unknown command — type 'help'")

    # ── Console loop ──────────────────────────────────────────────────────

    def console_loop(self) -> None:
        while rclpy.ok() and self.running:
            try:
                line = input("> ")
            except (EOFError, KeyboardInterrupt):
                self._auto_abort.set()
                self.running = False
                rclpy.shutdown()
                return
            self.handle_line(line)


# ---------------------------------------------------------------------------
def main(argv: Iterable[str] | None = None) -> None:
    rclpy.init(args=list(argv) if argv is not None else None)
    node = TestTuneNode()
    thread = threading.Thread(target=node.console_loop, daemon=True)
    thread.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._auto_abort.set()
    finally:
        node.running = False
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv[1:])
