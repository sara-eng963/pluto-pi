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
  AUTO VTEST <rpm> [duration_ms=2000]   PRIMARY source for wheel velocity PID tuning.
                                         Clean step-input; all wheels at <rpm> for <ms>.
                                         Metrics (rise/settle/overshoot/SS/osc) are valid.
  AUTO ROT  <angle_deg> [count=5]       Full-robot rotation validation (not for vel PID).
  AUTO MOVE <dist_m>    [count=3]       Full-robot move validation   (not for vel PID).
  AUTO TRIP <dist_m>    [count=3]       Full round-trip validation.

  VTEST <rpm> [duration_ms]            One-shot step test (no analysis loop).

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
        self._vkp_per_wheel: list[float] = [8.0,  6.0,  5.0,  6.5]
        self._vki_per_wheel: list[float] = [0.50, 0.35, 0.30, 0.35]
        self._vkd_per_wheel: list[float] = [0.03, 0.02, 0.02, 0.02]

        # Velocity sample collection (filled during auto modes)
        self._vel_samples: list[dict]              = []
        self._collecting_vel: bool                 = False
        self._vel_poll_stop                        = threading.Event()
        self._vel_poll_thread: Optional[threading.Thread] = None

        # Per-wheel suggestion history for oscillation dampening.
        # Stores recent suggestion directions: "KP↑", "KP↓", "KI↑", etc.
        self._suggest_history: list[list[str]] = [[], [], [], []]

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
        if verb not in ("MOVE", "ROTATE", "VTEST"):
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

    @staticmethod
    def _find_stable_windows(
        samples: list,
        wi: int,
        min_dur: float = 0.5,
        max_delta: float = 3.0,
        active_thr: float = 5.0,
    ) -> list:
        """
        Find contiguous windows where target[wi] is stable:
          - abs(target[wi]) > active_thr
          - consecutive target changes < max_delta RPM
          - window duration >= min_dur seconds

        Returns a list of windows; each window is a list of sample dicts.
        These are the only windows valid for SS-error, overshoot, settling-time,
        and oscillation analysis (MOVE ramps up/down and would corrupt all of
        those metrics if included).
        """
        windows: list = []
        current: list = []

        for s in samples:
            tgt = abs(s["target"][wi])

            if tgt <= active_thr:
                # Target inactive — close any open window
                if current:
                    dur = current[-1]["t"] - current[0]["t"]
                    if dur >= min_dur:
                        windows.append(current)
                    current = []
                continue

            if current:
                prev_tgt = abs(current[-1]["target"][wi])
                if abs(tgt - prev_tgt) < max_delta:
                    current.append(s)
                else:
                    # Significant target change — close old window, start new
                    dur = current[-1]["t"] - current[0]["t"]
                    if dur >= min_dur:
                        windows.append(current)
                    current = [s]
            else:
                current = [s]

        if current:
            dur = current[-1]["t"] - current[0]["t"]
            if dur >= min_dur:
                windows.append(current)

        return windows

    def _analyze_vel(self) -> dict:
        """
        Per-wheel time-domain and steady-state analysis using stable-target windows.

        Metrics per wheel:
          mean_err        — mean |target-measured| over all active (non-zero target) samples
          ss_error        — steady-state error from last 25 % of each stable window
          max_overshoot   — peak (|measured|-|target|) inside stable windows only
          overshoot_pct   — max_overshoot as % of mean target inside stable windows
          rise_time_s     — 10→90 % rise time inside the first stable window
          settling_time_s — first time wheel stays in ±15 % until end of each window
                            (None/NOT_SETTLED if it never settles in any window)
          oscillations    — sign changes in tracking error with ±2 RPM deadband
          stable_windows  — number of stable-target windows found
        """
        samples = self._vel_samples
        ACTIVE_THR  = 5.0   # RPM — below this target is considered "off"
        BAND        = 0.15  # ±15 % settling band
        OSC_DEADBAND = 2.0  # RPM — ignore sign changes smaller than this

        active = [s for s in samples
                  if any(abs(t) > ACTIVE_THR for t in s["target"])]

        result: dict = {
            "total":          len(samples),
            "active":         len(active),
            "mean_err":       [0.0]  * 4,
            "ss_error":       [0.0]  * 4,
            "max_overshoot":  [0.0]  * 4,
            "overshoot_pct":  [0.0]  * 4,
            "rise_time_s":    [None] * 4,
            "settling_time_s":[None] * 4,
            "oscillations":   [0]    * 4,
            "mean_target":    [0.0]  * 4,
            "mean_measured":  [0.0]  * 4,
            "stable_windows": [0]    * 4,
        }

        if not active:
            return result

        for wi in range(4):
            # ── Global stats (all active samples) ─────────────────────
            wa = [s for s in active if abs(s["target"][wi]) > ACTIVE_THR]
            if not wa:
                continue

            tgts_all = [s["target"][wi]   for s in wa]
            meas_all = [s["measured"][wi] for s in wa]
            errs_all = [abs(t - m) for t, m in zip(tgts_all, meas_all)]

            result["mean_target"][wi]   = sum(abs(t) for t in tgts_all) / len(tgts_all)
            result["mean_measured"][wi] = sum(abs(m) for m in meas_all) / len(meas_all)
            result["mean_err"][wi]      = sum(errs_all) / len(errs_all)

            # ── Stable-window analysis ────────────────────────────────
            windows = self._find_stable_windows(samples, wi)
            result["stable_windows"][wi] = len(windows)

            if not windows:
                # No stable window — fall back to last-25 % for SS error only
                tail = max(1, len(wa) // 4)
                result["ss_error"][wi] = sum(errs_all[-tail:]) / tail
                # Oscillations with deadband on all active samples
                signed = [t - m for t, m in zip(tgts_all, meas_all)]
                result["oscillations"][wi] = sum(
                    1 for i in range(1, len(signed))
                    if abs(signed[i]) > OSC_DEADBAND
                    and abs(signed[i - 1]) > OSC_DEADBAND
                    and signed[i] * signed[i - 1] < 0
                )
                continue

            all_ss_errs: list[float]       = []
            all_overshoots: list[float]    = []
            rise_times: list[float]        = []
            settling_times: list           = []   # float or None per window
            all_osc_signed: list[float]    = []

            for win in windows:
                w_tgts = [s["target"][wi]   for s in win]
                w_meas = [s["measured"][wi] for s in win]
                w_ts   = [s["t"]            for s in win]
                w_errs = [abs(t - m) for t, m in zip(w_tgts, w_meas)]

                # Steady-state error: last 25 % of this stable window
                tail = max(1, len(win) // 4)
                all_ss_errs.extend(w_errs[-tail:])

                # Overshoot: |measured| exceeds |target| inside stable window
                # (this excludes deceleration ramps that precede the window)
                for t_, m_ in zip(w_tgts, w_meas):
                    excess = abs(m_) - abs(t_)
                    if excess > 0:
                        all_overshoots.append(excess)

                # Rise time: 10 % → 90 % inside the first stable window only
                if not rise_times:
                    abs_tgt  = [abs(t) for t in w_tgts]
                    abs_meas = [abs(m) for m in w_meas]
                    pk = max(abs_tgt) if abs_tgt else 0.0
                    if pk > ACTIVE_THR:
                        t10, t90 = pk * 0.10, pk * 0.90
                        idx10 = next((i for i, m in enumerate(abs_meas) if m >= t10), None)
                        idx90 = next((i for i, m in enumerate(abs_meas) if m >= t90), None)
                        if idx10 is not None and idx90 is not None and idx90 > idx10:
                            rise_times.append(round(w_ts[idx90] - w_ts[idx10], 3))

                # Settling time: first index from which ALL subsequent samples
                # in this window stay within ±BAND of target.
                # Returns None if wheel never settles during this window.
                w_t0 = w_ts[0]
                n = len(win)
                settled_at = None
                for i in range(n):
                    if abs(w_tgts[i]) <= ACTIVE_THR:
                        continue
                    all_in_band = all(
                        abs(abs(w_meas[j]) - abs(w_tgts[j])) <= BAND * abs(w_tgts[j])
                        for j in range(i, n)
                        if abs(w_tgts[j]) > ACTIVE_THR
                    )
                    if all_in_band:
                        settled_at = round(w_ts[i] - w_t0, 3)
                        break
                settling_times.append(settled_at)

                # Oscillation signed errors for this window
                all_osc_signed.extend([t - m for t, m in zip(w_tgts, w_meas)])

            # Aggregate SS error
            if all_ss_errs:
                result["ss_error"][wi] = sum(all_ss_errs) / len(all_ss_errs)

            # Overshoot
            if all_overshoots:
                pk_over = max(all_overshoots)
                result["max_overshoot"][wi] = pk_over
                avg_tgt = result["mean_target"][wi]
                result["overshoot_pct"][wi] = (
                    pk_over / avg_tgt * 100.0 if avg_tgt > 0 else 0.0
                )

            # Rise time (first window only)
            if rise_times:
                result["rise_time_s"][wi] = rise_times[0]

            # Settling time: report worst-case settled time;
            # if ANY window never settled → NOT_SETTLED (None)
            settled_vals = [s for s in settling_times if s is not None]
            if len(settled_vals) == len(settling_times) and settled_vals:
                result["settling_time_s"][wi] = max(settled_vals)
            else:
                result["settling_time_s"][wi] = None  # never settled in ≥1 window

            # Oscillations with ±OSC_DEADBAND deadband
            result["oscillations"][wi] = sum(
                1 for i in range(1, len(all_osc_signed))
                if abs(all_osc_signed[i])     > OSC_DEADBAND
                and abs(all_osc_signed[i - 1]) > OSC_DEADBAND
                and all_osc_signed[i] * all_osc_signed[i - 1] < 0
            )

        return result

    def _print_vel_report(self, a: dict) -> None:
        print()
        print(f"  ── VELOCITY LOOP ANALYSIS ─────────────────────────────")
        wins_str = "  ".join(
            f"{n}:{a['stable_windows'][i]}w"
            for i, n in enumerate(["R1", "R2", "F1", "F2"])
        )
        print(f"  Samples: {a['active']} active / {a['total']} total  "
              f"stable-windows: {wins_str}")
        if a["active"] < 3:
            print(f"  Not enough active motion data.")
            return
        names = ["R1", "R2", "F1", "F2"]

        hdr = (f"  {'Whl':>4}  {'Target':>7}  {'Meas':>7}  "
               f"{'MeanErr':>8}  {'SSErr':>6}  "
               f"{'Over%':>6}  {'Rise':>6}  {'Settle':>8}  {'Osc':>4}")
        print(f"\n{hdr}")
        print("  " + "─" * (len(hdr) - 2))

        def _fmt(v, fmt):
            return "NO_SET" if v is None else format(v, fmt)

        for wi, wn in enumerate(names):
            settle_s = a["settling_time_s"][wi]
            settle_str = "NOT_SET" if settle_s is None else f"{settle_s:8.2f}"
            print(
                f"  {wn:>4}  "
                f"{a['mean_target'][wi]:>7.1f}  "
                f"{a['mean_measured'][wi]:>7.1f}  "
                f"{a['mean_err'][wi]:>8.2f}  "
                f"{a['ss_error'][wi]:>6.2f}  "
                f"{a['overshoot_pct'][wi]:>6.1f}  "
                f"{_fmt(a['rise_time_s'][wi], '6.3f')}  "
                f"{settle_str}  "
                f"{a['oscillations'][wi]:>4}"
            )

        avg_e  = sum(a["mean_err"]) / 4
        avg_ss = sum(a["ss_error"]) / 4
        avg_op = sum(a["overshoot_pct"]) / 4
        print(f"  {'AVG':>4}  {'':>7}  {'':>7}  "
              f"{avg_e:>8.2f}  {avg_ss:>6.2f}  {avg_op:>6.1f}")
        print(
            f"\n  MeanErr=all-active err(RPM)  SSErr=stable-window SS err  "
            f"Over%=stable-window overshoot  Rise=10→90%(s)  "
            f"Settle=first-settled(s)/NOT_SET  Osc=deadband sign-changes"
        )
        print()

    def _compute_vel_suggestions(self, a: dict) -> dict:
        """
        Per-wheel PID suggestions — ONE parameter type per wheel per cycle.

        Priority order:
          0. STALLED  — mean_measured < 15 % of mean_target despite long window
                        → increase KI (integral overcomes static friction deadband)
          1. KP ↓     — unstable / overshooting  (over% > 25 OR osc > 6)
          2. KP ↑     — slow / lagging            (rise > 0.8 s or mean_err > 10)
          3. KI ↑     — persistent SS error       (ss_err > 1.5 RPM, transient OK)
          4. KD ↑     — over + oscillation remain (over% > 15 AND osc > 4)
          5. KI ↓     — KI causing overshoot      (over% high, ss_err low)
          6. KD ↓     — KD adding noise, clean response
          7. OK       — all metrics within acceptable range

        Oscillation dampening:
          If the last two suggestions for a wheel were in opposite directions
          (e.g., KP↑ then KP↓), the step size is halved to prevent limit-cycling.

        OK criteria (all must hold):
          over% < 15%, settle != None or (osc <= 2 and over% < 10%), mean_err < 8 RPM
        """
        _WNAMES = ["R1", "R2", "F1", "F2"]
        _STALL_RATIO = 0.15   # mean_measured / mean_target — below this = stalled

        if a["active"] < 3:
            return {"msgs": ["not enough active motion data"], "cmds": {}}

        msgs: list[str] = []
        cmds: dict[str, float] = {}
        any_suggestion = False

        for wi, wn in enumerate(_WNAMES):
            mean_err    = a["mean_err"][wi]
            ss_err      = a["ss_error"][wi]
            over_pct    = a["overshoot_pct"][wi]
            rise        = a["rise_time_s"][wi]
            settle      = a["settling_time_s"][wi]
            osc         = a["oscillations"][wi]
            n_wins      = a["stable_windows"][wi]
            mean_tgt    = a["mean_target"][wi]
            mean_meas   = a["mean_measured"][wi]

            kp = self._vkp_per_wheel[wi]
            ki = self._vki_per_wheel[wi]
            kd = self._vkd_per_wheel[wi]

            # ── Oscillation dampening ──────────────────────────────────────
            hist = self._suggest_history[wi][-3:]  # last 3 suggestions
            def _oscillating(param: str) -> bool:
                """True if last 2 suggestions for this param alternated direction."""
                ups   = [h for h in hist if h == f"{param}↑"]
                downs = [h for h in hist if h == f"{param}↓"]
                if len(hist) >= 2 and hist[-1] != hist[-2]:
                    # last two differ AND both are for the same param
                    if (hist[-1].startswith(param) and hist[-2].startswith(param)):
                        return True
                return False

            def _step(base: float, direction_key: str, pct: float) -> float:
                """Apply pct step, halved if we've been oscillating on this param."""
                effective = pct * 0.5 if _oscillating(direction_key) else pct
                return effective

            def _record(action: str) -> None:
                self._suggest_history[wi].append(action)
                if len(self._suggest_history[wi]) > 6:
                    self._suggest_history[wi].pop(0)

            # ── Priority 0: STALLED wheel ──────────────────────────────────
            stall_ratio = mean_meas / mean_tgt if mean_tgt > 0 else 1.0
            if stall_ratio < _STALL_RATIO and n_wins > 0:
                # Wheel is not spinning despite a stable command.
                # KP alone won't overcome static friction — use KI to build up
                # integral drive, but only if anti-windup is in firmware.
                new_ki = round(min(ki + 0.15, _LIMITS["VKI"][1]), 3)
                cmds[f"VKI {wi}"] = new_ki
                msgs.append(
                    f"  {wn} [STALLED→KI↑]: meas {mean_meas:.1f} << tgt {mean_tgt:.1f}"
                    f" ({100*stall_ratio:.0f}%)  → KI {ki:.3f} → {new_ki}"
                    f"  (integral overcomes static friction)"
                )
                _record("KI↑")
                any_suggestion = True
                continue

            # ── Priority 1: Reduce KP (unstable / overshooting) ───────────
            if over_pct > 25.0 or (settle is None and osc > 6 and n_wins > 0):
                base_pct = 0.22 if over_pct > 40.0 or osc > 12 else 0.13
                effective_pct = _step(kp, "KP", base_pct)
                new_kp = round(max(kp * (1 - effective_pct), 0.5), 2)
                cmds[f"VKP {wi}"] = new_kp
                msgs.append(
                    f"  {wn} [KP↓]: over {over_pct:.0f}%  osc {osc}"
                    f"  settle={'n/s' if settle is None else f'{settle:.2f}s'}"
                    f" → KP {kp:.2f} → {new_kp}"
                    + ("  (halved — oscillating)" if _oscillating("KP") else f"  (↓{100*effective_pct:.0f}%)")
                )
                _record("KP↓")
                any_suggestion = True
                continue

            # ── Priority 2: Increase KP (slow rise / high tracking error) ──
            if (rise is not None and rise > 0.8 and over_pct < 10.0) or \
               (mean_err > 10.0 and over_pct < 15.0):
                base_pct = 0.20 if mean_err > 10.0 else 0.15
                effective_pct = _step(kp, "KP", base_pct)
                new_kp = round(min(kp * (1 + effective_pct), _LIMITS["VKP"][1]), 2)
                reason = (f"rise {rise:.2f}s" if rise is not None and rise > 0.8
                          else f"err {mean_err:.1f} RPM")
                cmds[f"VKP {wi}"] = new_kp
                msgs.append(
                    f"  {wn} [KP↑]: {reason}"
                    f" → KP {kp:.2f} → {new_kp}"
                    + ("  (halved — oscillating)" if _oscillating("KP") else f"  (+{100*effective_pct:.0f}%)")
                )
                _record("KP↑")
                any_suggestion = True
                continue

            if mean_err > 5.0 and over_pct < 15.0:
                effective_pct = _step(kp, "KP", 0.10)
                new_kp = round(min(kp * (1 + effective_pct), _LIMITS["VKP"][1]), 2)
                cmds[f"VKP {wi}"] = new_kp
                msgs.append(
                    f"  {wn} [KP↑]: err {mean_err:.1f} RPM moderate"
                    f" → KP {kp:.2f} → {new_kp}  (+{100*effective_pct:.0f}%)"
                )
                _record("KP↑")
                any_suggestion = True
                continue

            # ── Priority 3: Increase KI (persistent SS error) ─────────────
            transient_ok = (rise is None or rise < 1.0) and over_pct < 15.0
            if transient_ok and ss_err > 1.5 and ki < 0.5 and n_wins > 0:
                new_ki = round(min(ki + 0.03, _LIMITS["VKI"][1]), 3)
                cmds[f"VKI {wi}"] = new_ki
                msgs.append(
                    f"  {wn} [KI↑]: SS {ss_err:.2f} RPM (from {n_wins} window(s))"
                    f" → KI {ki:.3f} → {new_ki}"
                )
                _record("KI↑")
                any_suggestion = True
                continue

            # ── Priority 4: Increase KD (over + oscillation persist) ───────
            if over_pct > 15.0 and osc > 4:
                new_kd = round(min(kd + 0.015, _LIMITS["VKD"][1]), 3)
                cmds[f"VKD {wi}"] = new_kd
                msgs.append(
                    f"  {wn} [KD↑]: over {over_pct:.0f}%  osc {osc}"
                    f" → KD {kd:.3f} → {new_kd}  (damp)"
                )
                _record("KD↑")
                any_suggestion = True
                continue

            # ── Priority 5: Reduce KI (KI causing overshoot) ───────────────
            if ki > 0.0 and ss_err < 0.5 and over_pct > 10.0:
                new_ki = round(max(ki * 0.7, 0.0), 3)
                cmds[f"VKI {wi}"] = new_ki
                msgs.append(
                    f"  {wn} [KI↓]: KI may cause over {over_pct:.0f}%"
                    f" → KI {ki:.3f} → {new_ki}"
                )
                _record("KI↓")
                any_suggestion = True
                continue

            # ── Priority 6: Reduce KD (clean response, KD adding noise) ────
            if kd > 0.0 and osc <= 2 and over_pct < 5.0 and settle is not None:
                new_kd = round(max(kd * 0.7, 0.0), 3)
                cmds[f"VKD {wi}"] = new_kd
                msgs.append(
                    f"  {wn} [KD↓]: clean response, reduce noise risk"
                    f" → KD {kd:.3f} → {new_kd}"
                )
                _record("KD↓")
                any_suggestion = True
                continue

            # ── OK — all criteria met ─────────────────────────────────────
            # Strict OK: over% < 15%, either settled or (osc<=2 AND over%<10%), mean_err < 8
            ok_settle = (settle is not None) or (osc <= 2 and over_pct < 10.0)
            if over_pct < 15.0 and ok_settle and mean_err < 8.0:
                settle_str = "NOT_SET" if settle is None else f"{settle:.2f}s"
                rise_str   = "n/a"     if rise   is None else f"{rise:.2f}s"
                msgs.append(
                    f"  {wn}: ✓  err {mean_err:.2f}  ss {ss_err:.2f}  "
                    f"over {over_pct:.0f}%  rise {rise_str}  settle {settle_str}"
                    f"  osc {osc}  wins {n_wins} — OK"
                )
                _record("OK")
            else:
                # Not quite OK but no rule triggered cleanly — nudge KP slightly
                effective_pct = 0.05
                if over_pct >= 15.0:
                    new_kp = round(max(kp * (1 - effective_pct), 0.5), 2)
                    cmds[f"VKP {wi}"] = new_kp
                    msgs.append(
                        f"  {wn} [KP↓ gentle]: over {over_pct:.0f}%  settle={'n/s' if settle is None else f'{settle:.2f}s'}"
                        f" → KP {kp:.2f} → {new_kp}  (−5%)"
                    )
                    _record("KP↓")
                    any_suggestion = True
                else:
                    settle_str = "NOT_SET" if settle is None else f"{settle:.2f}s"
                    msgs.append(
                        f"  {wn}: ~ err {mean_err:.2f}  ss {ss_err:.2f}  "
                        f"over {over_pct:.0f}%  settle {settle_str}  osc {osc}  — MARGINAL"
                    )

        if not any_suggestion:
            msgs.append("\n  All wheels look good — no parameter changes suggested.")

        return {"msgs": msgs, "cmds": cmds}

    # ── Auto run loop (wraps auto methods with vel analysis + suggest) ─────

    def _run_auto_loop(self, run_fn, *args) -> None:
        """
        Runs run_fn(*args), then shows velocity analysis and suggestions.
        Loops until user types 'done' or 'q'.
        """
        # Reset suggestion history at the start of each new auto loop so that
        # oscillation dampening does not carry state between different RPM tests.
        self._suggest_history = [[], [], [], []]

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

    # ── AUTO VTEST ───────────────────────────────────────────────────────

    def _auto_vtest(self, rpm: float, duration_ms: int) -> None:
        """
        Dedicated wheel-velocity step test using the VTEST firmware command.

        All wheels are commanded to `rpm` simultaneously for `duration_ms`.
        This is a clean step-input so all velocity PID metrics (rise, settle,
        overshoot, SS error, oscillations) are valid — unlike MOVE where
        target RPM ramps up/down with position PID.
        """
        self._auto_abort.clear()
        dur_s = duration_ms / 1000.0
        print(f"\n  AUTO VTEST  rpm={rpm:.1f}  duration={duration_ms}ms  "
              f"(type ABORT to stop)\n")

        if not self._safe_reset_zero():
            return

        print(f"  Sending VTEST {rpm:.1f} {duration_ms} ...")
        done_timeout = dur_s + 5.0   # firmware runs for duration_ms then sends DONE
        done_text, ok = self._send_and_wait(
            f"VTEST {rpm:.1f} {duration_ms}",
            ack_timeout=_ACK_TIMEOUT,
            done_timeout=done_timeout,
        )

        if not ok:
            print(f"  ✗ VTEST did not complete cleanly: {done_text}")
            return

        print(f"  VTEST complete: {done_text}")
        time.sleep(0.1)

    # ── Scoring stubs (future: position / heading / rotate / final-yaw PID) ─
    #
    # TODO: implement _analyze_position() — metrics from AUTO MOVE runs:
    #   - final_dist_err_m    : |currentDistance - targetDistance| at DONE
    #   - overshoot_dist_m    : peak distance beyond target during approach
    #   - move_timeout        : True if MOVE FAULT TIMEOUT
    #   - stop_smoothness     : velocity jerk in last 200 ms (low = smooth stop)
    #
    # TODO: implement _analyze_heading() — metrics from straight MOVE runs:
    #   - rms_yaw_err_deg     : RMS of heading error sampled during MOVE
    #   - max_yaw_dev_deg     : peak heading deviation from commanded direction
    #
    # TODO: implement _analyze_rotate() — metrics from AUTO ROT runs:
    #   - final_yaw_err_deg   : |yaw - target| at DONE (from DONE ROTATE kv)
    #   - overshoot_angle_deg : peak signed overshoot beyond target
    #   - oscillation_count   : heading sign-changes near target before settling
    #   - rotation_time_s     : wall time from ACK to DONE
    #
    # TODO: implement _analyze_final_yaw() — metrics for small-angle correction:
    #   - corrected_without_buzz : True if final hold reached without oscillation
    #   - correction_time_s      : time to settle inside HEADING_TOLERANCE_DEG

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
                print("usage: AUTO ROT <angle> [count]  |  AUTO MOVE <dist> [count]  |  "
                      "AUTO TRIP <dist> [count]  |  AUTO VTEST <rpm> [duration_ms]")
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
            elif mode == "VTEST":
                # AUTO VTEST <rpm> [duration_ms=2000]
                # Primary source for wheel velocity PID tuning (clean step input).
                # Use AUTO MOVE/ROT to validate full-robot behaviour afterward.
                duration_ms = int(self._float(parts[3]) or 2000) if len(parts) >= 4 else 2000
                if duration_ms < 500 or duration_ms > 10000:
                    print("duration_ms must be 500–10000")
                    return
                if abs(val) > 60.0:
                    print("rpm > 60 blocked")
                    return
                self._run_auto_loop(self._auto_vtest, val, duration_ms)
            else:
                print(f"unknown auto mode '{mode}'  (ROT / MOVE / TRIP / VTEST)")
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

        # VTEST <rpm> [duration_ms=2000] — one-shot step test (no auto loop)
        if verb == "VTEST":
            if len(parts) < 2:
                print("usage: VTEST <rpm> [duration_ms=2000]")
                return
            rpm_val = self._float(parts[1])
            if rpm_val is None or abs(rpm_val) > 60.0:
                print("invalid or unsafe rpm (max ±60)")
                return
            dur_ms = int(self._float(parts[2]) or 2000) if len(parts) >= 3 else 2000
            if dur_ms < 500 or dur_ms > 10000:
                print("duration_ms must be 500–10000")
                return
            self._send(f"VTEST {rpm_val:.1f} {dur_ms}")
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
