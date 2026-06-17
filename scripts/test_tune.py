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
    "VKP":     (0.0,   5.0),
    "VKI":     (0.0,   2.0),
    "VKD":     (0.0,   1.0),
    "VKPALL":  (0.0,   5.0),
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

        if text.startswith("STATUS") and not self.show_status:
            return
        print(f"\r\033[KESP> {text}\n> ", end="", flush=True)

    # ── Low-level send ────────────────────────────────────────────────────

    def _send(self, command: str) -> None:
        msg = String()
        msg.data = command.strip()
        self.drive_pub.publish(msg)
        print(f"→ {msg.data}")

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
                self._auto_rotate(val, cnt)
            elif mode == "MOVE":
                if abs(val) > 3.0:
                    print("distance > 3 m blocked")
                    return
                self._auto_move(val, cnt)
            elif mode == "TRIP":
                if abs(val) > 3.0:
                    print("distance > 3 m blocked")
                    return
                self._auto_trip(val, cnt)
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
