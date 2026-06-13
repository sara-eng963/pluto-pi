#!/usr/bin/env python3

"""
Interactive PID tuning node for ESP1 drive control.

Publishes:
  /drive_cmd     std_msgs/msg/String
Subscribes:
  /drive_status  std_msgs/msg/String

Usage:
  ros2 run pluto test_pid.py
"""

import math
import threading
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


ACK_TIMEOUT_SEC = 5.0
ROTATE_TIMEOUT_SEC = 12.0
MOVE_TIMEOUT_SEC = 30.0
ANY_RESPONSE_TIMEOUT_SEC = 5.0
POSITION_TOLERANCE_M = 0.001

TUNING_KEYWORDS = {
    "PKP",
    "PKI",
    "HKP",
    "HKI",
    "RKP",
    "RKI",
    "RTOL",
    "VKP",
    "VKI",
    "VKPALL",
    "VKIALL",
    "HEADING",
    "HINVERT",
    "RINVERT",
    "STATUS",
    "STOP",
}

MOTION_KEYWORDS = {"MOVE", "ROTATE"}
FAULT_PREFIXES = ("FAULT", "ERR")


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass
class ParsedResponse:
    seq: int
    raw: str
    kind: str


def normalize_yaw_deg(yaw: float) -> float:
    while yaw > 180.0:
        yaw -= 360.0
    while yaw <= -180.0:
        yaw += 360.0
    return yaw


def heading_for_dx(dx: float) -> float:
    return 0.0 if dx >= 0.0 else 180.0


def heading_for_dy(dy: float) -> float:
    return 90.0 if dy >= 0.0 else -90.0


def parse_three_floats(text: str) -> Optional[Tuple[float, float, float]]:
    parts = text.strip().split()
    if len(parts) != 3:
        return None
    try:
        return float(parts[0]), float(parts[1]), float(parts[2])
    except ValueError:
        return None


def fmt_num(value: float, decimals: int = 3) -> str:
    text = f"{value:.{decimals}f}".rstrip("0").rstrip(".")
    return text if text else "0"


class TestPidNode(Node):
    def __init__(self) -> None:
        super().__init__("test_pid_node")
        self.cmd_pub = self.create_publisher(String, "/drive_cmd", 10)
        self.status_sub = self.create_subscription(String, "/drive_status", self._status_cb, 10)

        self.pose = Pose2D()

        self._seq = 0
        self._responses: List[ParsedResponse] = []
        self._lock = threading.Lock()
        self._cv = threading.Condition(self._lock)

    def _classify_response(self, text: str) -> str:
        upper = text.upper()
        if upper.startswith("ACK"):
            return "ACK"
        if upper.startswith("DONE"):
            if "MOVE" in upper:
                return "DONE_MOVE"
            if "ROTATE" in upper:
                return "DONE_ROTATE"
            return "DONE"
        if upper.startswith("STATUS"):
            return "STATUS"
        if upper.startswith(FAULT_PREFIXES):
            return "FAULT"
        return "OTHER"

    def _status_cb(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            return

        parsed = ParsedResponse(seq=0, raw=text, kind=self._classify_response(text))
        with self._cv:
            self._seq += 1
            parsed.seq = self._seq
            self._responses.append(parsed)
            if len(self._responses) > 500:
                self._responses = self._responses[-500:]
            self._cv.notify_all()

        print(f"ESP: {text}")

    def _publish(self, command: str) -> int:
        msg = String()
        msg.data = command
        with self._lock:
            start_seq = self._seq
        self.cmd_pub.publish(msg)
        print(f"SEND: {command}")
        return start_seq

    def _find_matching_since(
        self,
        start_seq: int,
        wanted_kinds: Optional[set] = None,
        allow_any: bool = False,
    ) -> Optional[ParsedResponse]:
        for item in self._responses:
            if item.seq <= start_seq:
                continue
            if allow_any:
                return item
            if wanted_kinds and item.kind in wanted_kinds:
                return item
        return None

    def _wait_for(
        self,
        start_seq: int,
        timeout_sec: float,
        wanted_kinds: Optional[set] = None,
        allow_any: bool = False,
    ) -> Optional[ParsedResponse]:
        deadline = time.time() + timeout_sec
        with self._cv:
            while rclpy.ok():
                match = self._find_matching_since(
                    start_seq=start_seq,
                    wanted_kinds=wanted_kinds,
                    allow_any=allow_any,
                )
                if match is not None:
                    return match

                remaining = deadline - time.time()
                if remaining <= 0.0:
                    return None
                self._cv.wait(timeout=min(0.05, remaining))
        return None

    def send_tuning_command(self, command: str) -> bool:
        start_seq = self._publish(command)
        response = self._wait_for(
            start_seq=start_seq,
            timeout_sec=ANY_RESPONSE_TIMEOUT_SEC,
            wanted_kinds={"ACK", "STATUS", "OTHER", "FAULT", "DONE", "DONE_MOVE", "DONE_ROTATE"},
            allow_any=False,
        )
        if response is None:
            print("Timeout waiting for ESP response.")
            return False
        if response.kind == "FAULT":
            print("Command failed due to ESP fault/error.")
            return False
        return True

    def _send_stop_on_failure(self) -> None:
        stop_start = self._publish("STOP")
        self._wait_for(stop_start, timeout_sec=2.0, allow_any=True)

    def send_motion_command(self, command: str) -> bool:
        upper = command.upper()
        done_kind = "DONE_MOVE" if upper.startswith("MOVE") else "DONE_ROTATE"
        motion_timeout = MOVE_TIMEOUT_SEC if done_kind == "DONE_MOVE" else ROTATE_TIMEOUT_SEC

        start_seq = self._publish(command)

        ack = self._wait_for(
            start_seq=start_seq,
            timeout_sec=ACK_TIMEOUT_SEC,
            wanted_kinds={"ACK", "FAULT"},
        )
        if ack is None:
            print("Timeout waiting for ACK. Sending STOP.")
            self._send_stop_on_failure()
            return False
        if ack.kind == "FAULT":
            print("ESP fault/error while waiting for ACK.")
            return False

        done = self._wait_for(
            start_seq=ack.seq,
            timeout_sec=motion_timeout,
            wanted_kinds={done_kind, "FAULT"},
        )
        if done is None:
            print("Timeout waiting for DONE. Sending STOP.")
            self._send_stop_on_failure()
            return False
        if done.kind == "FAULT":
            print("ESP fault/error while waiting for DONE.")
            return False
        return True

    def set_pose(self, x: float, y: float, yaw: float) -> None:
        self.pose = Pose2D(x=float(x), y=float(y), yaw=normalize_yaw_deg(float(yaw)))

    def apply_move(self, distance_m: float, heading_deg: float) -> None:
        rad = math.radians(heading_deg)
        self.pose.x += distance_m * math.cos(rad)
        self.pose.y += distance_m * math.sin(rad)

    def apply_rotate(self, heading_deg: float) -> None:
        self.pose.yaw = normalize_yaw_deg(heading_deg)

    def build_manhattan_sequence(self, target: Pose2D) -> List[str]:
        seq: List[str] = []

        dx = target.x - self.pose.x
        dy = target.y - self.pose.y

        if abs(dx) > POSITION_TOLERANCE_M:
            hx = heading_for_dx(dx)
            seq.append(f"ROTATE {fmt_num(hx)}")
            seq.append(f"MOVE {fmt_num(abs(dx))} {fmt_num(hx)}")

        if abs(dy) > POSITION_TOLERANCE_M:
            hy = heading_for_dy(dy)
            seq.append(f"ROTATE {fmt_num(hy)}")
            seq.append(f"MOVE {fmt_num(abs(dy))} {fmt_num(hy)}")

        seq.append(f"ROTATE {fmt_num(normalize_yaw_deg(target.yaw))}")
        return seq


def command_is_motion(command: str) -> bool:
    up = command.strip().upper()
    return up.startswith("MOVE") or up.startswith("ROTATE")


def command_is_tuning_or_control(command: str) -> bool:
    parts = command.strip().split()
    if not parts:
        return False
    keyword = parts[0].upper()
    return keyword in TUNING_KEYWORDS or keyword in MOTION_KEYWORDS


def print_help() -> None:
    print("Commands:")
    print("  help                 Show this help")
    print("  status               Send STATUS")
    print("  stop                 Send STOP")
    print("  pose                 Print logical pose")
    print("  setpose x y yaw      Set logical pose")
    print("  home                 Navigate to 0 0 0 (with tuning stage)")
    print("  exit                 Quit")
    print("")
    print("Direct ESP commands examples:")
    print("  PKP 0.8")
    print("  HKP 60")
    print("  RKI 0.2")
    print("  VKPALL 5")
    print("  MOVE 0.30 0")
    print("  ROTATE 90")
    print("  0.5 0 90   (coordinate target)")


def parse_setpose(line: str) -> Optional[Tuple[float, float, float]]:
    parts = line.strip().split()
    if len(parts) != 4 or parts[0].lower() != "setpose":
        return None
    try:
        return float(parts[1]), float(parts[2]), float(parts[3])
    except ValueError:
        return None


def execute_coordinate_target(node: TestPidNode, target: Pose2D) -> None:
    print(f"Current target: x={target.x:.2f} y={target.y:.2f} yaw={target.yaw:.1f}")
    print("Enter tuning commands now.")
    print("Examples:")
    print("  RKP 8")
    print("  RKI 0.2")
    print("  RTOL 1")
    print("  HKP 60")
    print("  HKI 2")
    print("  VKPALL 5")
    print("  done")
    print("  cancel")

    while rclpy.ok():
        try:
            line = input("tune> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nTune stage canceled.")
            return

        if not line:
            continue

        lower = line.lower()
        if lower == "done":
            break
        if lower == "cancel":
            print("Movement canceled.")
            return
        if lower == "help":
            print("Tune stage commands:")
            print("  PKP/PKI/HKP/HKI/RKP/RKI/RTOL/VKP/VKI/VKPALL/VKIALL")
            print("  HEADING ON|OFF, HINVERT, RINVERT, STATUS, STOP")
            print("  done, cancel")
            continue

        if not command_is_tuning_or_control(line):
            print("Unsupported tune command. Type help for tune commands.")
            continue

        if command_is_motion(line):
            print("Motion commands are not allowed in tune stage. Use done then execute generated sequence.")
            continue

        node.send_tuning_command(line)

    sequence = node.build_manhattan_sequence(target)
    print("Generated sequence:")
    for cmd in sequence:
        print(f"  {cmd}")

    for cmd in sequence:
        ok = node.send_motion_command(cmd)
        if not ok:
            print(f"Sequence aborted at: {cmd}")
            return

        upper = cmd.upper()
        parts = cmd.split()
        if upper.startswith("ROTATE") and len(parts) >= 2:
            node.apply_rotate(float(parts[1]))
        elif upper.startswith("MOVE") and len(parts) >= 3:
            node.apply_move(float(parts[1]), float(parts[2]))

    node.set_pose(target.x, target.y, target.yaw)
    print(f"Pose updated: x={node.pose.x:.2f} y={node.pose.y:.2f} yaw={node.pose.yaw:.1f}")


def main() -> None:
    rclpy.init()
    node = TestPidNode()

    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    print("test_pid_node started. Type help for commands.")

    try:
        while rclpy.ok():
            try:
                line = input("pid> ").strip()
            except EOFError:
                break
            except KeyboardInterrupt:
                print("")
                break

            if not line:
                continue

            lower = line.lower()

            if lower == "help":
                print_help()
                continue

            if lower == "exit":
                break

            if lower == "pose":
                print(
                    f"Pose: x={node.pose.x:.2f} y={node.pose.y:.2f} yaw={node.pose.yaw:.1f}"
                )
                continue

            if lower == "status":
                node.send_tuning_command("STATUS")
                continue

            if lower == "stop":
                node.send_tuning_command("STOP")
                continue

            if lower == "home":
                execute_coordinate_target(node, Pose2D(0.0, 0.0, 0.0))
                continue

            setpose_values = parse_setpose(line)
            if setpose_values is not None:
                x, y, yaw = setpose_values
                node.set_pose(x, y, yaw)
                print(f"Pose set: x={node.pose.x:.2f} y={node.pose.y:.2f} yaw={node.pose.yaw:.1f}")
                continue

            xyz = parse_three_floats(line)
            if xyz is not None:
                tx, ty, tyaw = xyz
                execute_coordinate_target(node, Pose2D(tx, ty, tyaw))
                continue

            if command_is_motion(line):
                ok = node.send_motion_command(line)
                if ok:
                    parts = line.split()
                    if line.upper().startswith("ROTATE") and len(parts) >= 2:
                        node.apply_rotate(float(parts[1]))
                    elif line.upper().startswith("MOVE") and len(parts) >= 3:
                        node.apply_move(float(parts[1]), float(parts[2]))
                        if len(parts) >= 3:
                            node.apply_rotate(float(parts[2]))
                continue

            if command_is_tuning_or_control(line):
                node.send_tuning_command(line)
                continue

            print("Unknown command. Type help.")

    finally:
        node.destroy_node()
        rclpy.shutdown()
        spinner.join(timeout=1.0)


if __name__ == "__main__":
    main()
