#!/usr/bin/env python3
"""
Interactive ESP1 tuning console for Pluto.

Default topics:
  publish motion commands: /esp1/drive_cmd      std_msgs/String
  publish tune commands:   /esp1/tune_cmd       std_msgs/String
  publish zero yaw:        /esp1/zero_yaw       std_msgs/String
  subscribe feedback:      /esp1/test_feedback  std_msgs/String

Put this file in: pluto/scripts/test_tune.py
Then chmod +x and install it in CMakeLists.txt.
"""

import shlex
import sys
import threading
from typing import Iterable

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


MENU = r"""
ESP1 TEST/TUNE CONSOLE

Motion commands:
  MOVE <meters>              example: MOVE 1
  ROTATE <deg>               example: ROTATE 90
  STOP
  STATUS
  ZERO                       zero IMU yaw

Wheel velocity PID:
  <wheel> KP <value>         example: R1 KP 1.0
  <wheel> KI <value>         example: R2 KI 0.10
  <wheel> KD <value>         example: F1 KD 0.00
  ALL KP <value>             set all wheel velocity Kp
  ALL KI <value>             set all wheel velocity Ki
  ALL KD <value>             set all wheel velocity Kd

Wheel names use ESP/code channel names:
  F1 = front right
  R1 = rear right
  R2 = rear left
  F2 = front left

Position / heading / rotate / final heading:
  POS KP <value>             example: POS KP 2.05
  POS KI <value>
  HEAD KP <value>            normal heading hold during MOVE
  HEAD KI <value>
  HEAD KD <value>
  ROT KP <value>             rotate-to-heading controller
  ROT KI <value>
  FINAL KP <value>           final heading Kp, current code default 600
  FINAL KI <value>
  FINAL KD <value>
  FINAL MAXRPM <value>       clamp final turn RPM
  TOL <deg>                  heading tolerance, example: TOL 1.1

Logger / terminal:
  p                          print live LOG telemetry lines
  e                          hide live LOG telemetry lines
  help                       print this menu
  q                          quit

Feedback expected from ESP1 on /esp1/test_feedback:
  TUNE_OK ...
  TUNE_ERR ...
  RESULT mode=... target=... actual=... error=... yaw_deg=... heading_error_deg=... fault=...
  LOG t=... mode=... dist=... yaw=... err=... rpm=... pwm=...
""".strip()


MOTION_WORDS = {"MOVE", "ROTATE", "STOP", "STATUS"}
WHEELS = {"R1", "R2", "F1", "F2", "ALL"}
WHEEL_GAINS = {"KP", "KI", "KD"}
TUNE_GROUPS = {"POS", "HEAD", "ROT", "FINAL", "TOL"}


class TestTuneNode(Node):
    def __init__(self) -> None:
        super().__init__("test_tune")

        self.declare_parameter("drive_topic", "/esp1/drive_cmd")
        self.declare_parameter("tune_topic", "/esp1/tune_cmd")
        self.declare_parameter("zero_topic", "/esp1/zero_yaw")
        self.declare_parameter("feedback_topic", "/esp1/test_feedback")

        drive_topic = self.get_parameter("drive_topic").get_parameter_value().string_value
        tune_topic = self.get_parameter("tune_topic").get_parameter_value().string_value
        zero_topic = self.get_parameter("zero_topic").get_parameter_value().string_value
        feedback_topic = self.get_parameter("feedback_topic").get_parameter_value().string_value

        self.drive_pub = self.create_publisher(String, drive_topic, 10)
        self.tune_pub = self.create_publisher(String, tune_topic, 10)
        self.zero_pub = self.create_publisher(String, zero_topic, 10)
        self.feedback_sub = self.create_subscription(String, feedback_topic, self.feedback_cb, 50)

        self.print_live_log = False
        self.running = True

        print(MENU)
        print("")
        self.get_logger().info(
            f"Publishing drive={drive_topic}, tune={tune_topic}, zero={zero_topic}; "
            f"subscribing feedback={feedback_topic}"
        )

    @staticmethod
    def _msg(data: str) -> String:
        msg = String()
        msg.data = data
        return msg

    def feedback_cb(self, msg: String) -> None:
        data = msg.data.strip()
        if data.startswith("LOG") and not self.print_live_log:
            return
        print(f"\nESP1> {data}\n> ", end="", flush=True)

    def publish_drive(self, command: str) -> None:
        self.drive_pub.publish(self._msg(command))
        print(f"sent drive: {command}")

    def publish_tune(self, command: str) -> None:
        self.tune_pub.publish(self._msg(command))
        print(f"sent tune: {command}")

    def publish_zero(self) -> None:
        self.zero_pub.publish(self._msg("ZERO"))
        print("sent zero yaw")

    @staticmethod
    def _is_float(text: str) -> bool:
        try:
            float(text)
            return True
        except ValueError:
            return False

    def handle_line(self, line: str) -> None:
        raw = line.strip()
        if not raw:
            return

        low = raw.lower()
        if low in {"q", "quit", "exit"}:
            self.running = False
            rclpy.shutdown()
            return
        if low in {"help", "h", "?"}:
            print(MENU)
            return
        if low == "p":
            self.print_live_log = True
            print("live LOG telemetry: enabled")
            return
        if low == "e":
            self.print_live_log = False
            print("live LOG telemetry: disabled")
            return
        if low in {"zero", "zero_yaw", "yaw0"}:
            self.publish_zero()
            return

        try:
            parts = shlex.split(raw)
        except ValueError as exc:
            print(f"bad command: {exc}")
            return

        if not parts:
            return

        parts_u = [p.upper() for p in parts]

        # Motion commands go to the existing drive command path.
        if parts_u[0] in MOTION_WORDS:
            self.publish_drive(" ".join(parts_u))
            return

        # Wheel tuning: R1 KP 1, ALL KI 0.1, etc.
        if len(parts_u) == 3 and parts_u[0] in WHEELS and parts_u[1] in WHEEL_GAINS and self._is_float(parts_u[2]):
            self.publish_tune(f"{parts_u[0]} {parts_u[1]} {float(parts_u[2]):.6g}")
            return

        # Controller tuning: POS KP 2.05, HEAD KD 25, ROT KI 5, FINAL MAXRPM 25.
        if parts_u[0] in TUNE_GROUPS:
            self.publish_tune(" ".join(parts_u))
            return

        print("unknown command. Type: help")

    def console_loop(self) -> None:
        while rclpy.ok() and self.running:
            try:
                line = input("> ")
            except (EOFError, KeyboardInterrupt):
                self.running = False
                rclpy.shutdown()
                return
            self.handle_line(line)


def main(argv: Iterable[str] | None = None) -> None:
    rclpy.init(args=list(argv) if argv is not None else None)
    node = TestTuneNode()

    thread = threading.Thread(target=node.console_loop, daemon=True)
    thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv[1:])
