#!/usr/bin/env python3

"""
navigation_node.py

Interactive coordinate-based Manhattan navigation node.

Purpose:
    Let the user type a target coordinate in the terminal:

        nav> 1 1 90

    Meaning:
        Go to x = 1 m
        Go to y = 1 m
        End at yaw = 90 deg

Architecture:
    Raspberry Pi:
        - Stores the robot's logical current pose: x, y, yaw
        - Converts target pose into ROTATE/MOVE commands
        - Sends commands one by one to ESP through /drive_cmd
        - Waits for ACK and DONE from /drive_status
        - Updates current pose only after full successful navigation

    ESP32:
        - Receives low-level movement commands:
            ROTATE <heading_deg>
            MOVE <distance_m> <heading_deg>
            STOP
            STATUS
        - Handles IMU, encoders, PID, motor control
        - Publishes:
            ACK ...
            DONE ...
            FAULT ...
            STATUS ...

Coordinate convention:
    0 deg    = +X
    180 deg  = -X
    90 deg   = +Y
    -90 deg  = -Y

Example:
    Current pose:
        x = 0, y = 0, yaw = 0

    User types:
        nav> 1 1 90

    Generated sequence:
        ROTATE 0
        MOVE 1.00 0
        ROTATE 90
        MOVE 1.00 90
        ROTATE 90

    After success:
        current_pose = (1, 1, 90)
"""

import time
from dataclasses import dataclass
from typing import List, Union

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# =============================================================================
# DATA STRUCTURE
# =============================================================================

@dataclass
class Pose2D:
    """
    Simple 2D pose.

    x:
        X position in meters.

    y:
        Y position in meters.

    yaw:
        Orientation in degrees.

    Important:
        This pose is the Pi's logical estimate.
        For now, we assume that if ESP says DONE MOVE, the robot reached that
        commanded distance correctly.
    """

    x: float
    y: float
    yaw: float


# =============================================================================
# NAVIGATION NODE
# =============================================================================

class NavigationNode(Node):
    """
    Interactive Manhattan navigation node.

    Publishes:
        /drive_cmd      std_msgs/msg/String

    Subscribes:
        /drive_status   std_msgs/msg/String
    """

    def __init__(self, current_pose: Pose2D, position_tolerance: float = 0.03):
        super().__init__("navigation_node")

        # ---------------------------------------------------------------------
        # ROS publisher to ESP
        # ---------------------------------------------------------------------
        # This sends commands like:
        #   ROTATE 90
        #   MOVE 0.30 90
        #   STOP
        #   STATUS
        self.cmd_pub = self.create_publisher(String, "/drive_cmd", 10)

        # ---------------------------------------------------------------------
        # ROS subscriber from ESP
        # ---------------------------------------------------------------------
        # This receives responses like:
        #   ACK ROTATE heading=90.0
        #   DONE ROTATE
        #   ACK MOVE distance=0.30 heading=90.0
        #   DONE MOVE
        #   STATUS ...
        #   FAULT TIMEOUT
        self.status_sub = self.create_subscription(
            String,
            "/drive_status",
            self.status_callback,
            10,
        )

        # Logical robot pose stored by the Pi.
        #
        # This is updated only after a full navigation sequence succeeds.
        self.current_pose = current_pose

        # If dx or dy is smaller than this, skip that movement.
        self.position_tolerance = position_tolerance

        # Latest ESP message.
        self.last_status = ""

        # Response flags used while waiting for ESP.
        self.ack_received = False
        self.done_received = False
        self.fault_received = False

        # Expected DONE type for current command.
        #
        # If command is MOVE, expect DONE MOVE.
        # If command is ROTATE, expect DONE ROTATE.
        self.expected_done_keyword = ""

        self.get_logger().info("Navigation node started.")
        self.log_current_pose()

    # -------------------------------------------------------------------------
    # CALLBACK FROM ESP
    # -------------------------------------------------------------------------

    def status_callback(self, msg: String):
        """
        Runs every time ESP publishes a message on /drive_status.

        The callback classifies incoming messages into:
            ACK
            DONE
            FAULT / ERR
            STATUS
        """

        text = msg.data.strip()
        self.last_status = text

        self.get_logger().info(f"ESP: {text}")

        # Command accepted.
        if text.startswith("ACK"):
            self.ack_received = True
            return

        # STATUS is a valid immediate response, not a motion completion.
        if text.startswith("STATUS"):
            self.ack_received = True
            return

        # STOP response may be "STOPPED" or similar.
        if text.startswith("STOP") or text.startswith("STOPPED"):
            self.ack_received = True
            return

        # Motion completion.
        if text.startswith("DONE"):
            if self.expected_done_keyword:
                if self.expected_done_keyword in text:
                    self.done_received = True
            else:
                self.done_received = True
            return

        # Error or fault.
        if text.startswith("FAULT") or text.startswith("ERR"):
            self.fault_received = True
            return

    # -------------------------------------------------------------------------
    # BASIC COMMAND FUNCTIONS
    # -------------------------------------------------------------------------

    def reset_wait_flags(self):
        """
        Clear previous command result flags.

        This prevents old ACK/DONE messages from affecting the next command.
        """

        self.last_status = ""
        self.ack_received = False
        self.done_received = False
        self.fault_received = False
        self.expected_done_keyword = ""

    def publish_command(self, command: str):
        """
        Publish one command to ESP.
        """

        command = command.strip()

        if not command:
            return

        msg = String()
        msg.data = command

        self.cmd_pub.publish(msg)
        self.get_logger().info(f"SEND: {command}")

    def send_stop(self):
        """
        Send STOP to ESP.
        """

        self.get_logger().warn("Sending STOP.")
        self.publish_command("STOP")

    def is_motion_command(self, command: str) -> bool:
        """
        MOVE and ROTATE require ACK then DONE.
        Other commands only require an immediate response.
        """

        upper = command.strip().upper()

        return upper.startswith("MOVE") or upper.startswith("ROTATE")

    def expected_done_from_command(self, command: str) -> str:
        """
        Determine expected DONE type.
        """

        upper = command.strip().upper()

        if upper.startswith("MOVE"):
            return "MOVE"

        if upper.startswith("ROTATE"):
            return "ROTATE"

        return ""

    # -------------------------------------------------------------------------
    # WAIT FUNCTIONS
    # -------------------------------------------------------------------------

    def wait_for_ack(self, timeout_sec: float) -> bool:
        """
        Wait for ESP to accept a motion command.
        """

        start_time = time.time()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

            if self.ack_received:
                return True

            if self.fault_received:
                self.get_logger().error(
                    f"ESP error while waiting for ACK: {self.last_status}"
                )
                return False

            if time.time() - start_time > timeout_sec:
                self.get_logger().error("Timeout waiting for ACK.")
                return False

        return False

    def wait_for_any_response(self, timeout_sec: float) -> bool:
        """
        Wait for any ESP response.

        Used for:
            STATUS
            STOP
            tuning commands
        """

        start_time = time.time()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

            if self.last_status:
                return True

            if self.fault_received:
                self.get_logger().error(f"ESP error: {self.last_status}")
                return False

            if time.time() - start_time > timeout_sec:
                self.get_logger().error("Timeout waiting for ESP response.")
                return False

        return False

    def wait_for_done(self, timeout_sec: float) -> bool:
        """
        Wait for ESP to finish a MOVE or ROTATE command.
        """

        start_time = time.time()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

            if self.done_received:
                return True

            if self.fault_received:
                self.get_logger().error(f"Motion failed: {self.last_status}")
                return False

            if time.time() - start_time > timeout_sec:
                self.get_logger().error("Timeout waiting for DONE.")
                self.send_stop()
                return False

        return False

    def send_command_and_wait(
        self,
        command: str,
        ack_timeout_sec: float = 5.0,
        motion_timeout_sec: float = 30.0,
    ) -> bool:
        """
        Send one ESP command and wait for the correct response.

        Non-motion commands:
            STATUS
            STOP

            Wait for any response.

        Motion commands:
            ROTATE
            MOVE

            Wait for ACK, then wait for DONE.
        """

        command = command.strip()

        if not command:
            return True

        self.reset_wait_flags()
        self.expected_done_keyword = self.expected_done_from_command(command)

        self.publish_command(command)

        # STATUS / STOP / tuning commands.
        if not self.is_motion_command(command):
            got_response = self.wait_for_any_response(timeout_sec=ack_timeout_sec)

            if not got_response:
                self.get_logger().error(f"No ESP response for command: {command}")
                return False

            return True

        # MOVE / ROTATE: first wait for ACK.
        got_ack = self.wait_for_ack(timeout_sec=ack_timeout_sec)

        if not got_ack:
            self.get_logger().error(f"No valid ACK for command: {command}")
            return False

        # Then wait for DONE.
        got_done = self.wait_for_done(timeout_sec=motion_timeout_sec)

        if not got_done:
            self.get_logger().error(f"Command did not finish: {command}")
            return False

        self.get_logger().info(f"Command completed: {command}")
        return True

    # -------------------------------------------------------------------------
    # NAVIGATION LOGIC
    # -------------------------------------------------------------------------

    @staticmethod
    def normalize_yaw_deg(yaw: float) -> float:
        """
        Normalize yaw into [-180, 180].

        Examples:
            270  -> -90
            360  -> 0
            -270 -> 90
        """

        while yaw > 180.0:
            yaw -= 360.0

        while yaw <= -180.0:
            yaw += 360.0

        return yaw

    def manhattan_commands(self, target_pose: Pose2D) -> List[str]:
        """
        Convert target pose into Manhattan-style ROTATE/MOVE commands.

        Movement order:
            1. X movement
            2. Y movement
            3. Final yaw

        Coordinate convention:
            +X  -> 0 deg
            -X  -> 180 deg
            +Y  -> 90 deg
            -Y  -> -90 deg
        """

        commands: List[str] = []

        dx = target_pose.x - self.current_pose.x
        dy = target_pose.y - self.current_pose.y

        self.get_logger().info(
            f"Planning from x={self.current_pose.x:.2f}, "
            f"y={self.current_pose.y:.2f}, "
            f"yaw={self.current_pose.yaw:.1f}"
        )

        self.get_logger().info(
            f"Target x={target_pose.x:.2f}, "
            f"y={target_pose.y:.2f}, "
            f"yaw={target_pose.yaw:.1f}"
        )

        self.get_logger().info(f"dx={dx:.2f}, dy={dy:.2f}")

        # ---------------------------------------------------------------------
        # X movement first
        # ---------------------------------------------------------------------
        if abs(dx) > self.position_tolerance:
            heading = 0.0 if dx > 0.0 else 180.0
            distance = abs(dx)

            commands.append(f"ROTATE {heading:.0f}")
            commands.append(f"MOVE {distance:.2f} {heading:.0f}")

        # ---------------------------------------------------------------------
        # Y movement second
        # ---------------------------------------------------------------------
        if abs(dy) > self.position_tolerance:
            heading = 90.0 if dy > 0.0 else -90.0
            distance = abs(dy)

            commands.append(f"ROTATE {heading:.0f}")
            commands.append(f"MOVE {distance:.2f} {heading:.0f}")

        # ---------------------------------------------------------------------
        # Final orientation
        # ---------------------------------------------------------------------
        final_yaw = self.normalize_yaw_deg(target_pose.yaw)
        commands.append(f"ROTATE {final_yaw:.0f}")

        return commands

    def execute_navigation_to(self, target_pose: Pose2D) -> bool:
        """
        Generate and execute the Manhattan command sequence.

        If successful:
            update current_pose = target_pose

        If failed:
            send STOP
            keep old current_pose
        """

        commands = self.manhattan_commands(target_pose)

        self.get_logger().info("Generated command sequence:")

        for command in commands:
            self.get_logger().info(f"  {command}")

        for index, command in enumerate(commands, start=1):
            self.get_logger().info(f"Navigation step {index}/{len(commands)}")

            ok = self.send_command_and_wait(command)

            if not ok:
                self.get_logger().error("Navigation failed. Sending STOP.")
                self.send_stop()
                return False

            # Short controlled pause between commands.
            # This improves reliability between motion segments.
            if self.is_motion_command(command):
                time.sleep(0.5)
            else:
                time.sleep(0.2)

        # Update logical pose only after full success.
        self.current_pose = Pose2D(
            x=target_pose.x,
            y=target_pose.y,
            yaw=self.normalize_yaw_deg(target_pose.yaw),
        )

        self.get_logger().info("Navigation succeeded.")
        self.log_current_pose()

        return True

    def log_current_pose(self):
        """
        Print current logical pose.
        """

        self.get_logger().info(
            f"Current logical pose: "
            f"x={self.current_pose.x:.2f}, "
            f"y={self.current_pose.y:.2f}, "
            f"yaw={self.current_pose.yaw:.1f}"
        )


# =============================================================================
# TERMINAL INPUT PARSING
# =============================================================================

def parse_target_input(line: str) -> Union[Pose2D, str, None]:
    """
    Parse user terminal input.

    Valid coordinate input:
        1 1 90
        0.5 0.2 0
        -0.3 1.2 -90

    Special commands:
        status
        stop
        pose
        home
        help
        exit
    """

    line = line.strip()

    if not line:
        return None

    lower = line.lower()

    if lower in ("exit", "quit", "q"):
        return "EXIT"

    if lower in ("help", "?"):
        return "HELP"

    if lower in ("status", "s"):
        return "STATUS"

    if lower in ("stop", "emergency", "estop"):
        return "STOP"

    if lower in ("pose", "p"):
        return "POSE"

    if lower in ("home", "h"):
        return Pose2D(0.0, 0.0, 0.0)

    parts = line.split()

    if len(parts) != 3:
        raise ValueError("Expected format: x y yaw   Example: 1 1 90")

    try:
        x = float(parts[0])
        y = float(parts[1])
        yaw = float(parts[2])
    except ValueError as exc:
        raise ValueError("x, y, yaw must be numbers. Example: 1 1 90") from exc

    return Pose2D(x=x, y=y, yaw=yaw)


def print_help():
    """
    Print interactive usage help.
    """

    print()
    print("Interactive navigation commands:")
    print()
    print("  x y yaw       navigate to target pose")
    print("                example: 1 1 90")
    print("                example: 0.3 0.3 90")
    print()
    print("  status        ask ESP for STATUS")
    print("  pose          print current logical pose")
    print("  home          navigate to 0 0 0")
    print("  stop          send STOP to ESP")
    print("  help          show this help")
    print("  exit          quit")
    print()
    print("Coordinate convention:")
    print("  0 deg    = +X")
    print("  180 deg  = -X")
    print("  90 deg   = +Y")
    print("  -90 deg  = -Y")
    print()


# =============================================================================
# MAIN
# =============================================================================

def main(args=None):
    """
    Main entry point.

    Starts an interactive terminal prompt:

        nav>

    User types target coordinates manually.
    """

    rclpy.init(args=args)

    # Initial logical pose.
    #
    # Place the robot physically at HOME before starting the node.
    #
    # HOME:
    #   x = 0
    #   y = 0
    #   yaw = 0
    current_pose = Pose2D(
        x=0.0,
        y=0.0,
        yaw=0.0,
    )

    node = NavigationNode(
        current_pose=current_pose,
        position_tolerance=0.03,
    )

    # Give ROS/micro-ROS discovery a moment.
    time.sleep(2.0)

    print_help()

    try:
        # First check that ESP is alive.
        node.send_command_and_wait("STATUS")

        while rclpy.ok():
            try:
                line = input("nav> ").strip()
            except EOFError:
                break

            if not line:
                continue

            try:
                result = parse_target_input(line)
            except ValueError as exc:
                print(f"Invalid input: {exc}")
                continue

            if result is None:
                continue

            if result == "EXIT":
                print("Exiting navigation node.")
                break

            if result == "HELP":
                print_help()
                continue

            if result == "STATUS":
                node.send_command_and_wait("STATUS")
                continue

            if result == "STOP":
                node.send_stop()
                continue

            if result == "POSE":
                node.log_current_pose()
                continue

            # At this point, result is a Pose2D target.
            target_pose = result

            print()
            print(
                f"Target received: "
                f"x={target_pose.x:.2f}, "
                f"y={target_pose.y:.2f}, "
                f"yaw={target_pose.yaw:.1f}"
            )

            success = node.execute_navigation_to(target_pose)

            if success:
                print("Navigation succeeded.")
            else:
                print("Navigation failed.")

            print()

    except KeyboardInterrupt:
        node.get_logger().warn("Keyboard interrupt detected. Sending STOP.")
        node.send_stop()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()