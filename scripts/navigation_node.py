#!/usr/bin/env python3

"""
navigation_node.py

Stage 3 Pi-side navigation node.

Purpose:
    Convert a target coordinate (x, y, yaw) into Manhattan-style drive commands.

Architecture:
    Raspberry Pi:
        - Stores the robot's logical current pose: x, y, yaw
        - Receives target coordinate
        - Generates ROTATE/MOVE command sequence
        - Sends commands one by one to ESP
        - Waits for DONE before sending next command
        - Updates current pose only after successful arrival

    ESP32:
        - Receives simple commands:
            ROTATE <heading_deg>
            MOVE <distance_m> <heading_deg>
            STOP
            STATUS
        - Handles real motor control, encoders, PID, and IMU yaw
        - Publishes ACK / DONE / FAULT responses

Coordinate convention:
    0 deg    = +X direction
    180 deg  = -X direction
    90 deg   = +Y direction
    -90 deg  = -Y direction

Example:
    Current pose:
        (0, 0, 0)

    Target:
        (1, 1, 90)

    Generated commands:
        ROTATE 0
        MOVE 1.00 0
        ROTATE 90
        MOVE 1.00 90
        ROTATE 90
"""

import argparse
import time
from dataclasses import dataclass
from typing import List

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


@dataclass
class Pose2D:
    """
    Simple 2D pose.

    x:
        Position on X axis in meters.

    y:
        Position on Y axis in meters.

    yaw:
        Robot orientation in degrees.

    Important:
        This is the Pi's logical pose estimate.
        The ESP still uses the IMU internally to execute ROTATE commands.
    """

    x: float
    y: float
    yaw: float


class NavigationNode(Node):
    """
    Coordinate-based Manhattan navigation node.

    Publishes:
        /drive_cmd      std_msgs/msg/String

    Subscribes:
        /drive_status   std_msgs/msg/String

    This node converts:
        target_x, target_y, target_yaw

    Into:
        ROTATE/MOVE/ROTATE/MOVE/ROTATE command sequence.
    """

    def __init__(self, current_pose: Pose2D, position_tolerance: float = 0.03):
        super().__init__("navigation_node")

        # Publisher to ESP drive command topic.
        self.cmd_pub = self.create_publisher(String, "/drive_cmd", 10)

        # Subscriber to ESP response/status topic.
        self.status_sub = self.create_subscription(
            String,
            "/drive_status",
            self.status_callback,
            10,
        )

        # Logical current pose stored by the Pi.
        #
        # For now, this is initialized manually.
        # Later, this could come from odometry/localization.
        self.current_pose = current_pose

        # If dx or dy is smaller than this, skip that movement.
        #
        # Example:
        #   If dx = 0.01 m and tolerance = 0.03 m,
        #   do not generate a MOVE command.
        self.position_tolerance = position_tolerance

        # Latest ESP response string.
        self.last_status = ""

        # Waiting flags.
        self.ack_received = False
        self.done_received = False
        self.fault_received = False

        # Expected DONE type for current command:
        #   "MOVE"
        #   "ROTATE"
        self.expected_done_keyword = ""

        self.get_logger().info("Navigation node started.")
        self.get_logger().info(
            f"Initial pose: x={self.current_pose.x:.2f}, "
            f"y={self.current_pose.y:.2f}, yaw={self.current_pose.yaw:.1f}"
        )

    # -------------------------------------------------------------------------
    # ROS CALLBACK
    # -------------------------------------------------------------------------

    def status_callback(self, msg: String):
        """
        Called whenever ESP publishes a message on /drive_status.

        It classifies ESP messages into:
            ACK
            DONE
            FAULT / ERR
            STATUS
        """

        text = msg.data.strip()
        self.last_status = text

        self.get_logger().info(f"ESP: {text}")

        if text.startswith("ACK"):
            self.ack_received = True
            return

        if text.startswith("STATUS"):
            self.ack_received = True
            return

        if text.startswith("STOP") or text.startswith("STOPPED"):
            self.ack_received = True
            return

        if text.startswith("DONE"):
            if self.expected_done_keyword:
                if self.expected_done_keyword in text:
                    self.done_received = True
            else:
                self.done_received = True
            return

        if text.startswith("FAULT") or text.startswith("ERR"):
            self.fault_received = True
            return

    # -------------------------------------------------------------------------
    # BASIC COMMAND HANDLING
    # -------------------------------------------------------------------------

    def reset_wait_flags(self):
        """
        Reset command-response flags before sending a new command.
        """

        self.last_status = ""
        self.ack_received = False
        self.done_received = False
        self.fault_received = False
        self.expected_done_keyword = ""

    def publish_command(self, command: str):
        """
        Publish one command string to the ESP.
        """

        msg = String()
        msg.data = command.strip()

        self.cmd_pub.publish(msg)

        self.get_logger().info(f"SEND: {msg.data}")

    def send_stop(self):
        """
        Send emergency STOP to ESP.
        """

        self.get_logger().warn("Sending STOP.")
        self.publish_command("STOP")

    def is_motion_command(self, command: str) -> bool:
        """
        Check whether command requires DONE.

        MOVE and ROTATE are motion commands.
        STATUS and STOP are immediate commands.
        """

        upper = command.strip().upper()

        return upper.startswith("MOVE") or upper.startswith("ROTATE")

    def expected_done_from_command(self, command: str) -> str:
        """
        Determine which DONE response should be expected.
        """

        upper = command.strip().upper()

        if upper.startswith("MOVE"):
            return "MOVE"

        if upper.startswith("ROTATE"):
            return "ROTATE"

        return ""

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
                self.get_logger().error(f"ESP error while waiting for ACK: {self.last_status}")
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

        Non-motion command:
            STATUS
            STOP

            Wait for any response.

        Motion command:
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

        if not self.is_motion_command(command):
            return self.wait_for_any_response(timeout_sec=ack_timeout_sec)

        got_ack = self.wait_for_ack(timeout_sec=ack_timeout_sec)

        if not got_ack:
            self.get_logger().error(f"No valid ACK for command: {command}")
            return False

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
        Normalize yaw to the range [-180, 180].

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
        Convert target pose into a Manhattan-style command sequence.

        Movement order:
            1. Move along X axis
            2. Move along Y axis
            3. Rotate to final yaw

        The ESP command format is:
            ROTATE <heading_deg>
            MOVE <distance_m> <heading_deg>

        Heading convention:
            +X  -> 0 deg
            -X  -> 180 deg
            +Y  -> 90 deg
            -Y  -> -90 deg
        """

        commands: List[str] = []

        dx = target_pose.x - self.current_pose.x
        dy = target_pose.y - self.current_pose.y

        self.get_logger().info(
            f"Planning from x={self.current_pose.x:.2f}, y={self.current_pose.y:.2f}, "
            f"yaw={self.current_pose.yaw:.1f}"
        )

        self.get_logger().info(
            f"Target x={target_pose.x:.2f}, y={target_pose.y:.2f}, "
            f"yaw={target_pose.yaw:.1f}"
        )

        self.get_logger().info(f"dx={dx:.2f}, dy={dy:.2f}")

        # Move along X first.
        if abs(dx) > self.position_tolerance:
            if dx > 0.0:
                heading = 0.0
            else:
                heading = 180.0

            distance = abs(dx)

            commands.append(f"ROTATE {heading:.0f}")
            commands.append(f"MOVE {distance:.2f} {heading:.0f}")

        # Then move along Y.
        if abs(dy) > self.position_tolerance:
            if dy > 0.0:
                heading = 90.0
            else:
                heading = -90.0

            distance = abs(dy)

            commands.append(f"ROTATE {heading:.0f}")
            commands.append(f"MOVE {distance:.2f} {heading:.0f}")

        # Finally face the requested final orientation.
        final_yaw = self.normalize_yaw_deg(target_pose.yaw)

        commands.append(f"ROTATE {final_yaw:.0f}")

        return commands

    def execute_navigation_to(self, target_pose: Pose2D) -> bool:
        """
        Generate Manhattan commands and execute them one by one.

        If all commands succeed:
            update current_pose to target_pose

        If any command fails:
            send STOP
            do not update current_pose
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

            # Small controlled pause between segments.
            # This improves reliability and gives mechanics time to settle.
            if self.is_motion_command(command):
                time.sleep(0.5)
            else:
                time.sleep(0.2)

        # Only update pose after full sequence succeeds.
        self.current_pose = Pose2D(
            x=target_pose.x,
            y=target_pose.y,
            yaw=self.normalize_yaw_deg(target_pose.yaw),
        )

        self.get_logger().info(
            f"Arrived. Updated current pose: "
            f"x={self.current_pose.x:.2f}, "
            f"y={self.current_pose.y:.2f}, "
            f"yaw={self.current_pose.yaw:.1f}"
        )

        return True


def parse_args():
    """
    Parse command-line target pose.

    Usage:
        ros2 run pluto navigation_node.py -- 1 1 90

    Meaning:
        target_x = 1 m
        target_y = 1 m
        target_yaw = 90 deg

    Optional current pose:
        ros2 run pluto navigation_node.py -- 1 1 90 --current-x 0 --current-y 0 --current-yaw 0
    """

    parser = argparse.ArgumentParser(description="Coordinate-based Manhattan navigation node")

    parser.add_argument("target_x", type=float, help="Target X position in meters")
    parser.add_argument("target_y", type=float, help="Target Y position in meters")
    parser.add_argument("target_yaw", type=float, help="Target final yaw in degrees")

    parser.add_argument("--current-x", type=float, default=0.0, help="Initial/current X position")
    parser.add_argument("--current-y", type=float, default=0.0, help="Initial/current Y position")
    parser.add_argument("--current-yaw", type=float, default=0.0, help="Initial/current yaw angle")

    parser.add_argument(
        "--position-tolerance",
        type=float,
        default=0.03,
        help="Ignore dx/dy smaller than this value in meters",
    )

    args, _ = parser.parse_known_args()

    return args


def main(args=None):
    """
    Main entry point.
    """

    cli_args = parse_args()

    rclpy.init(args=args)

    current_pose = Pose2D(
        x=cli_args.current_x,
        y=cli_args.current_y,
        yaw=cli_args.current_yaw,
    )

    target_pose = Pose2D(
        x=cli_args.target_x,
        y=cli_args.target_y,
        yaw=cli_args.target_yaw,
    )

    node = NavigationNode(
        current_pose=current_pose,
        position_tolerance=cli_args.position_tolerance,
    )

    # Give micro-ROS/ROS discovery a short moment.
    time.sleep(2.0)

    try:
        node.send_command_and_wait("STATUS")
        node.execute_navigation_to(target_pose)

    except KeyboardInterrupt:
        node.get_logger().warn("Keyboard interrupt detected. Sending STOP.")
        node.send_stop()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()