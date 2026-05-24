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
from std_msgs.msg import Bool
from std_msgs.msg import String


STATIC_AVOIDANCE_DISTANCE = 0.50
MAX_STATIC_AVOIDANCE_ATTEMPTS = 3
INTERRUPT_RESUMED = "INTERRUPT_RESUMED"
STATIC_AVOIDANCE_DONE = "STATIC_AVOIDANCE_DONE"
INTERRUPT_FAILED = "INTERRUPT_FAILED"


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
        self.mission_reset_pub = self.create_publisher(
            Bool,
            "/mission_reset_obstacle",
            10,
        )

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

        # Latest full STATUS text from ESP1.
        self.last_status_text = ""

        # ---- Obstacle handling flags ----------------------------------------
        self.obstacle_active = False
        self.waiting_dynamic_clear = False
        self.static_blocked = False
        self.interrupt_requested = False

        self.current_executing_command = ""
        self.interrupted_command = ""

        self.remaining_move_distance = 0.0
        self.interrupted_move_heading = 0.0
        self.interrupted_move_moved_distance = 0.0
        self.interrupted_move_target_distance = 0.0
        self.latest_static_obstacle_mask = 0
        self.has_remaining_move = False
        self.remaining_move_valid = False

        # Navigation execution state.
        # Used to ignore stale obstacle events received while idle.
        self.navigation_active = False
        self.command_active = False
        self.static_avoidance_active = False

        self.active_target_pose = None
        self.static_avoidance_count = 0
        # ---------------------------------------------------------------------

        # Subscriber for obstacle events from mission_node.
        self.obstacle_event_sub = self.create_subscription(
            String,
            "/obstacle_event",
            self.obstacle_event_callback,
            10,
        )

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
            self.last_status_text = text
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

    def obstacle_event_callback(self, msg: String):
        """
        Receives obstacle events from mission_node on /obstacle_event.

        OBSTACLE_DETECTED  – sends STOP immediately, sets interrupt flag
        DYNAMIC_OBSTACLE_CLEARED – clears flags so execute loop can resume
        STATIC_OBSTACLE    – sends STOP, marks static block
        """

        event = msg.data.strip()

        if self.static_avoidance_active:
            self.get_logger().info(
                f"Ignoring obstacle event during static avoidance: {event}"
            )
            return

        if not self.navigation_active:
            self.get_logger().info(
                f"Ignoring obstacle event while navigation idle: {event}"
            )
            return

        if not self.command_active:
            self.get_logger().info(
                f"Ignoring obstacle event because no command is active: {event}"
            )
            return

        if event.startswith("OBSTACLE_DETECTED"):
            self.get_logger().warn(f"OBSTACLE EVENT: {event}")
            self.obstacle_active = True
            self.waiting_dynamic_clear = True
            self.static_blocked = False
            self.interrupt_requested = True
            self.interrupted_command = self.current_executing_command
            self.send_stop()

        elif event.startswith("DYNAMIC_OBSTACLE_CLEARED"):
            self.get_logger().info("OBSTACLE EVENT: DYNAMIC_OBSTACLE_CLEARED")
            self.obstacle_active = False
            self.waiting_dynamic_clear = False
            self.static_blocked = False
            # Resume is handled by the execute_navigation_to loop.

        elif event.startswith("STATIC_OBSTACLE"):
            parsed_mask = 0
            for part in event.split():
                if part.startswith("mask="):
                    try:
                        parsed_mask = int(part[5:])
                    except ValueError:
                        parsed_mask = 0
                    break

            self.latest_static_obstacle_mask = parsed_mask
            self.get_logger().error(f"OBSTACLE EVENT: {event}")
            self.obstacle_active = True
            self.waiting_dynamic_clear = False
            self.static_blocked = True
            self.interrupt_requested = True
            self.send_stop()

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

    def reset_mission_obstacle_state(self):
        """
        Notify mission_node to clear STATIC_LOCKED after sidestep completion.
        """

        msg = Bool()
        msg.data = True
        self.mission_reset_pub.publish(msg)
        self.get_logger().info("Published /mission_reset_obstacle = true")

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
            rclpy.spin_once(self, timeout_sec=0.01)

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
            rclpy.spin_once(self, timeout_sec=0.01)

            if self.last_status:
                return True

            if self.fault_received:
                self.get_logger().error(f"ESP error: {self.last_status}")
                return False

            if time.time() - start_time > timeout_sec:
                self.get_logger().error("Timeout waiting for ESP response.")
                return False

        return False

    def wait_for_status_response(self, timeout_sec: float) -> bool:
        """
        Wait specifically for a STATUS response from ESP.

        STOPPED, ACK, DONE, and other non-STATUS responses are ignored.
        """

        start_time = time.time()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

            if self.last_status_text.startswith("STATUS"):
                return True

            if self.fault_received:
                self.get_logger().error(
                    f"ESP fault while waiting for STATUS: {self.last_status}"
                )
                return False

            if time.time() - start_time > timeout_sec:
                self.get_logger().error("Timeout waiting for STATUS response.")
                return False

        return False

    def wait_for_done(self, timeout_sec: float) -> str:
        """
        Wait for ESP to finish a MOVE or ROTATE command.

        Returns:
            "DONE"        – motion completed successfully
            "INTERRUPTED" – obstacle interrupt was requested
            "FAILED"      – fault or timeout
        """

        start_time = time.time()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

            if self.interrupt_requested and not self.static_avoidance_active:
                return "INTERRUPTED"

            if self.done_received:
                return "DONE"

            if self.fault_received:
                self.get_logger().error(f"Motion failed: {self.last_status}")
                return "FAILED"

            if time.time() - start_time > timeout_sec:
                self.get_logger().error("Timeout waiting for DONE.")
                self.send_stop()
                return "FAILED"

        return "FAILED"

    def send_command_and_wait(
        self,
        command: str,
        ack_timeout_sec: float = 5.0,
        motion_timeout_sec: float = 30.0,
    ) -> str:
        """
        Send one ESP command and wait for the correct response.

        Non-motion commands:
            STATUS, STOP – wait for any response.

        Motion commands:
            ROTATE, MOVE – wait for ACK, then wait for DONE.

        Returns:
            "DONE"        – completed successfully
            "INTERRUPTED" – obstacle interrupt during motion
            "FAILED"      – no response, fault, or timeout
        """

        command = command.strip()

        if not command:
            return "DONE"

        self.reset_wait_flags()
        self.expected_done_keyword = self.expected_done_from_command(command)

        self.publish_command(command)

        # STATUS / STOP / tuning commands.
        if not self.is_motion_command(command):
            got_response = self.wait_for_any_response(timeout_sec=ack_timeout_sec)

            if not got_response:
                self.get_logger().error(f"No ESP response for command: {command}")
                return "FAILED"

            return "DONE"

        # MOVE / ROTATE: first wait for ACK.
        got_ack = self.wait_for_ack(timeout_sec=ack_timeout_sec)

        if not got_ack:
            self.get_logger().error(f"No valid ACK for command: {command}")
            return "FAILED"

        # Then wait for DONE.
        result = self.wait_for_done(timeout_sec=motion_timeout_sec)

        if result == "DONE":
            self.get_logger().info(f"Command completed: {command}")
        elif result == "FAILED":
            self.get_logger().error(f"Command did not finish: {command}")

        return result

    # -------------------------------------------------------------------------
    # OBSTACLE HANDLING
    # -------------------------------------------------------------------------

    def parse_move_status(self, status_text: str):
        """
        Parse ESP1 STATUS response to extract move progress.

        ESP1 actual format:
            STATUS mode=MOVE_FORWARD dist=0.45 target=1.00 ... yaw=0.0 ...

        Returns:
            (target_distance, moved_distance, heading) or None if parsing fails.
        """

        target = None
        moved = None
        heading = None
        yaw = None

        parts = status_text.split()
        for part in parts:
            if part.startswith("target="):
                try:
                    target = float(part[7:])
                except ValueError:
                    pass
            elif part.startswith("dist="):
                try:
                    moved = float(part[5:])
                except ValueError:
                    pass
            elif part.startswith("heading="):
                try:
                    heading = float(part[8:])
                except ValueError:
                    pass
            elif part.startswith("yaw="):
                try:
                    yaw = float(part[4:])
                except ValueError:
                    pass

        if heading is None:
            heading = yaw

        if target is None or moved is None or heading is None:
            self.get_logger().error(
                f"STATUS parse failed "
                f"(target={target}, moved={moved}, heading={heading}): {status_text}"
            )
            return None

        if moved < 0.0:
            moved = 0.0
        if moved > target:
            moved = target

        return target, moved, heading

    def parse_status_active_flag(self, status_text: str):
        """
        Parse STATUS text and return drive active flag.

        Returns:
            True  -> active=1
            False -> active=0
            None  -> active field missing or invalid
        """

        for part in status_text.split():
            if part.startswith("active="):
                try:
                    return int(part[7:]) != 0
                except ValueError:
                    return None

        return None

    def wait_until_drive_idle(self, timeout_sec: float = 2.0) -> bool:
        """
        Poll ESP1 STATUS until active=0.

        Used after obstacle STOP so resume commands are sent only when ESP1
        is no longer busy.
        """

        start_time = time.time()

        while rclpy.ok():
            self.reset_wait_flags()
            self.last_status_text = ""
            self.publish_command("STATUS")

            if self.wait_for_status_response(timeout_sec=1.0):
                is_active = self.parse_status_active_flag(self.last_status_text)

                if is_active is False:
                    return True

                if is_active is True:
                    time.sleep(0.05)
                else:
                    self.get_logger().warn(
                        f"STATUS missing active flag: {self.last_status_text}"
                    )
                    time.sleep(0.05)
            else:
                # Keep retrying within timeout window.
                time.sleep(0.05)

            if time.time() - start_time > timeout_sec:
                self.get_logger().error("Timeout waiting for ESP1 drive idle.")
                return False

        return False

    def request_move_status_after_stop(self):
        """
        Send STATUS to ESP1 and parse the move-progress response.

        Returns:
            (target_distance, moved_distance, heading) or None if failed.
        """

        self.reset_wait_flags()
        self.last_status_text = ""
        self.publish_command("STATUS")

        if not self.wait_for_status_response(timeout_sec=5.0):
            self.get_logger().error("Failed to get STATUS from ESP1 after STOP.")
            return None

        parsed = self.parse_move_status(self.last_status_text)
        if parsed is None:
            self.get_logger().error("Cannot resume: STATUS parse failed.")
            return None

        return parsed

    def prepare_remaining_move_after_interrupt(self, command: str):
        """
        After STOP, query STATUS and compute remaining move distance.

        Sets:
            self.has_remaining_move
            self.remaining_move_distance
            self.interrupted_move_heading
        """

        self.remaining_move_valid = False
        self.has_remaining_move = False
        self.remaining_move_distance = 0.0

        parts = command.strip().split()
        if len(parts) < 2:
            return False

        try:
            commanded_distance = float(parts[1])
        except ValueError:
            return False

        parsed = self.request_move_status_after_stop()
        if parsed is None:
            self.get_logger().error(
                "Cannot compute remaining distance: STATUS failed."
            )
            return False

        target_distance, moved_distance, status_heading = parsed
        self.remaining_move_valid = True
        self.interrupted_move_moved_distance = moved_distance
        self.interrupted_move_target_distance = target_distance

        # Use the commanded heading from the original MOVE command (not the
        # drifted IMU yaw) so the resume travels in the same direction.
        try:
            commanded_heading = float(parts[2]) if len(parts) >= 3 else status_heading
        except ValueError:
            commanded_heading = status_heading

        remaining = target_distance - moved_distance
        if remaining < 0.0:
            remaining = 0.0

        self.get_logger().info(
            f"Interrupted MOVE: commanded={target_distance:.3f} "
            f"moved={moved_distance:.3f} remaining={remaining:.3f}"
        )

        if remaining <= self.position_tolerance:
            self.has_remaining_move = False
            self.remaining_move_distance = 0.0
            return True

        self.has_remaining_move = True
        self.remaining_move_distance = remaining
        self.interrupted_move_heading = commanded_heading
        return True

    def wait_for_dynamic_clear_or_static(self) -> str:
        """
        Block until mission_node publishes DYNAMIC_OBSTACLE_CLEARED or STATIC_OBSTACLE.

        Returns:
            "CLEARED" – obstacle cleared dynamically
            "STATIC"  – obstacle confirmed as static
        """

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

            if self.static_blocked:
                return "STATIC"

            if not self.waiting_dynamic_clear and not self.obstacle_active:
                return "CLEARED"

        return "STATIC"

    def handle_interrupted_command(self, command: str) -> str:
        """
        Handle a command interrupted by an obstacle.

        STOP has already been sent from obstacle_event_callback.

        Returns:
            INTERRUPT_RESUMED       – interrupted command successfully completed
            STATIC_AVOIDANCE_DONE   – static sidestep completed, caller should replan
            INTERRUPT_FAILED        – failed to recover
        """

        upper = command.strip().upper()

        if upper.startswith("MOVE"):
            success = self.prepare_remaining_move_after_interrupt(command)
            if not success:
                self.get_logger().error(
                    "Cannot resume MOVE because remaining distance is unknown."
                )
                return INTERRUPT_FAILED

        result = self.wait_for_dynamic_clear_or_static()

        if result == "STATIC":
            if not upper.startswith("MOVE"):
                self.get_logger().error(
                    "Static obstacle detected during non-MOVE command. Staying stopped."
                )
                return INTERRUPT_FAILED

            self.get_logger().warn("Static obstacle detected during MOVE.")

            if not self.remaining_move_valid:
                self.get_logger().error(
                    "Cannot perform static avoidance: missing interrupted MOVE STATUS."
                )
                return INTERRUPT_FAILED

            if not self.wait_until_drive_idle():
                self.get_logger().error("Cannot avoid: ESP1 still busy after STOP.")
                return INTERRUPT_FAILED

            interrupted_heading = self.normalize_yaw_deg(self.interrupted_move_heading)
            moved_distance = self.interrupted_move_moved_distance
            target_distance = self.interrupted_move_target_distance

            self.get_logger().info(
                f"STATUS: dist={moved_distance:.2f} "
                f"target={target_distance:.2f} heading={interrupted_heading:.0f}"
            )
            self.get_logger().info(
                f"Updating pose by interrupted moved distance: "
                f"{moved_distance:.2f} heading {interrupted_heading:.0f}"
            )
            self.apply_move_to_logical_pose(moved_distance, interrupted_heading)

            avoid_heading, avoid_side = self.choose_static_avoidance_heading(
                interrupted_heading,
                self.latest_static_obstacle_mask,
            )
            self.get_logger().info(
                f"Static obstacle mask={self.latest_static_obstacle_mask}, "
                f"avoiding {avoid_side}, heading={avoid_heading:.0f}"
            )

            # Clear original interruption state before running explicit sidestep
            # commands; otherwise wait_for_done may immediately return INTERRUPTED.
            self.interrupt_requested = False
            self.obstacle_active = False
            self.waiting_dynamic_clear = False
            self.static_blocked = False
            self.static_avoidance_active = True

            avoidance_commands = [
                f"ROTATE {avoid_heading:.0f}",
                f"MOVE {STATIC_AVOIDANCE_DISTANCE:.2f} {avoid_heading:.0f}",
                f"ROTATE {interrupted_heading:.0f}",
            ]

            for avoid_cmd in avoidance_commands:
                self.current_executing_command = avoid_cmd
                avoid_result = self.send_command_and_wait(avoid_cmd)
                if avoid_result != "DONE":
                    self.get_logger().error(
                        f"Static avoidance command failed: {avoid_cmd}"
                    )
                    self.static_avoidance_active = False
                    self.send_stop()
                    return INTERRUPT_FAILED

            self.get_logger().info(
                f"Updating pose by sidestep: "
                f"{STATIC_AVOIDANCE_DISTANCE:.2f} heading {avoid_heading:.0f}"
            )
            self.apply_move_to_logical_pose(STATIC_AVOIDANCE_DISTANCE, avoid_heading)
            self.current_pose.yaw = interrupted_heading
            self.static_avoidance_active = False

            self.interrupt_requested = False
            self.obstacle_active = False
            self.waiting_dynamic_clear = False
            self.static_blocked = False
            self.remaining_move_valid = False
            self.has_remaining_move = False
            self.remaining_move_distance = 0.0

            self.reset_mission_obstacle_state()
            return STATIC_AVOIDANCE_DONE

        # Dynamic clear received.
        self.interrupt_requested = False
        self.obstacle_active = False
        self.waiting_dynamic_clear = False

        if not self.wait_until_drive_idle():
            self.get_logger().error("Cannot resume: ESP1 still busy after STOP.")
            return INTERRUPT_FAILED

        if upper.startswith("MOVE"):
            if not self.remaining_move_valid:
                return INTERRUPT_FAILED

            if not self.has_remaining_move:
                self.get_logger().info("Interrupted MOVE effectively complete.")
                return INTERRUPT_RESUMED

            resume_cmd = (
                f"MOVE {self.remaining_move_distance:.2f} "
                f"{self.interrupted_move_heading:.0f}"
            )
            self.get_logger().info(f"Resuming MOVE: {resume_cmd}")
            self.current_executing_command = resume_cmd
            self.command_active = True
            resume_result = self.send_command_and_wait(resume_cmd)
            self.command_active = False

            if resume_result == "DONE":
                self.has_remaining_move = False
                self.remaining_move_distance = 0.0
                return INTERRUPT_RESUMED
            else:
                return INTERRUPT_FAILED

        if upper.startswith("ROTATE"):
            self.get_logger().info(f"Resuming ROTATE: {command}")
            self.current_executing_command = command
            self.command_active = True
            resume_result = self.send_command_and_wait(command)
            self.command_active = False
            return INTERRUPT_RESUMED if resume_result == "DONE" else INTERRUPT_FAILED

        return INTERRUPT_RESUMED

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

    def apply_move_to_logical_pose(self, distance: float, heading_deg: float):
        """
        Update logical (x, y) by moving distance along Manhattan heading.
        """

        heading = self.normalize_yaw_deg(heading_deg)

        if abs(heading - 0.0) < 1e-3:
            self.current_pose.x += distance
        elif abs(abs(heading) - 180.0) < 1e-3:
            self.current_pose.x -= distance
        elif abs(heading - 90.0) < 1e-3:
            self.current_pose.y += distance
        elif abs(heading + 90.0) < 1e-3:
            self.current_pose.y -= distance
        else:
            self.get_logger().warn(
                f"Non-Manhattan heading for pose update: {heading:.2f}. "
                "Ignoring pose translation update."
            )
            return

        self.log_current_pose()

    def left_of_heading(self, heading_deg: float) -> float:
        """
        Return heading to the left (+90 deg), normalized to [-180, 180].
        """

        return self.normalize_yaw_deg(heading_deg + 90.0)

    def right_of_heading(self, heading_deg: float) -> float:
        """
        Return heading to the right (-90 deg), normalized to [-180, 180].
        """

        return self.normalize_yaw_deg(heading_deg - 90.0)

    def choose_static_avoidance_heading(self, original_heading: float, mask: int):
        """
        Choose sidestep direction from static obstacle mask.

        Decision rule:
            left blocked, right clear  -> avoid right
            right blocked, left clear  -> avoid left
            otherwise                  -> avoid left (default)
        """

        left_blocked = (mask & 1) != 0
        right_blocked = (mask & 4) != 0

        if left_blocked and not right_blocked:
            return self.right_of_heading(original_heading), "right"

        if right_blocked and not left_blocked:
            return self.left_of_heading(original_heading), "left"

        return self.left_of_heading(original_heading), "left"

    def same_pose(self, a: Pose2D, b: Pose2D) -> bool:
        """
        Compare poses with small tolerance for target tracking.
        """

        return (
            abs(a.x - b.x) < 1e-6
            and abs(a.y - b.y) < 1e-6
            and abs(self.normalize_yaw_deg(a.yaw) - self.normalize_yaw_deg(b.yaw)) < 1e-6
        )

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

        normalized_target = Pose2D(
            x=target_pose.x,
            y=target_pose.y,
            yaw=self.normalize_yaw_deg(target_pose.yaw),
        )

        if self.active_target_pose is None or not self.same_pose(
            self.active_target_pose,
            normalized_target,
        ):
            self.active_target_pose = Pose2D(
                x=normalized_target.x,
                y=normalized_target.y,
                yaw=normalized_target.yaw,
            )
            self.static_avoidance_count = 0

        # Start each navigation request from an idle obstacle state, then drain
        # stale queued callbacks while idle so old obstacle events are ignored.
        self.navigation_active = False
        self.command_active = False
        self.obstacle_active = False
        self.waiting_dynamic_clear = False
        self.static_blocked = False
        self.interrupt_requested = False

        flush_start = time.time()
        while time.time() - flush_start < 0.2 and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

        self.navigation_active = True

        commands = self.manhattan_commands(normalized_target)

        self.get_logger().info("Generated command sequence:")

        for command in commands:
            self.get_logger().info(f"  {command}")

        for index, command in enumerate(commands, start=1):
            self.get_logger().info(f"Navigation step {index}/{len(commands)}")

            self.current_executing_command = command

            if self.is_motion_command(command):
                self.command_active = True

            result = self.send_command_and_wait(command)

            if result == "DONE":
                if self.is_motion_command(command):
                    self.command_active = False
                pass  # continue to next command

            elif result == "INTERRUPTED":
                interrupt_result = self.handle_interrupted_command(command)

                if self.is_motion_command(command):
                    self.command_active = False

                if interrupt_result == INTERRUPT_RESUMED:
                    pass
                elif interrupt_result == STATIC_AVOIDANCE_DONE:
                    self.static_avoidance_count += 1

                    if self.static_avoidance_count > MAX_STATIC_AVOIDANCE_ATTEMPTS:
                        self.get_logger().error(
                            "Exceeded max static avoidance attempts. Sending STOP."
                        )
                        self.send_stop()
                        self.command_active = False
                        self.navigation_active = False
                        self.active_target_pose = None
                        self.static_avoidance_count = 0
                        return False

                    self.get_logger().info(
                        "Replanning to original target from updated pose."
                    )
                    self.navigation_active = False
                    self.command_active = False
                    return self.execute_navigation_to(self.active_target_pose)
                else:
                    self.get_logger().error("Navigation stopped due to obstacle.")
                    self.navigation_active = False
                    self.active_target_pose = None
                    self.static_avoidance_count = 0
                    return False

            elif result == "FAILED":
                self.get_logger().error("Navigation failed. Sending STOP.")
                self.send_stop()
                self.command_active = False
                self.navigation_active = False
                self.active_target_pose = None
                self.static_avoidance_count = 0
                return False

            # Short controlled pause between commands.
            # This improves reliability between motion segments.
            if self.is_motion_command(command):
                time.sleep(0.5)
            else:
                time.sleep(0.2)

        # Update logical pose only after full success.
        self.current_pose = Pose2D(
            x=normalized_target.x,
            y=normalized_target.y,
            yaw=normalized_target.yaw,
        )

        self.get_logger().info("Navigation succeeded.")
        self.log_current_pose()

        self.command_active = False
        self.navigation_active = False
        self.active_target_pose = None
        self.static_avoidance_count = 0

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