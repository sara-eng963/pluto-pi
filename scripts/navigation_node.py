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
        - Updates current pose incrementally as commands complete
        - Applies a final snap correction to the requested target on success

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

import json
import time
import queue
import threading
from dataclasses import dataclass
from typing import List, Union

import rclpy
from geometry_msgs.msg import Pose2D as RosPose2D
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import String


STATIC_AVOIDANCE_DISTANCE = 0.50
FRUIT_EXIT_BACKUP_DISTANCE = 0.25
TARGET_APPROACH_IGNORE_TOLERANCE = 0.50
ESP_ACK_TIMEOUT_SEC = 15.0
ESP_MOTION_TIMEOUT_SEC = 60.0
ESP_STATUS_TIMEOUT_SEC = 5.0
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
        self.obstacle_ignore_pub = self.create_publisher(
            Bool,
            "/navigation/obstacle_ignore",
            10,
        )
        self.navigation_result_pub = self.create_publisher(
            String,
            "/navigation_result",
            10,
        )
        self.navigation_pose_pub = self.create_publisher(
            RosPose2D,
            "/navigation/pose",
            10,
        )
        self.navigation_status_pub = self.create_publisher(
            String,
            "/navigation/status",
            10,
        )
        self.debug_yaw_pub = self.create_publisher(
            Float32,
            "/debug_yaw",
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
        self.navigation_goal_sub = self.create_subscription(
            RosPose2D,
            "/navigation/goal",
            self.navigation_goal_callback,
            10,
        )
        self.navigation_control_sub = self.create_subscription(
            String,
            "/navigation/control",
            self.navigation_control_callback,
            10,
        )
        self.mission_state_sub = self.create_subscription(
            String,
            "/mission/state",
            self.mission_state_callback,
            10,
        )

        # Logical robot pose stored by the Pi.
        #
        # This is updated incrementally as commands complete.
        self.current_pose = current_pose
        self.goal_queue = queue.Queue()

        # If dx or dy is smaller than this, skip that movement.
        self.position_tolerance = position_tolerance

        # Latest ESP message.
        self.last_status = ""
        self.latest_mission_state = ""
        self.fruit_exit_backup_pending = False
        self.fruit_exit_backup_active = False

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
        self.real_yaw_from_esp = None

        # ---- Obstacle handling flags ----------------------------------------
        self.obstacle_active = False
        self.waiting_dynamic_clear = False
        self.static_blocked = False
        self.interrupt_requested = False

        self.current_executing_command = ""
        self.current_move_start_pose = None
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
        self.obstacle_ignore_active = False

        self.active_target_pose = None
        self.static_avoidance_count = 0
        self.next_axis_order = "XY"
        # ---------------------------------------------------------------------

        # Subscriber for obstacle events from mission_node.
        self.obstacle_event_sub = self.create_subscription(
            String,
            "/obstacle_event",
            self.obstacle_event_callback,
            10,
        )

        # ---- Manual / autonomous mode ---------------------------------------
        self.robot_mode = "autonomous"   # "autonomous" | "manual"
        self.manual_mode_interrupt = False

        self.robot_mode_sub = self.create_subscription(
            String,
            "/robot/mode",
            self.robot_mode_callback,
            10,
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_vel_callback,
            10,
        )
        # ---------------------------------------------------------------------

        self.debug_yaw_timer = self.create_timer(
            1.0,
            self.request_debug_yaw_status,
        )
        self.set_obstacle_ignore(False, force=True)

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

        if text.startswith("STATUS"):
            self.last_status_text = text

            yaw = self.extract_status_float(text, "yaw")
            if yaw is not None:
                self.real_yaw_from_esp = yaw

                yaw_msg = Float32()
                yaw_msg.data = float(yaw)
                self.debug_yaw_pub.publish(yaw_msg)

            return

        self.get_logger().info(f"ESP: {text}")

        # Command accepted.
        if text.startswith("ACK"):
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

        if self.fruit_exit_backup_active:
            self.get_logger().info(
                f"Ignoring obstacle event during fruit exit backup: {event}"
            )
            return

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

        if event.startswith("DYNAMIC_OBSTACLE_CLEARED"):
            if not self.obstacle_active:
                self.get_logger().info(
                    "Ignoring DYNAMIC_OBSTACLE_CLEARED because no MOVE obstacle is active."
                )
                return

            self.get_logger().info("OBSTACLE EVENT: DYNAMIC_OBSTACLE_CLEARED")
            self.obstacle_active = False
            self.waiting_dynamic_clear = False
            self.static_blocked = False
            # Resume is handled by the execute_navigation_to loop.
            return

        if not self.current_command_is_move():
            self.get_logger().info(
                f"Ignoring obstacle event during non-MOVE command: {event}"
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

    def navigation_goal_callback(self, msg: RosPose2D):
        """
        Queue ROS navigation goals for execution by the main loop.
        """

        target_pose = Pose2D(x=float(msg.x), y=float(msg.y), yaw=float(msg.theta))
        self.goal_queue.put(target_pose)
        self.publish_navigation_status("GOAL_RECEIVED", target_pose=target_pose)

    def navigation_control_callback(self, msg: String):
        """
        Handle out-of-band navigation control commands.
        """

        command = msg.data.strip().upper()
        if command == "STOP":
            self.send_stop()
            self.publish_navigation_result("NAV_STOPPED")
            self.publish_navigation_status("STOPPED")
        elif command:
            self.get_logger().warn(f"Ignoring unknown navigation control: {command}")

    def mission_state_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.latest_mission_state = ""
            return

        self.latest_mission_state = data.get("mission_state", "")

    def robot_mode_callback(self, msg: String):
        """
        Handle /robot/mode changes published by mission_node.

        On "manual":
            - Set manual_mode_interrupt so all blocking wait loops exit immediately.
            - Send STOP to ESP so the robot halts.
            - Drain any queued autonomous goals.

        On "autonomous":
            - Clear manual_mode_interrupt so the navigation loop can run again.
        """
        mode = msg.data.strip().lower()
        if mode not in ("manual", "autonomous"):
            return

        if self.robot_mode == mode:
            return

        self.robot_mode = mode
        self.get_logger().warn(f"Navigation: robot mode → {mode}")

        if mode == "manual":
            self.manual_mode_interrupt = True
            self.send_stop()
            self.navigation_active = False
            self.command_active = False
            # Drain all queued autonomous goals.
            while not self.goal_queue.empty():
                try:
                    self.goal_queue.get_nowait()
                except Exception:
                    break
            self.get_logger().info("Manual mode active. Awaiting /cmd_vel commands.")
        else:
            self.manual_mode_interrupt = False
            self.get_logger().info("Autonomous mode active. Ready for navigation goals.")

    def cmd_vel_callback(self, msg: Twist):
        """
        Forward GUI joystick commands to ESP1 as MANUAL vx vy wz.

        Only active in manual mode. Each component is reduced to a sign:
            +1  (positive)
             0  (zero)
            -1  (negative)

        ESP1 expected format:
            MANUAL vx vy wz
        e.g.  MANUAL 1 0 0   MANUAL 0 0 -1   MANUAL 0 0 0
        """
        if self.robot_mode != "manual":
            return

        def _sign(v: float) -> int:
            if v > 0.0:
                return 1
            if v < 0.0:
                return -1
            return 0

        vx = _sign(msg.linear.x)
        vy = _sign(msg.linear.y)
        wz = _sign(msg.angular.z)

        command = f"MANUAL {vx} {vy} {wz}"
        drive_msg = String()
        drive_msg.data = command
        self.cmd_pub.publish(drive_msg)
        self.get_logger().info(f"MANUAL CMD: {command}")



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
        self.publish_navigation_status("COMMAND_EXECUTING", command=command)

    def set_obstacle_ignore(self, active: bool, force: bool = False):
        active = bool(active)
        if not force and self.obstacle_ignore_active == active:
            return

        self.obstacle_ignore_active = active

        msg = Bool()
        msg.data = active
        self.obstacle_ignore_pub.publish(msg)
        self.get_logger().info(f"PUB /navigation/obstacle_ignore: {active}")

    def refresh_obstacle_ignore(self):
        self.set_obstacle_ignore(self.should_ignore_obstacles_now())

    def should_ignore_obstacles_now(self) -> bool:
        if self.latest_mission_state in ("visionChecking", "storing"):
            return True

        if self.latest_mission_state not in (
            "headingToFruit",
            "headingToCustomer",
            "visionFailedReturning",
        ):
            return False

        if not (
            self.navigation_active
            and self.command_active
            and self.current_command_is_move()
            and self.active_target_pose is not None
            and self.current_move_start_pose is not None
        ):
            return False

        return self.current_move_ends_at_active_target()

    def current_move_ends_at_active_target(self) -> bool:
        parts = self.current_executing_command.strip().split()
        if len(parts) < 3 or parts[0].upper() != "MOVE":
            return False

        try:
            distance = float(parts[1])
            heading = float(parts[2])
        except ValueError:
            return False

        start = self.current_move_start_pose
        heading = self.normalize_yaw_deg(heading)
        end_x = start.x
        end_y = start.y

        if abs(heading - 0.0) < 1e-3:
            end_x += distance
        elif abs(abs(heading) - 180.0) < 1e-3:
            end_x -= distance
        elif abs(heading - 90.0) < 1e-3:
            end_y += distance
        elif abs(heading + 90.0) < 1e-3:
            end_y -= distance
        else:
            return False

        return (
            abs(end_x - self.active_target_pose.x) <= TARGET_APPROACH_IGNORE_TOLERANCE
            and abs(end_y - self.active_target_pose.y) <= TARGET_APPROACH_IGNORE_TOLERANCE
        )

    def request_debug_yaw_status(self):
        """
        Request ESP STATUS once per second for debug yaw publishing.

        Do not request during active MOVE/ROTATE commands because STATUS replies
        can interfere with ACK/DONE waiting logic.
        """

        if self.command_active:
            return

        msg = String()
        msg.data = "STATUS"
        self.cmd_pub.publish(msg)

    def extract_status_float(self, status_text: str, key: str):
        prefix = key + "="
        for part in status_text.split():
            if part.startswith(prefix):
                try:
                    return float(part[len(prefix):])
                except ValueError:
                    return None
        return None

    def update_pose_after_successful_command(self, command: str, context: str):
        """
        Incrementally update logical pose after a successfully completed command.

        context is a short label for logging, e.g. "normal", "dynamic-resume".
        """

        parts = command.strip().split()
        if not parts:
            return

        kind = parts[0].upper()

        if kind == "MOVE" and len(parts) >= 3:
            try:
                distance = float(parts[1])
                heading = float(parts[2])
            except ValueError:
                self.get_logger().warn(f"Cannot parse MOVE for pose update: {command}")
                return

            self.apply_move_to_logical_pose(distance, heading)
            self.get_logger().info(
                f"Pose updated after {context} MOVE: "
                f"distance={distance:.2f}, heading={self.normalize_yaw_deg(heading):.0f}"
            )
            return

        if kind == "ROTATE" and len(parts) >= 2:
            try:
                heading = float(parts[1])
            except ValueError:
                self.get_logger().warn(
                    f"Cannot parse ROTATE for yaw update: {command}"
                )
                return

            self.current_pose.yaw = self.normalize_yaw_deg(heading)
            self.get_logger().info(
                f"Pose updated after {context} ROTATE: "
                f"yaw={self.current_pose.yaw:.0f}"
            )
            self.log_current_pose()
            return

    def current_command_is_move(self) -> bool:
        """
        True when the current executing command is MOVE.
        """

        return self.current_executing_command.strip().upper().startswith("MOVE")

    def send_stop(self):
        """
        Send STOP to ESP.
        """

        self.set_obstacle_ignore(False, force=True)
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

    def publish_navigation_result(self, event: str, target_pose=None):
        msg = String()

        if target_pose is None:
            msg.data = event
        else:
            msg.data = (
                f"{event} "
                f"x={target_pose.x:.3f} "
                f"y={target_pose.y:.3f} "
                f"yaw={target_pose.yaw:.1f}"
            )

        self.navigation_result_pub.publish(msg)
        self.get_logger().info(f"PUB /navigation_result: {msg.data}")

    def publish_current_pose(self):
        msg = RosPose2D()
        msg.x = float(self.current_pose.x)
        msg.y = float(self.current_pose.y)
        msg.theta = float(self.current_pose.yaw)
        self.navigation_pose_pub.publish(msg)

    def publish_navigation_status(
        self,
        status: str,
        target_pose=None,
        command: str = None,
    ):
        payload = {
            "status": status,
            "pose": {
                "x": self.current_pose.x,
                "y": self.current_pose.y,
                "yaw": self.current_pose.yaw,
            },
        }

        if target_pose is not None:
            payload["goal"] = {
                "x": target_pose.x,
                "y": target_pose.y,
                "yaw": target_pose.yaw,
            }

        if command is not None:
            payload["command"] = command

        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self.navigation_status_pub.publish(msg)

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

    def wait_for_ack(self, command: str, timeout_sec: float) -> bool:
        """
        Wait for ESP to accept a motion command.
        """

        start_time = time.time()
        last_ignore_refresh = 0.0

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            now = time.time()

            if now - last_ignore_refresh >= 0.2:
                self.refresh_obstacle_ignore()
                last_ignore_refresh = now

            if self.manual_mode_interrupt:
                self.get_logger().warn("wait_for_ack: manual mode interrupt — aborting.")
                return False

            if self.ack_received:
                return True

            if self.fault_received:
                self.get_logger().error(
                    f"ESP FAULT/ERR while waiting for ACK for {command}: {self.last_status}"
                )
                return False

            if time.time() - start_time > timeout_sec:
                self.get_logger().error(
                    f"ACK TIMEOUT for command: {command}, last_status={self.last_status}"
                )
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

    def wait_for_done(self, command: str, timeout_sec: float) -> str:
        """
        Wait for ESP to finish a MOVE or ROTATE command.

        Returns:
            "DONE"        – motion completed successfully
            "INTERRUPTED" – obstacle interrupt was requested
            "FAILED"      – fault or timeout
        """

        start_time = time.time()
        last_ignore_refresh = 0.0

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            now = time.time()

            if now - last_ignore_refresh >= 0.2:
                self.refresh_obstacle_ignore()
                last_ignore_refresh = now

            if self.manual_mode_interrupt:
                self.get_logger().warn("wait_for_done: manual mode interrupt — aborting.")
                return "FAILED"

            if self.interrupt_requested and not self.static_avoidance_active:
                return "INTERRUPTED"

            if self.done_received:
                return "DONE"

            if self.fault_received:
                self.get_logger().error(
                    f"ESP FAULT/ERR while waiting for DONE for {command}: {self.last_status}"
                )
                return "FAILED"

            if time.time() - start_time > timeout_sec:
                self.get_logger().error(
                    f"DONE TIMEOUT for command: {command}, last_status={self.last_status}"
                )
                self.send_stop()
                return "FAILED"

        return "FAILED"

    def send_command_and_wait(
        self,
        command: str,
        ack_timeout_sec: float = ESP_ACK_TIMEOUT_SEC,
        motion_timeout_sec: float = ESP_MOTION_TIMEOUT_SEC,
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

        self.get_logger().info(
            f"WAITING FOR ACK: {command}, timeout={ack_timeout_sec}s"
        )

        self.publish_command(command)
        self.refresh_obstacle_ignore()

        # STATUS / STOP / tuning commands.
        if not self.is_motion_command(command):
            self.set_obstacle_ignore(False, force=True)
            got_response = self.wait_for_any_response(timeout_sec=ack_timeout_sec)

            if not got_response:
                self.get_logger().error(f"No ESP response for command: {command}")
                self.set_obstacle_ignore(False, force=True)
                return "FAILED"

            return "DONE"

        # MOVE / ROTATE: first wait for ACK.
        got_ack = self.wait_for_ack(command, timeout_sec=ack_timeout_sec)

        if not got_ack:
            self.get_logger().error(f"No valid ACK for command: {command}")
            self.set_obstacle_ignore(False, force=True)
            return "FAILED"

        # Then wait for DONE.
        self.get_logger().info(
            f"WAITING FOR DONE: {command}, expected={self.expected_done_keyword}, "
            f"timeout={motion_timeout_sec}s"
        )
        result = self.wait_for_done(command, timeout_sec=motion_timeout_sec)

        if result == "DONE":
            self.get_logger().info(f"Command completed: {command}")
        elif result == "FAILED":
            self.get_logger().error(f"Command did not finish: {command}")

        self.set_obstacle_ignore(False, force=True)
        return result

    def execute_fruit_exit_backup(self) -> bool:
        yaw = self.normalize_yaw_deg(self.current_pose.yaw)
        command = f"MOVE {-FRUIT_EXIT_BACKUP_DISTANCE:.2f} {yaw:.0f}"

        self.get_logger().info(f"Executing fruit exit backup: {command}")

        self.current_executing_command = command
        self.current_move_start_pose = Pose2D(
            self.current_pose.x,
            self.current_pose.y,
            self.current_pose.yaw,
        )
        self.command_active = True
        self.fruit_exit_backup_active = True

        try:
            result = self.send_command_and_wait(command)

            if result == "DONE":
                self.update_pose_after_successful_command(
                    command,
                    context="fruit-exit-backup",
                )
                self.command_active = False
                self.fruit_exit_backup_pending = False
                self.get_logger().info(
                    "Fruit exit backup completed; continuing normal Manhattan navigation."
                )
                return True

            self.get_logger().error("Fruit exit backup failed. Sending STOP.")
            self.send_stop()
            self.command_active = False
            return False
        finally:
            self.fruit_exit_backup_active = False

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

    def request_move_status_now(self):
        """
        Send STATUS to ESP1 and parse the move-progress response.

        Returns:
            (target_distance, moved_distance, heading) or None if failed.
        """

        self.reset_wait_flags()
        self.last_status_text = ""
        self.publish_command("STATUS")

        if not self.wait_for_status_response(timeout_sec=ESP_STATUS_TIMEOUT_SEC):
            self.get_logger().error("Failed to get STATUS from ESP1.")
            return None

        parsed = self.parse_move_status(self.last_status_text)
        if parsed is None:
            self.get_logger().error("Move STATUS parse failed.")
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

        parsed = self.request_move_status_now()
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

            if self.manual_mode_interrupt:
                self.get_logger().warn("wait_for_dynamic_clear_or_static: manual mode interrupt — aborting.")
                return "STATIC"

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

            interrupted_axis = self.heading_axis(interrupted_heading)
            if interrupted_axis == "Y":
                self.next_axis_order = "YX"
            else:
                self.next_axis_order = "XY"

            self.get_logger().info(
                f"STATUS: dist={moved_distance:.2f} "
                f"target={target_distance:.2f} heading={interrupted_heading:.0f}"
            )
            self.get_logger().info(
                f"Updating pose by interrupted moved distance: "
                f"{moved_distance:.2f} heading {interrupted_heading:.0f}"
            )
            self.apply_move_to_logical_pose(moved_distance, interrupted_heading)
            self.get_logger().info("Pose updated after static moved-distance update.")

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
            self.get_logger().info("Pose updated after static sidestep update.")
            self.current_pose.yaw = interrupted_heading
            self.publish_current_pose()
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
                # No resume command is needed; reflect completed original MOVE.
                self.update_pose_after_successful_command(
                    command,
                    context="dynamic-resume",
                )
                self.get_logger().info(
                    "Pose updated after dynamic resume completion (no residual MOVE)."
                )
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
                # Resume completed the original interrupted MOVE.
                self.update_pose_after_successful_command(
                    command,
                    context="dynamic-resume",
                )
                self.get_logger().info(
                    "Pose updated after dynamic resume completion (resumed MOVE)."
                )
                return INTERRUPT_RESUMED
            else:
                return INTERRUPT_FAILED

        if upper.startswith("ROTATE"):
            self.get_logger().info(
                "Ignoring obstacle interrupt during ROTATE."
            )
            self.interrupt_requested = False
            self.obstacle_active = False
            self.waiting_dynamic_clear = False
            self.static_blocked = False
            return INTERRUPT_RESUMED

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

    def heading_axis(self, heading_deg: float):
        """
        Return axis label for Manhattan headings: "X", "Y", or None.
        """

        heading = self.normalize_yaw_deg(heading_deg)

        if abs(heading - 0.0) < 1e-3 or abs(abs(heading) - 180.0) < 1e-3:
            return "X"

        if abs(heading - 90.0) < 1e-3 or abs(heading + 90.0) < 1e-3:
            return "Y"

        return None

    def same_pose(self, a: Pose2D, b: Pose2D) -> bool:
        """
        Compare poses with small tolerance for target tracking.
        """

        return (
            abs(a.x - b.x) < 1e-6
            and abs(a.y - b.y) < 1e-6
            and abs(self.normalize_yaw_deg(a.yaw) - self.normalize_yaw_deg(b.yaw)) < 1e-6
        )

    def manhattan_commands(self, target_pose: Pose2D, axis_order: str = "XY") -> List[str]:
        """
        Convert target pose into Manhattan-style ROTATE/MOVE commands.

        Movement order:
            XY: X movement then Y movement
            YX: Y movement then X movement
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

        def append_x_segment():
            if abs(dx) > self.position_tolerance:
                heading = 0.0 if dx > 0.0 else 180.0
                distance = abs(dx)
                commands.append(f"ROTATE {heading:.0f}")
                commands.append(f"MOVE {distance:.2f} {heading:.0f}")

        def append_y_segment():
            if abs(dy) > self.position_tolerance:
                heading = 90.0 if dy > 0.0 else -90.0
                distance = abs(dy)
                commands.append(f"ROTATE {heading:.0f}")
                commands.append(f"MOVE {distance:.2f} {heading:.0f}")

        if axis_order == "YX":
            append_y_segment()
            append_x_segment()
        else:
            append_x_segment()
            append_y_segment()

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
            current_pose has already been updated incrementally; apply a final
            snap correction to target_pose.

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
            self.next_axis_order = "XY"

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

        if self.fruit_exit_backup_pending:
            if not self.execute_fruit_exit_backup():
                self.navigation_active = False
                self.active_target_pose = None
                self.static_avoidance_count = 0
                self.next_axis_order = "XY"
                return False

        commands = self.manhattan_commands(
            normalized_target,
            axis_order=self.next_axis_order,
        )

        self.get_logger().info("Generated command sequence:")

        for command in commands:
            self.get_logger().info(f"  {command}")

        for index, command in enumerate(commands, start=1):
            self.get_logger().info(f"Navigation step {index}/{len(commands)}")

            self.current_executing_command = command

            if command.strip().upper().startswith("MOVE"):
                self.current_move_start_pose = Pose2D(
                    self.current_pose.x,
                    self.current_pose.y,
                    self.current_pose.yaw,
                )
            else:
                self.current_move_start_pose = None

            if self.is_motion_command(command):
                self.command_active = True
                self.refresh_obstacle_ignore()

            result = self.send_command_and_wait(command)

            if result == "DONE":
                self.update_pose_after_successful_command(command, context="normal")
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

                    self.get_logger().info(
                        "Replanning to original target from updated pose."
                    )
                    self.navigation_active = False
                    self.command_active = False
                    return self.execute_navigation_to(self.active_target_pose)
                else:
                    self.get_logger().error("Navigation stopped due to obstacle.")
                    self.navigation_active = False
                    self.command_active = False
                    self.active_target_pose = None
                    self.static_avoidance_count = 0
                    self.next_axis_order = "XY"
                    return False

            elif result == "FAILED":
                self.get_logger().error("Navigation failed. Sending STOP.")
                self.send_stop()
                self.command_active = False
                self.navigation_active = False
                self.active_target_pose = None
                self.static_avoidance_count = 0
                self.next_axis_order = "XY"
                return False

            # Short controlled pause between commands.
            # This improves reliability between motion segments.
            if self.is_motion_command(command):
                time.sleep(0.5)
            else:
                time.sleep(0.2)

        # Final snap correction after all incremental updates and full success.
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
        self.next_axis_order = "XY"

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
        self.publish_current_pose()


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

def execute_navigation_request(node: NavigationNode, target_pose: Pose2D) -> bool:
    """
    Execute a queued ROS or terminal goal with the shared result lifecycle.
    """

    node.publish_navigation_result("NAV_STARTED", target_pose)
    node.publish_navigation_status("STARTED", target_pose=target_pose)
    success = node.execute_navigation_to(target_pose)

    if success:
        if node.latest_mission_state == "headingToFruit":
            node.fruit_exit_backup_pending = True
            node.get_logger().info("Fruit exit backup armed after reaching fruit.")

        node.publish_current_pose()
        node.publish_navigation_result("NAV_DONE", target_pose)
        node.publish_navigation_status("COMPLETED", target_pose=target_pose)
    else:
        node.publish_navigation_result("NAV_FAILED", target_pose)
        node.publish_navigation_status("FAILED", target_pose=target_pose)

    return success


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

    input_queue = queue.Queue()
    input_stop_event = threading.Event()

    def terminal_input_reader():
        while rclpy.ok() and not input_stop_event.is_set():
            try:
                line = input("nav> ")
            except EOFError:
                input_queue.put("__EOF__")
                break

            input_queue.put(line)

    input_thread = threading.Thread(target=terminal_input_reader, daemon=True)
    input_thread.start()

    try:
        # First check that ESP is alive.
        node.send_command_and_wait("STATUS")

        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

            try:
                queued_goal = node.goal_queue.get_nowait()
            except queue.Empty:
                queued_goal = None

            if queued_goal is not None:
                if node.robot_mode == "manual":
                    node.get_logger().warn(
                        "Discarding queued goal: robot is in manual mode."
                    )
                else:
                    execute_navigation_request(node, queued_goal)
                continue

            try:
                line = input_queue.get_nowait()
            except queue.Empty:
                continue

            if line == "__EOF__":
                node.get_logger().info(
                    "Terminal input reached EOF; continuing in ROS goal mode."
                )
                continue

            line = line.strip()

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
                node.publish_navigation_result("NAV_STOPPED")
                node.publish_navigation_status("STOPPED")
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

            success = execute_navigation_request(node, target_pose)

            if success:
                print("Navigation succeeded.")
            else:
                print("Navigation failed.")

            print()

    except KeyboardInterrupt:
        node.get_logger().warn("Keyboard interrupt detected. Sending STOP.")
        node.send_stop()

    input_stop_event.set()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
