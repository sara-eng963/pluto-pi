#!/usr/bin/env python3

"""
test_sequence_node.py

Stage 2 Pi-side drive test node.

Purpose:
    Send a fixed sequence of drive commands to the ESP32 through micro-ROS.

Architecture:
    Pi publishes command strings on:
        /drive_cmd

    ESP receives the command, runs its existing executeCommandLine(),
    then publishes responses on:
        /drive_status

This node proves:
    1. Pi can send commands automatically.
    2. ESP receives and executes them.
    3. Pi waits for ESP to finish before sending the next command.
    4. Faults/timeouts stop the sequence safely.

This is NOT the final navigation node yet.
This is only the automatic command sequencer.
"""

import time
from typing import List

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TestSequenceNode(Node):
    """
    ROS 2 node that sends a fixed drive command sequence to the ESP32.

    Publishes:
        /drive_cmd      std_msgs/msg/String

    Subscribes:
        /drive_status   std_msgs/msg/String

    Example command flow:
        Pi  -> "ROTATE 90"
        ESP -> "ACK ROTATE heading=90.00"
        ESP -> "DONE ROTATE"

        Pi  -> "MOVE 0.30 90"
        ESP -> "ACK MOVE distance=0.30 heading=90.00"
        ESP -> "DONE MOVE"
    """

    def __init__(self):
        super().__init__("test_sequence_node")

        # Publisher used to send text commands to the ESP.
        #
        # ESP should subscribe to this topic:
        #   /drive_cmd
        #
        # Message examples:
        #   "STATUS"
        #   "STOP"
        #   "ROTATE 90"
        #   "MOVE 0.30 90"
        self.cmd_pub = self.create_publisher(String, "/drive_cmd", 10)

        # Subscriber used to receive ESP responses.
        #
        # ESP should publish to this topic:
        #   /drive_status
        #
        # Message examples:
        #   "STATUS mode=IDLE ..."
        #   "ACK ROTATE heading=90.00"
        #   "DONE ROTATE"
        #   "FAULT TIMEOUT"
        #   "ERR BUSY"
        self.status_sub = self.create_subscription(
            String,
            "/drive_status",
            self.status_callback,
            10,
        )

        # Stores the latest message received from ESP.
        self.last_status = ""

        # Flags used while waiting for a command result.
        #
        # ack_received:
        #   True when ESP accepts a command or sends a valid immediate response.
        #
        # done_received:
        #   True when ESP finishes MOVE or ROTATE.
        #
        # fault_received:
        #   True when ESP reports FAULT or ERR.
        self.ack_received = False
        self.done_received = False
        self.fault_received = False

        # Used to avoid accepting the wrong DONE message.
        #
        # Example:
        #   If we send "MOVE 0.30 90",
        #   expected_done_keyword = "MOVE"
        #
        # Then only "DONE MOVE" is accepted.
        self.expected_done_keyword = ""

        self.get_logger().info("Test sequence node started.")
        self.get_logger().info("Publishing commands to /drive_cmd")
        self.get_logger().info("Listening for ESP responses on /drive_status")

    # -------------------------------------------------------------------------
    # ROS CALLBACK
    # -------------------------------------------------------------------------

    def status_callback(self, msg: String):
        """
        This function runs automatically whenever ESP publishes a message on
        /drive_status.

        It does three jobs:
            1. Print the ESP response.
            2. Classify the response as ACK / DONE / FAULT.
            3. Set flags so the waiting function can continue.
        """

        # Clean the received text.
        text = msg.data.strip()

        # Store latest ESP message.
        self.last_status = text

        # Print every ESP response.
        self.get_logger().info(f"ESP: {text}")

        # ACK messages mean:
        #   ESP accepted the command.
        #
        # Example:
        #   ACK ROTATE heading=90.00
        #   ACK MOVE distance=0.30 heading=90.00
        if text.startswith("ACK"):
            self.ack_received = True
            return

        # STATUS is a valid response, but your ESP returns:
        #   STATUS mode=IDLE ...
        #
        # It does NOT return:
        #   ACK STATUS
        #
        # So for STATUS command, this still counts as a valid response.
        if text.startswith("STATUS"):
            self.ack_received = True
            return

        # STOP may return different text depending on your ESP code.
        # Accept common stop responses as valid immediate responses.
        if text.startswith("STOP") or text.startswith("STOPPED"):
            self.ack_received = True
            return

        # DONE means a motion command actually finished.
        #
        # Examples:
        #   DONE ROTATE
        #   DONE MOVE
        if text.startswith("DONE"):
            # If we are expecting a specific DONE type, check it.
            if self.expected_done_keyword:
                if self.expected_done_keyword in text:
                    self.done_received = True
            else:
                # If no specific type is expected, accept any DONE.
                self.done_received = True
            return

        # FAULT / ERR means something failed.
        #
        # Examples:
        #   FAULT TIMEOUT
        #   ERR BUSY
        #   ERR UNKNOWN_COMMAND
        if text.startswith("FAULT") or text.startswith("ERR"):
            self.fault_received = True
            return

        # Any other response is still printed, but not classified.
        # This is intentional. Unknown text should not accidentally continue
        # the sequence.

    # -------------------------------------------------------------------------
    # BASIC COMMAND HELPERS
    # -------------------------------------------------------------------------

    def publish_command(self, command: str):
        """
        Publishes one command string to the ESP.

        This does not wait for anything.
        It only sends the command.
        """

        command = command.strip()

        # Empty commands are ignored.
        if not command:
            return

        msg = String()
        msg.data = command

        self.cmd_pub.publish(msg)

        self.get_logger().info(f"SEND: {command}")

    def send_stop(self):
        """
        Sends STOP to the ESP.

        Used when:
            - user interrupts the sequence
            - timeout happens
            - fault happens

        STOP should always be accepted by the ESP, even if it is busy.
        """

        self.get_logger().warn("Sending STOP.")
        self.publish_command("STOP")

    # -------------------------------------------------------------------------
    # COMMAND TYPE LOGIC
    # -------------------------------------------------------------------------

    def is_motion_command(self, command: str) -> bool:
        """
        Returns True if command is a motion command.

        Motion commands require:
            ACK first
            DONE later

        Non-motion commands only require an immediate response.

        Motion:
            MOVE
            ROTATE

        Non-motion:
            STATUS
            STOP
            RKP
            RMAX
            RTOL
            HMAX
            etc.
        """

        upper = command.strip().upper()

        return upper.startswith("MOVE") or upper.startswith("ROTATE")

    def expected_done_from_command(self, command: str) -> str:
        """
        Determines which DONE message we should wait for.

        If command is:
            MOVE 0.30 90

        We expect:
            DONE MOVE

        If command is:
            ROTATE 90

        We expect:
            DONE ROTATE
        """

        upper = command.strip().upper()

        if upper.startswith("MOVE"):
            return "MOVE"

        if upper.startswith("ROTATE"):
            return "ROTATE"

        return ""

    def reset_wait_flags(self):
        """
        Clears previous command result flags.

        This must be called before sending each new command.

        Otherwise an old DONE or ACK from the previous command could make
        the current command appear completed incorrectly.
        """

        self.last_status = ""

        self.ack_received = False
        self.done_received = False
        self.fault_received = False

        self.expected_done_keyword = ""

    # -------------------------------------------------------------------------
    # WAIT FUNCTIONS
    # -------------------------------------------------------------------------

    def wait_for_ack(self, timeout_sec: float) -> bool:
        """
        Waits for ESP to accept a command.

        Returns:
            True:
                ACK or accepted immediate response received.

            False:
                timeout happened or ESP returned FAULT/ERR.

        Used for:
            MOVE
            ROTATE
        """

        start_time = time.time()

        while rclpy.ok():
            # Let ROS process incoming /drive_status messages.
            #
            # Without this, status_callback() will not run.
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
        Waits for any response from ESP.

        Used for non-motion commands like:
            STATUS
            STOP
            RKP 25
            RMAX 100
            RTOL 6

        Reason:
            Your ESP responds to STATUS with:
                STATUS mode=IDLE ...

            not:
                ACK STATUS

        So for non-motion commands, any received response is enough.
        """

        start_time = time.time()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

            # Any message in last_status means ESP responded.
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
        Waits for ESP to finish a motion command.

        Used after MOVE or ROTATE was accepted.

        Returns:
            True:
                DONE MOVE or DONE ROTATE received.

            False:
                FAULT/ERR received or timeout happened.
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

    # -------------------------------------------------------------------------
    # MAIN COMMAND EXECUTION FUNCTION
    # -------------------------------------------------------------------------

    def send_command_and_wait(
        self,
        command: str,
        ack_timeout_sec: float = 2.0,
        motion_timeout_sec: float = 20.0,
    ) -> bool:
        """
        Sends one command and waits for the correct result.

        Behavior depends on command type.

        For non-motion commands:
            STATUS
            STOP
            RKP 25
            RMAX 100
            RTOL 6

            Steps:
                1. Send command.
                2. Wait for any ESP response.
                3. Continue.

        For motion commands:
            ROTATE 90
            MOVE 0.30 90

            Steps:
                1. Send command.
                2. Wait for ACK.
                3. Wait for DONE.
                4. Continue.

        If anything fails:
            return False
        """

        command = command.strip()

        if not command:
            return True

        # Clear old flags before sending a new command.
        self.reset_wait_flags()

        # Store expected DONE type if this is MOVE or ROTATE.
        self.expected_done_keyword = self.expected_done_from_command(command)

        # Publish the command to ESP.
        self.publish_command(command)

        # Non-motion commands do not produce DONE.
        # They only need one response.
        if not self.is_motion_command(command):
            got_response = self.wait_for_any_response(timeout_sec=ack_timeout_sec)

            if not got_response:
                self.get_logger().error(f"No ESP response for command: {command}")
                return False

            self.get_logger().info(f"Command responded: {command}")
            return True

        # Motion commands should first produce ACK.
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
    # SEQUENCE RUNNER
    # -------------------------------------------------------------------------

    def run_sequence(self, commands: List[str]) -> bool:
        """
        Runs a list of commands one by one.

        Rule:
            Never send the next motion command until the previous one is DONE.

        If any command fails:
            Send STOP.
            Abort the sequence.
        """

        self.get_logger().info("Starting command sequence.")

        for index, command in enumerate(commands, start=1):
            self.get_logger().info(f"Step {index}/{len(commands)}")

            ok = self.send_command_and_wait(command)

            if not ok:
                self.get_logger().error("Sequence aborted.")
                self.send_stop()
                return False

            # Small gap between commands.
            # This gives ESP and robot mechanics a short settling time.
            time.sleep(0.2)

        self.get_logger().info("Sequence finished successfully.")
        return True


def main(args=None):
    """
    Program entry point.

    This creates the node, defines the test command sequence,
    then runs it once.
    """

    rclpy.init(args=args)

    node = TestSequenceNode()

    # Fixed test sequence.
    #
    # Keep distances small while testing.
    #
    # This sequence means:
    #   1. Ask ESP status.
    #   2. Stop robot to guarantee clean start.
    #   3. Face 0 degrees.
    #   4. Move 30 cm along heading 0.
    #   5. Face 90 degrees.
    #   6. Move 30 cm along heading 90.
    #   7. Face 0 degrees again.
    commands = [
        "STATUS",
        "STOP",
        "ROTATE 0",
        "MOVE 0.30 0",
        "ROTATE 90",
        "MOVE 0.30 90",
        "ROTATE 0",
    ]

    # Short delay after node startup.
    #
    # This gives ROS discovery and micro-ROS bridge a moment to settle.
    time.sleep(2.0)

    try:
        node.run_sequence(commands)

    except KeyboardInterrupt:
        node.get_logger().warn("Keyboard interrupt detected.")
        node.send_stop()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()