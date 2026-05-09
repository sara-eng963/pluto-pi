#!/usr/bin/env python3

import time
from typing import List

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TestSequenceNode(Node):
    """
    Sends a fixed drive command sequence to the ESP32 through micro-ROS.

    Publishes:
        /drive_cmd      std_msgs/String

    Subscribes:
        /drive_status   std_msgs/String

    This node is Stage 2:
    - no fruit coordinates
    - no path planning
    - no GUI
    - only fixed command sequencing
    """

    def __init__(self):
        super().__init__("test_sequence_node")

        self.cmd_pub = self.create_publisher(String, "/drive_cmd", 10)

        self.status_sub = self.create_subscription(
            String,
            "/drive_status",
            self.status_callback,
            10,
        )

        self.last_status = ""

        self.ack_received = False
        self.done_received = False
        self.fault_received = False

        self.expected_done_keyword = ""

        self.get_logger().info("Test sequence node started.")
        self.get_logger().info("Publishing to /drive_cmd")
        self.get_logger().info("Listening on /drive_status")

    def status_callback(self, msg: String):
        """
        Called whenever ESP publishes on /drive_status.
        """
        text = msg.data.strip()
        self.last_status = text

        self.get_logger().info(f"ESP: {text}")

        if text.startswith("ACK"):
            self.ack_received = True

        elif text.startswith("DONE"):
            if self.expected_done_keyword:
                if self.expected_done_keyword in text:
                    self.done_received = True
            else:
                self.done_received = True

        elif text.startswith("FAULT") or text.startswith("ERR"):
            self.fault_received = True

    def publish_command(self, command: str):
        """
        Publishes one text command to the ESP.
        """
        msg = String()
        msg.data = command.strip()

        self.cmd_pub.publish(msg)
        self.get_logger().info(f"SEND: {msg.data}")

    def is_motion_command(self, command: str) -> bool:
        """
        MOVE and ROTATE are motion commands.
        They should produce DONE MOVE or DONE ROTATE.
        """
        upper = command.strip().upper()
        return upper.startswith("MOVE") or upper.startswith("ROTATE")

    def expected_done_from_command(self, command: str) -> str:
        """
        Determines what DONE message should be expected.
        """
        upper = command.strip().upper()

        if upper.startswith("MOVE"):
            return "MOVE"

        if upper.startswith("ROTATE"):
            return "ROTATE"

        return ""

    def reset_wait_flags(self):
        """
        Clears old command result flags before sending a new command.
        """
        self.ack_received = False
        self.done_received = False
        self.fault_received = False
        self.last_status = ""

    def send_stop(self):
        """
        Emergency stop command.
        """
        self.get_logger().warn("Sending STOP.")
        self.publish_command("STOP")

    def wait_for_ack(self, timeout_sec: float) -> bool:
        """
        Waits until ESP replies with ACK, ERR, or timeout.
        """
        start_time = time.time()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

            if self.ack_received:
                return True

            if self.fault_received:
                self.get_logger().error(f"ESP rejected command: {self.last_status}")
                return False

            if time.time() - start_time > timeout_sec:
                self.get_logger().error("Timeout waiting for ACK.")
                return False

        return False

    def wait_for_done(self, timeout_sec: float) -> bool:
        """
        Waits until ESP replies with DONE, FAULT, ERR, or timeout.
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
        ack_timeout_sec: float = 2.0,
        motion_timeout_sec: float = 20.0,
    ) -> bool:
        """
        Sends one command and waits for the correct response.

        For motion commands:
            wait for ACK
            then wait for DONE

        For non-motion commands:
            wait briefly for ACK/response only
        """
        command = command.strip()

        if not command:
            return True

        self.reset_wait_flags()
        self.expected_done_keyword = self.expected_done_from_command(command)

        self.publish_command(command)

        got_ack = self.wait_for_ack(timeout_sec=ack_timeout_sec)

        if not got_ack:
            self.get_logger().error(f"No valid ACK for command: {command}")
            return False

        if not self.is_motion_command(command):
            return True

        got_done = self.wait_for_done(timeout_sec=motion_timeout_sec)

        if not got_done:
            self.get_logger().error(f"Command did not finish: {command}")
            return False

        self.get_logger().info(f"Command completed: {command}")
        return True

    def run_sequence(self, commands: List[str]) -> bool:
        """
        Runs a list of commands one by one.
        """
        self.get_logger().info("Starting command sequence.")

        for index, command in enumerate(commands, start=1):
            self.get_logger().info(f"Step {index}/{len(commands)}")

            ok = self.send_command_and_wait(command)

            if not ok:
                self.get_logger().error("Sequence aborted.")
                self.send_stop()
                return False

            time.sleep(0.2)

        self.get_logger().info("Sequence finished successfully.")
        return True


def main(args=None):
    rclpy.init(args=args)

    node = TestSequenceNode()

    commands = [
        "STATUS",
        "STOP",
        "ROTATE 0",
        "MOVE 0.30 0",
        "ROTATE 90",
        "MOVE 0.30 90",
        "ROTATE 0",
    ]

    time.sleep(2.0)

    try:
        node.run_sequence(commands)

    except KeyboardInterrupt:
        node.get_logger().warn("Keyboard interrupt. Sending STOP.")
        node.send_stop()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()