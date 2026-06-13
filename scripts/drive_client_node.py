#!/usr/bin/env python3

import sys
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class DriveClientNode(Node):
    """
    Manual Pi-side client for the ESP drive controller.

    Publishes:
        /drive_cmd      std_msgs/String

    Subscribes:
        /drive_status   std_msgs/String

    This node does not do navigation yet.
    It only sends raw drive commands and prints ESP responses.
    """

    def __init__(self):
        super().__init__("drive_client_node")

        self.cmd_pub = self.create_publisher(String, "/drive_cmd", 10)

        self.status_sub = self.create_subscription(
            String,
            "/drive_status",
            self.status_callback,
            10,
        )

        self.last_status = ""
        self.last_done = ""
        self.last_fault = ""
        self.last_ack = ""

        self.running = True

        self.get_logger().info("Drive client node started.")
        self.get_logger().info("Publishing commands to /drive_cmd")
        self.get_logger().info("Listening for ESP responses on /drive_status")

    def status_callback(self, msg: String):
        """
        Called automatically whenever ESP publishes a response/status message.
        """
        text = msg.data.strip()
        self.last_status = text

        if text.startswith("ACK"):
            self.last_ack = text

        elif text.startswith("DONE"):
            self.last_done = text

        elif text.startswith("FAULT") or text.startswith("ERR"):
            self.last_fault = text

        print(f"\nESP: {text}")
        print("drive> ", end="", flush=True)

    def send_command(self, command: str):
        """
        Publish one command string to ESP.
        """
        command = command.strip()

        if not command:
            return

        msg = String()
        msg.data = command

        self.cmd_pub.publish(msg)
        self.get_logger().info(f"SENT: {command}")

    def print_help(self):
        print()
        print("Available local commands:")
        print("  help             show this help")
        print("  exit             close this node")
        print()
        print("Commands sent to ESP:")
        print("  STATUS")
        print("  STOP")
        print("  ROTATE 0")
        print("  ROTATE 90")
        print("  ROTATE 180")
        print("  ROTATE -90")
        print("  MOVE 0.3 0")
        print("  MOVE 0.3 90")
        print("  RKP 25")
        print("  RMAX 100")
        print("  RTOL 6")
        print("  RINVERT")
        print("  HKP 10")
        print("  HMAX 100")
        print("  HINVERT")
        print()

    def input_loop(self):
        """
        Runs in a separate thread so ROS can keep spinning while user types.
        """
        self.print_help()

        while self.running and rclpy.ok():
            try:
                command = input("drive> ").strip()
            except EOFError:
                self.running = False
                break
            except KeyboardInterrupt:
                self.running = False
                break

            if not command:
                continue

            lower = command.lower()

            if lower in ("exit", "quit", "q"):
                self.running = False
                break

            if lower in ("help", "h", "?"):
                self.print_help()
                continue

            self.send_command(command)

        self.get_logger().info("Input loop stopped.")


def main(args=None):
    rclpy.init(args=args)

    node = DriveClientNode()

    input_thread = threading.Thread(target=node.input_loop, daemon=True)
    input_thread.start()

    try:
        while rclpy.ok() and node.running:
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        pass

    node.running = False

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()