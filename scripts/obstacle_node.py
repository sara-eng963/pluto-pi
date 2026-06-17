#!/usr/bin/env python3

import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String


class ObstacleNode(Node):
    def __init__(self):
        super().__init__("obstacle_node")

        self.STATIC_CONFIRM_TIME = 4.0  # seconds

        self.state = "CLEAR"
        self.latest_mask = 0
        self.previous_mask = None
        self.obstacle_start_time = None
        self.current_traffic = None
        self.obstacle_ignore_active = False

        self.obstacle_sub = self.create_subscription(
            Int32,
            "/obstacle_status",
            self.obstacle_callback,
            10,
        )
        self.reset_sub = self.create_subscription(
            Bool,
            "/mission_reset_obstacle",
            self.reset_callback,
            10,
        )
        self.ignore_sub = self.create_subscription(
            Bool,
            "/navigation/obstacle_ignore",
            self.obstacle_ignore_callback,
            10,
        )

        self.obstacle_event_pub = self.create_publisher(
            String,
            "/obstacle_event",
            10,
        )
        self.esp2_traffic_pub = self.create_publisher(
            String,
            "/esp2/traffic_cmd",
            10,
        )

        self.get_logger().info("Obstacle node started.")
        self.get_logger().info("Obstacle logic active.")
        self.get_logger().info("Initial state: CLEAR")
        self.get_logger().info("Type 'c' then ENTER to reset obstacle state.")

        self.input_thread = threading.Thread(
            target=self.terminal_input_loop,
            daemon=True,
        )
        self.input_thread.start()

    def terminal_input_loop(self):
        while rclpy.ok():
            try:
                user_input = input().strip().lower()
            except EOFError:
                break

            if user_input == "c":
                self.get_logger().info("TERMINAL: c pressed")
                self.reset_obstacle_state()

    def reset_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info("RX: /mission_reset_obstacle = true")
            self.reset_obstacle_state()

    def obstacle_ignore_callback(self, msg: Bool):
        self.obstacle_ignore_active = bool(msg.data)

    def reset_obstacle_state(self):
        self.state = "CLEAR"
        self.obstacle_start_time = None
        self.previous_mask = self.latest_mask
        self._set_traffic("Y")

        self.get_logger().info("EVENT: OBSTACLE_STATE_RESET")
        self.get_logger().info("STATE: CLEAR")

    def _publish_obstacle_event(self, event: str):
        msg = String()
        msg.data = event
        self.obstacle_event_pub.publish(msg)
        self.get_logger().info(f"PUB /obstacle_event: {event}")

    def _set_traffic(self, signal: str):
        if self.current_traffic == signal:
            return

        self.current_traffic = signal

        msg = String()
        msg.data = signal
        self.esp2_traffic_pub.publish(msg)
        self.get_logger().info(f"PUB /esp2/traffic_cmd: {signal}")

    def obstacle_callback(self, msg: Int32):
        latest_mask = msg.data
        self.latest_mask = latest_mask

        if self.obstacle_ignore_active and self.latest_mask != 0:
            if self.state != "CLEAR":
                self.state = "CLEAR"
                self.obstacle_start_time = None
                self.previous_mask = 0
                self._set_traffic("Y")
                self.get_logger().info("STATE: CLEAR")

            self.get_logger().info(
                f"Ignoring obstacle mask={self.latest_mask} because navigation obstacle ignore is active."
            )
            return

        now = time.monotonic()

        # ---------------------------------------------------------------------
        # First received message
        # ---------------------------------------------------------------------
        if self.previous_mask is None:
            self.previous_mask = self.latest_mask

            if self.latest_mask == 0:
                self.get_logger().info("STATE: CLEAR")
            else:
                self.get_logger().info(
                    f"EVENT: OBSTACLE_DETECTED mask={self.latest_mask} "
                    f"({self.mask_to_text(self.latest_mask)})"
                )
                self.state = "WAITING_FOR_CLEAR"
                self.obstacle_start_time = now
                self.get_logger().info("STATE: WAITING_FOR_CLEAR")
                self._publish_obstacle_event(f"OBSTACLE_DETECTED mask={self.latest_mask}")
                self._set_traffic("R")

            return

        # ---------------------------------------------------------------------
        # STATE: CLEAR
        # ---------------------------------------------------------------------
        if self.state == "CLEAR":
            if self.latest_mask != 0:
                self.get_logger().info(
                    f"EVENT: OBSTACLE_DETECTED mask={self.latest_mask} "
                    f"({self.mask_to_text(self.latest_mask)})"
                )
                self.state = "WAITING_FOR_CLEAR"
                self.obstacle_start_time = now
                self.get_logger().info("STATE: WAITING_FOR_CLEAR")
                self._publish_obstacle_event(f"OBSTACLE_DETECTED mask={self.latest_mask}")
                self._set_traffic("R")

        # ---------------------------------------------------------------------
        # STATE: WAITING_FOR_CLEAR
        # ---------------------------------------------------------------------
        elif self.state == "WAITING_FOR_CLEAR":
            if self.latest_mask == 0:
                self.get_logger().info("EVENT: DYNAMIC_OBSTACLE_CLEARED")
                self.state = "CLEAR"
                self.obstacle_start_time = None
                self.get_logger().info("STATE: CLEAR")
                self._publish_obstacle_event("DYNAMIC_OBSTACLE_CLEARED")
                self._set_traffic("Y")

            else:
                self._set_traffic("R")
                elapsed = now - self.obstacle_start_time

                if elapsed >= self.STATIC_CONFIRM_TIME:
                    self.get_logger().info(
                        f"EVENT: STATIC_OBSTACLE mask={self.latest_mask} "
                        f"({self.mask_to_text(self.latest_mask)})"
                    )
                    self.state = "STATIC_LOCKED"
                    self.get_logger().info("STATE: STATIC_LOCKED")
                    self._publish_obstacle_event(f"STATIC_OBSTACLE mask={self.latest_mask}")
                    self._set_traffic("R")

        # ---------------------------------------------------------------------
        # STATE: STATIC_LOCKED
        # ---------------------------------------------------------------------
        elif self.state == "STATIC_LOCKED":
            # Intentionally do nothing.
            # After static classification, clearing the sensor should not reset
            # the state because the robot may be turning away to reroute.
            self._set_traffic("R")

        self.previous_mask = self.latest_mask

    def mask_to_text(self, mask: int) -> str:
        if mask == 0:
            return "CLEAR"
        elif mask == 1:
            return "LEFT"
        elif mask == 2:
            return "FRONT"
        elif mask == 3:
            return "LEFT + FRONT"
        elif mask == 4:
            return "RIGHT"
        elif mask == 5:
            return "LEFT + RIGHT"
        elif mask == 6:
            return "FRONT + RIGHT"
        elif mask == 7:
            return "LEFT + FRONT + RIGHT"
        else:
            return "INVALID MASK"


def main(args=None):
    rclpy.init(args=args)

    node = ObstacleNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn("Obstacle node stopped by keyboard interrupt.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
