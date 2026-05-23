#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class ObstacleDetectionTest(Node):
    def __init__(self):
        super().__init__("obstacle_detection_test")

        self.subscription = self.create_subscription(
            Int32,
            "/obstacle_status",
            self.obstacle_callback,
            10,
        )

        self.get_logger().info("Obstacle detection test node started.")
        self.get_logger().info("Waiting for /obstacle_status messages...")

    def obstacle_callback(self, msg: Int32):
        mask = msg.data
        self.get_logger().info(f"Received obstacle mask: {mask}")


def main(args=None):
    rclpy.init(args=args)

    node = ObstacleDetectionTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn("Obstacle detection test stopped.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()