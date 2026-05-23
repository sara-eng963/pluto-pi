#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class ObstacleDetectionTest(Node):
    def __init__(self):
        super().__init__("obstacle_detection_test")

        self.previous_mask = None
        self.latest_mask = 0
        self.message_count = 0

        self.obstacle_sub = self.create_subscription(
            Int32,
            "/obstacle_status",
            self.obstacle_callback,
            10,
        )

        self.get_logger().info("Level 2 obstacle detection test started.")
        self.get_logger().info("Printing only when mask changes...")

    def obstacle_callback(self, msg: Int32):
        self.message_count += 1
        self.latest_mask = msg.data

        if self.latest_mask == self.previous_mask:
            return

        old_mask = self.previous_mask
        self.previous_mask = self.latest_mask

        old_text = self.mask_to_text(old_mask)
        new_text = self.mask_to_text(self.latest_mask)

        self.get_logger().info(
            f"MASK CHANGE: {old_mask} ({old_text}) -> "
            f"{self.latest_mask} ({new_text})"
        )

    def mask_to_text(self, mask):
        if mask is None:
            return "NONE"

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

    node = ObstacleDetectionTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn("Obstacle detection test stopped.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()