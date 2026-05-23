#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class ObstacleDetectionTest(Node):
    def __init__(self):
        super().__init__("obstacle_detection_test")

        self.previous_mask = None
        self.latest_mask = 0

        self.obstacle_sub = self.create_subscription(
            Int32,
            "/obstacle_status",
            self.obstacle_callback,
            10,
        )

        self.get_logger().info("Level 3 obstacle detection test started.")
        self.get_logger().info("Detecting OBSTACLE_DETECTED and OBSTACLE_CLEARED events...")

    def obstacle_callback(self, msg: Int32):
        self.latest_mask = msg.data

        # First message initialization
        if self.previous_mask is None:
            self.previous_mask = self.latest_mask

            if self.latest_mask == 0:
                self.get_logger().info("INITIAL STATE: CLEAR")
            else:
                self.get_logger().info(
                    f"EVENT: OBSTACLE_DETECTED mask={self.latest_mask} "
                    f"({self.mask_to_text(self.latest_mask)})"
                )
            return

        previous_blocked = (self.previous_mask != 0)
        current_blocked = (self.latest_mask != 0)

        # 0 -> nonzero
        if (not previous_blocked) and current_blocked:
            self.get_logger().info(
                f"EVENT: OBSTACLE_DETECTED mask={self.latest_mask} "
                f"({self.mask_to_text(self.latest_mask)})"
            )

        # nonzero -> 0
        elif previous_blocked and (not current_blocked):
            self.get_logger().info("EVENT: OBSTACLE_CLEARED")

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

    node = ObstacleDetectionTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn("Obstacle detection test stopped.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()