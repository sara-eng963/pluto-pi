#!/usr/bin/env python3

"""
mission_node.py

Current version:
    Obstacle logic only.

Purpose:
    - Subscribe to /obstacle_status from ESP2
    - Detect dynamic obstacle clearing
    - Detect static obstacle locking
    - Support obstacle reset using:
        1. terminal input: c
        2. ROS topic: /mission_reset_obstacle

ESP2 publishes:
    /obstacle_status
    std_msgs/msg/Int32

Mask:
    0 = clear
    1 = left blocked
    2 = front blocked
    3 = left + front blocked
    4 = right blocked
    5 = left + right blocked
    6 = front + right blocked
    7 = left + front + right blocked

States:
    CLEAR
    WAITING_FOR_CLEAR
    STATIC_LOCKED
"""

import time
import threading
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool, String
from geometry_msgs.msg import Pose2D


class MissionNode(Node):
    def __init__(self):
        super().__init__("mission_node")

        self.STATIC_CONFIRM_TIME = 4.0  # seconds

        self.state = "CLEAR"
        self.latest_mask = 0
        self.previous_mask = None
        self.obstacle_start_time = None

        # Mission/order state
        self.mission_state = "idle"
        self.active_order_id = ""
        self.active_fruit = ""
        self.active_user_id = ""
        self.assigned_rfid = ""
        self.fault_type = "none"

        self.storage_sequence_running = False
        self.latest_valid = False
        self.waiting_for_valid = False
        self.storage_done_for_current_target = False

        self.waiting_for_rfid = False
        self.rfid_verified = False
        self.waiting_for_storage_close = False

        # Customer pose is HOME for this project
        self.customer_pose = (0.0, 0.0, 0.0)

        # Temporary hardcoded fruit poses for Stage 2
        # Later move these to YAML parameters.
        self.fruit_poses = {
            "Apple": (0.8, 0.0, 0.0),
            "Orange": (0.5, 0.5, 90.0),
            "Kiwi": (0.0, 0.5, 90.0),
        }

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
        self.esp2_gripper_pub = self.create_publisher(
            String,
            "/esp2/gripper_cmd",
            10,
        )
        self.esp2_position_pub = self.create_publisher(
            Int32,
            "/esp2/position_cmd",
            10,
        )

        self.navigation_result_sub = self.create_subscription(
            String,
            "/navigation_result",
            self.navigation_result_callback,
            10,
        )

        self.valid_sub = self.create_subscription(
            Bool,
            "/valid",
            self.valid_callback,
            10,
        )
        self.order_request_sub = self.create_subscription(
            String,
            "/mission/order_request",
            self.order_request_callback,
            10,
        )

        self.mission_control_sub = self.create_subscription(
            String,
            "/mission/control",
            self.mission_control_callback,
            10,
        )

        self.rfid_verification_sub = self.create_subscription(
            String,
            "/mission/rfid_verification",
            self.rfid_verification_callback,
            10,
        )

        self.storage_close_request_sub = self.create_subscription(
            String,
            "/mission/storage_close_request",
            self.storage_close_request_callback,
            10,
        )

        self.ordered_fruit_pub = self.create_publisher(
            String,
            "/ordered_fruit",
            10,
        )

        self.navigation_goal_pub = self.create_publisher(
            Pose2D,
            "/navigation/goal",
            10,
        )

        self.navigation_control_pub = self.create_publisher(
            String,
            "/navigation/control",
            10,
        )

        self.mission_state_pub = self.create_publisher(
            String,
            "/mission/state",
            10,
        )

        self.mission_event_pub = self.create_publisher(
            String,
            "/mission/event",
            10,
        )

        self.current_traffic = None

        self.get_logger().info("Mission node started.")
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

    def _publish_string(self, publisher, data: str, topic_name: str):
        msg = String()
        msg.data = data
        publisher.publish(msg)
        self.get_logger().info(f"PUB {topic_name}: {data}")


    def _publish_json(self, publisher, data: dict, topic_name: str):
        text = json.dumps(data, separators=(",", ":"))
        self._publish_string(publisher, text, topic_name)


    def _publish_mission_state(self):
        self._publish_json(
            self.mission_state_pub,
            {
                "order_id": self.active_order_id,
                "mission_state": self.mission_state,
                "fruit": self.active_fruit,
                "fault_type": self.fault_type,
                "storage_running": self.storage_sequence_running,
            },
            "/mission/state",
        )


    def _publish_mission_event(self, event: str, message: str, progress: int = 0):
        self._publish_json(
            self.mission_event_pub,
            {
                "order_id": self.active_order_id,
                "event": event,
                "message": message,
                "progress_percent": progress,
            },
            "/mission/event",
        )
    def _set_mission_state(self, state: str, event: str, message: str, progress: int):
        self.mission_state = state
        self._publish_mission_state()
        self._publish_mission_event(event, message, progress)


    def _publish_ordered_fruit(self, fruit: str):
        self._publish_string(
            self.ordered_fruit_pub,
            fruit,
            "/ordered_fruit",
        )


    def _publish_navigation_goal(self, x: float, y: float, theta: float):
        msg = Pose2D()
        msg.x = float(x)
        msg.y = float(y)
        msg.theta = float(theta)

        self.navigation_goal_pub.publish(msg)
        self.get_logger().info(
            f"PUB /navigation/goal: x={msg.x:.3f}, y={msg.y:.3f}, theta={msg.theta:.1f}"
        )


    def _publish_navigation_control(self, command: str):
        self._publish_string(
            self.navigation_control_pub,
            command,
            "/navigation/control",
        )

    def reset_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info("RX: /mission_reset_obstacle = true")
            self.reset_obstacle_state()

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

    def _send_gripper_cmd(self, command: str):
        msg = String()
        msg.data = command
        self.esp2_gripper_pub.publish(msg)
        self.get_logger().info(f"PUB /esp2/gripper_cmd: {command}")

    def _send_position_cmd(self, position: int):
        msg = Int32()
        msg.data = position
        self.esp2_position_pub.publish(msg)
        self.get_logger().info(f"PUB /esp2/position_cmd: {position}")

    def _parse_navigation_result_pose(self, text: str):
        x = 0.0
        y = 0.0
        yaw = 0.0

        for part in text.split():
            if part.startswith("x="):
                try:
                    x = float(part[2:])
                except ValueError:
                    pass
            elif part.startswith("y="):
                try:
                    y = float(part[2:])
                except ValueError:
                    pass
            elif part.startswith("yaw="):
                try:
                    yaw = float(part[4:])
                except ValueError:
                    pass

        return x, y, yaw

    def _open_storage_for_customer(self):
        self.get_logger().info("Opening storage for customer.")

        self._send_gripper_cmd("open_lock")
        time.sleep(1.0)

        self._send_gripper_cmd("open_lid")
        time.sleep(1.0)

        self.get_logger().info("Storage opened for customer.")


    def _start_open_storage_thread(self):
        threading.Thread(
            target=self._open_storage_for_customer,
            daemon=True,
        ).start()


    def _close_storage_after_customer(self):
        self.get_logger().info("Closing storage after customer collection.")

        self._send_gripper_cmd("close_lid")
        time.sleep(1.0)

        self._send_gripper_cmd("close_lock")
        time.sleep(1.0)

        self.get_logger().info("Storage closed after customer collection.")

        # Customer pose is HOME, so after closing storage the mission is complete.
        self._finish_mission_to_idle()


    def _start_close_storage_thread(self):
        threading.Thread(
            target=self._close_storage_after_customer,
            daemon=True,
        ).start()
        
    def run_storage_sequence(self):
        if self.storage_sequence_running:
            self.get_logger().warn("Storage sequence already running. Ignoring duplicate request.")
            return

        self.storage_sequence_running = True
        self.get_logger().info("Starting storage sequence.")

        self._send_gripper_cmd("open_lock")
        time.sleep(1.0)

        self._send_gripper_cmd("open_gripper")
        time.sleep(1.5)

        self._send_gripper_cmd("close_gripper")
        time.sleep(1.5)

        self._send_gripper_cmd("open_lid")
        time.sleep(1.0)

        self._send_position_cmd(90)
        time.sleep(4.0)

        self._send_gripper_cmd("open_gripper")
        time.sleep(1.5)

        self._send_position_cmd(0)
        time.sleep(4.0)

        self._send_gripper_cmd("close_lid")
        time.sleep(1.0)

        self._send_gripper_cmd("close_lock")
        time.sleep(1.0)

        self.storage_sequence_running = False
        self.get_logger().info("Fruit pickup storage sequence complete.")

        if self.mission_state == "storing":
            self._publish_mission_event(
                "storing",
                "Fruit collected and placed in storage.",
                50,
            )

            # Customer pose is HOME: x=0, y=0, yaw=0
            self._set_mission_state(
                "headingToCustomer",
                "headingToCustomer",
                "Robot heading to customer location.",
                55,
            )

            x, y, theta = self.customer_pose
            self._publish_navigation_goal(x, y, theta)

    def _start_storage_sequence_thread(self):
        threading.Thread(
            target=self.run_storage_sequence,
            daemon=True,
        ).start()

    def _finish_mission_to_idle(self):
        self._publish_mission_event(
            "idle",
            "Mission complete. Robot is idle.",
            100,
        )

        self.mission_state = "idle"
        self.active_order_id = ""
        self.active_fruit = ""
        self.active_user_id = ""
        self.assigned_rfid = ""
        self.fault_type = "none"

        self.waiting_for_valid = False
        self.storage_done_for_current_target = False
        self.waiting_for_rfid = False
        self.rfid_verified = False
        self.waiting_for_storage_close = False

        self._publish_mission_state()

    def _normalize_fruit_name(self, name: str) -> str:
        n = name.strip().lower()

        if n == "apple":
            return "Apple"
        if n == "orange":
            return "Orange"
        if n == "kiwi":
            return "Kiwi"

        return name.strip()

    def valid_callback(self, msg: Bool):
        self.latest_valid = msg.data

        if not msg.data:
            return

        if (
            self.waiting_for_valid
            and (not self.storage_sequence_running)
            and (not self.storage_done_for_current_target)
        ):
            if self.mission_state == "visionChecking":
                self.mission_state = "storing"
                self._publish_mission_state()
                self._publish_mission_event(
                    "storing",
                    f"{self.active_fruit} validated. Starting storage sequence.",
                    45,
                )
            self.get_logger().info("/valid became true. Starting storage sequence.")
            self.waiting_for_valid = False
            self.storage_done_for_current_target = True
            self._start_storage_sequence_thread()
            return

    def mission_control_callback(self, msg: String):
        self.get_logger().info(f"RX /mission/control: {msg.data}")

        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            data = {"command": msg.data.strip()}

        command = str(data.get("command", "")).upper()

        if command == "STOP":
            self.get_logger().warn("Mission STOP requested.")
            self._publish_navigation_control("STOP")

            self.mission_state = "idle"
            self.fault_type = "missionCancelled"
            self.waiting_for_valid = False
            self.storage_done_for_current_target = False
            self.waiting_for_rfid = False
            self.rfid_verified = False
            self.waiting_for_storage_close = False
            self.active_order_id = ""
            self.active_fruit = ""
            self.active_user_id = ""
            self.assigned_rfid = ""
            self.latest_valid = False

            self._set_traffic("O")
            self._publish_mission_state()
            self._publish_mission_event(
                "missionCancelled",
                "Mission stopped by GUI.",
                0,
            )
            return

        if command == "RESET":
            self.get_logger().warn("Mission RESET requested.")
            self.reset_obstacle_state()

            self.mission_state = "idle"
            self.fault_type = "none"
            self.waiting_for_valid = False
            self.storage_done_for_current_target = False
            self.waiting_for_rfid = False
            self.rfid_verified = False
            self.waiting_for_storage_close = False
            self.active_order_id = ""
            self.active_user_id = ""
            self.assigned_rfid = ""
            self.active_fruit = ""
            self.latest_valid = False

            self._set_traffic("Y")
            self._publish_mission_state()
            self._publish_mission_event(
                "missionReset",
                "Mission state reset by GUI.",
                0,
            )
            return

        self.get_logger().warn(f"Unknown mission control command: {command}")

    def navigation_result_callback(self, msg: String):
        text = msg.data.strip()
        self.get_logger().info(f"RX /navigation_result: {text}")

        if text.startswith("NAV_STARTED"):
            self._set_traffic("Y")
            return

        if text.startswith("NAV_STOPPED"):
            self._set_traffic("O")
            return

        if text.startswith("NAV_FAILED"):
            self._set_traffic("R")

            # Only navigation states are allowed to fail the mission.
            # Ignore stale NAV_FAILED after robot already reached customer / opened storage.
            if self.mission_state in ["headingToFruit", "headingToCustomer"]:
                self.mission_state = "failed"
                self.fault_type = "navigationFailed"
                self._publish_mission_state()
                self._publish_mission_event(
                    "error",
                    "Navigation failed.",
                    0,
                )
                return
            else:
                self.get_logger().warn(
                    f"Ignoring NAV_FAILED because mission_state={self.mission_state}"
                )
            return

        if not text.startswith("NAV_DONE"):
            return

        self._set_traffic("G")
        x, y, yaw = self._parse_navigation_result_pose(text)

        # Case 1: robot reached fruit stock area
        if self.mission_state == "headingToFruit":
            self.mission_state = "visionChecking"
            self._publish_mission_state()
            self._publish_mission_event(
                "visionChecking",
                f"Reached {self.active_fruit}. Waiting for vision validation.",
                30,
            )

            self.waiting_for_valid = True
            self.storage_done_for_current_target = False
            self.get_logger().info("Reached fruit target. Waiting for /valid = true.")

            if self.latest_valid:
                if (not self.storage_sequence_running) and (not self.storage_done_for_current_target):
                    self.get_logger().info("/valid already true at NAV_DONE. Starting storage sequence.")

                    self.mission_state = "storing"
                    self._publish_mission_state()
                    self._publish_mission_event(
                        "storing",
                        f"{self.active_fruit} validated. Starting storage sequence.",
                        45,
                    )

                    self.waiting_for_valid = False
                    self.storage_done_for_current_target = True
                    self._start_storage_sequence_thread()

        # Case 2: robot reached customer pose, which is HOME = 0,0,0
        if self.mission_state == "headingToCustomer":
            self.waiting_for_rfid = True
            self.rfid_verified = False

            self._set_mission_state(
                "rfidAwaiting",
                "rfidAwaiting",
                "Robot reached customer. Waiting for RFID verification.",
                70,
            )
            return

        # Any other NAV_DONE is ignored safely
        self.get_logger().info(
            f"NAV_DONE ignored for mission_state={self.mission_state}, pose=({x:.3f},{y:.3f},{yaw:.1f})"
        )

    def rfid_verification_callback(self, msg: String):
        self.get_logger().info(f"RX /mission/rfid_verification: {msg.data}")

        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("Invalid RFID JSON.")
            return

        if self.mission_state != "rfidAwaiting":
            self.get_logger().warn(
                f"Ignoring RFID because mission_state={self.mission_state}, not rfidAwaiting."
            )
            return

        order_id = data.get("order_id", "")
        success = bool(data.get("success", False))
        rfid_card_id = data.get("rfid_card_id", "")

        if order_id and order_id != self.active_order_id:
            self.get_logger().warn(
                f"RFID order mismatch: received={order_id}, active={self.active_order_id}"
            )
            return

        if (not success) or (rfid_card_id != self.assigned_rfid):
            self.get_logger().error(
                f"RFID failed: success={success}, card={rfid_card_id}, expected={self.assigned_rfid}"
            )

            self.waiting_for_rfid = False
            self.rfid_verified = False
            self.mission_state = "failed"
            self.fault_type = "rfidFailed"

            self._publish_mission_state()
            self._publish_mission_event(
                "error",
                "RFID verification failed.",
                0,
            )
            return

        self.get_logger().info("RFID verification passed.")

        self.waiting_for_rfid = False
        self.rfid_verified = True
        self.waiting_for_storage_close = True

        self._set_mission_state(
            "storageOpened",
            "storageOpened",
            "RFID verified. Storage opened for customer.",
            80,
        )

        self._start_open_storage_thread()


    def storage_close_request_callback(self, msg: String):
        order_id = msg.data.strip()
        self.get_logger().info(
            f"RX /mission/storage_close_request: order_id={order_id}"
        )

        if self.mission_state != "storageOpened":
            self.get_logger().warn(
                f"Ignoring storage close request because mission_state={self.mission_state}, not storageOpened."
            )
            return

        if order_id and order_id != self.active_order_id:
            self.get_logger().warn(
                f"Storage close order mismatch: received={order_id}, active={self.active_order_id}"
            )
            return

        self.waiting_for_storage_close = False

        self._set_mission_state(
            "storageClosed",
            "storageClosed",
            "Customer confirmed collection. Closing storage.",
            90,
        )

        self._start_close_storage_thread()

    def order_request_callback(self, msg: String):
        self.get_logger().info(f"RX /mission/order_request: {msg.data}")

        if self.mission_state != "idle":
            self.get_logger().warn(
                f"Rejecting order because mission is active: state={self.mission_state}"
            )
            self._publish_mission_event(
                "orderRejected",
                f"Mission already active: {self.mission_state}",
                0,
            )
            return

        try:
            order = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error("Invalid order JSON.")
            self.fault_type = "badOrder"
            self._publish_mission_event("orderRejected", "Invalid order JSON.", 0)
            return

        items = order.get("items", [])
        if not items:
            self.get_logger().error("Order has no items.")
            self.fault_type = "badOrder"
            self._publish_mission_event("orderRejected", "Order has no items.", 0)
            return

        if len(items) != 1:
            self.get_logger().error("Stage 2 supports exactly one item only.")
            self.fault_type = "unsupportedOrder"
            self._publish_mission_event(
                "orderRejected",
                "Only one item is supported in this stage.",
                0,
            )
            return

        item = items[0]
        quantity = int(item.get("quantity", 1))

        if quantity != 1:
            self.get_logger().error("Stage 2 supports quantity 1 only.")
            self.fault_type = "unsupportedOrder"
            self._publish_mission_event(
                "orderRejected",
                "Only quantity 1 is supported in this stage.",
                0,
            )
            return

        fruit_raw = item.get("product_name", item.get("name", "")).strip()
        fruit = self._normalize_fruit_name(fruit_raw)

        if fruit not in self.fruit_poses:
            self.get_logger().error(f"Unknown fruit: {fruit_raw}")
            self.fault_type = "unknownFruit"
            self._publish_mission_event(
                "orderRejected",
                f"Unknown fruit: {fruit_raw}",
                0,
            )
            return

        self.active_order_id = order.get("order_id", order.get("id", ""))
        self.active_user_id = order.get("user_id", "")
        self.assigned_rfid = order.get("assigned_rfid", "")
        self.active_fruit = fruit
        self.fault_type = "none"
        self.latest_valid = False
        self.waiting_for_valid = False
        self.storage_done_for_current_target = False


        self.mission_state = "missionReceived"
        self._publish_mission_state()
        self._publish_mission_event(
            "orderReceived",
            f"Order received for {fruit}.",
            5,
        )

        # Tell vision what fruit to validate.
        self._publish_ordered_fruit(fruit)

        # Send robot to fruit pose.
        x, y, theta = self.fruit_poses[fruit]
        self.mission_state = "headingToFruit"
        self._publish_mission_state()
        self._publish_mission_event(
            "headingToFruit",
            f"Heading to {fruit} stock position.",
            15,
        )
        self._publish_navigation_goal(x, y, theta)
        
    def obstacle_callback(self, msg: Int32):
        self.latest_mask = msg.data
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

    node = MissionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn("Mission node stopped by keyboard interrupt.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
