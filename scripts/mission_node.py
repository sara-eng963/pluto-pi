#!/usr/bin/env python3

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool, String
from geometry_msgs.msg import Pose2D


class MissionNode(Node):
    def __init__(self):
        super().__init__("mission_node")

        self.REQUIRED_VALID_READINGS = 3

        # Mission/order state
        self.mission_state = "idle"
        self.active_order_id = ""
        self.active_fruit = ""
        self.active_user_id = ""
        self.assigned_rfid = ""
        self.fault_type = "none"

        self.storage_sequence_running = False
        self.esp2_sequence_name = None
        self.esp2_sequence_steps = []
        self.esp2_sequence_index = 0
        self.esp2_expected_status = None
        self.latest_valid = False
        self.valid_reading_count = 0
        self.valid_true_count = 0
        self.latest_detected_fruit = ""
        self.waiting_for_valid = False
        self.storage_done_for_current_target = False

        self.RFID_MAX_ATTEMPTS = 3
        self.rfid_failed_attempts = 0

        self.waiting_for_rfid = False
        self.rfid_verified = False
        self.waiting_for_storage_close = False
        self.robot_mode = "autonomous"  # "autonomous" | "manual"

        # Customer pose is HOME for this project
        self.customer_pose = (0.0, 0.0, 0.0)

        # Temporary hardcoded fruit poses for Stage 2
        # Later move these to YAML parameters.
        self.fruit_poses = {
            "Apple": (1.24, 0.0, 0.0),
            "Orange": (1.24, 0.35, 0),
            "Kiwi": (0.8, 1.4, 0),
        }

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
        self.obstacle_reset_pub = self.create_publisher(
            Bool,
            "/mission_reset_obstacle",
            10,
        )
        self.esp2_status_sub = self.create_subscription(
            String,
            "/esp2_status",
            self.esp2_status_callback,
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
        self.detected_fruit_sub = self.create_subscription(
            String,
            "/detected_fruit",
            self.detected_fruit_callback,
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

        self.customer_rfid_sub = self.create_subscription(
            String,
            "/customer_rfid",
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

        self.robot_mode_pub = self.create_publisher(
            String,
            "/robot/mode",
            10,
        )

        self.current_traffic = None
        self.startup_default_timer = self.create_timer(
            1.0,
            self._send_startup_default_storage_position,
        )

        self.get_logger().info("Mission node started.")

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

    def request_obstacle_reset(self):
        msg = Bool()
        msg.data = True
        self.obstacle_reset_pub.publish(msg)
        self.get_logger().info("PUB /mission_reset_obstacle: true")

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

    def _send_startup_default_storage_position(self):
        self.startup_default_timer.cancel()
        self.destroy_timer(self.startup_default_timer)
        self.startup_default_timer = None

        self.get_logger().info("Setting startup storage defaults: open lock, close lid, gripper open.")
        self._send_gripper_cmd("open_lock")
        self._send_gripper_cmd("close_lid")
        self._send_gripper_cmd("open_gripper")

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

    def _cancel_esp2_sequence(self):
        self.storage_sequence_running = False
        self.esp2_sequence_name = None
        self.esp2_sequence_steps = []
        self.esp2_sequence_index = 0
        self.esp2_expected_status = None

    def _start_esp2_sequence(self, name: str, steps: list):
        if self.storage_sequence_running:
            self.get_logger().warn(
                f"ESP2 sequence already running ({self.esp2_sequence_name}). Ignoring {name} request."
            )
            return False

        self.storage_sequence_running = True
        self.esp2_sequence_name = name
        self.esp2_sequence_steps = steps
        self.esp2_sequence_index = 0
        self.esp2_expected_status = None

        self.get_logger().info(f"Starting ESP2 sequence: {name}")
        self._send_current_esp2_sequence_step()
        return True

    def _send_current_esp2_sequence_step(self):
        if not self.storage_sequence_running:
            return

        if self.esp2_sequence_index >= len(self.esp2_sequence_steps):
            self._complete_esp2_sequence()
            return

        step = self.esp2_sequence_steps[self.esp2_sequence_index]
        self.esp2_expected_status = step["expected_status"]

        if step["command_type"] == "gripper":
            self._send_gripper_cmd(step["command_value"])
        elif step["command_type"] == "position":
            self._send_position_cmd(step["command_value"])
        else:
            self.get_logger().error(f"Invalid ESP2 sequence command type: {step['command_type']}")
            self._cancel_esp2_sequence()

    def esp2_status_callback(self, msg: String):
        status = msg.data.strip()
        self.get_logger().info(f"RX /esp2_status: {status}")

        if not self.storage_sequence_running:
            return

        if status != self.esp2_expected_status:
            return

        self.esp2_sequence_index += 1
        self._send_current_esp2_sequence_step()

    def _complete_esp2_sequence(self):
        sequence_name = self.esp2_sequence_name
        self._cancel_esp2_sequence()

        if sequence_name == "fruit_pickup_storage":
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

        elif sequence_name == "customer_open":
            self.get_logger().info("Storage opened for customer (mechanism complete).")

        elif sequence_name == "customer_close":
            self.get_logger().info("Storage closed after customer collection.")

            # Customer pose is HOME, so after closing storage the mission is complete.
            self._finish_mission_to_idle()

    def _storage_step(self, command_type: str, command_value, expected_status: str):
        return {
            "command_type": command_type,
            "command_value": command_value,
            "expected_status": expected_status,
        }

    def _start_open_storage_thread(self):
        self.get_logger().info("Opening storage for customer.")
        self._start_esp2_sequence(
            "customer_open",
            [
                self._storage_step("gripper", "open_lock", "opened lock"),
                self._storage_step("gripper", "open_lid", "opened lid"),
            ],
        )

    def _start_close_storage_thread(self):
        self.get_logger().info("Closing storage after customer collection.")
        self._start_esp2_sequence(
            "customer_close",
            [
                self._storage_step("gripper", "close_lid", "closed lid"),
            ],
        )

    def run_storage_sequence(self):
        self.get_logger().info("Starting storage sequence.")
        self._start_esp2_sequence(
            "fruit_pickup_storage",
            [
                self._storage_step("gripper", "open_lock", "opened lock"),
                self._storage_step("gripper", "open_gripper", "opened gripper"),
                self._storage_step("gripper", "close_gripper", "closed gripper"),
                self._storage_step("gripper", "open_lid", "opened lid"),
                self._storage_step("position", 90, "position 90 reached"),
                self._storage_step("gripper", "open_gripper", "opened gripper"),
                self._storage_step("position", 0, "position 0 reached"),
                self._storage_step("gripper", "close_lid", "closed lid"),
            ],
        )

    def _start_storage_sequence_thread(self):
        self.run_storage_sequence()

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
        self._reset_valid_readings()
        self.storage_done_for_current_target = False
        self.waiting_for_rfid = False
        self.rfid_verified = False
        self.waiting_for_storage_close = False
        self.rfid_failed_attempts = 0
        self.latest_detected_fruit = ""

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

        if not (
            self.waiting_for_valid
            and self.mission_state == "visionChecking"
            and not self.storage_sequence_running
            and not self.storage_done_for_current_target
        ):
            return

        self.valid_reading_count += 1
        if msg.data:
            self.valid_true_count += 1

        self.get_logger().info(
            f"Vision valid sample {self.valid_reading_count}/{self.REQUIRED_VALID_READINGS}: {msg.data}"
        )

        if self.valid_reading_count < self.REQUIRED_VALID_READINGS:
            return

        if self.valid_true_count == self.REQUIRED_VALID_READINGS:
            self.mission_state = "storing"
            self._publish_mission_state()
            self._publish_mission_event(
                "storing",
                f"{self.active_fruit} validated after {self.REQUIRED_VALID_READINGS} readings. Starting storage sequence.",
                45,
            )
            self.get_logger().info(
                f"/valid true for {self.REQUIRED_VALID_READINGS} readings. Starting storage sequence."
            )
            self.waiting_for_valid = False
            self._reset_valid_readings()
            self.storage_done_for_current_target = True
            self._start_storage_sequence_thread()
            return

        self.get_logger().warn(
            f"Vision failed after {self.REQUIRED_VALID_READINGS} readings: {self.valid_true_count} true."
        )
        self._handle_vision_fail()

    def detected_fruit_callback(self, msg: String):
        self.latest_detected_fruit = msg.data.strip()

    def _reset_valid_readings(self):
        self.valid_reading_count = 0
        self.valid_true_count = 0

    def _handle_vision_fail(self):
        detected = self.latest_detected_fruit or "Unknown"
        self.get_logger().warn(
            f"Vision FAIL: ordered={self.active_fruit}, detected={detected}. Returning to base."
        )

        self.waiting_for_valid = False
        self._reset_valid_readings()
        self.storage_done_for_current_target = False

        self._set_mission_state(
            "visionFailedReturning",
            "visionFailed",
            f"Vision check failed: expected {self.active_fruit}, detected {detected}. Returning to base.",
            25,
        )

        x, y, theta = self.customer_pose
        self._publish_navigation_goal(x, y, theta)

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
            self._cancel_esp2_sequence()

            self.mission_state = "idle"
            self.fault_type = "missionCancelled"
            self.waiting_for_valid = False
            self._reset_valid_readings()
            self.storage_done_for_current_target = False
            self.waiting_for_rfid = False
            self.rfid_verified = False
            self.waiting_for_storage_close = False
            self.active_order_id = ""
            self.active_fruit = ""
            self.active_user_id = ""
            self.assigned_rfid = ""
            self.latest_valid = False
            self.latest_detected_fruit = ""
            self.rfid_failed_attempts = 0

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
            self.request_obstacle_reset()
            self._publish_navigation_control("RESET")
            self._cancel_esp2_sequence()

            self.mission_state = "idle"
            self.fault_type = "none"
            self.waiting_for_valid = False
            self._reset_valid_readings()
            self.storage_done_for_current_target = False
            self.waiting_for_rfid = False
            self.rfid_verified = False
            self.waiting_for_storage_close = False
            self.active_order_id = ""
            self.active_user_id = ""
            self.assigned_rfid = ""
            self.active_fruit = ""
            self.latest_valid = False
            self.latest_detected_fruit = ""
            self.rfid_failed_attempts = 0

            self._set_traffic("Y")
            self._publish_mission_state()
            self._publish_mission_event(
                "missionReset",
                "Mission state reset by GUI.",
                0,
            )
            return

        if command == "SET_MODE":
            mode = str(data.get("mode", "autonomous")).lower()
            if mode not in ("manual", "autonomous"):
                self.get_logger().warn(f"Unknown mode in SET_MODE: {mode}")
                return

            if self.robot_mode == mode:
                self.get_logger().info(f"Robot already in mode: {mode}")
                self._publish_robot_mode()
                return

            self.robot_mode = mode
            self.get_logger().warn(f"Robot mode → {mode}")

            if mode == "manual":
                # Abort any active mission cleanly.
                if self.mission_state != "idle":
                    self._publish_navigation_control("STOP")
                    self._cancel_esp2_sequence()

                    self.mission_state = "idle"
                    self.fault_type = "modeSwitchedToManual"
                    self.waiting_for_valid = False
                    self._reset_valid_readings()
                    self.storage_done_for_current_target = False
                    self.waiting_for_rfid = False
                    self.rfid_verified = False
                    self.waiting_for_storage_close = False
                    self.active_order_id = ""
                    self.active_fruit = ""
                    self.active_user_id = ""
                    self.assigned_rfid = ""
                    self.latest_valid = False
                    self.latest_detected_fruit = ""
                    self.rfid_failed_attempts = 0

                    self._set_traffic("O")
                    self._publish_mission_state()
                    self._publish_mission_event(
                        "missionAborted",
                        "Mission aborted: switched to manual control.",
                        0,
                    )

            self._publish_robot_mode()
            return

        self.get_logger().warn(f"Unknown mission control command: {command}")

    def _publish_robot_mode(self):
        msg = String()
        msg.data = self.robot_mode
        self.robot_mode_pub.publish(msg)
        self.get_logger().info(f"PUB /robot/mode: {self.robot_mode}")

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
            if self.mission_state in ["headingToFruit", "headingToCustomer", "visionFailedReturning"]:
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
            self._reset_valid_readings()
            self.storage_done_for_current_target = False
            self.get_logger().info(
                f"Reached fruit target. Waiting for {self.REQUIRED_VALID_READINGS} fresh /valid readings."
            )
            return

        # Case 2: robot reached customer pose, which is HOME = 0,0,0
        if self.mission_state == "headingToCustomer":
            self.waiting_for_rfid = True
            self.rfid_verified = False
            self.rfid_failed_attempts = 0

            self._set_mission_state(
                "rfidAwaiting",
                "rfidAwaiting",
                "Robot reached customer. Waiting for RFID verification.",
                70,
            )
            return

        # Case 3: robot returned home after vision fail — no RFID, no storage, straight to idle
        if self.mission_state == "visionFailedReturning":
            self.get_logger().info("Returned to base after vision fail. Ready for new orders.")
            self._publish_mission_event(
                "visionFailedIdle",
                f"Vision check failed. Robot returned to base. Ready for new orders.",
                0,
            )
            self.mission_state = "idle"
            self.active_order_id = ""
            self.active_fruit = ""
            self.active_user_id = ""
            self.assigned_rfid = ""
            self.fault_type = "visionFailed"
            self.waiting_for_valid = False
            self._reset_valid_readings()
            self.storage_done_for_current_target = False
            self.waiting_for_rfid = False
            self.rfid_verified = False
            self.waiting_for_storage_close = False
            self.rfid_failed_attempts = 0
            self.latest_detected_fruit = ""
            self._publish_mission_state()
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
            self.rfid_failed_attempts += 1
            remaining_attempts = self.RFID_MAX_ATTEMPTS - self.rfid_failed_attempts

            self.get_logger().error(
                f"RFID failed attempt {self.rfid_failed_attempts}/{self.RFID_MAX_ATTEMPTS}: "
                f"success={success}, card={rfid_card_id}, expected={self.assigned_rfid}"
            )

            self.rfid_verified = False

            if self.rfid_failed_attempts < self.RFID_MAX_ATTEMPTS:
                self.waiting_for_rfid = True
                self.mission_state = "rfidAwaiting"

                self._publish_mission_state()
                self._publish_mission_event(
                    "rfidRetry",
                    f"Wrong RFID card. {remaining_attempts} attempt(s) remaining.",
                    70,
                )
                return

            self.waiting_for_rfid = False
            self.mission_state = "failed"
            self.fault_type = "rfidFailed"
            self._send_gripper_cmd("close_lock")

            self._publish_mission_state()
            self._publish_mission_event(
                "error",
                "RFID verification failed after 3 attempts.",
                0,
            )
            return

        self.get_logger().info("RFID verification passed.")

        self.waiting_for_rfid = False
        self.rfid_verified = True
        self.waiting_for_storage_close = True
        self.rfid_failed_attempts = 0

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

        if self.robot_mode == "manual":
            self.get_logger().warn("Rejecting order: robot is in manual mode.")
            self._publish_mission_event(
                "orderRejected",
                "Robot is in manual control mode. Switch to autonomous to place orders.",
                0,
            )
            return

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
        self.latest_detected_fruit = ""
        self.waiting_for_valid = False
        self._reset_valid_readings()
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
