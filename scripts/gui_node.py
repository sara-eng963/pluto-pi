#!/usr/bin/env python3
"""
gui_node.py

Phase 1 GUI bridge for Pluto.

Purpose:
    Bridge the fixed sim_server.py dashboard WebSocket to ROS 2.

Important:
    - Does NOT own mission logic.
    - Does NOT publish /drive_cmd.
    - Does NOT publish ESP2 mechanism topics.
    - Only translates safe GUI/dashboard messages into semantic ROS topics.
    - Sends selected ROS telemetry/events back to dashboard WebSocket as JSON logs.

Backend:
    sim_server.py must already be running.
    Default WebSocket URL:
        ws://127.0.0.1:8000/dashboard/ws

Run:
    ros2 run pluto gui_node.py --ros-args -p backend_ws_url:=ws://127.0.0.1:8000/dashboard/ws
"""

import asyncio
import json
import threading
import time
from datetime import datetime, timezone
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool, Float32, Int32
from geometry_msgs.msg import Pose2D

try:
    import websockets
except ImportError as exc:
    raise ImportError(
        "Missing Python package 'websockets'. Install it in the ROS Python environment.\n"
        "Try: sudo apt install python3-websockets\n"
        "or inside the active environment: pip install websockets"
    ) from exc


def now_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.000Z")


def compact_json(data: Dict[str, Any]) -> str:
    return json.dumps(data, separators=(",", ":"), ensure_ascii=False)


class GuiNode(Node):
    def __init__(self):
        super().__init__("gui_node")

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter(
            "backend_ws_url",
            "ws://127.0.0.1:8000/dashboard/ws",
        )
        self.backend_ws_url = (
            self.get_parameter("backend_ws_url")
            .get_parameter_value()
            .string_value
        )

        # GUI heartbeat messages required by the PDF contract
        self.robot_status_timer = self.create_timer(
            2.0,
            self.robot_status_heartbeat_callback,
        )

        self.system_health_timer = self.create_timer(
            10.0,
            self.system_health_timer_callback,
        )

        self.battery_timer = self.create_timer(
            30.0,
            self.battery_timer_callback,
        )

        # ------------------------------------------------------------------
        # ROS publishers: GUI -> ROS semantic commands
        # ------------------------------------------------------------------
        self.order_request_pub = self.create_publisher(
            String,
            "/mission/order_request",
            10,
        )

        self.mission_control_pub = self.create_publisher(
            String,
            "/mission/control",
            10,
        )

        self.rfid_verification_pub = self.create_publisher(
            String,
            "/mission/rfid_verification",
            10,
        )

        self.storage_close_request_pub = self.create_publisher(
            String,
            "/mission/storage_close_request",
            10,
        )

        # ------------------------------------------------------------------
        # ROS subscribers: ROS -> GUI/dashboard log
        # ------------------------------------------------------------------
        self.navigation_result_sub = self.create_subscription(
            String,
            "/navigation_result",
            self.navigation_result_callback,
            10,
        )

        self.detected_fruit_sub = self.create_subscription(
            String,
            "/detected_fruit",
            self.detected_fruit_callback,
            10,
        )

        self.detected_confidence_sub = self.create_subscription(
            Float32,
            "/detected_confidence",
            self.detected_confidence_callback,
            10,
        )

        self.valid_sub = self.create_subscription(
            Bool,
            "/valid",
            self.valid_callback,
            10,
        )

        self.obstacle_status_sub = self.create_subscription(
            Int32,
            "/obstacle_status",
            self.obstacle_status_callback,
            10,
        )

        self.drive_status_sub = self.create_subscription(
            String,
            "/drive_status",
            self.drive_status_callback,
            10,
        )

        self.mission_state_sub = self.create_subscription(
            String,
            "/mission/state",
            self.mission_state_callback,
            10,
        )

        self.mission_event_sub = self.create_subscription(
            String,
            "/mission/event",
            self.mission_event_callback,
            10,
        )

        self.navigation_status_sub = self.create_subscription(
            String,
            "/navigation/status",
            self.navigation_status_callback,
            10,
        )

        self.navigation_pose_sub = self.create_subscription(
            Pose2D,
            "/navigation/pose",
            self.navigation_pose_callback,
            10,
        )

        self.esp2_voltage_sub = self.create_subscription(
            Float32,
            "/esp2/voltage",
            self.esp2_voltage_callback,
            10,
        )

        # ------------------------------------------------------------------
        # Cached ROS state for GUI messages
        # ------------------------------------------------------------------
        self.latest_detected_fruit = "Unknown"
        self.latest_detected_confidence = 0.0
        self.latest_valid = False
        self.latest_mission_state = "idle"
        self.latest_active_order_id = ""
        self.latest_fault_type = "none"
        self.latest_current_fruit = ""
        self.latest_storage_state = "closed"
        self.latest_obstacle_detected = False
        self.latest_distance_remaining = 0.0

        self.last_drive_status_gui_time = 0.0
        self.drive_status_throttle_sec = 1.0

        # Extra cached telemetry/state for PDF-compatible GUI messages
        self.latest_battery_voltage = 11.4
        self.latest_battery_percent = 82
        self.latest_is_charging = False

        self.latest_rfid_card_id = ""
        self.latest_rfid_order_id = ""

        self.start_time_monotonic = time.monotonic()
        self.last_ros_message_time = time.monotonic()
        self.last_drive_status_time = 0.0
        self.last_vision_time = 0.0
        self.last_obstacle_time = 0.0

        # ------------------------------------------------------------------
        # WebSocket threading
        # ------------------------------------------------------------------
        self.ws_loop: Optional[asyncio.AbstractEventLoop] = None
        self.ws_queue: Optional[asyncio.Queue] = None
        self.ws_connected = False
        self.stop_event = threading.Event()

        self.ws_thread = threading.Thread(
            target=self.websocket_thread_main,
            daemon=True,
        )
        self.ws_thread.start()

        self.get_logger().info("GUI bridge node started.")
        self.get_logger().info(f"Backend WebSocket: {self.backend_ws_url}")
        self.get_logger().info("Safety: gui_node will not publish /drive_cmd or ESP2 mechanism topics.")

    # ======================================================================
    # WebSocket lifecycle
    # ======================================================================

    def websocket_thread_main(self):
        self.ws_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.ws_loop)

        self.ws_queue = asyncio.Queue()

        try:
            self.ws_loop.run_until_complete(self.websocket_main())
        except Exception as exc:
            self.get_logger().error(f"WebSocket loop crashed: {exc}")
        finally:
            self.ws_loop.close()

    async def websocket_main(self):
        while rclpy.ok() and not self.stop_event.is_set():
            try:
                self.get_logger().info(f"Connecting to dashboard WS: {self.backend_ws_url}")

                async with websockets.connect(self.backend_ws_url) as ws:
                    self.ws_connected = True
                    self.get_logger().info("Connected to dashboard WebSocket.")

                    await self.send_dashboard_json_direct(
                        ws,
                        {
                            "type": "event.log",
                            "level": "system",
                            "event_type": "gui_node_connected",
                            "message": "ROS GUI bridge connected to dashboard WebSocket.",
                            "timestamp": now_iso(),
                        },
                    )

                    await self.send_dashboard_json_direct(
                        ws,
                        {
                            "type": "connection.status",
                            "state": "connected",
                            "latency_ms": 0,
                            "timestamp": now_iso(),
                        },
                    )

                    sender_task = asyncio.create_task(self.websocket_sender(ws))
                    receiver_task = asyncio.create_task(self.websocket_receiver(ws))

                    done, pending = await asyncio.wait(
                        {sender_task, receiver_task},
                        return_when=asyncio.FIRST_EXCEPTION,
                    )

                    for task in pending:
                        task.cancel()

                    for task in done:
                        exc = task.exception()
                        if exc:
                            raise exc

            except asyncio.CancelledError:
                break

            except Exception as exc:
                self.ws_connected = False
                self.get_logger().warn(f"Dashboard WS disconnected/error: {exc}")
                await asyncio.sleep(2.0)

        self.ws_connected = False

    async def websocket_sender(self, ws):
        while rclpy.ok() and not self.stop_event.is_set():
            msg = await self.ws_queue.get()
            await self.send_dashboard_json_direct(ws, msg)

    async def send_dashboard_json_direct(self, ws, msg: Dict[str, Any]):
        text = compact_json(msg)
        await ws.send(text)

    async def websocket_receiver(self, ws):
        async for raw in ws:
            try:
                payload = json.loads(raw)
            except json.JSONDecodeError:
                self.get_logger().warn(f"Ignoring non-JSON WS payload: {raw}")
                continue

            self.handle_dashboard_payload(payload)

    def send_dashboard_json(self, msg: Dict[str, Any]):
        """
        Thread-safe enqueue from ROS callbacks to WebSocket thread.
        """
        if self.ws_loop is None or self.ws_queue is None:
            return

        try:
            asyncio.run_coroutine_threadsafe(
                self.ws_queue.put(msg),
                self.ws_loop,
            )
        except RuntimeError:
            pass

    # ======================================================================
    # Dashboard WS -> ROS
    # ======================================================================

    def handle_dashboard_payload(self, payload: Dict[str, Any]):
        """
        Process messages coming from sim_server dashboard websocket.

        sim_server sends:
            {"type":"init", ...}
            {"type":"log", "entry": {"dir":"IN"/"OUT"/"SYS", "msg": {...}}}

        We only process:
            payload.type == "log"
            payload.entry.dir == "IN"

        This prevents simulator OUT logs from entering ROS.
        """

        if payload.get("type") == "init":
            self.get_logger().info("Received dashboard init payload.")
            return

        if payload.get("type") != "log":
            return

        entry = payload.get("entry", {})
        direction = entry.get("dir", "")

        if direction != "IN":
            return

        msg = entry.get("msg", {})
        if not isinstance(msg, dict):
            return

        msg_type = msg.get("type", "")

        # Ignore gui_node-generated log messages echoed back by the server.
        if msg_type in ("event.log", "robot.status", "vision.fruit_detection",
                        "navigation.obstacle_status", "navigation.path_status"):
            return

        self.handle_gui_message(msg)

    def publish_string(self, publisher, data: str, topic_name: str):
        msg = String()
        msg.data = data
        publisher.publish(msg)
        self.get_logger().info(f"PUB {topic_name}: {data}")

    def handle_gui_message(self, msg: Dict[str, Any]):
        msg_type = msg.get("type", "")

        # --------------------------------------------------------------
        # Accepted commands
        # --------------------------------------------------------------

        if msg_type == "order.place":
            order = dict(msg)

            if not order.get("order_id"):
                order["order_id"] = f"dashboard_order_{int(time.time() * 1000)}"

            self.publish_string(
                self.order_request_pub,
                compact_json(order),
                "/mission/order_request",
            )
            return

        if msg_type in ("mission.stop", "order.cancel"):
            control = {
                "command": "STOP",
                "source": "gui",
                "order_id": msg.get("order_id", ""),
                "reason": msg.get("reason", msg_type),
                "timestamp": now_iso(),
            }

            self.publish_string(
                self.mission_control_pub,
                compact_json(control),
                "/mission/control",
            )
            return

        if msg_type == "debug.reset":
            control = {
                "command": "RESET",
                "source": "gui",
                "timestamp": now_iso(),
            }

            self.publish_string(
                self.mission_control_pub,
                compact_json(control),
                "/mission/control",
            )
            return

        if msg_type in ("debug.rfid_simulate", "rfid.verification"):
            rfid = {
                "source": "dashboard" if msg_type == "debug.rfid_simulate" else "gui",
                "order_id": msg.get("order_id", ""),
                "rfid_card_id": msg.get("rfid_card_id", ""),
                "success": bool(msg.get("should_succeed", True)),
                "timestamp": now_iso(),
            }

            self.latest_rfid_card_id = rfid.get("rfid_card_id", "")
            self.latest_rfid_order_id = rfid.get("order_id", "")

            self.publish_string(
                self.rfid_verification_pub,
                compact_json(rfid),
                "/mission/rfid_verification",
            )
            return

        if msg_type == "storage.close_request":
            order_id = msg.get("order_id", "")

            self.publish_string(
                self.storage_close_request_pub,
                str(order_id),
                "/mission/storage_close_request",
            )
            return

        # --------------------------------------------------------------
        # Rejected unsafe simulator/debug commands
        # --------------------------------------------------------------

        rejected = {
            "debug.obstacle_inject",
            "debug.obstacle_release",
            "debug.force_state",
            "debug.battery_drain",
            "debug.battery_charge",
        }

        if msg_type in rejected:
            self.get_logger().warn(
                f"Rejected unsafe dashboard command: {msg_type}. "
                "Real robot state remains owned by ROS/ESP."
            )

            self.send_dashboard_json(
                {
                    "type": "event.log",
                    "level": "warning",
                    "event_type": "gui_command_rejected",
                    "message": f"Rejected unsafe GUI command: {msg_type}",
                    "timestamp": now_iso(),
                }
            )
            return

        # --------------------------------------------------------------
        # Ignored backend/session/payment messages
        # --------------------------------------------------------------

        ignored = {
            "user.session",
            "payment.request",
            "payment.status",
            "mission.start",
        }

        if msg_type in ignored:
            self.get_logger().info(f"Ignored backend-owned GUI message: {msg_type}")
            return

        if msg_type:
            self.get_logger().info(f"Ignored unhandled GUI message type: {msg_type}")

    # ======================================================================
    # ROS -> Dashboard WS
    # ======================================================================

    def _battery_percent_from_voltage(self, voltage: float) -> int:
        """
        Simple linear estimate:
        7.5 V  -> 0%
        12.6 V -> 100%

        This is only GUI telemetry estimation, not BMS logic.
        """

        percent = int(round(((voltage - 7.5) / (12.6 - 7.5)) * 100.0))
        return max(0, min(100, percent))


    def _send_robot_status(self):
        self.send_dashboard_json(
            {
                "type": "robot.status",
                "battery_percent": int(self.latest_battery_percent),
                "is_charging": bool(self.latest_is_charging),
                "mode": "autonomous",
                "mission_state": self.latest_mission_state,
                "storage_state": self.latest_storage_state,
                "fault_type": self.latest_fault_type,
                "active_order_id": self.latest_active_order_id,
                "linear_speed": 0.0,
                "angular_speed": 0.0,
                "distance_remaining": float(self.latest_distance_remaining),
                "obstacle_detected": bool(self.latest_obstacle_detected),
                "current_fruit": self.latest_current_fruit.lower() if self.latest_current_fruit else "",
                "timestamp": now_iso(),
            }
        )


    def _send_order_status_update(self, order_id: str, event: str, message: str):
        if not order_id:
            return

        status_map = {
            "orderReceived": "accepted",
            "missionReceived": "accepted",
            "headingToFruit": "navigating_to_fruit",
            "visionChecking": "checking_stock",
            "storing": "collecting_fruit",
            "headingToCustomer": "delivering",
            "rfidAwaiting": "waiting_for_rfid",
            "storageOpened": "ready_for_pickup",
            "storageClosed": "collected",
            "returning": "returning",
            "idle": "completed",
            "error": "failed",
            "failed": "failed",
        }

        self.send_dashboard_json(
            {
                "type": "order.status_update",
                "order_id": order_id,
                "status": status_map.get(event, event),
                "message": message,
                "timestamp": now_iso(),
            }
        )


    def _send_storage_messages_for_event(self, order_id: str, event: str):
        if not order_id:
            return

        if event == "storing":
            self.send_dashboard_json(
                {
                    "type": "storage.pickup_status",
                    "state": "gripping",
                    "fruit": self.latest_current_fruit.lower() if self.latest_current_fruit else "",
                    "attempt": 1,
                    "timestamp": now_iso(),
                }
            )

            self.send_dashboard_json(
                {
                    "type": "storage.status",
                    "state": "opening",
                    "order_id": order_id,
                    "timestamp": now_iso(),
                }
            )
            return

        if event == "storageOpened":
            self.send_dashboard_json(
                {
                    "type": "storage.status",
                    "state": "open",
                    "order_id": order_id,
                    "timestamp": now_iso(),
                }
            )

            self.send_dashboard_json(
                {
                    "type": "storage.open",
                    "state": "open",
                    "order_id": order_id,
                    "timestamp": now_iso(),
                }
            )
            return

        if event == "storageClosed":
            self.send_dashboard_json(
                {
                    "type": "storage.status",
                    "state": "closed",
                    "order_id": order_id,
                    "timestamp": now_iso(),
                }
            )

            self.send_dashboard_json(
                {
                    "type": "storage.closed",
                    "state": "closing",
                    "order_id": order_id,
                    "timestamp": now_iso(),
                }
            )
            return


    def _send_rfid_result_for_event(self, order_id: str, event: str, message: str):
        if event == "storageOpened":
            self.send_dashboard_json(
                {
                    "type": "rfid.result",
                    "success": True,
                    "rfid_card_id": self.latest_rfid_card_id,
                    "order_id": order_id,
                    "message": "Identity verified ✓",
                    "timestamp": now_iso(),
                }
            )
            return

        if event in ("error", "failed") and self.latest_fault_type == "rfidFailed":
            self.send_dashboard_json(
                {
                    "type": "rfid.result",
                    "success": False,
                    "rfid_card_id": self.latest_rfid_card_id,
                    "order_id": order_id,
                    "message": message or "RFID verification failed.",
                    "timestamp": now_iso(),
                }
            )
            return


    def _send_battery_telemetry(self):
        self.send_dashboard_json(
            {
                "type": "telemetry.battery",
                "battery_percent": int(self.latest_battery_percent),
                "is_charging": bool(self.latest_is_charging),
                "voltage": round(float(self.latest_battery_voltage), 2),
                "estimated_minutes_remaining": int(max(0, self.latest_battery_percent * 2)),
                "timestamp": now_iso(),
            }
        )


    def _send_system_health(self):
        now = time.monotonic()

        self.send_dashboard_json(
            {
                "type": "system.health",
                "level": "nominal",
                "uptime_seconds": int(now - self.start_time_monotonic),
                "ros2_active": True,
                "micro_ros_active": (now - self.last_drive_status_time) < 5.0 if self.last_drive_status_time else False,
                "camera_active": (now - self.last_vision_time) < 5.0 if self.last_vision_time else False,
                "lidar_active": (now - self.last_obstacle_time) < 5.0 if self.last_obstacle_time else False,
                "timestamp": now_iso(),
            }
        )
    def mission_state_callback(self, msg: String):
            """
            Convert ROS /mission/state JSON into GUI robot.status JSON.
            """

            try:
                data = json.loads(msg.data)
            except json.JSONDecodeError:
                self.get_logger().warn(f"Invalid /mission/state JSON: {msg.data}")
                return

            mission_state = data.get("mission_state", "idle")
            order_id = data.get("order_id", "")
            fruit = data.get("fruit", "")
            fault_type = data.get("fault_type", "none")

            self.latest_mission_state = mission_state
            self.latest_active_order_id = order_id
            self.latest_current_fruit = fruit
            self.latest_fault_type = fault_type

            # Temporary storage state inference for current stage.
            # Later mission_node should publish exact storage state.
            storage_state = "closed"
            if mission_state == "storageOpened":
                storage_state = "open"
            elif mission_state == "storageClosed":
                storage_state = "closed"
            elif mission_state == "storing":
                storage_state = "closed"

            self.latest_storage_state = storage_state

            self.last_ros_message_time = time.monotonic()
            self._send_robot_status()   


    def mission_event_callback(self, msg: String):
            """
            Convert ROS /mission/event JSON into GUI mission.event and event.log JSON.
            """

            try:
                data = json.loads(msg.data)
            except json.JSONDecodeError:
                self.get_logger().warn(f"Invalid /mission/event JSON: {msg.data}")
                return

            order_id = data.get("order_id", "")
            event = data.get("event", "")
            message = data.get("message", "")
            progress_percent = int(data.get("progress_percent", 0))

            # Main timeline event for Flutter/GUI.
            self.send_dashboard_json(
                {
                    "type": "mission.event",
                    "order_id": order_id,
                    "event": event,
                    "message": message,
                    "progress_percent": progress_percent,
                    "timestamp": now_iso(),
                }
            )

            # Human-readable event log.
            self.send_dashboard_json(
                {
                    "type": "event.log",
                    "level": "mission",
                    "event_type": event,
                    "message": message,
                    "order_id": order_id,
                    "timestamp": now_iso(),
                }
            )

            self.last_ros_message_time = time.monotonic()

            # Extra PDF-required messages generated from mission events
            self._send_order_status_update(order_id, event, message)
            self._send_storage_messages_for_event(order_id, event)
            self._send_rfid_result_for_event(order_id, event, message)


    def navigation_status_callback(self, msg: String):
        """
        Convert ROS /navigation/status JSON into GUI navigation.path_status JSON.
        """

        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Invalid /navigation/status JSON: {msg.data}")
            return

        status = data.get("status", "")
        goal = data.get("goal", {}) or {}

        state = "clear"
        if status in ("FAILED", "STOPPED"):
            state = "blocked"

        target_x = float(goal.get("x", 0.0))
        target_y = float(goal.get("y", 0.0))

        self.last_ros_message_time = time.monotonic()

        self.send_dashboard_json(
            {
                "type": "navigation.path_status",
                "state": state,
                "target_x": target_x,
                "target_y": target_y,
                "distance_to_goal": float(self.latest_distance_remaining),
                "estimated_seconds": 0,
                "timestamp": now_iso(),
            }
        )

    def navigation_pose_callback(self, msg: Pose2D):
            """
            Convert ROS /navigation/pose into GUI navigation.pose JSON.
            """
            self.last_ros_message_time = time.monotonic()

            self.send_dashboard_json(
                {
                    "type": "navigation.pose",
                    "x": float(msg.x),
                    "y": float(msg.y),
                    "heading_degrees": float(msg.theta),
                    "timestamp": now_iso(),
                }
            )

    def navigation_result_callback(self, msg: String):
        text = msg.data.strip()

        self.send_dashboard_json(
            {
                "type": "event.log",
                "level": "navigation",
                "event_type": "navigation_result",
                "message": text,
                "timestamp": now_iso(),
            }
        )

    def esp2_voltage_callback(self, msg: Float32):
        voltage = float(msg.data)

        self.latest_battery_voltage = voltage
        self.latest_battery_percent = self._battery_percent_from_voltage(voltage)
        self.last_ros_message_time = time.monotonic()

        self._send_battery_telemetry()

    def drive_status_callback(self, msg: String):
        self.last_ros_message_time = time.monotonic()
        self.last_drive_status_time = time.monotonic()
        text = msg.data.strip()

        # STATUS can be high-rate. Throttle it.
        if text.startswith("STATUS"):
            now = time.monotonic()
            if now - self.last_drive_status_gui_time < self.drive_status_throttle_sec:
                return
            self.last_drive_status_gui_time = now

        self.send_dashboard_json(
            {
                "type": "event.log",
                "level": "drive",
                "event_type": "drive_status",
                "message": text,
                "timestamp": now_iso(),
            }
        )

    def obstacle_status_callback(self, msg: Int32):

        self.last_ros_message_time = time.monotonic()
        self.last_obstacle_time = time.monotonic()
        mask = int(msg.data)
        blocking = mask != 0
        self.latest_obstacle_detected = blocking

        self.send_dashboard_json(
            {
                "type": "navigation.obstacle_status",
                "state": "detected" if blocking else "clear",
                "raw_mask": mask,
                "distance_meters": None,
                "angle_degrees": None,
                "is_blocking_path": blocking,
                "timestamp": now_iso(),
            }
        )

    def detected_fruit_callback(self, msg: String):
        self.last_ros_message_time = time.monotonic()
        self.last_vision_time = time.monotonic()
        self.latest_detected_fruit = msg.data.strip()
        self.publish_cached_vision_to_gui()

    def detected_confidence_callback(self, msg: Float32):
        self.last_ros_message_time = time.monotonic()
        self.last_vision_time = time.monotonic()
        self.latest_detected_confidence = float(msg.data)
        self.publish_cached_vision_to_gui()

    def valid_callback(self, msg: Bool):
        self.last_ros_message_time = time.monotonic()
        self.last_vision_time = time.monotonic()
        self.latest_valid = bool(msg.data)
        self.publish_cached_vision_to_gui()

    def publish_cached_vision_to_gui(self):
        fruit = self.latest_detected_fruit
        conf = float(self.latest_detected_confidence)
        valid = bool(self.latest_valid)

        self.send_dashboard_json(
            {
                "type": "vision.fruit_detection",
                "fruit": fruit,
                "detected": fruit not in ("", "Unknown"),
                "confidence": round(conf, 3),
                "valid": valid,
                "bounding_box": None,
                "timestamp": now_iso(),
            }
        )

    def robot_status_heartbeat_callback(self):
        self._send_robot_status()


    def system_health_timer_callback(self):
        self._send_system_health()


    def battery_timer_callback(self):
        self._send_battery_telemetry()

    # ======================================================================
    # Shutdown
    # ======================================================================

    def destroy_node(self):
        self.stop_event.set()

        self.send_dashboard_json(
            {
                "type": "event.log",
                "level": "system",
                "event_type": "gui_node_shutdown",
                "message": "ROS GUI bridge shutting down.",
                "timestamp": now_iso(),
            }
        )

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = GuiNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().warn("gui_node stopped by keyboard interrupt.")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()