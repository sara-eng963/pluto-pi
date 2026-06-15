#!/usr/bin/env python3

import asyncio
import json
import threading
import time
from datetime import datetime, timezone
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D, Twist
from std_msgs.msg import Bool, Float32, Int32, String

try:
    import websockets
except ImportError as exc:
    raise ImportError(
        "Missing Python package 'websockets'.\n"
        "  sudo apt install python3-websockets\n"
        "  or: pip install websockets"
    ) from exc


def now_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.000Z")


def compact_json(data: Dict[str, Any]) -> str:
    return json.dumps(data, separators=(",", ":"), ensure_ascii=False)


class GuiNode(Node):
    def __init__(self):
        super().__init__("gui_node")

        # ──────────────────────────────────────────────────────────────────
        # Parameters
        # ──────────────────────────────────────────────────────────────────
        self.declare_parameter(
            "backend_ws_url",
            "ws://127.0.0.1:8000/ros_bridge/ws",   # pi_server endpoint
        )
        self.backend_ws_url = (
            self.get_parameter("backend_ws_url")
            .get_parameter_value()
            .string_value
        )

        # Periodic GUI heartbeats
        self.robot_status_timer  = self.create_timer(2.0,  self.robot_status_heartbeat_callback)
        self.system_health_timer = self.create_timer(10.0, self.system_health_timer_callback)
        self.battery_timer       = self.create_timer(30.0, self.battery_timer_callback)

        # ──────────────────────────────────────────────────────────────────
        # ROS publishers  (GUI → ROS)
        # ──────────────────────────────────────────────────────────────────
        self.order_request_pub        = self.create_publisher(String, "/mission/order_request",        10)
        self.mission_control_pub      = self.create_publisher(String, "/mission/control",              10)
        self.rfid_verification_pub    = self.create_publisher(String, "/mission/rfid_verification",    10)
        self.storage_close_request_pub = self.create_publisher(String, "/mission/storage_close_request", 10)
        self.cmd_vel_pub              = self.create_publisher(Twist,  "/cmd_vel",                      10)  # Gap 1

        # ──────────────────────────────────────────────────────────────────
        # ROS subscribers  (ROS → Flutter)
        # ──────────────────────────────────────────────────────────────────
        self.navigation_result_sub = self.create_subscription(
            String,  "/navigation_result",  self.navigation_result_callback,  10)
        self.detected_fruit_sub = self.create_subscription(
            String,  "/detected_fruit",     self.detected_fruit_callback,     10)
        self.detected_confidence_sub = self.create_subscription(
            Float32, "/detected_confidence", self.detected_confidence_callback, 10)
        self.valid_sub = self.create_subscription(
            Bool,    "/valid",              self.valid_callback,              10)
        self.obstacle_status_sub = self.create_subscription(
            Int32,   "/obstacle_status",    self.obstacle_status_callback,    10)
        self.drive_status_sub = self.create_subscription(
            String,  "/drive_status",       self.drive_status_callback,       10)
        self.mission_state_sub = self.create_subscription(
            String,  "/mission/state",      self.mission_state_callback,      10)
        self.mission_event_sub = self.create_subscription(
            String,  "/mission/event",      self.mission_event_callback,      10)
        self.navigation_status_sub = self.create_subscription(
            String,  "/navigation/status",  self.navigation_status_callback,  10)
        self.navigation_pose_sub = self.create_subscription(
            Pose2D,  "/navigation/pose",    self.navigation_pose_callback,    10)
        self.esp2_voltage_sub = self.create_subscription(
            Float32, "/esp2/voltage",       self.esp2_voltage_callback,       10)
        # Gap 4 – subscribe to /robot/mode so latest_mode stays current
        self.robot_mode_sub = self.create_subscription(
            String,  "/robot/mode",         self.robot_mode_callback,         10)

        # ──────────────────────────────────────────────────────────────────
        # Cached ROS state
        # ──────────────────────────────────────────────────────────────────
        self.latest_detected_fruit       = "Unknown"
        self.latest_detected_confidence  = 0.0
        self.latest_valid                = False
        self.latest_mission_state        = "idle"
        self.latest_active_order_id      = ""
        self.latest_fault_type           = "none"
        self.latest_current_fruit        = ""
        self.latest_storage_state        = "closed"
        self.latest_obstacle_detected    = False
        self.latest_distance_remaining   = 0.0
        self.latest_mode                 = "autonomous"   # Gap 4 – updated by /robot/mode

        self.latest_battery_voltage  = 11.4
        self.latest_battery_percent  = 82
        self.latest_is_charging      = False

        self.latest_rfid_card_id  = ""   # Gap 3 – populated from user.session
        self.latest_rfid_order_id = ""

        self.start_time_monotonic     = time.monotonic()
        self.last_ros_message_time    = time.monotonic()
        self.last_drive_status_time   = 0.0
        self.last_vision_time         = 0.0
        self.last_obstacle_time       = 0.0
        self.last_drive_status_gui_time = 0.0
        self.drive_status_throttle_sec  = 1.0

        # ──────────────────────────────────────────────────────────────────
        # WebSocket thread
        # ──────────────────────────────────────────────────────────────────
        self.ws_loop:  Optional[asyncio.AbstractEventLoop] = None
        self.ws_queue: Optional[asyncio.Queue]             = None
        self.ws_connected = False
        self.stop_event   = threading.Event()

        self.ws_thread = threading.Thread(target=self._ws_thread_main, daemon=True)
        self.ws_thread.start()

        self.get_logger().info("GUI bridge node started  (Phase 2 – Pi relay).")
        self.get_logger().info(f"Backend WS: {self.backend_ws_url}")

    # ══════════════════════════════════════════════════════════════════════
    # WebSocket lifecycle
    # ══════════════════════════════════════════════════════════════════════

    def _ws_thread_main(self):
        self.ws_loop  = asyncio.new_event_loop()
        asyncio.set_event_loop(self.ws_loop)
        self.ws_queue = asyncio.Queue()
        try:
            self.ws_loop.run_until_complete(self._ws_main())
        except Exception as exc:
            self.get_logger().error(f"WebSocket loop crashed: {exc}")
        finally:
            self.ws_loop.close()

    async def _ws_main(self):
        while rclpy.ok() and not self.stop_event.is_set():
            try:
                self.get_logger().info(f"Connecting → {self.backend_ws_url}")

                async with websockets.connect(self.backend_ws_url) as ws:
                    self.ws_connected = True
                    self.get_logger().info("Connected to pi_server /ros_bridge/ws.")

                    sender   = asyncio.create_task(self._ws_sender(ws))
                    receiver = asyncio.create_task(self._ws_receiver(ws))

                    done, pending = await asyncio.wait(
                        {sender, receiver},
                        return_when=asyncio.FIRST_EXCEPTION,
                    )
                    for t in pending:
                        t.cancel()
                    for t in done:
                        exc = t.exception()
                        if exc:
                            raise exc

            except asyncio.CancelledError:
                break
            except Exception as exc:
                self.ws_connected = False
                self.get_logger().warn(f"/ros_bridge/ws error: {exc}")
                await asyncio.sleep(2.0)

        self.ws_connected = False

    async def _ws_sender(self, ws):
        while rclpy.ok() and not self.stop_event.is_set():
            msg  = await self.ws_queue.get()
            text = compact_json(msg)
            await ws.send(text)

    async def _ws_receiver(self, ws):
        async for raw in ws:
            try:
                payload = json.loads(raw)
            except json.JSONDecodeError:
                self.get_logger().warn(f"Non-JSON from pi_server: {raw[:80]}")
                continue
            self._handle_incoming(payload)

    def send_to_flutter(self, msg: Dict[str, Any]):
        """Thread-safe enqueue → WebSocket sender → pi_server → Flutter."""
        if self.ws_loop is None or self.ws_queue is None:
            return
        try:
            asyncio.run_coroutine_threadsafe(self.ws_queue.put(msg), self.ws_loop)
        except RuntimeError:
            pass

    # ══════════════════════════════════════════════════════════════════════
    # Incoming: pi_server → gui_node  (originally from Flutter or HTTP)
    # ══════════════════════════════════════════════════════════════════════

    def _handle_incoming(self, msg: Dict[str, Any]):
        """
        pi_server sends raw Flutter messages directly.
        No log-entry wrapper to unwrap (unlike sim_server /dashboard/ws).
        """
        msg_type = msg.get("type", "")

        # Handshake / internal pi_server messages
        if msg_type in ("bridge.connected", "bridge.status_request"):
            if msg_type == "bridge.status_request":
                self._send_robot_status()
            return

        self._handle_gui_message(msg)

    def _pub_string(self, publisher, data: str, topic: str):
        m      = String()
        m.data = data
        publisher.publish(m)
        self.get_logger().info(f"PUB {topic}: {data[:120]}")

    def _handle_gui_message(self, msg: Dict[str, Any]):
        msg_type = msg.get("type", "")

        # ── Gap 1: Teleop → /cmd_vel ──────────────────────────────────────
        # Flutter sends integer flags: vx (-1/0/1), vy (-1/0/1), w (-1/0/1)
        # The Pi drive node applies its own speed scaling.
        if msg_type == "teleop.command":
            twist           = Twist()
            twist.linear.x  = float(msg.get("vx", 0))
            twist.linear.y  = float(msg.get("vy", 0))
            twist.angular.z = float(msg.get("w",  0))
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info(
                f"PUB /cmd_vel  dir={msg.get('direction','?')}  "
                f"vx={int(twist.linear.x)}  vy={int(twist.linear.y)}  w={int(twist.angular.z)}"
            )
            return

        # ── Gap 2: Clear faults → /mission/control ────────────────────────
        if msg_type == "worker.clear_faults":
            self._pub_string(
                self.mission_control_pub,
                compact_json({"command": "CLEAR_FAULTS", "source": "gui", "timestamp": now_iso()}),
                "/mission/control",
            )
            return

        # ── Orders / mission ──────────────────────────────────────────────
        if msg_type == "order.place":
            order = dict(msg)
            if not order.get("order_id"):
                order["order_id"] = f"pi_order_{int(time.time() * 1000)}"
            self._pub_string(self.order_request_pub, compact_json(order), "/mission/order_request")
            return

        if msg_type in ("mission.stop", "order.cancel"):
            self._pub_string(
                self.mission_control_pub,
                compact_json({
                    "command":   "STOP",
                    "source":    "gui",
                    "order_id":  msg.get("order_id", ""),
                    "reason":    msg.get("reason", msg_type),
                    "timestamp": now_iso(),
                }),
                "/mission/control",
            )
            return

        if msg_type == "debug.reset":
            self._pub_string(
                self.mission_control_pub,
                compact_json({"command": "RESET", "source": "gui", "timestamp": now_iso()}),
                "/mission/control",
            )
            return

        # ── Robot mode (from HTTP /robot/mode → pi_server → here) ─────────
        if msg_type == "robot.set_mode":
            self._pub_string(
                self.mission_control_pub,
                compact_json({
                    "command":   "SET_MODE",
                    "mode":      msg.get("mode", "autonomous"),
                    "source":    "gui",
                    "timestamp": now_iso(),
                }),
                "/mission/control",
            )
            return

        # ── RFID ──────────────────────────────────────────────────────────
        if msg_type in ("debug.rfid_simulate", "rfid.verification"):
            rfid = {
                "source":      "dashboard" if msg_type == "debug.rfid_simulate" else "gui",
                "order_id":    msg.get("order_id", ""),
                "rfid_card_id": msg.get("rfid_card_id", ""),
                "success":     bool(msg.get("should_succeed", True)),
                "timestamp":   now_iso(),
            }
            self.latest_rfid_card_id  = rfid["rfid_card_id"]
            self.latest_rfid_order_id = rfid["order_id"]
            self._pub_string(self.rfid_verification_pub, compact_json(rfid), "/mission/rfid_verification")
            return

        # ── Storage ───────────────────────────────────────────────────────
        if msg_type == "storage.close_request":
            self._pub_string(
                self.storage_close_request_pub,
                msg.get("order_id", ""),
                "/mission/storage_close_request",
            )
            return

        if msg_type == "storage.open_request":
            self._pub_string(
                self.mission_control_pub,
                compact_json({
                    "command":   "STORAGE_OPEN",
                    "source":    "gui",
                    "order_id":  msg.get("order_id", ""),
                    "timestamp": now_iso(),
                }),
                "/mission/control",
            )
            return

        # ── Gap 3: user.session – cache rfid_card_id ──────────────────────
        if msg_type == "user.session":
            is_logout = msg.get("is_logout", False)
            rfid      = msg.get("rfid_card_id", "")
            username  = msg.get("username", "?")
            if not is_logout and rfid:
                self.latest_rfid_card_id = rfid
                self.get_logger().info(f"[Session] Cached RFID for {username}: {rfid}")
            else:
                self.get_logger().info(f"[Session] Logout for {username}.")
            return

        # ── Gap 5: Rejected unsafe commands ───────────────────────────────
        rejected = {
            "debug.obstacle_inject",
            "debug.obstacle_release",
            "debug.force_state",
            "debug.battery_drain",
            "debug.battery_charge",
            "debug.vision_simulate",    # Gap 5 – was falling through
        }
        if msg_type in rejected:
            self.get_logger().warn(f"Rejected unsafe GUI command: {msg_type}")
            self.send_to_flutter({
                "type":        "event.log",
                "level":       "warning",
                "event_type":  "gui_command_rejected",
                "message":     f"Rejected unsafe GUI command: {msg_type}",
                "timestamp":   now_iso(),
            })
            return

        # ── Pass-through (backend-owned) ──────────────────────────────────
        ignored = {"payment.request", "payment.status", "mission.start"}
        if msg_type in ignored:
            self.get_logger().info(f"Ignored backend-owned message: {msg_type}")
            return

        if msg_type:
            self.get_logger().info(f"Unhandled message type: {msg_type}")

    # ══════════════════════════════════════════════════════════════════════
    # Outgoing helpers  (ROS state → send_to_flutter → pi_server → Flutter)
    # ══════════════════════════════════════════════════════════════════════

    def _battery_percent_from_voltage(self, voltage: float) -> int:
        """Simple linear estimate: 7.5 V = 0%, 12.6 V = 100%."""
        return max(0, min(100, int(round(((voltage - 7.5) / (12.6 - 7.5)) * 100.0))))

    def _send_robot_status(self):
        self.send_to_flutter({
            "type":               "robot.status",
            "battery_percent":    int(self.latest_battery_percent),
            "is_charging":        bool(self.latest_is_charging),
            "mode":               self.latest_mode,           # Gap 4 – no longer hardcoded
            "mission_state":      self.latest_mission_state,
            "storage_state":      self.latest_storage_state,
            "fault_type":         self.latest_fault_type,
            "active_order_id":    self.latest_active_order_id,
            "linear_speed":       0.0,
            "angular_speed":      0.0,
            "distance_remaining": float(self.latest_distance_remaining),
            "obstacle_detected":  bool(self.latest_obstacle_detected),
            "current_fruit":      self.latest_current_fruit.lower() if self.latest_current_fruit else "",
            "timestamp":          now_iso(),
        })

    def _send_order_status_update(self, order_id: str, event: str, message: str):
        if not order_id:
            return
        status_map = {
            "orderReceived":    "accepted",
            "missionReceived":  "accepted",
            "headingToFruit":   "navigating_to_fruit",
            "visionChecking":   "checking_stock",
            "storing":          "collecting_fruit",
            "headingToCustomer":"delivering",
            "rfidAwaiting":     "waiting_for_rfid",
            "storageOpened":    "ready_for_pickup",
            "storageClosed":    "collected",
            "returning":        "returning",
            "idle":             "completed",
            "error":            "failed",
            "failed":           "failed",
        }
        self.send_to_flutter({
            "type":      "order.status_update",
            "order_id":  order_id,
            "status":    status_map.get(event, event),
            "message":   message,
            "timestamp": now_iso(),
        })

    def _send_storage_messages_for_event(self, order_id: str, event: str):
        if not order_id:
            return
        if event == "storing":
            self.send_to_flutter({
                "type": "storage.status", "state": "opening",
                "order_id": order_id, "timestamp": now_iso(),
            })
        elif event == "storageOpened":
            self.send_to_flutter({
                "type": "storage.status", "state": "open",
                "order_id": order_id, "timestamp": now_iso(),
            })
            self.send_to_flutter({
                "type": "storage.open", "state": "open",
                "order_id": order_id, "timestamp": now_iso(),
            })
        elif event == "storageClosed":
            self.send_to_flutter({
                "type": "storage.status", "state": "closed",
                "order_id": order_id, "timestamp": now_iso(),
            })
            self.send_to_flutter({
                "type": "storage.closed", "state": "closing",
                "order_id": order_id, "timestamp": now_iso(),
            })

    def _send_rfid_result_for_event(self, order_id: str, event: str, message: str):
        if event == "storageOpened":
            self.send_to_flutter({
                "type": "rfid.result", "success": True,
                "rfid_card_id": self.latest_rfid_card_id,
                "order_id": order_id, "message": "Identity verified ✓",
                "timestamp": now_iso(),
            })
        elif event in ("error", "failed") and self.latest_fault_type == "rfidFailed":
            self.send_to_flutter({
                "type": "rfid.result", "success": False,
                "rfid_card_id": self.latest_rfid_card_id,
                "order_id": order_id,
                "message": message or "RFID verification failed.",
                "timestamp": now_iso(),
            })

    def _send_battery_telemetry(self):
        self.send_to_flutter({
            "type":                        "telemetry.battery",
            "battery_percent":             int(self.latest_battery_percent),
            "is_charging":                 bool(self.latest_is_charging),
            "voltage":                     round(float(self.latest_battery_voltage), 2),
            "estimated_minutes_remaining": int(max(0, self.latest_battery_percent * 2)),
            "timestamp":                   now_iso(),
        })

    def _send_system_health(self):
        now = time.monotonic()
        self.send_to_flutter({
            "type":              "system.health",
            "level":             "nominal",
            "uptime_seconds":    int(now - self.start_time_monotonic),
            "ros2_active":       True,
            "micro_ros_active":  (now - self.last_drive_status_time) < 5.0
                                 if self.last_drive_status_time else False,
            "camera_active":     (now - self.last_vision_time) < 5.0
                                 if self.last_vision_time else False,
            "lidar_active":      (now - self.last_obstacle_time) < 5.0
                                 if self.last_obstacle_time else False,
            "timestamp":         now_iso(),
        })

    # ══════════════════════════════════════════════════════════════════════
    # ROS subscribers
    # ══════════════════════════════════════════════════════════════════════

    def robot_mode_callback(self, msg: String):
        """Gap 4 – keep latest_mode in sync with /robot/mode topic."""
        mode = msg.data.strip()
        if mode:
            self.latest_mode = mode
            self.get_logger().info(f"Robot mode updated: {mode}")

    def mission_state_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Invalid /mission/state JSON: {msg.data}")
            return

        self.latest_mission_state     = data.get("mission_state", "idle")
        self.latest_active_order_id   = data.get("order_id",      "")
        self.latest_current_fruit     = data.get("fruit",         "")
        self.latest_fault_type        = data.get("fault_type",    "none")

        storage_state = "closed"
        if self.latest_mission_state == "storageOpened":
            storage_state = "open"
        self.latest_storage_state = storage_state

        self.last_ros_message_time = time.monotonic()
        self._send_robot_status()

    def mission_event_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Invalid /mission/event JSON: {msg.data}")
            return

        order_id         = data.get("order_id",         "")
        event            = data.get("event",             "")
        message          = data.get("message",           "")
        progress_percent = int(data.get("progress_percent", 0))

        # Primary mission timeline event (drives Flutter order-tracking UI)
        self.send_to_flutter({
            "type":             "mission.event",
            "order_id":         order_id,
            "event":            event,
            "message":          message,
            "progress_percent": progress_percent,
            "timestamp":        now_iso(),
        })
        # Human-readable event log
        self.send_to_flutter({
            "type":       "event.log",
            "level":      "mission",
            "event_type": event,
            "message":    message,
            "order_id":   order_id,
            "timestamp":  now_iso(),
        })

        self.last_ros_message_time = time.monotonic()
        self._send_order_status_update(order_id, event, message)
        self._send_storage_messages_for_event(order_id, event)
        self._send_rfid_result_for_event(order_id, event, message)

    def navigation_status_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        status  = data.get("status", "")
        goal    = data.get("goal", {}) or {}
        state   = "blocked" if status in ("FAILED", "STOPPED") else "clear"

        self.last_ros_message_time = time.monotonic()
        self.send_to_flutter({
            "type":              "navigation.path_status",
            "state":             state,
            "target_x":         float(goal.get("x", 0.0)),
            "target_y":         float(goal.get("y", 0.0)),
            "distance_to_goal": float(self.latest_distance_remaining),
            "estimated_seconds": 0,
            "timestamp":         now_iso(),
        })

    def navigation_pose_callback(self, msg: Pose2D):
        self.last_ros_message_time = time.monotonic()
        self.send_to_flutter({
            "type":            "navigation.pose",
            "x":               float(msg.x),
            "y":               float(msg.y),
            "heading_degrees": float(msg.theta),
            "timestamp":       now_iso(),
        })

    def navigation_result_callback(self, msg: String):
        self.send_to_flutter({
            "type":       "event.log",
            "level":      "navigation",
            "event_type": "navigation_result",
            "message":    msg.data.strip(),
            "timestamp":  now_iso(),
        })

    def esp2_voltage_callback(self, msg: Float32):
        voltage = float(msg.data)
        self.latest_battery_voltage = voltage
        self.latest_battery_percent = self._battery_percent_from_voltage(voltage)
        self.last_ros_message_time  = time.monotonic()
        self._send_battery_telemetry()

    def drive_status_callback(self, msg: String):
        self.last_ros_message_time  = time.monotonic()
        self.last_drive_status_time = time.monotonic()
        text = msg.data.strip()

        # Throttle high-rate STATUS messages
        if text.startswith("STATUS"):
            now = time.monotonic()
            if now - self.last_drive_status_gui_time < self.drive_status_throttle_sec:
                return
            self.last_drive_status_gui_time = now

        self.send_to_flutter({
            "type":       "event.log",
            "level":      "drive",
            "event_type": "drive_status",
            "message":    text,
            "timestamp":  now_iso(),
        })

    def obstacle_status_callback(self, msg: Int32):
        self.last_ros_message_time = time.monotonic()
        self.last_obstacle_time    = time.monotonic()
        mask     = int(msg.data)
        blocking = mask != 0
        self.latest_obstacle_detected = blocking
        self.send_to_flutter({
            "type":            "navigation.obstacle_status",
            "state":           "detected" if blocking else "clear",
            "raw_mask":        mask,
            "distance_meters": None,
            "angle_degrees":   None,
            "is_blocking_path": blocking,
            "timestamp":       now_iso(),
        })

    def detected_fruit_callback(self, msg: String):
        self.last_ros_message_time = time.monotonic()
        self.last_vision_time      = time.monotonic()
        self.latest_detected_fruit = msg.data.strip()
        self._publish_cached_vision()

    def detected_confidence_callback(self, msg: Float32):
        self.last_ros_message_time       = time.monotonic()
        self.last_vision_time            = time.monotonic()
        self.latest_detected_confidence  = float(msg.data)
        self._publish_cached_vision()

    def valid_callback(self, msg: Bool):
        self.last_ros_message_time = time.monotonic()
        self.last_vision_time      = time.monotonic()
        self.latest_valid          = bool(msg.data)
        self._publish_cached_vision()

    def _publish_cached_vision(self):
        fruit = self.latest_detected_fruit
        self.send_to_flutter({
            "type":        "vision.fruit_detection",
            "fruit":       fruit,
            "detected":    fruit not in ("", "Unknown"),
            "confidence":  round(float(self.latest_detected_confidence), 3),
            "valid":       bool(self.latest_valid),
            "bounding_box": None,
            "timestamp":   now_iso(),
        })

    # ──────────────────────────────────────────────────────────────────────
    # Heartbeat callbacks
    # ──────────────────────────────────────────────────────────────────────

    def robot_status_heartbeat_callback(self):
        self._send_robot_status()

    def system_health_timer_callback(self):
        self._send_system_health()

    def battery_timer_callback(self):
        self._send_battery_telemetry()

    # ══════════════════════════════════════════════════════════════════════
    # Shutdown
    # ══════════════════════════════════════════════════════════════════════

    def destroy_node(self):
        self.stop_event.set()
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
