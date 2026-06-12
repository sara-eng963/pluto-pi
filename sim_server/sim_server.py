#!/usr/bin/env python3
"""
Endpoints:
    GET  /                 → Dashboard HTML
    GET  /products         → Product list (JSON)
    POST /auth/login       → Login (JSON)
    POST /orders           → Place order → auto-starts mission
    POST /robot/mode       → Set robot mode
    POST /robot/storage/open
    POST /robot/storage/close
    WS   /ws               → Flutter client (send/receive JSON)
    WS   /dashboard/ws     → Dashboard control (send/receive JSON)
"""

import asyncio
import hashlib
import json
import uuid
from datetime import datetime, timezone
from typing import Optional

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse, JSONResponse

# App Setup

app = FastAPI(title="Pluto Robot Sim Server", version="1.0.0")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# Helpers 

def _sha256(s: str) -> str:
    return hashlib.sha256(s.encode("utf-8")).hexdigest()

def _now() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.000Z")

def _uid() -> str:
    return uuid.uuid4().hex[:8]

# Users  (mirrors MockAuthDataSource) 

_USERS_RAW = [
    ("user_001", "omar",   "12345678", "Omar",  "customer", "RFID_OM4R99", 500.0),
    ("user_002", "shady",  "12345678", "Shady", "customer", "RFID_SH4DY7", 500.0),
    ("user_003", "kaboo",  "12345678", "Kaboo", "worker",   "RFID_K4B0099",  0.0),
]

USERS: dict[str, dict] = {
    username: {
        "id": uid, "username": username, "password_hash": _sha256(pw),
        "name": name, "role": role, "rfid_card_id": rfid, "credits": credits,
    }
    for uid, username, pw, name, role, rfid, credits in _USERS_RAW
}

# Mutable credits (changes during session)
user_credits: dict[str, float] = {u: USERS[u]["credits"] for u in USERS}
# Map rfid_card_id → username for RFID verification
rfid_to_user: dict[str, str] = {USERS[u]["rfid_card_id"]: u for u in USERS}

# Products  (mirrors MockProductDataSource) 

PRODUCTS: list[dict] = [
    {"id": "prod_001", "name": "Apple",  "price": 5.0, "stock": 20, "image_url": "", "unit": "kg", "category": "Classic", "description": "Crisp & Juicy",  "emoji": "🍎"},
    {"id": "prod_002", "name": "Orange", "price": 4.5, "stock": 20, "image_url": "", "unit": "kg", "category": "Citrus",  "description": "Sweet & Fresh",  "emoji": "🍊"},
    {"id": "prod_003", "name": "Kiwi",   "price": 6.0, "stock": 20, "image_url": "", "unit": "kg", "category": "Tropical","description": "Tangy & Sweet", "emoji": "🥝"},
]

product_stock: dict[str, int] = {p["id"]: p["stock"] for p in PRODUCTS}

# Robot State

class RobotState:
    def __init__(self):
        self.battery: int = 82
        self.is_charging: bool = False
        self.mode: str = "autonomous"
        self.mission_state: str = "idle"
        self.storage_state: str = "closed"
        self.fault_type: str = "none"
        self.active_order_id: Optional[str] = None
        self.linear_speed: float = 0.0
        self.angular_speed: float = 0.0
        self.distance_remaining: float = 0.0
        self.obstacle_detected: bool = False
        self.current_fruit: Optional[str] = None
        # Obstacle control
        self.paused: bool = False
        # RFID control — set in startup (needs event loop)
        self.rfid_event: Optional[asyncio.Event] = None
        self.rfid_success: bool = False
        self.rfid_card_id: str = ""

        # Active Flutter session (populated by user.session message)
        self.active_session_user_id:  Optional[str] = None
        self.active_session_username: str = ""
        self.active_session_rfid:     str = ""
        self.active_session_credits:  float = 0.0
        self.active_session_token:    str = ""

    def status_msg(self) -> dict:
        return {
            "type": "robot.status",
            "battery_percent": self.battery,
            "is_charging": self.is_charging,
            "mode": self.mode,
            "mission_state": self.mission_state,
            "storage_state": self.storage_state,
            "fault_type": self.fault_type,
            "active_order_id": self.active_order_id,
            "linear_speed": self.linear_speed,
            "angular_speed": self.angular_speed,
            "distance_remaining": round(self.distance_remaining, 2),
            "obstacle_detected": self.obstacle_detected,
            "current_fruit": self.current_fruit,
            "timestamp": _now(),
        }

    def to_ui(self) -> dict:
        """Compact dict for dashboard UI display."""
        return {
            "battery": self.battery,
            "mode": self.mode,
            "mission_state": self.mission_state,
            "storage_state": self.storage_state,
            "fault_type": self.fault_type,
            "active_order_id": self.active_order_id,
            "distance_remaining": round(self.distance_remaining, 2),
            "obstacle_detected": self.obstacle_detected,
            "current_fruit": self.current_fruit,
            "paused": self.paused,
        }

robot = RobotState()
mission_task: Optional[asyncio.Task] = None

# Connection Management 

flutter_clients: set[WebSocket] = set()
dashboard_clients: set[WebSocket] = set()
message_log: list[dict] = []   # rolling 200-entry log

#  Broadcast Helpers 

async def _send_safe(ws: WebSocket, text: str) -> bool:
    """Try to send to a client. Returns False if dead."""
    try:
        await ws.send_text(text)
        return True
    except Exception:
        return False

async def broadcast(msg: dict):
    """Send JSON msg to all Flutter clients and log it."""
    text = json.dumps(msg, ensure_ascii=False)
    entry = {"dir": "OUT", "msg": msg, "ts": _now()}
    _log(entry)

    dead = {ws for ws in flutter_clients if not await _send_safe(ws, text)}
    flutter_clients.difference_update(dead)

    await _push_log_to_dashboard(entry)

async def _push_log_to_dashboard(entry: dict):
    """Push a log entry + robot state to all dashboard clients."""
    payload = json.dumps({"type": "log", "entry": entry, "robot": robot.to_ui()})
    dead = {ws for ws in dashboard_clients if not await _send_safe(ws, payload)}
    dashboard_clients.difference_update(dead)

async def dash_push(msg: dict):
    """Send directly to dashboard only (control confirmations)."""
    text = json.dumps(msg)
    dead = {ws for ws in dashboard_clients if not await _send_safe(ws, text)}
    dashboard_clients.difference_update(dead)

def _log(entry: dict):
    message_log.append(entry)
    if len(message_log) > 200:
        message_log.pop(0)

# Mission State Machine

async def _navigate(total_seconds: float, start_distance: float, order_id: str):
    """
    Simulate movement over total_seconds, checking for obstacle pauses.
    Updates robot.distance_remaining and broadcasts status + path_status.
    """
    steps = int(total_seconds / 0.8)
    dist_per_step = start_distance / max(steps, 1)

    for i in range(steps):
        # Obstacle pause loop
        while robot.paused:
            robot.obstacle_detected = True
            robot.fault_type = "obstacleBlocked"
            robot.linear_speed = 0.0
            await broadcast(robot.status_msg())
            await broadcast({
                "type": "navigation.obstacle_status",
                "state": "detected",
                "distance_meters": 0.35,
                "angle_degrees": 0.0,
                "is_blocking_path": True,
                "timestamp": _now(),
            })
            await asyncio.sleep(0.5)

        # Cleared obstacle
        if robot.obstacle_detected:
            robot.obstacle_detected = False
            robot.fault_type = "none"
            # keep mission_state as-is (headingToFruit / headingToCustomer)
            robot.linear_speed = 0.25
            await broadcast({
                "type": "navigation.obstacle_status",
                "state": "clear",
                "distance_meters": 2.5,
                "angle_degrees": 0.0,
                "is_blocking_path": False,
                "timestamp": _now(),
            })

        robot.distance_remaining = max(0.0, start_distance - (i + 1) * dist_per_step)
        robot.linear_speed = 0.25 if robot.distance_remaining > 0 else 0.0

        await broadcast({
            "type": "navigation.path_status",
            "state": "clear",
            "target_x": 2.0,
            "target_y": 1.5,
            "distance_to_goal": round(robot.distance_remaining, 2),
            "estimated_seconds": max(0, int((steps - i - 1) * 0.8)),
            "timestamp": _now(),
        })
        await broadcast(robot.status_msg())
        await asyncio.sleep(0.8)

async def _emit_event(order_id: str, event: str, message: str, progress: int):
    await broadcast({
        "type": "mission.event",
        "order_id": order_id,
        "event": event,
        "message": message,
        "progress_percent": progress,
        "timestamp": _now(),
    })
    await broadcast({
        "type": "event.log",
        "level": "mission",
        "event_type": event,
        "message": message,
        "order_id": order_id,
        "timestamp": _now(),
    })
    await broadcast(robot.status_msg())

async def run_mission(order_id: str, items: list):
    global robot

    fruit_name = items[0].get("product_name", "Fruit") if items else "Fruit"
    robot.current_fruit = fruit_name.lower()
    robot.active_order_id = order_id
    robot.fault_type = "none"
    robot.paused = False

    try:
        #  1. Mission Received 
        robot.mission_state = "missionReceived"
        await _emit_event(order_id, "orderReceived", "Mission received! Processing order...", 5)
        # Hold missionReceived long enough for Flutter to navigate to the
        # tracking screen and render the first timeline step as green.
        # Re-broadcast status every second so late-joining clients catch it.
        for _ in range(4):
            await asyncio.sleep(1.0)
            await broadcast(robot.status_msg())

        #  2. Heading to Fruit 
        robot.mission_state = "headingToFruit"
        robot.linear_speed = 0.25
        robot.distance_remaining = 4.0
        await _emit_event(order_id, "headingToFruit", f"Robot heading to {fruit_name} stock area...", 15)
        await _navigate(total_seconds=8.0, start_distance=4.0, order_id=order_id)
        robot.linear_speed = 0.0
        robot.distance_remaining = 0.0

        #  3. Vision Checking 
        robot.mission_state = "visionChecking"
        await _emit_event(order_id, "visionChecking", "Vision module checking stock availability...", 30)
        await broadcast({
            "type": "vision.fruit_detection",
            "fruit": robot.current_fruit,
            "detected": True,
            "confidence": 0.94,
            "bounding_box": {"x": 118, "y": 82, "w": 58, "h": 52},
            "timestamp": _now(),
        })
        await asyncio.sleep(3.0)

        #  4. Storing 
        robot.mission_state = "storing"
        robot.storage_state = "opening"
        await _emit_event(order_id, "storing", f"Collecting {fruit_name} into storage box...", 45)
        await broadcast({
            "type": "storage.status",
            "state": "opening",
            "order_id": order_id,
            "timestamp": _now(),
        })
        await asyncio.sleep(3.5)
        robot.storage_state = "closed"

        #  5. Heading to Customer 
        robot.mission_state = "headingToCustomer"
        robot.linear_speed = 0.25
        robot.distance_remaining = 4.8
        await _emit_event(order_id, "headingToCustomer", "Robot heading to customer location!", 55)
        await _navigate(total_seconds=9.0, start_distance=4.8, order_id=order_id)
        robot.linear_speed = 0.0
        robot.distance_remaining = 0.0

        #  6. RFID Awaiting 
        robot.mission_state = "rfidAwaiting"
        robot.rfid_event = asyncio.Event()
        robot.rfid_success = False
        await _emit_event(order_id, "rfidAwaiting", "Please tap your RFID card on the robot.", 65)
        await broadcast({
            "type": "event.log",
            "level": "warning",
            "event_type": "rfidAwaiting",
            "message": "⏳ Waiting for RFID scan (60 s timeout)...",
            "order_id": order_id,
            "timestamp": _now(),
        })

        # Wait for RFID signal (timeout 60 s)
        try:
            await asyncio.wait_for(robot.rfid_event.wait(), timeout=60.0)
        except asyncio.TimeoutError:
            robot.mission_state = "failed"
            robot.fault_type = "rfidFailed"
            await _emit_event(order_id, "error", "RFID scan timed out. Mission failed.", 65)
            await _end_mission(order_id)
            return

        # RFID result
        if not robot.rfid_success:
            robot.mission_state = "failed"
            robot.fault_type = "rfidFailed"
            await broadcast({
                "type": "rfid.result",
                "success": False,
                "rfid_card_id": robot.rfid_card_id,
                "order_id": order_id,
                "message": "RFID verification failed.",
                "timestamp": _now(),
            })
            await _emit_event(order_id, "error", "RFID verification failed. Mission aborted.", 65)
            await _end_mission(order_id)
            return

        #  7. Storage Opened 
        robot.mission_state = "storageOpened"
        robot.fault_type = "none"
        robot.storage_state = "open"
        await broadcast({
            "type": "rfid.result",
            "success": True,
            "rfid_card_id": robot.rfid_card_id,
            "order_id": order_id,
            "message": "Identity verified ✓",
            "timestamp": _now(),
        })
        await broadcast({
            "type": "storage.open",
            "state": "open",
            "order_id": order_id,
            "timestamp": _now(),
        })
        await _emit_event(order_id, "storageOpened", "Storage opened. Please collect your order!", 75)
        await broadcast({
            "type": "storage.status",
            "state": "open",
            "order_id": order_id,
            "timestamp": _now(),
        })

        # Wait for customer "I took my order" signal (storage.close_request) — 120 s timeout
        robot.storage_close_event = asyncio.Event()
        try:
            await asyncio.wait_for(robot.storage_close_event.wait(), timeout=120.0)
        except asyncio.TimeoutError:
            pass  # auto-close after timeout

        # 8. Storage Closed 
        robot.mission_state = "storageClosed"
        robot.storage_state = "closing"
        await broadcast({
            "type": "storage.closed",
            "state": "closing",
            "order_id": order_id,
            "timestamp": _now(),
        })
        await _emit_event(order_id, "storageClosed", "Storage closed. Thank you! 🎉", 88)
        await asyncio.sleep(2.5)
        robot.storage_state = "closed"

        # 9. Returning 
        robot.mission_state = "returning"
        robot.linear_speed = 0.18
        await _emit_event(order_id, "returning", "Robot returning to home position.", 95)
        await asyncio.sleep(5.0)
        robot.linear_speed = 0.0
        robot.current_fruit = None

    except asyncio.CancelledError:
        pass
    finally:
        await _end_mission(order_id)

async def _end_mission(order_id: str):
    await asyncio.sleep(2.5)
    robot.mission_state = "idle"
    robot.active_order_id = None
    robot.linear_speed = 0.0
    robot.distance_remaining = 0.0
    robot.fault_type = "none"
    robot.current_fruit = None
    robot.paused = False
    await broadcast(robot.status_msg())
    await broadcast({
        "type": "event.log",
        "level": "system",
        "event_type": "idle",
        "message": "Robot returned to idle. Ready for next order.",
        "order_id": order_id,
        "timestamp": _now(),
    })

# Background Tasks 

async def battery_drain_task():
    """Drain 1% battery every 30 s. Broadcast telemetry.battery on each drain."""
    while True:
        await asyncio.sleep(30)
        if not robot.is_charging and robot.battery > 0:
            robot.battery -= 1
        if robot.battery <= 10:
            robot.fault_type = "criticalBattery"
        elif robot.battery <= 20:
            robot.fault_type = "lowBattery" if robot.fault_type == "none" else robot.fault_type

        await broadcast({
            "type": "telemetry.battery",
            "battery_percent": robot.battery,
            "is_charging": robot.is_charging,
            "voltage": round(9.0 + robot.battery * 0.03, 1),
            "estimated_minutes_remaining": robot.battery * 2,
            "timestamp": _now(),
        })

async def health_heartbeat_task():
    """Broadcast system.health every 10 s and robot.status every 2 s."""
    tick = 0
    while True:
        await asyncio.sleep(2)
        tick += 1
        if flutter_clients:
            await broadcast(robot.status_msg())
        if tick % 5 == 0:
            await broadcast({
                "type": "system.health",
                "level": "nominal",
                "cpu_percent": 28 + (tick % 15),
                "ram_percent": 45 + (tick % 10),
                "uptime_seconds": tick * 2,
                "ros2_active": True,
                "micro_ros_active": True,
                "camera_active": True,
                "lidar_active": True,
                "timestamp": _now(),
            })

# Incoming Message Handler 

async def handle_incoming(msg: dict, source: str = "flutter"):
    """
    Handle a JSON message from Flutter client or dashboard.
    source = 'flutter' | 'dashboard'
    """
    global mission_task, robot

    msg_type = msg.get("type", "")

    # Log incoming
    entry = {"dir": "IN", "src": source, "msg": msg, "ts": _now()}
    _log(entry)
    await _push_log_to_dashboard(entry)

    # Mission / Order 

    if msg_type == "order.place":
        order_id = f"ord_{_uid()}"
        items = msg.get("items", [])
        _cancel_mission()
        mission_task = asyncio.create_task(run_mission(order_id, items))
        await dash_push({"type": "control_ack", "action": "mission_started", "order_id": order_id})

    elif msg_type == "order.cancel" or msg_type == "mission.stop":
        _cancel_mission()
        robot.mission_state = "idle"
        robot.fault_type = "missionCancelled"
        robot.active_order_id = None
        await broadcast(robot.status_msg())
        await broadcast({
            "type": "event.log", "level": "warning", "event_type": "error",
            "message": "Mission cancelled by user.", "order_id": msg.get("order_id", ""),
            "timestamp": _now(),
        })

    elif msg_type == "mission.start":
        # Re-start mission for existing order (from checkout flow)
        order_id = msg.get("order_id", _uid())
        _cancel_mission()
        mission_task = asyncio.create_task(run_mission(order_id, []))

    # RFID / Payment 

    elif msg_type == "rfid.verification":
        rfid_id = msg.get("rfid_card_id", "")
        robot.rfid_card_id = rfid_id
        robot.rfid_success = True
        if robot.rfid_event:
            robot.rfid_event.set()

    elif msg_type == "storage.close_request":
        # Customer pressed "I took my order" → unblock the storageOpened wait
        if hasattr(robot, "storage_close_event") and robot.storage_close_event:
            robot.storage_close_event.set()

    elif msg_type == "payment.request":
        # Simulate instant approval
        asyncio.create_task(_approve_payment(msg.get("order_id", ""), msg.get("amount", 0.0)))

    # Session

    elif msg_type == "user.session":
        is_logout = msg.get("is_logout", False)
        username  = msg.get("username", "")
        user_id   = msg.get("user_id", "")
        credits   = float(msg.get("credits", 0.0))
        token     = msg.get("session_token", "")

        if is_logout:
            # Clear active session info on the backend
            robot.active_session_user_id   = None
            robot.active_session_username  = ""
            robot.active_session_rfid      = ""
            robot.active_session_credits   = 0.0
            robot.active_session_token     = ""
            ack_msg = "Session cleared"
            active = False
            print(f"[AUTH] User logged out: {username}")
        else:
            # Store the active session so mission logic can use rfid / credits
            robot.active_session_user_id  = user_id
            robot.active_session_username = username
            robot.active_session_rfid     = msg.get("rfid_card_id", "")
            robot.active_session_credits  = credits
            robot.active_session_token    = token
            # Also sync live credits from the server's mutable store
            live_credits = user_credits.get(username, credits)
            robot.active_session_credits  = live_credits
            ack_msg = f"Session registered for {msg.get('name', username)}"
            active = True
            print(f"[AUTH] User session registered: {username}  credits={live_credits}")

        # Reply with user.session_ack (Flutter can refresh its credit balance)
        await broadcast({
            "type": "user.session_ack",
            "user_id": user_id,
            "username": username,
            "credits": robot.active_session_credits if active else 0.0,
            "active_session": active,
            "message": ack_msg,
            "timestamp": _now(),
        })
        await dash_push({
            "type": "control_ack",
            "action": "session_updated",
            "username": username,
            "active": active,
        })

    # Debug Controls 

    elif msg_type == "debug.obstacle_inject":
        robot.paused = True
        await dash_push({"type": "control_ack", "action": "obstacle_injected"})

    elif msg_type == "debug.obstacle_release":
        robot.paused = False
        robot.obstacle_detected = False
        robot.fault_type = "none"
        # keep current mission_state — it was headingToFruit or headingToCustomer
        await broadcast(robot.status_msg())
        await dash_push({"type": "control_ack", "action": "obstacle_released"})

    elif msg_type == "debug.rfid_simulate":
        should_succeed = msg.get("should_succeed", True)
        robot.rfid_card_id = msg.get("rfid_card_id", "RFID_A1B2C3")
        robot.rfid_success = should_succeed
        if robot.rfid_event:
            robot.rfid_event.set()
        await dash_push({"type": "control_ack", "action": "rfid_simulated", "success": should_succeed})

    elif msg_type == "debug.battery_drain":
        amount = int(msg.get("amount", 20))
        robot.battery = max(0, robot.battery - amount)
        await broadcast({
            "type": "telemetry.battery",
            "battery_percent": robot.battery,
            "is_charging": False,
            "voltage": round(9.0 + robot.battery * 0.03, 1),
            "estimated_minutes_remaining": robot.battery * 2,
            "timestamp": _now(),
        })
        await dash_push({"type": "control_ack", "action": "battery_drained", "battery": robot.battery})

    elif msg_type == "debug.battery_charge":
        robot.battery = min(100, robot.battery + int(msg.get("amount", 30)))
        robot.is_charging = False
        await broadcast(robot.status_msg())

    elif msg_type == "debug.force_state":
        state = msg.get("state", "idle")
        robot.mission_state = state
        await broadcast(robot.status_msg())
        # Also emit mission.event so Flutter timeline + event log both update
        order_id = robot.active_order_id or "debug"
        await _emit_event(order_id, state, f"[Force] State set to: {state}", 0)
        await dash_push({"type": "control_ack", "action": "state_forced", "state": state})

    elif msg_type == "debug.reset":
        _cancel_mission()
        robot.mission_state = "idle"
        robot.battery = 82
        robot.fault_type = "none"
        robot.active_order_id = None
        robot.linear_speed = 0.0
        robot.distance_remaining = 0.0
        robot.current_fruit = None
        robot.paused = False
        robot.obstacle_detected = False
        for pid in product_stock:
            orig = next((p["stock"] for p in PRODUCTS if p["id"] == pid), 0)
            product_stock[pid] = orig
        await broadcast(robot.status_msg())
        await dash_push({"type": "control_ack", "action": "full_reset"})

def _cancel_mission():
    global mission_task
    if mission_task and not mission_task.done():
        mission_task.cancel()
    robot.paused = False
    robot.rfid_event = asyncio.Event()

async def _approve_payment(order_id: str, amount: float):
    await asyncio.sleep(1.0)
    await broadcast({
        "type": "event.log",
        "level": "mission",
        "event_type": "paymentProcessing",
        "message": f"Payment of {amount:.2f} EGP approved ✓",
        "order_id": order_id,
        "timestamp": _now(),
    })

# WebSocket Endpoints 

@app.websocket("/ws")
async def flutter_ws(websocket: WebSocket):
    await websocket.accept()
    flutter_clients.add(websocket)
    print(f"[WS] Flutter client connected. Total: {len(flutter_clients)}")

    # Welcome burst
    await websocket.send_text(json.dumps({
        "type": "connection.status",
        "state": "connected",
        "latency_ms": 1,
        "timestamp": _now(),
    }))
    await websocket.send_text(json.dumps(robot.status_msg()))

    await _push_log_to_dashboard({
        "dir": "SYS", "msg": {"info": f"Flutter client connected ({len(flutter_clients)} total)"}, "ts": _now()
    })

    try:
        while True:
            raw = await websocket.receive_text()
            try:
                await handle_incoming(json.loads(raw), source="flutter")
            except json.JSONDecodeError:
                pass
    except WebSocketDisconnect:
        flutter_clients.discard(websocket)
        print(f"[WS] Flutter client disconnected. Total: {len(flutter_clients)}")
        await _push_log_to_dashboard({
            "dir": "SYS", "msg": {"info": f"Flutter client disconnected ({len(flutter_clients)} total)"}, "ts": _now()
        })

@app.websocket("/dashboard/ws")
async def dashboard_ws_handler(websocket: WebSocket):
    await websocket.accept()
    dashboard_clients.add(websocket)
    print(f"[WS] Dashboard connected. Total: {len(dashboard_clients)}")

    # Send full init payload (replay last 80 log entries + robot state)
    await websocket.send_text(json.dumps({
        "type": "init",
        "robot": robot.to_ui(),
        "flutter_clients": len(flutter_clients),
        "log": message_log[-80:],
    }))

    try:
        while True:
            raw = await websocket.receive_text()
            try:
                await handle_incoming(json.loads(raw), source="dashboard")
            except json.JSONDecodeError:
                pass
    except WebSocketDisconnect:
        dashboard_clients.discard(websocket)

# HTTP Endpoints 

@app.post("/auth/login")
async def login(request: Request):
    body = await request.json()
    username = body.get("username", "").strip()
    password = body.get("password", "").strip()

    user = USERS.get(username)
    if not user or user["password_hash"] != _sha256(password):
        raise HTTPException(status_code=401, detail="Invalid username or password")

    user_data = {k: v for k, v in user.items() if k != "password_hash"}
    user_data["credits"] = user_credits.get(username, user["credits"])

    return {
        "session_token": f"tok_{uuid.uuid4().hex[:20]}",
        "user": user_data,
    }

@app.get("/products")
async def get_products():
    result = []
    for p in PRODUCTS:
        item = dict(p)
        item["stock"] = product_stock.get(p["id"], p["stock"])
        result.append(item)
    return result

@app.post("/orders")
async def create_order(request: Request):
    global mission_task
    body = await request.json()

    user_id   = body.get("user_id", "")
    rfid      = body.get("assigned_rfid", "")
    items_raw = body.get("items", [])

    # Validate user
    user = next((u for u in USERS.values() if u["id"] == user_id), None)
    if not user:
        raise HTTPException(status_code=404, detail="User not found")

    # Build order items + calculate total
    total = 0.0
    order_items = []
    for raw in items_raw:
        prod_id = raw.get("product_id", raw.get("productId", ""))
        qty     = int(raw.get("quantity", 1))
        prod    = next((p for p in PRODUCTS if p["id"] == prod_id), None)
        if not prod:
            raise HTTPException(status_code=404, detail=f"Product {prod_id} not found")
        if product_stock.get(prod_id, 0) < qty:
            raise HTTPException(status_code=422, detail=f"{prod['name']} is out of stock")
        subtotal = prod["price"] * qty
        total += subtotal
        product_stock[prod_id] -= qty
        order_items.append({
            "id": _uid(),
            "order_id": "",   # filled below
            "product_id": prod_id,
            "product_name": prod["name"],
            "unit_price": prod["price"],
            "quantity": qty,
        })

    # Check credits
    uname = user["username"]
    avail = user_credits.get(uname, user["credits"])
    if avail < total:
        raise HTTPException(status_code=422, detail="Insufficient credits")

    order_id = f"ord_{_uid()}"
    for item in order_items:
        item["order_id"] = order_id

    remaining = avail - total
    user_credits[uname] = remaining

    order = {
        "id": order_id,
        "user_id": user_id,
        "assigned_rfid": rfid,
        "items": order_items,
        "total_price": total,
        "status": "pending",
        "created_at": _now(),
    }

    # Kick off mission
    _cancel_mission()
    mission_task = asyncio.create_task(run_mission(order_id, order_items))

    return {"order": order, "remaining_credits": remaining}

@app.post("/robot/mode")
async def set_mode(request: Request):
    body = await request.json()
    robot.mode = body.get("mode", "autonomous")
    await broadcast(robot.status_msg())
    return {"success": True, "mode": robot.mode}

@app.post("/robot/storage/open")
async def open_storage(request: Request):
    body = await request.json()
    robot.storage_state = "open"
    await broadcast({
        "type": "storage.open",
        "state": "open",
        "order_id": body.get("order_id", robot.active_order_id),
        "timestamp": _now(),
    })
    return {"success": True, "storage_state": "open"}

@app.post("/robot/storage/close")
async def close_storage():
    robot.storage_state = "closed"
    await broadcast({
        "type": "storage.closed",
        "state": "closed",
        "order_id": robot.active_order_id,
        "timestamp": _now(),
    })
    return {"success": True, "storage_state": "closed"}

@app.get("/orders/{order_id}")
async def get_order(order_id: str):
    # Return a placeholder — Flutter just needs the order ID
    return {"id": order_id, "status": robot.mission_state}

@app.get("/health")
async def health():
    return {
        "status": "ok",
        "flutter_clients": len(flutter_clients),
        "dashboard_clients": len(dashboard_clients),
        "robot": robot.to_ui(),
    }

@app.get("/", response_class=HTMLResponse)
async def dashboard():
    try:
        with open("sim_dashboard.html", encoding="utf-8") as f:
            return HTMLResponse(f.read())
    except FileNotFoundError:
        return HTMLResponse("<h1>sim_dashboard.html not found in sim_server/ directory</h1>", status_code=404)

# Startup 

@app.on_event("startup")
async def startup():
    robot.rfid_event = asyncio.Event()
    asyncio.create_task(battery_drain_task())
    asyncio.create_task(health_heartbeat_task())
    print("=" * 52)
    print("  Pluto Robot Simulation Server")
    print("  Dashboard  : http://localhost:8000")
    print("  Flutter WS : ws://localhost:8000/ws")
    print("  Flutter API: http://localhost:8000")
    print("=" * 52)

# Entry Point

if __name__ == "__main__":
    uvicorn.run("sim_server:app", host="0.0.0.0", port=8000, reload=False, log_level="info")
