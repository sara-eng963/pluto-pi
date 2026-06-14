#!/usr/bin/env python3
"""
pi_server.py  –  Pluto Pi Relay Server v2.0

PURPOSE
-------
Production HTTP + WebSocket relay server for the real Raspberry Pi deployment.
Replaces sim_server.py on the robot.

This server does NOT simulate anything.
  • All HTTP endpoints handle auth / products / orders (book-keeping only).
  • Every Flutter message is forwarded raw to gui_node via /ros_bridge/ws.
  • Every message gui_node sends is broadcast raw to all Flutter clients via /ws.

ARCHITECTURE
------------
  Flutter App
      │  HTTP  (login, products, place-order)
      │  WS    /ws
      ▼
  pi_server.py   (this file, runs on Pi at 0.0.0.0:8000)
      │  WS    /ros_bridge/ws
      ▼
  gui_node.py    (ROS 2 node, also on Pi)
      │  ROS 2 topics
      ▼
  mission_node / nav / vision / ESP firmware …

ENDPOINTS
---------
  POST /auth/login
  GET  /products
  POST /orders          ← validates + forwards order.place to gui_node (NO simulation)
  POST /robot/mode      ← forwards robot.set_mode to gui_node
  POST /robot/storage/open   ← forwards storage.open_request to gui_node
  POST /robot/storage/close  ← forwards storage.close_request to gui_node
  GET  /health
  WS   /ws              ← Flutter clients
  WS   /ros_bridge/ws   ← gui_node

RUN
---
  pip install fastapi uvicorn websockets
  python3 pi_server.py
"""

import asyncio
import hashlib
import json
import uuid
from datetime import datetime, timezone
from typing import Set

import uvicorn
from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware

# ─── App ──────────────────────────────────────────────────────────────────────

app = FastAPI(title="Pluto Pi Server", version="2.0.0")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# ─── Helpers ──────────────────────────────────────────────────────────────────

def _sha256(s: str) -> str:
    return hashlib.sha256(s.encode("utf-8")).hexdigest()


def _now() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.000Z")


def _uid() -> str:
    return uuid.uuid4().hex[:8]


# ─── Users  (mirrors MockAuthDataSource) ──────────────────────────────────────

_USERS_RAW = [
    ("user_001", "Abood",   "ThrustBearing",     "Omar",    "customer", "81:EC:12:0D", 500.0),
    ("user_002", "Dr.Shady Maged",   "DrShadyIsTheBest",  "Dr. Shady",   "customer", "50:8F:E5:10", 100000.0),
    ("user_003", "kaboo",   "PlutoLover",        "Kaboo",   "worker",   "81:C5:8B:0D",   0.0),
    ("user_004", "Mohanna", "NurseAid",          "Mohanna", "customer", "50:3D:3B:10", 500.0),
    ("user_005", "Dina",    "FordFiesta",        "Dina",    "customer", "81:E2:10:0D", 500.0),
    ("user_006", "Amr",     "SharpEdges",        "Amoor",   "customer", "30:94:5F:10", 500.0),
    ("user_007", "Nagaty",  "SafetyCircuit",     "Nagaty",  "customer", "81:60:1F:4B", 500.0),
    ("user_008", "Mostafa",    "SasaPcb",           "Mostafa", "customer", "D2:04:F9:52", 500.0),
    ("user_009", "Roaa",    "RoaaESP",           "Roaa",    "customer", "50:8F:3A:10", 500.0),
    ("user_010", "Sara",    "SaraGazebo",        "Sara",    "customer", "D2:A0:1B:52", 500.0),
]

USERS: dict = {
    u: {
        "id": uid, "username": u, "password_hash": _sha256(pw),
        "name": name, "role": role, "rfid_card_id": rfid, "credits": cred,
    }
    for uid, u, pw, name, role, rfid, cred in _USERS_RAW
}

# Mutable credits (deducted on each order)
user_credits: dict = {u: USERS[u]["credits"] for u in USERS}

# ─── Products  (mirrors MockProductDataSource) ────────────────────────────────

PRODUCTS: list = [
    {"id": "prod_001", "name": "Apple",  "price": 5.0, "stock": 20, "image_url": "",
     "unit": "kg", "category": "Classic",  "description": "Crisp & Juicy",  "emoji": "🍎"},
    {"id": "prod_002", "name": "Orange", "price": 4.5, "stock": 20, "image_url": "",
     "unit": "kg", "category": "Citrus",   "description": "Sweet & Fresh",  "emoji": "🍊"},
    {"id": "prod_003", "name": "Kiwi",   "price": 6.0, "stock": 20, "image_url": "",
     "unit": "kg", "category": "Tropical", "description": "Tangy & Sweet",  "emoji": "🥝"},
]

product_stock: dict = {p["id"]: p["stock"] for p in PRODUCTS}

# ─── Connection Sets ───────────────────────────────────────────────────────────

flutter_clients:    Set[WebSocket] = set()   # /ws   – Flutter app
ros_bridge_clients: Set[WebSocket] = set()   # /ros_bridge/ws – gui_node

# ─── Low-level helpers ────────────────────────────────────────────────────────

async def _send_safe(ws: WebSocket, text: str) -> bool:
    try:
        await ws.send_text(text)
        return True
    except Exception:
        return False


async def _broadcast_to_flutter(msg: dict):
    """Send a dict (from gui_node / ROS) to all connected Flutter clients."""
    if not flutter_clients:
        return
    text = json.dumps(msg, ensure_ascii=False)
    dead = {ws for ws in flutter_clients if not await _send_safe(ws, text)}
    flutter_clients.difference_update(dead)


async def _broadcast_to_ros(msg: dict):
    """Forward a dict (from Flutter / HTTP) to all connected gui_node instances."""
    if not ros_bridge_clients:
        return
    text = json.dumps(msg, ensure_ascii=False)
    dead = {ws for ws in ros_bridge_clients if not await _send_safe(ws, text)}
    ros_bridge_clients.difference_update(dead)


# ─── WebSocket /ws  (Flutter) ─────────────────────────────────────────────────

@app.websocket("/ws")
async def flutter_ws(ws: WebSocket):
    await ws.accept()
    flutter_clients.add(ws)
    print(f"[Flutter WS] connected  total={len(flutter_clients)}")

    # Connection acknowledgement – Flutter expects this on connect.
    await ws.send_text(json.dumps({
        "type": "connection.status",
        "state": "connected",
        "latency_ms": 1,
        "timestamp": _now(),
    }))

    # Ask gui_node to push its current robot status right away.
    await _broadcast_to_ros({"type": "bridge.status_request", "timestamp": _now()})

    try:
        while True:
            raw = await ws.receive_text()
            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                continue
            # Relay Flutter message → gui_node  (no simulation here)
            await _broadcast_to_ros(msg)

    except WebSocketDisconnect:
        flutter_clients.discard(ws)
        print(f"[Flutter WS] disconnected  total={len(flutter_clients)}")


# ─── WebSocket /ros_bridge/ws  (gui_node) ─────────────────────────────────────

@app.websocket("/ros_bridge/ws")
async def ros_bridge_ws(ws: WebSocket):
    await ws.accept()
    ros_bridge_clients.add(ws)
    print(f"[ROS Bridge WS] gui_node connected  total={len(ros_bridge_clients)}")

    # Handshake so gui_node knows it is connected.
    await ws.send_text(json.dumps({
        "type": "bridge.connected",
        "timestamp": _now(),
    }))

    try:
        while True:
            raw = await ws.receive_text()
            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                continue
            # Relay ROS telemetry/events → all Flutter clients
            await _broadcast_to_flutter(msg)

    except WebSocketDisconnect:
        ros_bridge_clients.discard(ws)
        print(f"[ROS Bridge WS] gui_node disconnected  total={len(ros_bridge_clients)}")


# ─── HTTP: Auth ───────────────────────────────────────────────────────────────

@app.post("/auth/login")
async def login(request: Request):
    body = await request.json()
    username = body.get("username", "").strip()
    password = body.get("password", "").strip()

    user = USERS.get(username)
    if not user or user["password_hash"] != _sha256(password):
        raise HTTPException(status_code=401, detail="Invalid username or password")

    data = {k: v for k, v in user.items() if k != "password_hash"}
    data["credits"] = user_credits.get(username, user["credits"])

    return {
        "session_token": f"tok_{uuid.uuid4().hex[:20]}",
        "user": data,
    }


# ─── HTTP: Products ───────────────────────────────────────────────────────────

@app.get("/products")
async def get_products():
    return [
        dict(p, stock=product_stock.get(p["id"], p["stock"]))
        for p in PRODUCTS
    ]


# ─── HTTP: Orders ─────────────────────────────────────────────────────────────

@app.post("/orders")
async def create_order(request: Request):
    body = await request.json()

    user_id   = body.get("user_id", "")
    rfid      = body.get("assigned_rfid", "")
    items_raw = body.get("items", [])

    user = next((u for u in USERS.values() if u["id"] == user_id), None)
    if not user:
        raise HTTPException(status_code=404, detail="User not found")

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
        total += prod["price"] * qty
        product_stock[prod_id] -= qty
        order_items.append({
            "id": _uid(),
            "order_id": "",          # filled below
            "product_id": prod_id,
            "product_name": prod["name"],
            "unit_price": prod["price"],
            "quantity": qty,
        })

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

    # ── Forward to gui_node so ROS mission_node starts the REAL mission ──
    # No asyncio simulation here.
    await _broadcast_to_ros({
        "type": "order.place",
        "order_id": order_id,
        "user_id": user_id,
        "assigned_rfid": rfid,
        "items": order_items,
        "timestamp": _now(),
    })

    return {"order": order, "remaining_credits": remaining}


# ─── HTTP: Robot mode ─────────────────────────────────────────────────────────

@app.post("/robot/mode")
async def set_mode(request: Request):
    body = await request.json()
    mode = body.get("mode", "autonomous")
    await _broadcast_to_ros({
        "type": "robot.set_mode",
        "mode": mode,
        "timestamp": _now(),
    })
    return {"success": True, "mode": mode}


# ─── HTTP: Storage ────────────────────────────────────────────────────────────

@app.post("/robot/storage/open")
async def open_storage(request: Request):
    body = await request.json()
    await _broadcast_to_ros({
        "type": "storage.open_request",
        "order_id": body.get("order_id", ""),
        "timestamp": _now(),
    })
    return {"success": True}


@app.post("/robot/storage/close")
async def close_storage():
    await _broadcast_to_ros({
        "type": "storage.close_request",
        "timestamp": _now(),
    })
    return {"success": True}


# ─── HTTP: Health ─────────────────────────────────────────────────────────────

@app.get("/health")
async def health():
    return {
        "status": "ok",
        "flutter_clients": len(flutter_clients),
        "ros_bridge_clients": len(ros_bridge_clients),
        "timestamp": _now(),
    }


# ─── Startup ──────────────────────────────────────────────────────────────────

@app.on_event("startup")
async def startup():
    print("=" * 56)
    print("  Pluto Pi Relay Server  v2.0")
    print("  Flutter WS   : ws://0.0.0.0:8000/ws")
    print("  ROS Bridge   : ws://0.0.0.0:8000/ros_bridge/ws")
    print("  Flutter API  : http://0.0.0.0:8000")
    print()
    print("  ✓ No mission simulation – all commands go to ROS 2")
    print("=" * 56)


# ─── Entry Point ──────────────────────────────────────────────────────────────

if __name__ == "__main__":
    uvicorn.run("pi_server:app", host="0.0.0.0", port=8000, reload=False, log_level="info")
