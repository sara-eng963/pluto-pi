#!/usr/bin/env python3

"""
rfid_node.py

Purpose:
    Read MFRC522 RFID cards using Raspberry Pi SPI.
    Publish scanned UID to /mission/rfid_verification.

Architecture:
    RFID hardware node does NOT own mission logic.
    It only publishes physical card scans.

Publishes:
    /mission/rfid_verification   std_msgs/String JSON

Example published JSON:
    {
      "source": "rfid_node",
      "order_id": "",
      "rfid_card_id": "D2:A0:1B:52",
      "success": true,
      "timestamp": "2026-06-12T20:00:00.000Z"
    }

Important:
    order_id is intentionally empty.
    mission_node accepts empty order_id and validates only the active mission RFID.
"""

import json
import os
import time
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import spidev


# =============================================================================
# MFRC522 register map
# =============================================================================

COMMAND_REG = 0x01
COM_IEN_REG = 0x02
DIV_IEN_REG = 0x03
COM_IRQ_REG = 0x04
DIV_IRQ_REG = 0x05
ERROR_REG = 0x06
STATUS_1_REG = 0x07
STATUS_2_REG = 0x08
FIFO_DATA_REG = 0x09
FIFO_LEVEL_REG = 0x0A
CONTROL_REG = 0x0C
BIT_FRAMING_REG = 0x0D
COLL_REG = 0x0E

MODE_REG = 0x11
TX_MODE_REG = 0x12
RX_MODE_REG = 0x13
TX_CONTROL_REG = 0x14
TX_ASK_REG = 0x15

T_MODE_REG = 0x2A
T_PRESCALER_REG = 0x2B
T_RELOAD_REG_H = 0x2C
T_RELOAD_REG_L = 0x2D

VERSION_REG = 0x37


# =============================================================================
# MFRC522 commands
# =============================================================================

PCD_IDLE = 0x00
PCD_AUTHENT = 0x0E
PCD_TRANSCEIVE = 0x0C
PCD_SOFT_RESET = 0x0F

PICC_REQIDL = 0x26
PICC_ANTICOLL = 0x93
PICC_HALT = 0x50

MI_OK = 0
MI_NOTAGERR = 1
MI_ERR = 2


def now_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.000Z")


class MFRC522:
    def __init__(self, bus: int = 0, device: int = 0, speed_hz: int = 1_000_000):
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.max_speed_hz = speed_hz
        self.spi.mode = 0

        self.init_reader()

    def close(self):
        self.spi.close()

    def read_reg(self, reg: int) -> int:
        addr = ((reg << 1) & 0x7E) | 0x80
        response = self.spi.xfer2([addr, 0x00])
        return response[1]

    def write_reg(self, reg: int, value: int):
        addr = (reg << 1) & 0x7E
        self.spi.xfer2([addr, value])

    def set_bit_mask(self, reg: int, mask: int):
        current = self.read_reg(reg)
        self.write_reg(reg, current | mask)

    def clear_bit_mask(self, reg: int, mask: int):
        current = self.read_reg(reg)
        self.write_reg(reg, current & (~mask))

    def antenna_on(self):
        value = self.read_reg(TX_CONTROL_REG)
        if not (value & 0x03):
            self.set_bit_mask(TX_CONTROL_REG, 0x03)

    def antenna_off(self):
        self.clear_bit_mask(TX_CONTROL_REG, 0x03)

    def init_reader(self):
        self.write_reg(COMMAND_REG, PCD_SOFT_RESET)
        time.sleep(0.15)  # increased: 50 ms was too short when many nodes start together

        self.write_reg(T_MODE_REG, 0x8D)
        self.write_reg(T_PRESCALER_REG, 0x3E)
        self.write_reg(T_RELOAD_REG_L, 30)
        self.write_reg(T_RELOAD_REG_H, 0)

        self.write_reg(TX_ASK_REG, 0x40)
        self.write_reg(MODE_REG, 0x3D)

        self.antenna_on()

    def read_version(self) -> int:
        return self.read_reg(VERSION_REG)

    def to_card(self, command: int, send_data: list[int]):
        back_data = []
        back_len = 0
        status = MI_ERR

        irq_en = 0x00
        wait_irq = 0x00

        if command == PCD_AUTHENT:
            irq_en = 0x12
            wait_irq = 0x10
        elif command == PCD_TRANSCEIVE:
            irq_en = 0x77
            wait_irq = 0x30

        self.write_reg(COM_IEN_REG, irq_en | 0x80)
        self.clear_bit_mask(COM_IRQ_REG, 0x80)
        self.set_bit_mask(FIFO_LEVEL_REG, 0x80)

        self.write_reg(COMMAND_REG, PCD_IDLE)

        for value in send_data:
            self.write_reg(FIFO_DATA_REG, value)

        self.write_reg(COMMAND_REG, command)

        if command == PCD_TRANSCEIVE:
            self.set_bit_mask(BIT_FRAMING_REG, 0x80)

        timeout = 2000
        irq_value = 0

        while timeout:
            irq_value = self.read_reg(COM_IRQ_REG)

            if irq_value & wait_irq:
                break

            if irq_value & 0x01:
                break

            timeout -= 1

        self.clear_bit_mask(BIT_FRAMING_REG, 0x80)

        if timeout == 0:
            return MI_ERR, [], 0

        if self.read_reg(ERROR_REG) & 0x1B:
            return MI_ERR, [], 0

        status = MI_OK

        if irq_value & irq_en & 0x01:
            status = MI_NOTAGERR

        if command == PCD_TRANSCEIVE:
            fifo_count = self.read_reg(FIFO_LEVEL_REG)
            last_bits = self.read_reg(CONTROL_REG) & 0x07

            if last_bits:
                back_len = (fifo_count - 1) * 8 + last_bits
            else:
                back_len = fifo_count * 8

            if fifo_count == 0:
                fifo_count = 1

            if fifo_count > 16:
                fifo_count = 16

            for _ in range(fifo_count):
                back_data.append(self.read_reg(FIFO_DATA_REG))

        return status, back_data, back_len

    def request(self):
        self.write_reg(BIT_FRAMING_REG, 0x07)

        status, back_data, back_bits = self.to_card(
            PCD_TRANSCEIVE,
            [PICC_REQIDL],
        )

        if status != MI_OK or back_bits != 0x10:
            return MI_ERR

        return MI_OK

    def anticoll(self):
        self.write_reg(BIT_FRAMING_REG, 0x00)

        status, back_data, _ = self.to_card(
            PCD_TRANSCEIVE,
            [PICC_ANTICOLL, 0x20],
        )

        if status != MI_OK:
            return MI_ERR, None

        if len(back_data) != 5:
            return MI_ERR, None

        checksum = 0
        for i in range(4):
            checksum ^= back_data[i]

        if checksum != back_data[4]:
            return MI_ERR, None

        uid = back_data[:4]
        return MI_OK, uid

    def halt(self):
        self.to_card(PCD_TRANSCEIVE, [PICC_HALT, 0x00])

    def read_uid(self):
        if self.request() != MI_OK:
            return None

        status, uid = self.anticoll()

        if status != MI_OK:
            return None

        self.halt()
        return uid


class RFIDNode(Node):
    def __init__(self):
        super().__init__("rfid_node")

        self.declare_parameter("spi_bus", 0)
        self.declare_parameter("spi_device", 0)
        self.declare_parameter("spi_speed_hz", 1_000_000)
        self.declare_parameter("poll_hz", 5.0)
        self.declare_parameter("cooldown_sec", 2.0)

        self.spi_bus = int(self.get_parameter("spi_bus").value)
        self.spi_device = int(self.get_parameter("spi_device").value)
        self.spi_speed_hz = int(self.get_parameter("spi_speed_hz").value)
        self.poll_hz = float(self.get_parameter("poll_hz").value)
        self.cooldown_sec = float(self.get_parameter("cooldown_sec").value)

        self.pub = self.create_publisher(
            String,
            "/mission/rfid_verification",
            10,
        )

        self.reader = self._init_reader_with_retry()

        self.get_logger().info("RFID node started.")
        self.get_logger().info(f"RFID executable path: {os.path.abspath(__file__)}")
        self.get_logger().info(
            f"MFRC522 SPI bus={self.spi_bus}, device={self.spi_device}, speed={self.spi_speed_hz}"
        )
        self.get_logger().info(f"MFRC522 VersionReg = 0x{self.reader.read_version():02X}")
        self.get_logger().info("Publishing scans to /mission/rfid_verification")

        self.last_uid = ""
        self.last_publish_time = 0.0
        self.poll_count = 0
        self._consecutive_errors = 0
        self._REINIT_THRESHOLD = 30  # reinit after 30 consecutive poll failures (~6 s at 5 Hz)

        period = 1.0 / max(self.poll_hz, 0.1)
        self.timer = self.create_timer(period, self.poll_once)

    def _init_reader_with_retry(self, retries: int = 5, delay_sec: float = 2.0) -> "MFRC522":
        for attempt in range(1, retries + 1):
            try:
                reader = MFRC522(
                    bus=self.spi_bus,
                    device=self.spi_device,
                    speed_hz=self.spi_speed_hz,
                )
                # Validate chip is actually responding — 0x00 / 0xFF means bad state
                version = reader.read_version()
                if version in (0x00, 0xFF):
                    raise RuntimeError(
                        f"MFRC522 VersionReg=0x{version:02X} — chip not responding (bad init state)"
                    )
                return reader
            except Exception as exc:
                self.get_logger().warn(
                    f"MFRC522 SPI init attempt {attempt}/{retries} failed: {exc}"
                )
                if attempt < retries:
                    time.sleep(delay_sec)
        raise RuntimeError(
            f"Failed to open SPI bus={self.spi_bus} device={self.spi_device} "
            f"after {retries} attempts."
        )

    def format_uid(self, uid: list[int]) -> str:
        return ":".join(f"{byte:02X}" for byte in uid)

    def publish_uid(self, uid_text: str):
        payload = {
            "source": "rfid_node",
            "order_id": "",
            "rfid_card_id": uid_text,
            "success": True,
            "timestamp": now_iso(),
        }

        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self.pub.publish(msg)

        self.get_logger().info(f"RFID scanned: {uid_text}")
        self.get_logger().info(f"PUB /mission/rfid_verification: {msg.data}")

    def _try_reinit(self):
        self.get_logger().warn("RFID: reinitialising reader after consecutive failures...")
        try:
            self.reader.close()
        except Exception:
            pass
        try:
            self.reader = self._init_reader_with_retry()
            self._consecutive_errors = 0
            self.get_logger().info("RFID: reader reinitialised successfully.")
        except Exception as exc:
            self.get_logger().error(f"RFID: reinit failed: {exc}")

    def poll_once(self):
        self.poll_count += 1

        try:
            uid = self.reader.read_uid()
        except Exception as exc:
            self.get_logger().error(f"RFID poll error: {exc}")
            self._consecutive_errors += 1
            if self._consecutive_errors >= self._REINIT_THRESHOLD:
                self._try_reinit()
            return

        if uid is None:
            self._consecutive_errors += 1
            if self._consecutive_errors >= self._REINIT_THRESHOLD:
                self._try_reinit()
            return

        self._consecutive_errors = 0

        uid_text = self.format_uid(uid)
        now = time.monotonic()

        duplicate = (
            uid_text == self.last_uid
            and (now - self.last_publish_time) < self.cooldown_sec
        )

        if duplicate:
            return

        self.last_uid = uid_text
        self.last_publish_time = now

        self.publish_uid(uid_text)

    def destroy_node(self):
        try:
            self.reader.close()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = RFIDNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().warn("RFID node stopped by keyboard interrupt.")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
