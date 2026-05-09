#!/usr/bin/env python3

import spidev
import time
from datetime import datetime


# ============================================================
# MFRC522 Register Addresses
# These are internal memory-like locations inside the MFRC522.
# The Raspberry Pi reads/writes these registers over SPI.
# ============================================================

CommandReg      = 0x01
ComIrqReg       = 0x04
DivIrqReg       = 0x05
ErrorReg        = 0x06
FIFODataReg     = 0x09
FIFOLevelReg    = 0x0A
ControlReg      = 0x0C
BitFramingReg   = 0x0D
ModeReg         = 0x11
TxControlReg    = 0x14
TxASKReg        = 0x15
TModeReg        = 0x2A
TPrescalerReg   = 0x2B
TReloadRegH     = 0x2C
TReloadRegL     = 0x2D
CRCResultRegH   = 0x21
CRCResultRegL   = 0x22


# ============================================================
# MFRC522 Commands
# These are commands written into CommandReg.
# ============================================================

PCD_IDLE        = 0x00
PCD_CALCCRC     = 0x03
PCD_TRANSCEIVE  = 0x0C
PCD_SOFTRESET   = 0x0F


# ============================================================
# PICC / Card Commands
# These are commands sent through RF from MFRC522 to the card.
# ============================================================

PICC_REQA       = 0x26
PICC_ANTICOLL_CL1 = 0x93
PICC_ANTICOLL_CL2 = 0x95
PICC_ANTICOLL_CL3 = 0x97


# ============================================================
# Status Values
# ============================================================

STATUS_OK       = 0
STATUS_ERROR    = 1
STATUS_TIMEOUT  = 2


class MFRC522:
    def __init__(self, bus=0, device=0, speed_hz=1_000_000):
        """
        bus=0, device=0 means /dev/spidev0.0
        This matches:
        SDA/SS -> GPIO8 -> CE0
        """
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.max_speed_hz = speed_hz
        self.spi.mode = 0

        self.init_chip()

    # ------------------------------------------------------------
    # Low-level SPI register write
    # ------------------------------------------------------------
    def write_reg(self, reg, value):
        """
        Write one byte to an MFRC522 register.

        MFRC522 SPI write format:
        bit 7 = 0 for write
        bits 6:1 = register address
        bit 0 = 0
        """
        address = (reg << 1) & 0x7E
        self.spi.xfer2([address, value])

    # ------------------------------------------------------------
    # Low-level SPI register read
    # ------------------------------------------------------------
    def read_reg(self, reg):
        """
        Read one byte from an MFRC522 register.

        MFRC522 SPI read format:
        bit 7 = 1 for read
        bits 6:1 = register address
        bit 0 = 0

        Second byte is dummy data.
        It exists only so the Pi creates clock pulses
        and the MFRC522 can send data back on MISO.
        """
        address = ((reg << 1) & 0x7E) | 0x80
        response = self.spi.xfer2([address, 0x00])
        return response[1]

    # ------------------------------------------------------------
    # Bit helpers
    # ------------------------------------------------------------
    def set_bit_mask(self, reg, mask):
        current = self.read_reg(reg)
        self.write_reg(reg, current | mask)

    def clear_bit_mask(self, reg, mask):
        current = self.read_reg(reg)
        self.write_reg(reg, current & (~mask))

    # ------------------------------------------------------------
    # Initialize MFRC522
    # ------------------------------------------------------------
    def init_chip(self):
        """
        Reset and configure the MFRC522.

        This prepares timers, modulation settings,
        and turns on the antenna field.
        """
        self.write_reg(CommandReg, PCD_SOFTRESET)
        time.sleep(0.05)

        self.write_reg(TModeReg, 0x8D)
        self.write_reg(TPrescalerReg, 0x3E)
        self.write_reg(TReloadRegL, 30)
        self.write_reg(TReloadRegH, 0)

        self.write_reg(TxASKReg, 0x40)
        self.write_reg(ModeReg, 0x3D)

        self.antenna_on()

    # ------------------------------------------------------------
    # Turn antenna on
    # ------------------------------------------------------------
    def antenna_on(self):
        """
        Enable antenna driver pins.
        Without this, the card receives no RF energy.
        """
        value = self.read_reg(TxControlReg)
        if not (value & 0x03):
            self.set_bit_mask(TxControlReg, 0x03)

    # ------------------------------------------------------------
    # Send data to card through MFRC522
    # ------------------------------------------------------------
    def to_card(self, command, send_data):
        """
        Core MFRC522 communication function.

        Raspberry Pi writes bytes into MFRC522 FIFO.
        MFRC522 sends them through RF to the card.
        MFRC522 receives card response.
        Raspberry Pi reads response from MFRC522 FIFO over SPI.
        """

        back_data = []
        back_bits = 0

        if command == PCD_TRANSCEIVE:
            wait_irq = 0x30
        else:
            wait_irq = 0x00

        # Clear interrupt flags
        self.write_reg(ComIrqReg, 0x7F)

        # Flush FIFO buffer
        self.set_bit_mask(FIFOLevelReg, 0x80)

        # Stop any active command
        self.write_reg(CommandReg, PCD_IDLE)

        # Write outgoing data to FIFO
        for byte in send_data:
            self.write_reg(FIFODataReg, byte)

        # Start command
        self.write_reg(CommandReg, command)

        # For Transceive, StartSend begins RF transmission
        if command == PCD_TRANSCEIVE:
            self.set_bit_mask(BitFramingReg, 0x80)

        # Wait until command finishes or times out
        timeout_counter = 2000
        while True:
            irq_value = self.read_reg(ComIrqReg)
            timeout_counter -= 1

            command_done = irq_value & wait_irq
            timer_timeout = irq_value & 0x01

            if command_done or timer_timeout or timeout_counter == 0:
                break

        # Stop StartSend
        self.clear_bit_mask(BitFramingReg, 0x80)

        if timeout_counter == 0:
            return STATUS_TIMEOUT, [], 0

        error = self.read_reg(ErrorReg)

        # Check for protocol/collision/parity/buffer errors
        if error & 0x1B:
            return STATUS_ERROR, [], 0

        # Read received bytes from FIFO
        fifo_level = self.read_reg(FIFOLevelReg)
        last_bits = self.read_reg(ControlReg) & 0x07

        if last_bits:
            back_bits = (fifo_level - 1) * 8 + last_bits
        else:
            back_bits = fifo_level * 8

        for _ in range(fifo_level):
            back_data.append(self.read_reg(FIFODataReg))

        return STATUS_OK, back_data, back_bits

    # ------------------------------------------------------------
    # Request card presence
    # ------------------------------------------------------------
    def request_card(self):
        """
        Send REQA command.

        REQA asks:
        'Is there any idle RFID card in the RF field?'
        """
        self.write_reg(BitFramingReg, 0x07)

        status, back_data, back_bits = self.to_card(
            PCD_TRANSCEIVE,
            [PICC_REQA]
        )

        self.write_reg(BitFramingReg, 0x00)

        if status != STATUS_OK or back_bits != 0x10:
            return False

        return True

    # ------------------------------------------------------------
    # Anti-collision for one cascade level
    # ------------------------------------------------------------
    def anticollision(self, cascade_level):
        """
        Ask the card for UID bytes at one cascade level.

        For a simple 4-byte UID card:
        cascade level 1 returns:
        UID0 UID1 UID2 UID3 BCC

        BCC = UID0 XOR UID1 XOR UID2 XOR UID3
        """
        self.write_reg(BitFramingReg, 0x00)

        status, data, bits = self.to_card(
            PCD_TRANSCEIVE,
            [cascade_level, 0x20]
        )

        if status != STATUS_OK:
            return None

        if len(data) != 5:
            return None

        bcc = data[0] ^ data[1] ^ data[2] ^ data[3]

        if bcc != data[4]:
            return None

        return data

    # ------------------------------------------------------------
    # Calculate CRC using MFRC522 hardware
    # ------------------------------------------------------------
    def calculate_crc(self, data):
        """
        MFRC522 can calculate CRC internally.
        SELECT commands need CRC bytes.
        """
        self.write_reg(CommandReg, PCD_IDLE)
        self.write_reg(DivIrqReg, 0x04)
        self.set_bit_mask(FIFOLevelReg, 0x80)

        for byte in data:
            self.write_reg(FIFODataReg, byte)

        self.write_reg(CommandReg, PCD_CALCCRC)

        timeout_counter = 255
        while True:
            irq = self.read_reg(DivIrqReg)
            timeout_counter -= 1

            if irq & 0x04 or timeout_counter == 0:
                break

        crc_low = self.read_reg(CRCResultRegL)
        crc_high = self.read_reg(CRCResultRegH)

        return [crc_low, crc_high]

    # ------------------------------------------------------------
    # Select card at one cascade level
    # ------------------------------------------------------------
    def select_tag(self, cascade_level, serial_bytes):
        """
        Selects the card after anti-collision.

        The card replies with SAK.
        SAK tells us whether more UID cascade levels exist.
        """
        command = [cascade_level, 0x70] + serial_bytes
        crc = self.calculate_crc(command)
        command += crc

        status, data, bits = self.to_card(PCD_TRANSCEIVE, command)

        if status == STATUS_OK and bits == 0x18 and len(data) > 0:
            return data[0]

        return None

    # ------------------------------------------------------------
    # Full UID read
    # ------------------------------------------------------------
    def read_uid(self):
        """
        Complete UID reading sequence:

        1. Check if card exists.
        2. Run anti-collision cascade level 1.
        3. Select card.
        4. If UID is longer, continue cascade level 2/3.
        5. Return UID as clean string.
        """

        if not self.request_card():
            return None

        uid = []

        cascade_levels = [
            PICC_ANTICOLL_CL1,
            PICC_ANTICOLL_CL2,
            PICC_ANTICOLL_CL3
        ]

        for cascade_level in cascade_levels:
            serial = self.anticollision(cascade_level)

            if serial is None:
                return None

            sak = self.select_tag(cascade_level, serial)

            if sak is None:
                return None

            # 0x88 means Cascade Tag, not a real UID byte
            if serial[0] == 0x88:
                uid.extend(serial[1:4])
            else:
                uid.extend(serial[0:4])

            # SAK bit 2 means UID continues to another cascade level
            if not (sak & 0x04):
                return self.format_uid(uid)

        return self.format_uid(uid)

    # ------------------------------------------------------------
    # UID formatting
    # ------------------------------------------------------------
    def format_uid(self, uid_bytes):
        """
        Convert UID bytes into stable readable format.

        Example:
        [0x83, 0x27, 0x9A, 0x1C]
        becomes:
        83:27:9A:1C
        """
        return ":".join(f"{byte:02X}" for byte in uid_bytes)

    # ------------------------------------------------------------
    # Close SPI
    # ------------------------------------------------------------
    def close(self):
        self.spi.close()


def main():
    reader = MFRC522(bus=0, device=0)

    print("RFID UID reader started.")
    print("Place one card near the MFRC522.")
    print("Remove the card before scanning it again.")
    print("Press Ctrl+C to stop.\n")

    scan_count = 0
    last_uid = None
    last_scan_time = 0

    try:
        while True:
            uid = reader.read_uid()

            if uid is not None:
                current_time = time.time()

                # Prevent terminal spam if the same card is left on the reader
                if uid != last_uid or (current_time - last_scan_time) > 2.0:
                    scan_count += 1
                    timestamp = datetime.now().strftime("%H:%M:%S")

                    print(f"[{scan_count:02d}] {timestamp} | UID = {uid}")

                    last_uid = uid
                    last_scan_time = current_time

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nStopped by user.")

    finally:
        reader.close()


if __name__ == "__main__":
    main()
