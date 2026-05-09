import spidev
import time

spi = spidev.SpiDev()
spi.open(0, 0)              # bus 0, CE0 = /dev/spidev0.0
spi.max_speed_hz = 1000000
spi.mode = 0

VERSION_REG = 0x37

def read_reg(reg):
    addr = ((reg << 1) & 0x7E) | 0x80
    response = spi.xfer2([addr, 0x00])
    return response[1]

time.sleep(0.1)

version = read_reg(VERSION_REG)
print(f"MFRC522 VersionReg = 0x{version:02X}")

spi.close()
