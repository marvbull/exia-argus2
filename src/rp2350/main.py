"""
exia-argus2 — RP2350 SBUS Bridge
Reads SBUS from Herelink Air Unit and forwards decoded frames to Jetson via UART.

Wiring:
    Herelink SBUS-Out  →  GP1  (UART0 RX, inverted)
    GP4 (UART1 TX)     →  Jetson /dev/ttyTHS2 RX
    GND                →  GND (all three devices)

Output to Jetson: raw 25-byte SBUS frames at 115200 8N1
Jetson reads with: python test_sbus.py --port /dev/ttyTHS2
"""

from machine import UART, Pin
import rp2
import time

# ---------------------------------------------------------------------------
# PIO program — reads inverted UART (SBUS) without hardware inverter
# ---------------------------------------------------------------------------
# The RP2350 UART supports inversion via the invert flag in MicroPython.
# We use UART0 with invert=True for SBUS input.

# SBUS input  — UART0 RX on GP1 (inverted, 100000 baud, 8E2)
sbus_uart = UART(0,
                 baudrate=100000,
                 bits=8,
                 parity=1,        # Even parity
                 stop=2,          # 2 stop bits
                 rx=Pin(1),
                 invert=UART.INV_RX)  # Invert RX for SBUS

# Output to Jetson — UART1 TX on GP4 (normal, 115200 baud, 8N1)
jetson_uart = UART(1,
                   baudrate=115200,
                   bits=8,
                   parity=None,
                   stop=1,
                   tx=Pin(4))

SBUS_FRAME_LEN  = 25
SBUS_STARTBYTE  = 0x0F
SBUS_ENDBYTE    = 0x00

# LED for status feedback
led = Pin(25, Pin.OUT)  # onboard LED


def decode_channels(frame: bytes) -> list:
    """Decode 16 x 11-bit SBUS channels from bytes 1-22."""
    b = frame[1:23]
    return [
        ((b[0]       | b[1]  << 8) & 0x07FF),
        ((b[1]  >> 3 | b[2]  << 5) & 0x07FF),
        ((b[2]  >> 6 | b[3]  << 2 | b[4] << 10) & 0x07FF),
        ((b[4]  >> 1 | b[5]  << 7) & 0x07FF),
        ((b[5]  >> 4 | b[6]  << 4) & 0x07FF),
        ((b[6]  >> 7 | b[7]  << 1 | b[8] << 9) & 0x07FF),
        ((b[8]  >> 2 | b[9]  << 6) & 0x07FF),
        ((b[9]  >> 5 | b[10] << 3) & 0x07FF),
        ((b[11]      | b[12] << 8) & 0x07FF),
        ((b[12] >> 3 | b[13] << 5) & 0x07FF),
        ((b[13] >> 6 | b[14] << 2 | b[15] << 10) & 0x07FF),
        ((b[15] >> 1 | b[16] << 7) & 0x07FF),
        ((b[16] >> 4 | b[17] << 4) & 0x07FF),
        ((b[17] >> 7 | b[18] << 1 | b[19] << 9) & 0x07FF),
        ((b[19] >> 2 | b[20] << 6) & 0x07FF),
        ((b[20] >> 5 | b[21] << 3) & 0x07FF),
    ]


def main():
    buf = bytearray()
    frame_count = 0
    error_count = 0

    print("RP2350 SBUS Bridge started")
    print("Waiting for SBUS frames from Herelink...")

    while True:
        # Read available bytes
        n = sbus_uart.any()
        if n:
            buf.extend(sbus_uart.read(n))

        # Process complete frames from buffer
        while len(buf) >= SBUS_FRAME_LEN:
            # Sync: find start byte
            if buf[0] != SBUS_STARTBYTE:
                buf.pop(0)
                error_count += 1
                continue

            # Check if we have a full frame
            if len(buf) < SBUS_FRAME_LEN:
                break

            frame = bytes(buf[:SBUS_FRAME_LEN])

            # Validate end byte
            if frame[24] != SBUS_ENDBYTE:
                buf.pop(0)
                error_count += 1
                continue

            # Valid frame — forward to Jetson
            jetson_uart.write(frame)
            buf = buf[SBUS_FRAME_LEN:]

            frame_count += 1

            # Blink LED every 100 frames (~700ms)
            if frame_count % 100 == 0:
                led.toggle()
                flags = frame[23]
                failsafe = bool(flags & 0x08)
                lost     = bool(flags & 0x04)
                ch = decode_channels(frame)
                print(f"frames={frame_count} errors={error_count} "
                      f"ch1={ch[0]} ch2={ch[1]} ch3={ch[2]} ch4={ch[3]} "
                      f"failsafe={failsafe} lost={lost}")

        time.sleep_ms(1)


main()
