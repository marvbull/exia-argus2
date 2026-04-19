#!/usr/bin/env python3
"""
Jetson → STM32 RC Bridge
Reads SBUS from Herelink via FTDI inverter, forwards to STM32 PWM slave.

Hardware setup:
    Herelink SBUS → FTDI+Inverter → Jetson /dev/ttyUSB0 (100k 8E2)
    Jetson USB → STM32 Nucleo /dev/ttyACM0 (115k 8N1)
    STM32 PA0 → Linear Servo signal

Usage:
    python3 jetson_to_stm32_bridge.py
    python3 jetson_to_stm32_bridge.py --sbus-port /dev/ttyUSB1 --stm32-port /dev/ttyACM1
"""

import sys
import time
import struct
import argparse
import serial
import threading
from typing import Optional

# SBUS config (from your test_sbus.py)
SBUS_BAUD = 100000
SBUS_STARTBYTE = 0x0F
SBUS_ENDBYTE = 0x00
SBUS_FRAME_LEN = 25
SBUS_MIN = 172
SBUS_MID = 992
SBUS_MAX = 1811

# STM32 output config
STM32_BAUD = 115200
STM32_RATE_HZ = 50
FRAME_SYNC = [0xAA, 0x55]

# RC mapping: SBUS → your CH1 range
RC_MIN = 300   # maps to servo retracted
RC_MID = 600   # failsafe
RC_MAX = 1600  # maps to servo extended


def sbus_to_rc(sbus_val: int) -> int:
    """Map SBUS value (172-1811) to RC range (300-1600)."""
    clamped = max(SBUS_MIN, min(SBUS_MAX, sbus_val))
    return int(RC_MIN + (clamped - SBUS_MIN) / (SBUS_MAX - SBUS_MIN) * (RC_MAX - RC_MIN))


def crc8_dallas(data: bytes) -> int:
    """CRC8 Dallas/Maxim (same as STM32 code)."""
    crc = 0
    for b in data:
        for _ in range(8):
            mix = (crc ^ b) & 1
            crc >>= 1
            if mix:
                crc ^= 0x8C
            b >>= 1
    return crc


def build_stm32_frame(ch1: int, ch2: int, ch3: int, ch4: int) -> bytes:
    """Build 11-byte frame for STM32 (same format as sim_jetson.py)."""
    payload = struct.pack("<HHHH", ch1, ch2, ch3, ch4)
    return bytes(FRAME_SYNC) + payload + bytes([crc8_dallas(payload)])


def decode_sbus_frame(frame: bytes) -> Optional[dict]:
    """Decode 25-byte SBUS frame (from your test_sbus.py)."""
    if len(frame) != SBUS_FRAME_LEN:
        return None
    if frame[0] != SBUS_STARTBYTE or frame[24] != SBUS_ENDBYTE:
        return None

    # Unpack 16 x 11-bit channels from bytes 1-22
    b = frame[1:23]
    raw = [
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

    flags = frame[23]
    return {
        'channels': raw,
        'failsafe': bool(flags & 0x08),
        'lost_frame': bool(flags & 0x04),
    }


def sync_to_sbus_frame(ser: serial.Serial) -> Optional[bytes]:
    """Read until we find a valid SBUS frame."""
    buf = bytearray()
    deadline = time.monotonic() + 2.0

    while time.monotonic() < deadline:
        byte = ser.read(1)
        if not byte:
            continue
        buf.append(byte[0])

        if len(buf) >= SBUS_FRAME_LEN:
            # Slide window looking for valid frame
            for i in range(len(buf) - SBUS_FRAME_LEN + 1):
                candidate = bytes(buf[i:i + SBUS_FRAME_LEN])
                if candidate[0] == SBUS_STARTBYTE and candidate[24] == SBUS_ENDBYTE:
                    return candidate
            buf = buf[-(SBUS_FRAME_LEN - 1):]
    return None


class RCBridge:
    def __init__(self, sbus_port: str, stm32_port: str):
        self.running = False
        self.sbus_ok = False
        self.stm32_ok = False

        # Open ports
        print(f"Opening SBUS: {sbus_port} @ {SBUS_BAUD} baud (8E2)")
        self.sbus = serial.Serial(
            port=sbus_port,
            baudrate=SBUS_BAUD,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_EVEN,
            stopbits=serial.STOPBITS_TWO,
            timeout=0.1
        )

        print(f"Opening STM32: {stm32_port} @ {STM32_BAUD} baud (8N1)")
        self.stm32 = serial.Serial(stm32_port, STM32_BAUD, timeout=0)

        # Current RC values (for failsafe)
        self.last_ch = [RC_MID, RC_MID, RC_MID, RC_MID]
        self.last_valid = time.monotonic()

    def close(self):
        self.running = False
        if hasattr(self, 'sbus'):
            self.sbus.close()
        if hasattr(self, 'stm32'):
            self.stm32.close()

    def sync_sbus(self):
        """Initial SBUS sync."""
        print("Syncing to SBUS...")
        frame = sync_to_sbus_frame(self.sbus)
        if frame is None:
            print("ERROR: No SBUS frames found!")
            print("Check: Herelink → FTDI → /dev/ttyUSB0, hardware inverter")
            return False
        print("SBUS sync OK!")
        return True

    def bridge_loop(self):
        """Main bridge loop."""
        if not self.sync_sbus():
            return

        print("Bridge active: Controller → SBUS → Jetson → STM32 → Servo")
        print("Ctrl+C to stop")

        period = 1.0 / STM32_RATE_HZ
        next_send = time.monotonic()
        buf = bytearray()

        self.running = True

        while self.running:
            now = time.monotonic()

            # Read SBUS data
            data = self.sbus.read(SBUS_FRAME_LEN * 2)  # read more for buffering
            if data:
                buf.extend(data)

                # Process complete frames
                while len(buf) >= SBUS_FRAME_LEN:
                    if buf[0] != SBUS_STARTBYTE:
                        buf.pop(0)
                        continue

                    frame = bytes(buf[:SBUS_FRAME_LEN])
                    decoded = decode_sbus_frame(frame)

                    if decoded is not None:
                        if not decoded['failsafe'] and not decoded['lost_frame']:
                            # Valid frame
                            ch = decoded['channels']
                            self.last_ch = [
                                sbus_to_rc(ch[0]),  # CH1 → servo
                                sbus_to_rc(ch[1]),  # CH2
                                sbus_to_rc(ch[2]),  # CH3
                                sbus_to_rc(ch[3])   # CH4
                            ]
                            self.last_valid = now
                            self.sbus_ok = True
                        else:
                            # SBUS failsafe
                            self.sbus_ok = False

                    buf = buf[SBUS_FRAME_LEN:]

            # Failsafe timeout (no valid SBUS for 200ms)
            if now - self.last_valid > 0.2:
                self.sbus_ok = False
                self.last_ch = [RC_MID, RC_MID, RC_MID, RC_MID]

            # Send to STM32 at 50 Hz
            if now >= next_send:
                frame = build_stm32_frame(*self.last_ch)
                self.stm32.write(frame)
                next_send += period

                # Status print
                status = "LINK OK" if self.sbus_ok else "FAILSAFE"
                print(f"CH1: {self.last_ch[0]:4d} | {status}", end='\\r')

            # Small sleep to avoid busy loop
            time.sleep(0.001)


def main():
    parser = argparse.ArgumentParser(description='SBUS → STM32 RC Bridge')
    parser.add_argument('--sbus-port', default='/dev/ttyUSB0',
                        help='SBUS input port (Herelink via FTDI)')
    parser.add_argument('--stm32-port', default='/dev/ttyACM0',
                        help='STM32 output port (Nucleo USB)')
    args = parser.parse_args()

    bridge = None
    try:
        bridge = RCBridge(args.sbus_port, args.stm32_port)
        bridge.bridge_loop()
    except serial.SerialException as e:
        print(f"Serial error: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\\nStopping...")
    finally:
        if bridge:
            bridge.close()


if __name__ == '__main__':
    main()