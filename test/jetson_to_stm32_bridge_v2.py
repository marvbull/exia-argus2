#!/usr/bin/env python3
"""
Jetson → STM32 RC Bridge v2
CH3-based control with split functionality:
  CH3 300-1200: Gas control (output on CH2)
  CH3 1200-1600: Servo control (output on CH1)

Hardware setup:
    Herelink SBUS → FTDI+Inverter → Jetson /dev/ttyUSB0 (100k 8E2)
    Jetson USB → STM32 Nucleo /dev/ttyACM0 (115k 8N1)
    STM32 PA0 → Linear Servo signal
    STM32 PA1 → Gas/Motor control (future)

Usage:
    python3 jetson_to_stm32_bridge_v2.py
"""

import sys
import time
import struct
import argparse
import serial
from typing import Optional, Tuple

# SBUS config
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

# Split-channel mapping - ADJUSTABLE SETTINGS
SPLIT_POINT_SBUS = 992        # gas/brake boundary in SBUS values (adjust this!)
RC_MIN = 300
RC_MAX = 1600
RC_FAILSAFE_SERVO = 600       # servo failsafe (mid position)
RC_FAILSAFE_GAS = 300         # gas failsafe (off)

# SERVO POSITION SETTINGS - adjust these for your brake setup
SERVO_BRAKE_RELEASED = 1600   # servo fully extended (brake off)
SERVO_BRAKE_ENGAGED = 1200    # servo retracted (brake on)
SERVO_HOLD_DEADBAND = 20      # ±20 units around hold position (prevents jitter)


def sbus_to_split_control(sbus_val: int, current_servo: int) -> Tuple[int, int, str]:
    """
    Map CH3 SBUS value to servo + gas control with position hold.

    INVERTED LOGIC (stick down = higher SBUS values):
    SBUS < SPLIT_POINT_SBUS: BRAKE control, gas off
    SBUS > SPLIT_POINT_SBUS: GAS control, servo HOLD

    Returns: (servo_value, gas_value, mode)
    """
    clamped = max(SBUS_MIN, min(SBUS_MAX, sbus_val))

    if clamped < SPLIT_POINT_SBUS:
        # Stick up: brake control, gas off
        brake_range = SERVO_BRAKE_RELEASED - SERVO_BRAKE_ENGAGED
        # Map SBUS_MIN..SPLIT_POINT_SBUS to SERVO_BRAKE_ENGAGED..SERVO_BRAKE_RELEASED
        servo_val = SERVO_BRAKE_ENGAGED + int((clamped - SBUS_MIN) / (SPLIT_POINT_SBUS - SBUS_MIN) * brake_range)
        gas_val = RC_FAILSAFE_GAS
        mode = "BRAKE"
        return (servo_val, gas_val, mode)
    else:
        # Stick down: gas control, servo HOLDS current position
        gas_range = 1200 - RC_MIN  # 1200 - 300 = 900
        # Map SPLIT_POINT_SBUS..SBUS_MAX to RC_MIN..1200
        gas_val = RC_MIN + int((clamped - SPLIT_POINT_SBUS) / (SBUS_MAX - SPLIT_POINT_SBUS) * gas_range)

        # Hold servo at current position (don't move it during gas mode)
        servo_val = current_servo
        mode = "GAS"
        return (servo_val, gas_val, mode)


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
    """Build 11-byte frame for STM32."""
    payload = struct.pack("<HHHH", ch1, ch2, ch3, ch4)
    return bytes(FRAME_SYNC) + payload + bytes([crc8_dallas(payload)])


def decode_sbus_frame(frame: bytes) -> Optional[dict]:
    """Decode 25-byte SBUS frame."""
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

        # Current values
        self.servo_val = RC_FAILSAFE_SERVO
        self.gas_val = RC_FAILSAFE_GAS
        self.current_mode = "INIT"
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
            return False
        print("SBUS sync OK!")
        return True

    def bridge_loop(self):
        """Main bridge loop."""
        if not self.sync_sbus():
            return

        print(f"Bridge active: CH3 split control (boundary: {SPLIT_POINT_SBUS})")
        print("Stick UP (<992): BRAKE mode | Stick DOWN (>992): GAS mode")
        print("Ctrl+C to stop")

        period = 1.0 / STM32_RATE_HZ
        next_send = time.monotonic()
        buf = bytearray()

        self.running = True

        while self.running:
            now = time.monotonic()

            # Read SBUS data
            data = self.sbus.read(SBUS_FRAME_LEN * 2)
            if data:
                buf.extend(data)

                while len(buf) >= SBUS_FRAME_LEN:
                    if buf[0] != SBUS_STARTBYTE:
                        buf.pop(0)
                        continue

                    frame = bytes(buf[:SBUS_FRAME_LEN])
                    decoded = decode_sbus_frame(frame)

                    if decoded is not None:
                        if not decoded['failsafe'] and not decoded['lost_frame']:
                            # Valid frame - use CH3 (index 2)
                            ch3_sbus = decoded['channels'][2]
                            self.servo_val, self.gas_val, self.current_mode = sbus_to_split_control(ch3_sbus, self.servo_val)
                            self.last_valid = now
                            self.sbus_ok = True
                        else:
                            # SBUS failsafe
                            self.sbus_ok = False
                            self.current_mode = "FAILSAFE"

                    buf = buf[SBUS_FRAME_LEN:]

            # Timeout failsafe (no valid SBUS for 200ms)
            if now - self.last_valid > 0.2:
                self.sbus_ok = False
                self.servo_val = RC_FAILSAFE_SERVO
                self.gas_val = RC_FAILSAFE_GAS
                self.current_mode = "TIMEOUT"

            # Send to STM32 at 50 Hz
            # CH1 = servo, CH2 = gas, CH3/4 = unused
            if now >= next_send:
                frame = build_stm32_frame(self.servo_val, self.gas_val, 600, 600)
                self.stm32.write(frame)
                next_send += period

                # Status print
                status = "LINK OK" if self.sbus_ok else "FAILSAFE"
                print(f"Servo: {self.servo_val:4d} | Gas: {self.gas_val:4d} | Mode: {self.current_mode} | {status}", end='\r')

            time.sleep(0.001)


def main():
    parser = argparse.ArgumentParser(description='SBUS → STM32 RC Bridge v2 (CH3 split)')
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
        print("\nStopping...")
    finally:
        if bridge:
            bridge.close()


if __name__ == '__main__':
    main()