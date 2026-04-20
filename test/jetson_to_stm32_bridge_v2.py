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
SPLIT_POINT_SBUS = 1173        # CH3 stick boundary: below = BRAKE, above = GAS

# Servo (CH1) settings — values are PWM µs, sent directly to STM32
# Physical inversion: 1900µs = rod extended = brake OFF, 1100µs = rod retracted = brake ON
SERVO_BRAKE_RELEASED = 1600   # brake OFF (rod extended)
SERVO_BRAKE_ENGAGED  = 1150   # brake ON  (rod retracted)
SERVO_HOLD_DEADBAND  = 10     # ±µs, suppresses jitter when holding position

# Gas (CH2) settings
GAS_MIN = 300                 # gas off (motor idle)
GAS_MAX = 1200                # gas full throttle

# Gear servo (CH3 in frame) — values are PWM µs, sent directly to STM32
GEAR_NEUTRAL_US  = 1100       # neutral
GEAR_DRIVE_US    = 1200       # drive
GEAR_REVERSE_US  = 1300       # reverse
GEAR_FAILSAFE_US = 1200       # failsafe → neutral

SBUS_BUTTON_HIGH = 1000       # SBUS value above this = button pressed

# Safety interlock: time (seconds) to wait after BRAKE→GAS transition before
# allowing throttle. Servo must physically travel to BRAKE_RELEASED first.
BRAKE_RELEASE_DELAY_S = 0.8   # tune to actual servo travel time

# Gear-shift interlock: gas is blocked for this long after any gear change.
# Braking is NEVER blocked. Tune to actual gear servo travel time.
GEAR_SHIFT_DELAY_S = 1.0

# Failsafe values sent on SBUS timeout / invalid frame
RC_FAILSAFE_SERVO = 1150      # failsafe → brake ON
RC_FAILSAFE_GAS   = 300


def sbus_to_split_control(sbus_val: int, current_servo: int) -> Tuple[int, int, str]:
    """
    Map CH3 SBUS value to (servo_value, gas_value, mode).

    Stick UP   (SBUS 172..1173):  GAS mode   — gas 1200..300, servo holds
    Stick DOWN (SBUS 1173..1811): BRAKE mode — servo 1700..1900µs, gas off
    """
    clamped = max(SBUS_MIN, min(SBUS_MAX, sbus_val))

    if clamped <= SPLIT_POINT_SBUS:
        # Stick UP (SBUS 172..1173): GAS mode, servo holds
        t = (clamped - SBUS_MIN) / (SPLIT_POINT_SBUS - SBUS_MIN)
        gas_val = GAS_MAX - int(t * (GAS_MAX - GAS_MIN))  # 172→full, 1173→off
        return (current_servo, gas_val, "GAS")
    else:
        # Stick DOWN (SBUS 1173..1811): BRAKE mode, gas off
        t = (clamped - SPLIT_POINT_SBUS) / (SBUS_MAX - SPLIT_POINT_SBUS)
        servo_val = SERVO_BRAKE_RELEASED - int(t * (SERVO_BRAKE_RELEASED - SERVO_BRAKE_ENGAGED))
        return (servo_val, RC_FAILSAFE_GAS, "BRAKE")


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
        self.gear_us = GEAR_FAILSAFE_US
        self.gear_name = "NEUTRAL"
        self.ch3_raw = 0
        self.current_mode = "INIT"
        self.last_valid = time.monotonic()

        # Gear button rising-edge state
        self._ch6_prev = False
        self._ch7_prev = False
        self._ch8_prev = False

        # Brake-release interlock
        self._prev_raw_mode = "INIT"
        self._brake_release_time = 0.0

        # Gear-shift interlock
        self._gear_shift_time = 0.0

    def close(self):
        self.running = False
        if hasattr(self, 'sbus'):
            self.sbus.close()
        if hasattr(self, 'stm32'):
            self.stm32.close()

    def _update_gear(self, ch6: int, ch7: int, ch8: int) -> None:
        """Rising-edge latch: one press sets the gear, no hold required."""
        ch6_now = ch6 > SBUS_BUTTON_HIGH
        ch7_now = ch7 > SBUS_BUTTON_HIGH
        ch8_now = ch8 > SBUS_BUTTON_HIGH

        # Allow gear shift when brake is at least 30% engaged
        brake_30pct = SERVO_BRAKE_RELEASED - int(0.3 * (SERVO_BRAKE_RELEASED - SERVO_BRAKE_ENGAGED))
        brake_full = self.servo_val <= brake_30pct

        if ch6_now and not self._ch6_prev and brake_full:
            self.gear_us, self.gear_name = GEAR_DRIVE_US, "DRIVE"
            self._gear_shift_time = time.monotonic() + GEAR_SHIFT_DELAY_S
        elif ch7_now and not self._ch7_prev and brake_full:
            self.gear_us, self.gear_name = GEAR_REVERSE_US, "REVERSE"
            self._gear_shift_time = time.monotonic() + GEAR_SHIFT_DELAY_S
        elif ch8_now and not self._ch8_prev and brake_full:
            self.gear_us, self.gear_name = GEAR_NEUTRAL_US, "NEUTRAL"
            self._gear_shift_time = time.monotonic() + GEAR_SHIFT_DELAY_S

        self._ch6_prev = ch6_now
        self._ch7_prev = ch7_now
        self._ch8_prev = ch8_now

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
        print("Stick UP (<992): GAS mode | Stick DOWN (>992): BRAKE mode")
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
                            ch3_sbus = decoded['channels'][2]
                            self.ch3_raw = ch3_sbus
                            self.servo_val, self.gas_val, raw_mode = sbus_to_split_control(ch3_sbus, self.servo_val)

                            # Brake-release interlock: on BRAKE→GAS, block gas until servo travels
                            if raw_mode == "GAS":
                                if self._prev_raw_mode == "BRAKE":
                                    self._brake_release_time = now + BRAKE_RELEASE_DELAY_S
                                    self.servo_val = SERVO_BRAKE_RELEASED
                                if now < self._brake_release_time:
                                    self.gas_val = RC_FAILSAFE_GAS
                                    self.current_mode = "RELEASING"
                                elif now < self._gear_shift_time:
                                    # Gear-shift interlock: block gas, braking always allowed
                                    self.gas_val = RC_FAILSAFE_GAS
                                    self.current_mode = "SHIFTING"
                                else:
                                    self.current_mode = "GAS"
                            else:
                                self.current_mode = raw_mode
                            self._prev_raw_mode = raw_mode

                            self._update_gear(
                                decoded['channels'][5],
                                decoded['channels'][6],
                                decoded['channels'][7],
                            )
                            self.last_valid = now
                            self.sbus_ok = True
                        else:
                            self.servo_val = RC_FAILSAFE_SERVO
                            self.gas_val = RC_FAILSAFE_GAS
                            self.gear_us = GEAR_FAILSAFE_US
                            self.gear_name = "NEUTRAL"
                            self.sbus_ok = False
                            self.current_mode = "FAILSAFE"

                    buf = buf[SBUS_FRAME_LEN:]

            # Timeout failsafe (no valid SBUS for 200ms)
            if now - self.last_valid > 0.2:
                self.sbus_ok = False
                self.servo_val = RC_FAILSAFE_SERVO
                self.gas_val = RC_FAILSAFE_GAS
                self.gear_us = GEAR_FAILSAFE_US
                self.gear_name = "NEUTRAL"
                self.current_mode = "TIMEOUT"

            # Send to STM32 at 50 Hz
            # CH1 = brake servo, CH2 = gas, CH3 = gear servo
            if now >= next_send:
                frame = build_stm32_frame(self.servo_val, self.gas_val, self.gear_us, 600)
                self.stm32.write(frame)
                next_send += period

                # Status print
                link = "LINK OK" if self.sbus_ok else "NO LINK"
                print(f"CH3_RAW: {self.ch3_raw:4d} | Brake: {self.servo_val:4d} | Gas: {self.gas_val:4d} | Gear: {self.gear_name:<8} | Mode: {self.current_mode:<8} | {link}", end='\r')

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