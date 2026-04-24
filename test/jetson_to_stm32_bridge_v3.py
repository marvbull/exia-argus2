#!/usr/bin/env python3
"""
Jetson → STM32 RC Bridge v3
All v2 features preserved + Waveshare Serial Bus Servo throttle control via USB-C.

CH3-based control with split functionality:
  CH3 300-1200: Gas control (output on CH2)
  CH3 1200-1600: Servo control (output on CH1)

Throttle (new):
  CH2 (SBUS 172-1811) → Waveshare 30kg Serial Bus Servo position (0-4095)
  Connected via USB-C directly to Jetson (not STM32)

Hardware setup:
    Herelink SBUS  → FTDI+Inverter → Jetson /dev/ttyUSB0 (100k 8E2)
    Jetson USB     → STM32 Nucleo  → /dev/ttyACM0 (115k 8N1)
    Jetson USB-C   → Waveshare Serial Bus Servo Driver Board (default /dev/ttyUSB1)
    STM32 PA0      → Linear Servo signal (brake)
    STM32 PA1      → Gear servo

Usage:
    python3 jetson_to_stm32_bridge_v3.py
    python3 jetson_to_stm32_bridge_v3.py --throttle-port /dev/ttyUSB2
    python3 jetson_to_stm32_bridge_v3.py --no-throttle
"""

import sys
import time
import struct
import argparse
import serial
from typing import Optional, Tuple

# ── SBUS config ───────────────────────────────────────────────────────────────
SBUS_BAUD       = 100000
SBUS_STARTBYTE  = 0x0F
SBUS_ENDBYTE    = 0x00
SBUS_FRAME_LEN  = 25
SBUS_MIN        = 172
SBUS_MID        = 992
SBUS_MAX        = 1811

# ── STM32 output config ───────────────────────────────────────────────────────
STM32_BAUD    = 115200
STM32_RATE_HZ = 50
FRAME_SYNC    = [0xAA, 0x55]

# ── Split-channel mapping ─────────────────────────────────────────────────────
SPLIT_POINT_SBUS = 1173   # CH3 boundary: below = GAS, above = BRAKE

# Brake servo (CH1) — PWM µs sent to STM32
SERVO_BRAKE_RELEASED = 1600
SERVO_BRAKE_ENGAGED  = 1150
SERVO_HOLD_DEADBAND  = 10

# Gas (CH2 to STM32) — legacy motor control path
GAS_MIN = 300
GAS_MAX = 1200

# Gear servo (CH3 in STM32 frame) — PWM µs
GEAR_NEUTRAL_US  = 1100
GEAR_DRIVE_US    = 1200
GEAR_REVERSE_US  = 1300
GEAR_FAILSAFE_US = 1200

SBUS_BUTTON_HIGH = 1000   # SBUS value above this = button pressed

# Safety interlocks
BRAKE_RELEASE_DELAY_S = 0.8
GEAR_SHIFT_DELAY_S    = 1.0

# Failsafe values
RC_FAILSAFE_SERVO = 1150
RC_FAILSAFE_GAS   = 300

# ── Waveshare Serial Bus Servo (Feetech SMS/SCS protocol) ────────────────────
THROTTLE_BAUD        = 1000000   # 1 Mbps — standard for SMS series
THROTTLE_SERVO_ID    = 1         # default servo ID from factory
THROTTLE_POS_MIN     = 0         # position at zero throttle
THROTTLE_POS_MAX     = 4095      # position at full throttle (12-bit)
THROTTLE_FAILSAFE_POS = 0        # safe position on link loss
THROTTLE_RATE_HZ      = 50        # send at same rate as STM32
THROTTLE_SPEED        = 1500      # steps/s for position moves (0=max speed, but some FW treat 0 as stop)
THROTTLE_ACC          = 50        # acceleration in 100 steps/s² units

# Feetech/SMS register addresses
_SMS_ACC             = 0x29      # 1-byte acceleration — WritePosEx starts here
_SMS_TORQUE_ENABLE   = 0x28      # 1-byte: 0=off, 1=on


# ── Waveshare Serial Bus Servo driver ─────────────────────────────────────────

def _sms_checksum(servo_id: int, length: int, instruction: int, params: bytes) -> int:
    """Feetech SMS/SCS checksum: ~(ID + LEN + CMD + sum(params)) & 0xFF."""
    total = servo_id + length + instruction + sum(params)
    return (~total) & 0xFF


def _sms_write_cmd(servo_id: int, address: int, data: bytes) -> bytes:
    """Build a generic Feetech SMS WRITE_DATA (0x03) packet."""
    params = bytes([address]) + data
    instruction = 0x03
    length = len(params) + 2
    checksum = _sms_checksum(servo_id, length, instruction, params)
    return bytes([0xFF, 0xFF, servo_id, length, instruction]) + params + bytes([checksum])


def build_write_pos_ex(servo_id: int, position: int,
                       speed: int = THROTTLE_SPEED,
                       acc: int = THROTTLE_ACC) -> bytes:
    """
    Equivalent of Feetech's WritePosEx — the only reliable way to move the servo.

    Writes 7 bytes starting at register 0x29 (ACC):
      [ACC][POS_L][POS_H][TIME_L][TIME_H][SPEED_L][SPEED_H]

    Verified against the official Feetech C library source (genWrite / WritePosEx).
    Without the speed bytes the servo firmware treats speed=0 and refuses to move.
    """
    position = max(THROTTLE_POS_MIN, min(THROTTLE_POS_MAX, position))
    # Sign-magnitude encoding: bit15 of the 16-bit word is the direction sign
    pos_enc = position  # 0-4095 are always positive, no sign bit needed
    data = bytes([
        acc & 0xFF,
        pos_enc & 0xFF,
        (pos_enc >> 8) & 0xFF,
        0x00,               # time low  (0 = no time limit)
        0x00,               # time high
        speed & 0xFF,
        (speed >> 8) & 0xFF,
    ])
    return _sms_write_cmd(servo_id, _SMS_ACC, data)


class ThrottleServo:
    """Waveshare Serial Bus Servo controller (Feetech SMS/SCS protocol over USB)."""

    def __init__(self, port: str, servo_id: int = THROTTLE_SERVO_ID,
                 baud: int = THROTTLE_BAUD):
        self.servo_id = servo_id
        self.position = THROTTLE_FAILSAFE_POS

        print(f"Opening Throttle Servo: {port} @ {baud} baud (servo ID {servo_id})")
        self._serial = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1,
        )

        # 1. Enable torque (register 0x28 = 1) — servo ignores position commands without this
        self._serial.write(_sms_write_cmd(servo_id, _SMS_TORQUE_ENABLE, bytes([0x01])))
        self._serial.flush()
        time.sleep(0.1)

        # 2. Move to failsafe position using the full WritePosEx format
        self._serial.write(build_write_pos_ex(servo_id, THROTTLE_FAILSAFE_POS))
        self._serial.flush()
        time.sleep(0.1)

        self._serial.timeout = 0
        print(f"Throttle Servo: torque enabled, ready at position {THROTTLE_FAILSAFE_POS}")

    def send_position(self, position: int) -> None:
        position = max(THROTTLE_POS_MIN, min(THROTTLE_POS_MAX, position))
        self.position = position
        self._serial.write(build_write_pos_ex(self.servo_id, position))
        self._serial.flush()

    def failsafe(self) -> None:
        self._serial.write(build_write_pos_ex(self.servo_id, THROTTLE_FAILSAFE_POS))
        self._serial.flush()

    def close(self) -> None:
        try:
            self.failsafe()
            time.sleep(0.05)
        except Exception:
            pass
        self._serial.close()


def gas_val_to_throttle_position(gas_val: int) -> int:
    """Map gas_val (GAS_MIN=300 .. GAS_MAX=1200) → throttle servo position (0-4095).
    gas_val=GAS_MIN (brake/failsafe) → 0, gas_val=GAS_MAX (full gas) → 4095."""
    t = (gas_val - GAS_MIN) / (GAS_MAX - GAS_MIN)
    return int(max(0.0, min(1.0, t)) * THROTTLE_POS_MAX)


# ── Existing v2 helpers ────────────────────────────────────────────────────────

def sbus_to_split_control(sbus_val: int, current_servo: int) -> Tuple[int, int, str]:
    """
    Map CH3 SBUS value to (servo_value, gas_value, mode).

    Stick UP   (SBUS 172..1173):  GAS mode   — gas 1200..300, servo holds
    Stick DOWN (SBUS 1173..1811): BRAKE mode — servo 1700..1900µs, gas off
    """
    clamped = max(SBUS_MIN, min(SBUS_MAX, sbus_val))

    if clamped <= SPLIT_POINT_SBUS:
        t = (clamped - SBUS_MIN) / (SPLIT_POINT_SBUS - SBUS_MIN)
        gas_val = GAS_MAX - int(t * (GAS_MAX - GAS_MIN))
        return (current_servo, gas_val, "GAS")
    else:
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


# ── Main bridge ────────────────────────────────────────────────────────────────

class RCBridge:
    def __init__(self, sbus_port: str, stm32_port: str,
                 throttle_port: Optional[str] = None,
                 throttle_servo_id: int = THROTTLE_SERVO_ID,
                 throttle_baud: int = THROTTLE_BAUD):
        self.running = False
        self.sbus_ok = False

        print(f"Opening SBUS: {sbus_port} @ {SBUS_BAUD} baud (8E2)")
        self.sbus = serial.Serial(
            port=sbus_port,
            baudrate=SBUS_BAUD,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_EVEN,
            stopbits=serial.STOPBITS_TWO,
            timeout=0.1,
        )

        print(f"Opening STM32: {stm32_port} @ {STM32_BAUD} baud (8N1)")
        self.stm32 = serial.Serial(stm32_port, STM32_BAUD, timeout=0)

        # Optional throttle servo
        self.throttle: Optional[ThrottleServo] = None
        if throttle_port:
            try:
                self.throttle = ThrottleServo(throttle_port, throttle_servo_id,
                                              baud=throttle_baud)
            except serial.SerialException as e:
                print(f"WARNING: Throttle servo unavailable ({e}) — continuing without it")

        # Current values
        self.servo_val   = RC_FAILSAFE_SERVO
        self.gas_val     = RC_FAILSAFE_GAS
        self.gear_us     = GEAR_FAILSAFE_US
        self.gear_name   = "NEUTRAL"

        self.ch3_raw     = 0
        self.throttle_pos = THROTTLE_FAILSAFE_POS
        self.current_mode = "INIT"
        self.last_valid  = time.monotonic()

        # Gear button rising-edge state
        self._ch6_prev = False
        self._ch7_prev = False
        self._ch8_prev = False

        # Brake-release interlock
        self._prev_raw_mode   = "INIT"
        self._brake_release_time = 0.0

        # Gear-shift interlock
        self._gear_shift_time = 0.0

    def close(self):
        self.running = False
        if self.throttle:
            self.throttle.close()
        if hasattr(self, 'sbus'):
            self.sbus.close()
        if hasattr(self, 'stm32'):
            self.stm32.close()

    def _update_gear(self, ch6: int, ch7: int, ch8: int) -> None:
        """Rising-edge latch: one press sets the gear, no hold required."""
        ch6_now = ch6 > SBUS_BUTTON_HIGH
        ch7_now = ch7 > SBUS_BUTTON_HIGH
        ch8_now = ch8 > SBUS_BUTTON_HIGH

        brake_30pct = SERVO_BRAKE_RELEASED - int(0.3 * (SERVO_BRAKE_RELEASED - SERVO_BRAKE_ENGAGED))
        brake_full  = self.servo_val <= brake_30pct

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

    def sync_sbus(self) -> bool:
        """Initial SBUS sync."""
        print("Syncing to SBUS...")
        frame = sync_to_sbus_frame(self.sbus)
        if frame is None:
            print("ERROR: No SBUS frames found!")
            return False
        print("SBUS sync OK!")
        return True

    def _apply_throttle_failsafe(self) -> None:
        self.throttle_pos = THROTTLE_FAILSAFE_POS
        if self.throttle:
            self.throttle.failsafe()

    def bridge_loop(self):
        """Main bridge loop."""
        if not self.sync_sbus():
            return

        throttle_info = f"Throttle servo: {'ENABLED (servo ID ' + str(self.throttle.servo_id) + ')' if self.throttle else 'DISABLED'}"
        print(f"Bridge active: CH3 split control (boundary: {SPLIT_POINT_SBUS})")
        print(f"Stick UP (<{SPLIT_POINT_SBUS}): GAS mode | Stick DOWN (>{SPLIT_POINT_SBUS}): BRAKE mode")
        print(f"CH2 → {throttle_info}")
        print("Ctrl+C to stop")

        period    = 1.0 / STM32_RATE_HZ
        next_send = time.monotonic()
        buf       = bytearray()

        self.running = True

        while self.running:
            now = time.monotonic()

            # ── Read SBUS ─────────────────────────────────────────────────────
            data = self.sbus.read(SBUS_FRAME_LEN * 2)
            if data:
                buf.extend(data)

                while len(buf) >= SBUS_FRAME_LEN:
                    if buf[0] != SBUS_STARTBYTE:
                        buf.pop(0)
                        continue

                    frame   = bytes(buf[:SBUS_FRAME_LEN])
                    decoded = decode_sbus_frame(frame)

                    if decoded is not None:
                        if not decoded['failsafe'] and not decoded['lost_frame']:
                            ch3_sbus = decoded['channels'][2]
                            self.ch3_raw = ch3_sbus

                            # CH3 → brake servo + gas (split control)
                            self.servo_val, self.gas_val, raw_mode = \
                                sbus_to_split_control(ch3_sbus, self.servo_val)

                            # Brake-release interlock
                            if raw_mode == "GAS":
                                if self._prev_raw_mode == "BRAKE":
                                    self._brake_release_time = now + BRAKE_RELEASE_DELAY_S
                                    self.servo_val = SERVO_BRAKE_RELEASED
                                if now < self._brake_release_time:
                                    self.gas_val = RC_FAILSAFE_GAS
                                    self.current_mode = "RELEASING"
                                elif now < self._gear_shift_time:
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

                            # gas_val (from CH3 split) → Waveshare throttle servo
                            # interlocks (RELEASING/SHIFTING) already force gas_val=GAS_MIN → pos=0
                            self.throttle_pos = gas_val_to_throttle_position(self.gas_val)

                            self.last_valid = now
                            self.sbus_ok    = True

                        else:
                            # SBUS failsafe / lost frame
                            self.servo_val  = RC_FAILSAFE_SERVO
                            self.gas_val    = RC_FAILSAFE_GAS
                            self.gear_us    = GEAR_FAILSAFE_US
                            self.gear_name  = "NEUTRAL"
                            self._apply_throttle_failsafe()
                            self.sbus_ok    = False
                            self.current_mode = "FAILSAFE"

                    buf = buf[SBUS_FRAME_LEN:]

            # ── Timeout failsafe (no valid SBUS for 200 ms) ───────────────────
            if now - self.last_valid > 0.2:
                self.sbus_ok   = False
                self.servo_val = RC_FAILSAFE_SERVO
                self.gas_val   = RC_FAILSAFE_GAS
                self.gear_us   = GEAR_FAILSAFE_US
                self.gear_name = "NEUTRAL"
                self._apply_throttle_failsafe()
                self.current_mode = "TIMEOUT"

            # ── Send at 50 Hz ─────────────────────────────────────────────────
            if now >= next_send:
                # STM32: CH1=brake servo, CH2=gas, CH3=gear servo
                stm_frame = build_stm32_frame(self.servo_val, self.gas_val, self.gear_us, 600)
                self.stm32.write(stm_frame)

                # Waveshare throttle servo
                if self.throttle and self.sbus_ok:
                    self.throttle.send_position(self.throttle_pos)

                next_send += period

                # Status line
                link         = "LINK OK" if self.sbus_ok else "NO LINK"
                throttle_str = f"Throttle: {self.throttle_pos:4d}" if self.throttle else "Throttle: N/A"
                print(
                    f"CH3: {self.ch3_raw:4d} | Brake: {self.servo_val:4d} | Gas: {self.gas_val:4d} | "
                    f"Gear: {self.gear_name:<8} | {throttle_str} | "
                    f"Mode: {self.current_mode:<9} | {link}",
                    end='\r',
                )

            time.sleep(0.001)


def main():
    parser = argparse.ArgumentParser(
        description='SBUS → STM32 RC Bridge v3 (CH3 split + Waveshare throttle servo)',
    )
    parser.add_argument('--sbus-port',     default='/dev/ttyUSB0',
                        help='SBUS input port (Herelink via FTDI, default: %(default)s)')
    parser.add_argument('--stm32-port',    default='/dev/ttyACM0',
                        help='STM32 output port (Nucleo USB, default: %(default)s)')
    parser.add_argument('--throttle-port', default='/dev/ttyUSB1',
                        help='Waveshare Serial Bus Servo USB port (default: %(default)s)')
    parser.add_argument('--throttle-id',   default=THROTTLE_SERVO_ID, type=int,
                        help='Waveshare servo ID (default: %(default)s)')
    parser.add_argument('--throttle-baud', default=THROTTLE_BAUD, type=int,
                        help='Waveshare servo baud rate (default: %(default)s, try 115200 or 500000)')
    parser.add_argument('--no-throttle',   action='store_true',
                        help='Disable Waveshare throttle servo entirely')
    args = parser.parse_args()

    throttle_port = None if args.no_throttle else args.throttle_port

    bridge = None
    try:
        bridge = RCBridge(
            sbus_port=args.sbus_port,
            stm32_port=args.stm32_port,
            throttle_port=throttle_port,
            throttle_servo_id=args.throttle_id,
            throttle_baud=args.throttle_baud,
        )
        bridge.bridge_loop()
    except serial.SerialException as e:
        print(f"\nSerial error: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        if bridge:
            bridge.close()


if __name__ == '__main__':
    main()
