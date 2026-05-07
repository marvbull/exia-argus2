#!/usr/bin/env python3
"""
Jetson → STM32 RC Bridge v4

Real-time + crash-resistant version of v3.

Key changes vs v3:
  • STM32 + Phidget update rate: 50 Hz → 100 Hz (10ms cycle)
  • Brake-release interlock:  0.8s → 0.2s
  • Gear-shift interlock:     1.0s → 0.3s
  • Status print throttled to 20 Hz (sonst spamt 100 Hz)
  • Every device (SBUS / STM32 / Throttle / Steering) has auto-reconnect:
        on USB drop / write error / Phidget detach → reconnect attempt every 2s
  • Each loop iteration wrapped in try/except — Bridge läuft NIE durch eine
    Exception kaputt. Devices die nicht erreichbar sind werden ausgelassen,
    der Rest funktioniert weiter.

Hardware setup (stable symlinks via /etc/udev/rules.d/99-exia-argus.rules):
    Herelink SBUS  → FTDI+Inverter → /dev/sbus      (100k 8E2)
    Jetson USB     → STM32 Nucleo  → /dev/stm32     (115k 8N1)
    Jetson USB-C   → Waveshare Servo Driver         → /dev/waveshare
    Jetson USB     → Phidget VINT Hub → DCC1000 (steering motor) on Hub-Port 3
    STM32 PA0      → Linear Servo signal (brake)
    STM32 PA1      → Gear servo

Usage:
    python3 jetson_to_stm32_bridge_v4.py
    python3 jetson_to_stm32_bridge_v4.py --no-throttle
    python3 jetson_to_stm32_bridge_v4.py --no-steering
"""

import sys
import time
import struct
import argparse
import traceback
import serial
from typing import Optional, Tuple

try:
    from Phidget22.Devices.MotorPositionController import MotorPositionController
    from Phidget22.PhidgetException import PhidgetException
    PHIDGET_AVAILABLE = True
except ImportError:
    PHIDGET_AVAILABLE = False
    class PhidgetException(Exception):    # type: ignore
        pass

# ── SBUS config ───────────────────────────────────────────────────────────────
SBUS_BAUD       = 100000
SBUS_STARTBYTE  = 0x0F
SBUS_ENDBYTE    = 0x00
SBUS_FRAME_LEN  = 25
SBUS_MIN        = 172
SBUS_MID        = 992
SBUS_MAX        = 1811

# ── STM32 output config ───────────────────────────────────────────────────────
STM32_BAUD      = 115200
STM32_RATE_HZ   = 100              # ↑ 50 → 100 Hz (10ms cycle)
FRAME_SYNC      = [0xAA, 0x55]

# ── Loop / robustness ─────────────────────────────────────────────────────────
LOOP_SLEEP_S          = 0.0005     # 0.5ms — finer than v3's 1ms
RECONNECT_INTERVAL_S  = 2.0        # how often we retry a dead device
SBUS_TIMEOUT_S        = 0.2        # SBUS link-loss threshold
STATUS_PRINT_HZ       = 20

# ── Split-channel mapping ─────────────────────────────────────────────────────
SPLIT_POINT_SBUS = 1173

# Brake servo (CH1) — PWM µs
SERVO_BRAKE_RELEASED = 1600
SERVO_BRAKE_ENGAGED  = 1150

# Gas (CH2) — legacy STM32 path
GAS_MIN = 300
GAS_MAX = 1200

# Gear servo (CH3) — PWM µs (mechanisch kalibriert)
GEAR_NEUTRAL_US  = 1375
GEAR_DRIVE_US    = 1450
GEAR_REVERSE_US  = 1300
GEAR_FAILSAFE_US = 1375

SBUS_BUTTON_HIGH = 1000

# Safety interlocks (reduziert für mehr "Echtzeitgefühl")
BRAKE_RELEASE_DELAY_S = 0.2        # ↓ 0.8 → 0.2
GEAR_SHIFT_DELAY_S    = 0.3        # ↓ 1.0 → 0.3

# Failsafe values
RC_FAILSAFE_SERVO = 1150
RC_FAILSAFE_GAS   = 300

# ── Waveshare Throttle Servo (Feetech SMS/SCS) ────────────────────────────────
THROTTLE_BAUD         = 1000000
THROTTLE_SERVO_ID     = 1
THROTTLE_POS_MIN      = 0
THROTTLE_POS_MAX      = 4095
THROTTLE_POS_IDLE     = 796        # ~70° (Standgas)
THROTTLE_POS_FULL     = 114        # ~10° (Vollgas)
THROTTLE_FAILSAFE_POS = THROTTLE_POS_IDLE
THROTTLE_SPEED        = 1500
THROTTLE_ACC          = 50

_SMS_ACC            = 0x29
_SMS_TORQUE_ENABLE  = 0x28

# ── Phidget Steering Motor (DCC1000 + HKT22) ─────────────────────────────────
STEERING_HUB_PORT       = 3
STEERING_ENCODER_PPR    = 300
STEERING_QUADRATURE     = 4
STEERING_GEARBOX_1      = 4.25
STEERING_GEARBOX_2      = 76 / 13
STEERING_RESCALE_FACTOR = -360.0 / (
    STEERING_ENCODER_PPR * STEERING_QUADRATURE *
    STEERING_GEARBOX_1   * STEERING_GEARBOX_2
)
STEERING_MAX_DEG        = 1750.0
STEERING_DEFAULT_RANGE  = 500
STEERING_VELOCITY_LIMIT = 1500.0
STEERING_ACCELERATION   = 5000.0
STEERING_CURRENT_LIMIT  = 15.0
STEERING_DEAD_BAND      = 2.0
STEERING_KP             = 450.0
STEERING_KI             = 0.0
STEERING_KD             = 200.0
STEERING_ATTACH_TO_MS   = 1000     # 1s (war 5s) — schnellere Reconnect-Versuche


# ─────────────────────────────────────────────────────────────────────────────
# Helpers (unchanged from v3)
# ─────────────────────────────────────────────────────────────────────────────

def crc8_dallas(data: bytes) -> int:
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
    payload = struct.pack("<HHHH", ch1, ch2, ch3, ch4)
    return bytes(FRAME_SYNC) + payload + bytes([crc8_dallas(payload)])


def decode_sbus_frame(frame: bytes) -> Optional[dict]:
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


def sbus_to_split_control(sbus_val: int, current_servo: int) -> Tuple[int, int, str]:
    clamped = max(SBUS_MIN, min(SBUS_MAX, sbus_val))
    if clamped <= SPLIT_POINT_SBUS:
        t = (clamped - SBUS_MIN) / (SPLIT_POINT_SBUS - SBUS_MIN)
        gas_val = GAS_MAX - int(t * (GAS_MAX - GAS_MIN))
        return (current_servo, gas_val, "GAS")
    else:
        t = (clamped - SPLIT_POINT_SBUS) / (SBUS_MAX - SPLIT_POINT_SBUS)
        servo_val = SERVO_BRAKE_RELEASED - int(t * (SERVO_BRAKE_RELEASED - SERVO_BRAKE_ENGAGED))
        return (servo_val, RC_FAILSAFE_GAS, "BRAKE")


def gas_val_to_throttle_position(gas_val: int) -> int:
    t = (gas_val - GAS_MIN) / (GAS_MAX - GAS_MIN)
    t = max(0.0, min(1.0, t))
    return int(THROTTLE_POS_IDLE - t * (THROTTLE_POS_IDLE - THROTTLE_POS_FULL))


def ch1_to_steering_deg(ch1: int, max_deg: float, invert: bool = False) -> float:
    ch1 = max(SBUS_MIN, min(SBUS_MAX, ch1))
    centered = ch1 - SBUS_MID
    half_range = max(SBUS_MAX - SBUS_MID, SBUS_MID - SBUS_MIN)
    normalized = centered / half_range
    if invert:
        normalized = -normalized
    return normalized * max_deg


# ── Feetech SMS protocol (for ThrottleServo) ──────────────────────────────────

def _sms_checksum(servo_id: int, length: int, instruction: int, params: bytes) -> int:
    return (~(servo_id + length + instruction + sum(params))) & 0xFF


def _sms_write_cmd(servo_id: int, address: int, data: bytes) -> bytes:
    params = bytes([address]) + data
    instruction = 0x03
    length = len(params) + 2
    checksum = _sms_checksum(servo_id, length, instruction, params)
    return bytes([0xFF, 0xFF, servo_id, length, instruction]) + params + bytes([checksum])


def build_write_pos_ex(servo_id: int, position: int,
                       speed: int = THROTTLE_SPEED,
                       acc: int = THROTTLE_ACC) -> bytes:
    position = max(THROTTLE_POS_MIN, min(THROTTLE_POS_MAX, position))
    data = bytes([
        acc & 0xFF,
        position & 0xFF,
        (position >> 8) & 0xFF,
        0x00, 0x00,
        speed & 0xFF,
        (speed >> 8) & 0xFF,
    ])
    return _sms_write_cmd(servo_id, _SMS_ACC, data)


# ─────────────────────────────────────────────────────────────────────────────
# Self-managing devices with auto-reconnect
# ─────────────────────────────────────────────────────────────────────────────

class SbusReader:
    """SBUS reader with auto-reconnect on USB drop / read errors."""

    def __init__(self, port: str):
        self.port = port
        self.alive = False
        self._next_retry = 0.0
        self._ser: Optional[serial.Serial] = None
        self._buf = bytearray()
        self._connect()

    def _connect(self) -> None:
        try:
            self._ser = serial.Serial(
                port=self.port, baudrate=SBUS_BAUD,
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_EVEN,
                stopbits=serial.STOPBITS_TWO, timeout=0.005,
            )
            self._buf = bytearray()
            self.alive = True
            print(f"[SBUS] connected on {self.port}")
        except Exception as e:
            self.alive = False
            print(f"[SBUS] open failed: {e}")

    def maybe_reconnect(self) -> None:
        if self.alive:
            return
        now = time.monotonic()
        if now < self._next_retry:
            return
        self._next_retry = now + RECONNECT_INTERVAL_S
        self._connect()

    def read_frames(self):
        """Generator yielding decoded SBUS frame dicts. Never raises."""
        if not self.alive or self._ser is None:
            return
        try:
            data = self._ser.read(SBUS_FRAME_LEN * 2)
            if data:
                self._buf.extend(data)
        except Exception as e:
            self._fail(e)
            return

        while len(self._buf) >= SBUS_FRAME_LEN:
            if self._buf[0] != SBUS_STARTBYTE:
                self._buf.pop(0)
                continue
            frame = bytes(self._buf[:SBUS_FRAME_LEN])
            decoded = decode_sbus_frame(frame)
            if decoded is not None:
                yield decoded
            self._buf = self._buf[SBUS_FRAME_LEN:]

    def _fail(self, e: Exception) -> None:
        if self.alive:
            print(f"\n[SBUS] read failed: {e} → reconnecting")
        self.alive = False
        try:
            if self._ser:
                self._ser.close()
        except Exception:
            pass
        self._ser = None
        self._next_retry = time.monotonic() + RECONNECT_INTERVAL_S

    def close(self) -> None:
        try:
            if self._ser:
                self._ser.close()
        except Exception:
            pass


class Stm32Writer:
    """STM32 frame writer with auto-reconnect."""

    def __init__(self, port: str):
        self.port = port
        self.alive = False
        self._next_retry = 0.0
        self._ser: Optional[serial.Serial] = None
        self._connect()

    def _connect(self) -> None:
        try:
            self._ser = serial.Serial(self.port, STM32_BAUD, timeout=0)
            self.alive = True
            print(f"[STM32] connected on {self.port}")
        except Exception as e:
            self.alive = False
            print(f"[STM32] open failed: {e}")

    def maybe_reconnect(self) -> None:
        if self.alive:
            return
        now = time.monotonic()
        if now < self._next_retry:
            return
        self._next_retry = now + RECONNECT_INTERVAL_S
        self._connect()

    def write_frame(self, frame: bytes) -> None:
        if not self.alive or self._ser is None:
            return
        try:
            self._ser.write(frame)
        except Exception as e:
            self._fail(e)

    def _fail(self, e: Exception) -> None:
        if self.alive:
            print(f"\n[STM32] write failed: {e} → reconnecting")
        self.alive = False
        try:
            if self._ser:
                self._ser.close()
        except Exception:
            pass
        self._ser = None
        self._next_retry = time.monotonic() + RECONNECT_INTERVAL_S

    def close(self) -> None:
        try:
            if self._ser:
                self._ser.close()
        except Exception:
            pass


class ThrottleServo:
    """Waveshare throttle servo with auto-reconnect."""

    def __init__(self, port: str, servo_id: int = THROTTLE_SERVO_ID,
                 baud: int = THROTTLE_BAUD):
        self.port = port
        self.servo_id = servo_id
        self.baud = baud
        self.position = THROTTLE_FAILSAFE_POS
        self.alive = False
        self._next_retry = 0.0
        self._ser: Optional[serial.Serial] = None
        self._connect()

    def _connect(self) -> None:
        try:
            ser = serial.Serial(
                port=self.port, baudrate=self.baud,
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE, timeout=0.05,
            )
            ser.write(_sms_write_cmd(self.servo_id, _SMS_TORQUE_ENABLE, bytes([0x01])))
            ser.flush()
            time.sleep(0.05)
            ser.write(build_write_pos_ex(self.servo_id, THROTTLE_FAILSAFE_POS))
            ser.flush()
            ser.timeout = 0
            self._ser = ser
            self.alive = True
            print(f"[Throttle] connected on {self.port} (servo ID {self.servo_id})")
        except Exception as e:
            self.alive = False
            self._ser = None
            print(f"[Throttle] open failed: {e}")

    def maybe_reconnect(self) -> None:
        if self.alive:
            return
        now = time.monotonic()
        if now < self._next_retry:
            return
        self._next_retry = now + RECONNECT_INTERVAL_S
        self._connect()

    def send_position(self, position: int) -> None:
        if not self.alive or self._ser is None:
            return
        position = max(THROTTLE_POS_MIN, min(THROTTLE_POS_MAX, position))
        self.position = position
        try:
            self._ser.write(build_write_pos_ex(self.servo_id, position))
            self._ser.flush()
        except Exception as e:
            self._fail(e)

    def failsafe(self) -> None:
        self.send_position(THROTTLE_FAILSAFE_POS)

    def _fail(self, e: Exception) -> None:
        if self.alive:
            print(f"\n[Throttle] write failed: {e} → reconnecting")
        self.alive = False
        try:
            if self._ser:
                self._ser.close()
        except Exception:
            pass
        self._ser = None
        self._next_retry = time.monotonic() + RECONNECT_INTERVAL_S

    def close(self) -> None:
        try:
            self.failsafe()
            time.sleep(0.05)
        except Exception:
            pass
        try:
            if self._ser:
                self._ser.close()
        except Exception:
            pass


class SteeringMotor:
    """Phidget DCC1000 steering motor with auto-reconnect."""

    def __init__(self, hub_port: int = STEERING_HUB_PORT):
        if not PHIDGET_AVAILABLE:
            raise RuntimeError("Phidget22 library not installed (pip install Phidget22)")
        self.hub_port = hub_port
        self.target_deg = 0.0
        self.position = 0.0
        self.alive = False
        self._next_retry = 0.0
        self._m: Optional[MotorPositionController] = None
        self._connect()

    def _connect(self) -> None:
        m = MotorPositionController()
        try:
            m.setHubPort(self.hub_port)
            m.setIsHubPortDevice(False)
            m.openWaitForAttachment(STEERING_ATTACH_TO_MS)

            m.setRescaleFactor(STEERING_RESCALE_FACTOR)
            m.setCurrentLimit(STEERING_CURRENT_LIMIT)
            m.setVelocityLimit(STEERING_VELOCITY_LIMIT)
            m.setAcceleration(STEERING_ACCELERATION)
            m.setDeadBand(STEERING_DEAD_BAND)
            m.setKp(STEERING_KP)
            m.setKi(STEERING_KI)
            m.setKd(STEERING_KD)
            try:
                m.addPositionOffset(-m.getPosition())
            except PhidgetException:
                pass
            m.setEngaged(True)
            self._m = m
            self.alive = True
            print(f"[Steering] attached on Hub-Port {self.hub_port}, position referenced to 0°")
        except Exception as e:
            try:
                m.close()
            except Exception:
                pass
            self._m = None
            self.alive = False
            print(f"[Steering] connect failed: {e}")

    def maybe_reconnect(self) -> None:
        if self.alive:
            return
        now = time.monotonic()
        if now < self._next_retry:
            return
        self._next_retry = now + RECONNECT_INTERVAL_S
        self._connect()

    def send_target(self, deg: float) -> None:
        if not self.alive or self._m is None:
            return
        deg = max(-STEERING_MAX_DEG, min(STEERING_MAX_DEG, deg))
        self.target_deg = deg
        try:
            self._m.setTargetPosition(deg)
        except PhidgetException as e:
            self._fail(e)
        except Exception as e:
            self._fail(e)

    def update_position(self) -> None:
        """Aktualisiert self.position für Status-Anzeige (nur bei Status-Print rufen)."""
        if not self.alive or self._m is None:
            return
        try:
            self.position = self._m.getPosition()
        except PhidgetException as e:
            self._fail(e)
        except Exception as e:
            self._fail(e)

    def failsafe(self) -> None:
        self.send_target(0.0)

    def _fail(self, e: Exception) -> None:
        if self.alive:
            print(f"\n[Steering] error: {e} → reconnecting")
        self.alive = False
        try:
            if self._m:
                self._m.close()
        except Exception:
            pass
        self._m = None
        self._next_retry = time.monotonic() + RECONNECT_INTERVAL_S

    def close(self) -> None:
        try:
            if self.alive and self._m:
                self._m.setTargetPosition(0.0)
                time.sleep(0.1)
                self._m.setEngaged(False)
        except Exception:
            pass
        try:
            if self._m:
                self._m.close()
        except Exception:
            pass


# ─────────────────────────────────────────────────────────────────────────────
# Bridge
# ─────────────────────────────────────────────────────────────────────────────

class RCBridge:
    def __init__(self,
                 sbus_port: str,
                 stm32_port: str,
                 throttle_port: Optional[str] = None,
                 throttle_servo_id: int = THROTTLE_SERVO_ID,
                 throttle_baud: int = THROTTLE_BAUD,
                 steering_enabled: bool = True,
                 steering_hub_port: int = STEERING_HUB_PORT,
                 steering_max_deg: float = STEERING_DEFAULT_RANGE,
                 steering_invert: bool = False):
        self.running = False

        # Devices (all self-recovering, never raise on init)
        self.sbus  = SbusReader(sbus_port)
        self.stm32 = Stm32Writer(stm32_port)

        self.throttle: Optional[ThrottleServo] = None
        if throttle_port:
            try:
                self.throttle = ThrottleServo(throttle_port, throttle_servo_id, throttle_baud)
            except Exception as e:
                print(f"[Throttle] init failed: {e} — disabled")
                self.throttle = None

        self.steering: Optional[SteeringMotor] = None
        self.steering_max_deg = steering_max_deg
        self.steering_invert  = steering_invert
        if steering_enabled:
            try:
                self.steering = SteeringMotor(hub_port=steering_hub_port)
            except Exception as e:
                print(f"[Steering] init failed: {e} — disabled")
                self.steering = None

        # State
        self.servo_val   = RC_FAILSAFE_SERVO
        self.gas_val     = RC_FAILSAFE_GAS
        self.gear_us     = GEAR_FAILSAFE_US
        self.gear_name   = "NEUTRAL"
        self.ch1_raw     = SBUS_MID
        self.ch3_raw     = 0
        self.throttle_pos  = THROTTLE_FAILSAFE_POS
        self.steering_deg  = 0.0
        self.current_mode  = "INIT"
        self.last_valid    = time.monotonic()
        self.sbus_ok       = False

        # Gear button rising-edge state
        self._ch6_prev = False
        self._ch7_prev = False
        self._ch8_prev = False

        # Brake-release / gear-shift interlocks
        self._prev_raw_mode      = "INIT"
        self._brake_release_time = 0.0
        self._gear_shift_time    = 0.0

    def close(self) -> None:
        self.running = False
        if self.throttle: self.throttle.close()
        if self.steering: self.steering.close()
        self.sbus.close()
        self.stm32.close()

    # ── Internal handlers ──────────────────────────────────────────────────────

    def _update_gear(self, ch6: int, ch7: int, ch8: int) -> None:
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

    def _handle_sbus_frame(self, decoded: dict, now: float) -> None:
        if decoded['failsafe'] or decoded['lost_frame']:
            self._apply_failsafe()
            self.current_mode = "FAILSAFE"
            return

        ch1 = decoded['channels'][0]
        ch3 = decoded['channels'][2]
        self.ch1_raw = ch1
        self.ch3_raw = ch3

        # CH1 → steering target
        self.steering_deg = ch1_to_steering_deg(
            ch1, self.steering_max_deg, self.steering_invert
        )

        # CH3 → split control (brake + gas)
        self.servo_val, self.gas_val, raw_mode = sbus_to_split_control(ch3, self.servo_val)

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

        # Gear via buttons
        self._update_gear(
            decoded['channels'][5],
            decoded['channels'][6],
            decoded['channels'][7],
        )

        # gas_val (from CH3 split) → throttle position
        self.throttle_pos = gas_val_to_throttle_position(self.gas_val)

        self.last_valid = now
        self.sbus_ok    = True

    def _apply_failsafe(self) -> None:
        self.servo_val   = RC_FAILSAFE_SERVO
        self.gas_val     = RC_FAILSAFE_GAS
        self.gear_us     = GEAR_FAILSAFE_US
        self.gear_name   = "NEUTRAL"
        self.throttle_pos = THROTTLE_FAILSAFE_POS
        self.steering_deg = 0.0
        self.sbus_ok      = False

    def _send_outputs(self) -> None:
        # STM32: CH1=brake servo, CH2=gas, CH3=gear servo
        frame = build_stm32_frame(self.servo_val, self.gas_val, self.gear_us, 600)
        self.stm32.write_frame(frame)

        # Throttle (always send — failsafe value is safe)
        if self.throttle:
            self.throttle.send_position(self.throttle_pos)

        # Steering (always send — failsafe = 0°)
        if self.steering:
            self.steering.send_target(self.steering_deg)

    def _print_status(self) -> None:
        if self.steering:
            self.steering.update_position()
        link = "LINK OK" if self.sbus_ok else "NO LINK"

        # Device-Health-Indikator
        flags = []
        flags.append("S" if self.sbus.alive  else "s")
        flags.append("M" if self.stm32.alive else "m")
        if self.throttle: flags.append("T" if self.throttle.alive else "t")
        if self.steering: flags.append("P" if self.steering.alive else "p")
        health = "".join(flags)

        if self.steering:
            steer_str = f"Steer:{self.steering_deg:+6.1f}°(act:{self.steering.position:+6.1f}°)"
        else:
            steer_str = "Steer:--"
        throt_str = f"Throt:{self.throttle_pos:4d}" if self.throttle else "Throt:--"

        print(
            f"[{health}] CH1:{self.ch1_raw:4d} CH3:{self.ch3_raw:4d} | "
            f"Brake:{self.servo_val:4d} Gas:{self.gas_val:4d} | "
            f"Gear:{self.gear_name:<7} | {throt_str} | {steer_str} | "
            f"{self.current_mode:<9} | {link}",
            end='\r',
        )

    # ── Main loop ──────────────────────────────────────────────────────────────

    def bridge_loop(self) -> None:
        print(f"\nBridge v4 active — STM32 @ {STM32_RATE_HZ}Hz, status @ {STATUS_PRINT_HZ}Hz")
        print(f"Steering range: ±{self.steering_max_deg:.0f}° (invert={self.steering_invert})")
        print("Devices flag: uppercase=alive, lowercase=disconnected (auto-reconnect every 2s)")
        print("S=SBUS  M=STM32  T=Throttle  P=Phidget(steering)")
        print("Ctrl+C to stop\n")

        period_send  = 1.0 / STM32_RATE_HZ
        period_print = 1.0 / STATUS_PRINT_HZ
        next_send    = time.monotonic()
        next_print   = time.monotonic()

        self.running = True

        while self.running:
            try:
                now = time.monotonic()

                # ── Reconnect attempts (cheap; gated by 2s timer per device) ──
                self.sbus.maybe_reconnect()
                self.stm32.maybe_reconnect()
                if self.throttle: self.throttle.maybe_reconnect()
                if self.steering: self.steering.maybe_reconnect()

                # ── Read SBUS (generator never raises) ────────────────────────
                for decoded in self.sbus.read_frames():
                    try:
                        self._handle_sbus_frame(decoded, now)
                    except Exception as e:
                        print(f"\n[bridge] sbus handler error: {e}")

                # ── Timeout failsafe (no valid SBUS for SBUS_TIMEOUT_S) ───────
                if now - self.last_valid > SBUS_TIMEOUT_S:
                    if self.sbus_ok or self.current_mode != "TIMEOUT":
                        # Erstmaliges Erreichen des Timeouts
                        pass
                    self._apply_failsafe()
                    self.current_mode = "TIMEOUT"

                # ── Send outputs at STM32_RATE_HZ ─────────────────────────────
                if now >= next_send:
                    self._send_outputs()
                    next_send += period_send
                    if next_send < now:        # falls weit hinterher: catch up
                        next_send = now + period_send

                # ── Print status at STATUS_PRINT_HZ ───────────────────────────
                if now >= next_print:
                    self._print_status()
                    next_print += period_print
                    if next_print < now:
                        next_print = now + period_print

            except KeyboardInterrupt:
                raise
            except Exception as e:
                # Letzte Verteidigung — eine Tick-Iteration darf nichts kaputt machen.
                print(f"\n[bridge] tick exception: {e}")
                traceback.print_exc()

            time.sleep(LOOP_SLEEP_S)


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='SBUS → STM32 RC Bridge v4 (real-time + crash-resistant)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument('--sbus-port',         default='/dev/sbus')
    parser.add_argument('--stm32-port',        default='/dev/stm32')
    parser.add_argument('--throttle-port',     default='/dev/waveshare')
    parser.add_argument('--throttle-id',       default=THROTTLE_SERVO_ID, type=int)
    parser.add_argument('--throttle-baud',     default=THROTTLE_BAUD,     type=int)
    parser.add_argument('--no-throttle',       action='store_true')
    parser.add_argument('--no-steering',       action='store_true')
    parser.add_argument('--steering-hub-port', default=STEERING_HUB_PORT, type=int)
    parser.add_argument('--max-steer-deg',     default=STEERING_DEFAULT_RANGE, type=float)
    parser.add_argument('--steering-invert',   action='store_true')
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
            steering_enabled=not args.no_steering,
            steering_hub_port=args.steering_hub_port,
            steering_max_deg=args.max_steer_deg,
            steering_invert=args.steering_invert,
        )
        bridge.bridge_loop()
    except KeyboardInterrupt:
        print("\nStopping...")
    except Exception as e:
        # Selbst hier nicht crashen — sauber loggen, dann beenden.
        print(f"\n[main] fatal error: {e}")
        traceback.print_exc()
    finally:
        if bridge is not None:
            try:
                bridge.close()
            except Exception:
                pass


if __name__ == '__main__':
    main()
