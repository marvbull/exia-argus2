#!/usr/bin/env python3
"""
Phidget Steering RC Test — SBUS CH1 → DCC1000 Position Control

Liest CH1 vom Herelink (SBUS via FTDI) und mappt es auf ±MAX_STEER_DEG am Lenkmotor.
Verwendet wieder den hardwareseitigen Position-Controller (kein Software-PID).

Mapping:
    CH1 SBUS = 172  (Stick links)   → -MAX_STEER_DEG
    CH1 SBUS = 992  (Mitte)         →   0°
    CH1 SBUS = 1811 (Stick rechts)  → +MAX_STEER_DEG

Failsafe:
    SBUS-Failsafe-Flag, lost frame, oder >200ms ohne gültige Frame → Target 0°.

Usage:
    python3 test_phidget_steering_rc.py                    # Default ±200°, Hub-Port 3
    python3 test_phidget_steering_rc.py --max-steer 500    # größerer Lenkbereich
    python3 test_phidget_steering_rc.py --invert           # Lenkrichtung umkehren
"""

import sys
import time
import argparse
import signal
from contextlib import contextmanager
from typing import Optional

import serial

try:
    from Phidget22.Devices.MotorPositionController import MotorPositionController
    from Phidget22.PhidgetException import PhidgetException
except ImportError:
    print("FEHLER: Phidget22 Library nicht gefunden (pip install Phidget22)")
    sys.exit(1)


# ── SBUS config ───────────────────────────────────────────────────────────────
SBUS_PORT       = '/dev/sbus'
SBUS_BAUD       = 100000
SBUS_STARTBYTE  = 0x0F
SBUS_ENDBYTE    = 0x00
SBUS_FRAME_LEN  = 25
SBUS_MIN        = 172
SBUS_MID        = 992
SBUS_MAX        = 1811
SBUS_TIMEOUT_S  = 0.2

# ── Phidget config (übernommen aus test_phidget_steering.py) ─────────────────
HUB_PORT          = 3
ENCODER_PPR       = 300
QUADRATURE        = 4
GEARBOX_RATIO_1   = 4.25
GEARBOX_RATIO_2   = 76 / 13
RESCALE_FACTOR    = -360.0 / (ENCODER_PPR * QUADRATURE * GEARBOX_RATIO_1 * GEARBOX_RATIO_2)
MAX_STEERING_DEG  = 1750.0           # absolutes Sicherheitslimit (Lenkanschlag)

VELOCITY_LIMIT    = 400.0
ACCELERATION      = 1200.0
CURRENT_LIMIT     = 10.0
DEAD_BAND         = 3.0
KP                = 400.0
KI                = 0.0
KD                = 250.0

CONTROL_RATE_HZ   = 50
ATTACH_TIMEOUT_MS = 5000


# ── SBUS decode ───────────────────────────────────────────────────────────────

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
    ]
    flags = frame[23]
    return {
        'channels': raw,
        'failsafe': bool(flags & 0x08),
        'lost_frame': bool(flags & 0x04),
    }


def sync_to_sbus_frame(ser: serial.Serial) -> Optional[bytes]:
    buf = bytearray()
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        byte = ser.read(1)
        if not byte:
            continue
        buf.append(byte[0])
        if len(buf) >= SBUS_FRAME_LEN:
            for i in range(len(buf) - SBUS_FRAME_LEN + 1):
                cand = bytes(buf[i:i + SBUS_FRAME_LEN])
                if cand[0] == SBUS_STARTBYTE and cand[24] == SBUS_ENDBYTE:
                    return cand
            buf = buf[-(SBUS_FRAME_LEN - 1):]
    return None


# ── Mapping ───────────────────────────────────────────────────────────────────

def ch1_to_steering_deg(ch1: int, max_steer_deg: float, invert: bool) -> float:
    """SBUS CH1 (172..1811) → ±max_steer_deg, Mitte 992 → 0°."""
    ch1 = max(SBUS_MIN, min(SBUS_MAX, ch1))
    centered = ch1 - SBUS_MID
    half_range = max(SBUS_MAX - SBUS_MID, SBUS_MID - SBUS_MIN)  # 819
    normalized = centered / half_range          # -1.0 .. +1.0
    if invert:
        normalized = -normalized
    return normalized * max_steer_deg


# ── Phidget setup ─────────────────────────────────────────────────────────────

@contextmanager
def open_motor():
    m = MotorPositionController()
    m.setHubPort(HUB_PORT)
    m.setIsHubPortDevice(False)

    print(f"Verbinde mit DCC1000 auf Hub-Port {HUB_PORT}...")
    try:
        m.openWaitForAttachment(ATTACH_TIMEOUT_MS)
    except PhidgetException as e:
        print(f"FEHLER: Phidget-Attach fehlgeschlagen: {e}")
        sys.exit(1)

    try:
        m.setRescaleFactor(RESCALE_FACTOR)
        m.setCurrentLimit(CURRENT_LIMIT)
        m.setVelocityLimit(VELOCITY_LIMIT)
        m.setAcceleration(ACCELERATION)
        m.setDeadBand(DEAD_BAND)
        m.setKp(KP); m.setKi(KI); m.setKd(KD)
        try:
            m.addPositionOffset(-m.getPosition())
        except PhidgetException:
            pass
        m.setEngaged(True)
        time.sleep(0.2)

        print(f"Phidget bereit — Position auf 0 referenziert")
        yield m
    finally:
        try:
            m.setTargetPosition(0.0)
            time.sleep(0.3)
            m.setEngaged(False)
            m.close()
            print("\nPhidget disengaged und geschlossen.")
        except PhidgetException:
            pass


# ── Main loop ─────────────────────────────────────────────────────────────────

def run(max_steer_deg: float, invert: bool):
    print(f"Öffne SBUS: {SBUS_PORT} @ {SBUS_BAUD} (8E2)")
    sbus = serial.Serial(
        port=SBUS_PORT, baudrate=SBUS_BAUD,
        bytesize=serial.EIGHTBITS, parity=serial.PARITY_EVEN,
        stopbits=serial.STOPBITS_TWO, timeout=0.1,
    )
    print("Synce auf SBUS...")
    if sync_to_sbus_frame(sbus) is None:
        print("FEHLER: Keine SBUS-Frames empfangen")
        sbus.close()
        sys.exit(1)
    print("SBUS sync OK")

    with open_motor() as m:
        print(f"Bridge aktiv — CH1 → ±{max_steer_deg}° (invert={invert})")
        print("Ctrl+C zum Beenden\n")

        period       = 1.0 / CONTROL_RATE_HZ
        next_send    = time.monotonic()
        last_valid   = time.monotonic()
        ch1_raw      = SBUS_MID
        target_deg   = 0.0
        sbus_ok      = False
        buf          = bytearray()

        while True:
            now = time.monotonic()

            # SBUS lesen
            data = sbus.read(SBUS_FRAME_LEN * 2)
            if data:
                buf.extend(data)
                while len(buf) >= SBUS_FRAME_LEN:
                    if buf[0] != SBUS_STARTBYTE:
                        buf.pop(0); continue
                    frame = bytes(buf[:SBUS_FRAME_LEN])
                    decoded = decode_sbus_frame(frame)
                    if decoded is not None:
                        if not decoded['failsafe'] and not decoded['lost_frame']:
                            ch1_raw    = decoded['channels'][0]
                            target_deg = ch1_to_steering_deg(ch1_raw, max_steer_deg, invert)
                            last_valid = now
                            sbus_ok    = True
                        else:
                            target_deg = 0.0
                            sbus_ok    = False
                    buf = buf[SBUS_FRAME_LEN:]

            # Timeout-Failsafe
            if now - last_valid > SBUS_TIMEOUT_S:
                target_deg = 0.0
                sbus_ok    = False

            # Mit Control-Rate an Phidget senden
            if now >= next_send:
                try:
                    m.setTargetPosition(target_deg)
                    actual = m.getPosition()
                except PhidgetException as e:
                    actual = float('nan')
                    print(f"\n[Phidget-Fehler] {e}")

                next_send += period
                link = "LINK OK " if sbus_ok else "NO LINK!"
                print(
                    f"CH1: {ch1_raw:4d} | Target: {target_deg:+7.1f}° | "
                    f"Pos: {actual:+7.1f}° | err: {target_deg - actual:+6.1f}° | {link}",
                    end='\r',
                )

            time.sleep(0.001)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--max-steer', type=float, default=200.0,
                   help='maximaler Lenkausschlag in Grad bei Stick voll (default: 200)')
    p.add_argument('--invert',    action='store_true',
                   help='Lenkrichtung umkehren')
    args = p.parse_args()

    if abs(args.max_steer) > MAX_STEERING_DEG:
        print(f"FEHLER: --max-steer {args.max_steer} > {MAX_STEERING_DEG} (Lenkanschlag)")
        sys.exit(1)

    signal.signal(signal.SIGINT, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))

    try:
        run(args.max_steer, args.invert)
    except KeyboardInterrupt:
        print("\nAbbruch durch Benutzer.")
    except serial.SerialException as e:
        print(f"\nSerial-Fehler: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()
