#!/usr/bin/env python3
"""
exia-argus2 — SBUS Reader for Jetson
Reads SBUS from Herelink Air Unit SBUS-output pin via UART.

Wiring:
    Herelink SBUS-out → Hardware inverter → Jetson UART RX (/dev/ttyTHS2)

SBUS protocol:
    Baud:    100000
    Format:  8E2  (8 data bits, Even parity, 2 stop bits)
    Signal:  Inverted (requires hardware inverter or inverting level shifter)
    Frame:   25 bytes @ ~7ms intervals
             [0]    = 0x0F  (start byte)
             [1-22] = 16 channels, 11 bits each (packed)
             [23]   = flags (failsafe, lost frame, ch17, ch18)
             [24]   = 0x00  (end byte)
    Range:   172 (min) … 992 (center) … 1811 (max)  → mapped to 1000-2000 µs

Usage:
    python test_sbus.py                        # default /dev/ttyTHS2
    python test_sbus.py --port /dev/ttyTHS1
    python test_sbus.py --port /dev/ttyTHS2 --raw
"""

import sys
import time
import argparse
import serial

SBUS_PORT      = '/dev/ttyTHS2'
SBUS_BAUD      = 100000
SBUS_STARTBYTE = 0x0F
SBUS_ENDBYTE   = 0x00
SBUS_FRAME_LEN = 25

# Raw SBUS value range
SBUS_MIN = 172
SBUS_MID = 992
SBUS_MAX = 1811

# Output µs range
RC_MIN = 1000
RC_MID = 1500
RC_MAX = 2000


def sbus_to_us(value: int) -> int:
    """Map raw SBUS value to microseconds."""
    clamped = max(SBUS_MIN, min(SBUS_MAX, value))
    return int(RC_MIN + (clamped - SBUS_MIN) / (SBUS_MAX - SBUS_MIN) * (RC_MAX - RC_MIN))


def decode_frame(frame: bytes) -> dict | None:
    """Decode a 25-byte SBUS frame. Returns dict with channels and flags."""
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
        'channels':    raw,
        'ch17':        bool(flags & 0x01),
        'ch18':        bool(flags & 0x02),
        'lost_frame':  bool(flags & 0x04),
        'failsafe':    bool(flags & 0x08),
    }


def open_sbus_port(port: str) -> serial.Serial:
    return serial.Serial(
        port     = port,
        baudrate = SBUS_BAUD,
        bytesize = serial.EIGHTBITS,
        parity   = serial.PARITY_EVEN,
        stopbits = serial.STOPBITS_TWO,
        timeout  = 0.1,
    )


def sync_to_frame(ser: serial.Serial) -> bytes | None:
    """Read bytes until we find a valid frame start."""
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


def main():
    parser = argparse.ArgumentParser(
        description='SBUS reader for Herelink Air Unit',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument('--port', default=SBUS_PORT,
                        help='UART device connected to Herelink SBUS output')
    parser.add_argument('--raw', action='store_true',
                        help='Show raw SBUS values instead of µs')
    args = parser.parse_args()

    print(f"Opening {args.port} @ {SBUS_BAUD} baud (8E2, inverted SBUS) ...")
    try:
        ser = open_sbus_port(args.port)
    except serial.SerialException as e:
        print(f"ERROR: {e}")
        sys.exit(1)

    print("Syncing to SBUS frame ...")
    first = sync_to_frame(ser)
    if first is None:
        print("ERROR: No valid SBUS frame found.")
        print()
        print("Check:")
        print("  1. Herelink SBUS-out pin wired to Jetson RX")
        print("  2. Hardware inverter in the signal path")
        print("  3. Correct UART device (try --port /dev/ttyTHS1)")
        sys.exit(1)

    print("SBUS OK!")
    print()

    hdr = "  ch1   ch2   ch3   ch4   ch5   ch6   ch7   ch8  flags"
    print(hdr)
    print("-" * len(hdr))

    buf = bytearray()
    while True:
        data = ser.read(SBUS_FRAME_LEN)
        if not data:
            continue

        buf.extend(data)

        while len(buf) >= SBUS_FRAME_LEN:
            if buf[0] != SBUS_STARTBYTE:
                buf.pop(0)
                continue

            frame = bytes(buf[:SBUS_FRAME_LEN])
            decoded = decode_frame(frame)
            if decoded is None:
                buf.pop(0)
                continue

            buf = buf[SBUS_FRAME_LEN:]
            ch = decoded['channels']

            if args.raw:
                vals = [f"{v:5d}" for v in ch[:8]]
            else:
                vals = [f"{sbus_to_us(v):5d}" for v in ch[:8]]

            flags = ""
            if decoded['failsafe']:
                flags += " FAILSAFE"
            if decoded['lost_frame']:
                flags += " LOST"

            print("  " + "  ".join(vals) + "  " + (flags or "OK"), end='\r')


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")
