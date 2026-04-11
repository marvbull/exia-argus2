#!/usr/bin/env python3
"""
exia-argus2 — Herelink RC Bridge
Reads SBUS channel values from the Herelink Air Unit via ADB,
converts them to MAVLink RC_CHANNELS and either:
  - prints them live (--mode print)
  - outputs MAVLink on a UART (--mode uart --port /dev/ttyTHS1)
  - serves them on a UDP port (--mode udp --udp-port 14560)

Usage:
    python herelink_rc_bridge.py                          # print mode
    python herelink_rc_bridge.py --mode uart --port /dev/ttyTHS1
    python herelink_rc_bridge.py --mode udp --udp-port 14560
"""

import sys
import time
import socket
import threading
import argparse
import subprocess
from pymavlink import mavutil

ADB_DEVICE   = '192.168.43.1:5555'
SBUS_FILE    = '/tmp/sbus_values'
POLL_HZ      = 50          # how often to read sbus_values per second
SBUS_MIN     = 0
SBUS_MID     = 1024
SBUS_MAX     = 2047
RC_MIN       = 1000        # µs
RC_MID       = 1500
RC_MAX       = 2000


# ---------------------------------------------------------------------------
# SBUS reader
# ---------------------------------------------------------------------------

def read_sbus_values(device: str) -> list[int] | None:
    """Read 32 SBUS channel values from the Herelink via ADB."""
    try:
        result = subprocess.run(
            ['adb', '-s', device, 'shell', f'cat {SBUS_FILE}'],
            capture_output=True, text=True, timeout=1,
        )
        if result.returncode != 0:
            return None
        values = [int(v) for v in result.stdout.strip().split(',')]
        return values if len(values) >= 16 else None
    except Exception:
        return None


def sbus_to_us(value: int) -> int:
    """Convert raw SBUS value (0-2047) to microseconds (1000-2000)."""
    clamped = max(SBUS_MIN, min(SBUS_MAX, value))
    return int(RC_MIN + (clamped / SBUS_MAX) * (RC_MAX - RC_MIN))


# ---------------------------------------------------------------------------
# MAVLink sender
# ---------------------------------------------------------------------------

class MavlinkSender:
    def __init__(self, mode: str, port: str, udp_port: int):
        self.mode = mode
        if mode == 'uart':
            print(f"Opening MAVLink UART on {port} ...")
            self.conn = mavutil.mavlink_connection(port, baud=115200, source_system=1)
        elif mode == 'udp':
            print(f"Opening MAVLink UDP on 0.0.0.0:{udp_port} ...")
            self.conn = mavutil.mavlink_connection(f'udpout:127.0.0.1:{udp_port}', source_system=1)
        else:
            self.conn = None

    def send(self, channels: list[int]):
        if self.conn is None:
            return
        # Pad to 18 channels
        ch = channels[:18] + [65535] * (18 - min(len(channels), 18))
        self.conn.mav.rc_channels_send(
            int(time.time() * 1000) & 0xFFFFFFFF,  # time_boot_ms
            min(len(channels), 18),                 # chancount
            *ch[:18],                               # chan1..chan18_raw
            0,                                      # rssi (unknown)
        )


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

def print_header():
    print(f"{'ch1':>6} {'ch2':>6} {'ch3':>6} {'ch4':>6} "
          f"{'ch5':>6} {'ch6':>6} {'ch7':>6} {'ch8':>6}  (µs)")
    print("-" * 60)


def main():
    parser = argparse.ArgumentParser(
        description='Herelink SBUS → MAVLink RC bridge',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument('--adb-device', default=ADB_DEVICE,
                        help='ADB device address')
    parser.add_argument('--mode', choices=['print', 'uart', 'udp'], default='print',
                        help='Output mode')
    parser.add_argument('--port', default='/dev/ttyTHS1',
                        help='UART device (used with --mode uart)')
    parser.add_argument('--udp-port', type=int, default=14560,
                        help='UDP port (used with --mode udp)')
    parser.add_argument('--rate', type=int, default=POLL_HZ,
                        help='Poll rate in Hz')
    args = parser.parse_args()

    # Verify ADB connection
    print(f"Checking ADB connection to {args.adb_device} ...")
    test = read_sbus_values(args.adb_device)
    if test is None:
        print(f"ERROR: Cannot read {SBUS_FILE} via ADB. Is Herelink connected?")
        print(f"  Run: adb connect {args.adb_device}")
        sys.exit(1)
    print(f"ADB OK — got {len(test)} channels")

    sender = MavlinkSender(args.mode, args.port, args.udp_port)

    if args.mode == 'print':
        print_header()

    interval = 1.0 / args.rate
    errors = 0

    while True:
        t0 = time.monotonic()

        values = read_sbus_values(args.adb_device)

        if values is None:
            errors += 1
            if errors % 10 == 1:
                print(f"  [read error #{errors} — check ADB connection]")
            time.sleep(interval)
            continue

        errors = 0
        us_values = [sbus_to_us(v) for v in values]

        if args.mode == 'print':
            print(
                f"{us_values[0]:>6d} {us_values[1]:>6d} {us_values[2]:>6d} {us_values[3]:>6d} "
                f"{us_values[4]:>6d} {us_values[5]:>6d} {us_values[6]:>6d} {us_values[7]:>6d}",
                end='\r',
            )
        else:
            sender.send(us_values)

        elapsed = time.monotonic() - t0
        sleep_for = interval - elapsed
        if sleep_for > 0:
            time.sleep(sleep_for)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")
