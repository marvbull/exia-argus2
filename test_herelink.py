#!/usr/bin/env python3
"""
exia-argus2 — Herelink UART Test
Reads MAVLink from Herelink Air Unit via UART and prints live joystick + button state.
"""

import sys
import time
import threading
import argparse
from pymavlink import mavutil

UART_PORT = '/dev/ttyTHS1'
BAUD = 115200

# RC channel range expected from Herelink (microseconds)
RC_MIN, RC_MID, RC_MAX = 1000, 1500, 2000


def parse_buttons(bitmask: int) -> str:
    names = {0: 'A', 1: 'B', 2: 'C', 3: 'D', 4: 'Wheel'}
    pressed = [names[i] for i in names if bitmask & (1 << i)]
    return ', '.join(pressed) if pressed else '-'


def heartbeat_sender(conn, stop_event):
    """Send a GCS heartbeat at 1 Hz so the Herelink streams RC data."""
    while not stop_event.is_set():
        conn.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0,
        )
        stop_event.wait(1.0)


def sniff_mode(conn, duration: int):
    """Print every MAVLink message type seen for `duration` seconds."""
    print(f"Sniffing all MAVLink messages for {duration}s — move sticks now ...")
    print("-" * 60)
    deadline = time.monotonic() + duration
    seen: dict[str, int] = {}
    while time.monotonic() < deadline:
        msg = conn.recv_match(blocking=True, timeout=0.5)
        if msg is None:
            continue
        t = msg.get_type()
        seen[t] = seen.get(t, 0) + 1
        # Print RC-related messages in detail
        if t in ('MANUAL_CONTROL', 'RC_CHANNELS', 'RC_CHANNELS_RAW',
                  'RC_CHANNELS_OVERRIDE', 'RC_CHANNELS_SCALED'):
            print(f"  {t}: {msg.to_dict()}")
        else:
            print(f"  {t} (count={seen[t]})", end='\r')
    print()
    print("Message types seen:")
    for t, n in sorted(seen.items()):
        print(f"  {t:40s} x{n}")


def rc_channels_mode(conn):
    """Display RC_CHANNELS (Herelink often sends this instead of MANUAL_CONTROL)."""
    print("-" * 60)
    header = f"{'ch1':>6} {'ch2':>6} {'ch3':>6} {'ch4':>6} {'ch5':>6} {'ch6':>6} {'ch7':>6} {'ch8':>6}"
    print(header)
    print("-" * 60)
    while True:
        msg = conn.recv_match(type='RC_CHANNELS', blocking=True, timeout=2)
        if msg is None:
            print("  [timeout — no RC_CHANNELS received]")
            continue
        print(
            f"{msg.chan1_raw:>6d} {msg.chan2_raw:>6d} {msg.chan3_raw:>6d} {msg.chan4_raw:>6d}"
            f" {msg.chan5_raw:>6d} {msg.chan6_raw:>6d} {msg.chan7_raw:>6d} {msg.chan8_raw:>6d}",
            end='\r',
        )


def manual_control_mode(conn):
    """Display MANUAL_CONTROL (normalized joystick axes)."""
    print("-" * 60)
    print(f"{'x (pitch)':>12} {'y (roll)':>10} {'z (thr)':>9} {'r (yaw)':>9}  buttons")
    print("-" * 60)
    while True:
        msg = conn.recv_match(type='MANUAL_CONTROL', blocking=True, timeout=2)
        if msg is None:
            print("  [timeout — no MANUAL_CONTROL received]")
            continue
        print(
            f"{msg.x:>+12d} {msg.y:>+10d} {msg.z:>9d} {msg.r:>+9d}  {parse_buttons(msg.buttons)}",
            end='\r',
        )


def main(port: str, baud: int, mode: str, sniff_duration: int):
    print(f"Connecting to {port} @ {baud} baud ...")
    conn = mavutil.mavlink_connection(port, baud=baud, source_system=255)

    # Start sending heartbeats immediately so Herelink begins streaming
    stop_hb = threading.Event()
    hb_thread = threading.Thread(target=heartbeat_sender, args=(conn, stop_hb), daemon=True)
    hb_thread.start()

    print("Waiting for heartbeat ...")
    hb = conn.wait_heartbeat(timeout=10)
    if hb is None:
        print("ERROR: No heartbeat received. Check wiring and pairing.")
        sys.exit(1)

    print(f"Heartbeat OK — system={conn.target_system} component={conn.target_component}")

    try:
        if mode == 'sniff':
            sniff_mode(conn, sniff_duration)
        elif mode == 'rc':
            rc_channels_mode(conn)
        else:
            manual_control_mode(conn)
    finally:
        stop_hb.set()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Herelink UART MAVLink test')
    parser.add_argument('--port', default=UART_PORT, help=f'UART device (default: {UART_PORT})')
    parser.add_argument('--baud', type=int, default=BAUD, help=f'Baud rate (default: {BAUD})')
    parser.add_argument(
        '--mode', choices=['manual', 'rc', 'sniff'], default='manual',
        help='manual=MANUAL_CONTROL, rc=RC_CHANNELS, sniff=print all messages (default: manual)',
    )
    parser.add_argument(
        '--sniff-duration', type=int, default=15,
        help='Seconds to sniff in sniff mode (default: 15)',
    )
    args = parser.parse_args()

    try:
        main(args.port, args.baud, args.mode, args.sniff_duration)
    except KeyboardInterrupt:
        print("\nStopped.")
