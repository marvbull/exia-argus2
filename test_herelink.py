#!/usr/bin/env python3
"""
exia-argus2 — Herelink UART Test
Reads MAVLink from Herelink Air Unit via UART and prints live joystick + button state.
"""

import sys
import argparse
from pymavlink import mavutil

UART_PORT = '/dev/ttyTHS1'
BAUD = 115200


def parse_buttons(bitmask: int) -> str:
    names = {0: 'A', 1: 'B', 2: 'C', 3: 'D', 4: 'Wheel'}
    pressed = [names[i] for i in names if bitmask & (1 << i)]
    return ', '.join(pressed) if pressed else '-'


def main(port: str, baud: int):
    print(f"Connecting to {port} @ {baud} baud ...")
    conn = mavutil.mavlink_connection(port, baud=baud)

    print("Waiting for heartbeat ...")
    hb = conn.wait_heartbeat(timeout=10)
    if hb is None:
        print("ERROR: No heartbeat received. Check wiring and pairing.")
        sys.exit(1)

    print(f"Heartbeat OK — system={conn.target_system} component={conn.target_component}")
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
            end='\r'
        )


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Herelink UART MAVLink test')
    parser.add_argument('--port', default=UART_PORT, help=f'UART device (default: {UART_PORT})')
    parser.add_argument('--baud', type=int, default=BAUD, help=f'Baud rate (default: {BAUD})')
    args = parser.parse_args()

    try:
        main(args.port, args.baud)
    except KeyboardInterrupt:
        print("\nStopped.")
