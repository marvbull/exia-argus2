#!/usr/bin/env python3
"""
exia-argus2 — Autopilot Heartbeat Test
Meldet sich als Autopilot (statt GCS) gegenüber dem Herelink,
um RC-Routing freizuschalten.
"""

import sys
import time
import threading
import argparse
from pymavlink import mavutil

UART_PORT = '/dev/ttyTHS1'
BAUD      = 115200

RC_TYPES = {
    'MANUAL_CONTROL',
    'RC_CHANNELS',
    'RC_CHANNELS_RAW',
    'RC_CHANNELS_SCALED',
    'RC_CHANNELS_OVERRIDE',
}

NOISE = {'HEARTBEAT', 'RADIO_STATUS', 'TIMESYNC', 'SCALED_PRESSURE'}


def heartbeat_sender(conn, stop: threading.Event):
    while not stop.is_set():
        conn.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_QUADROTOR,
            mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
            0, 0, 0,
        )
        stop.wait(0.5)


def main(port: str, baud: int, duration: int):
    print(f"Connecting to {port} @ {baud} baud ...")
    conn = mavutil.mavlink_connection(port, baud=baud, source_system=1)

    stop = threading.Event()
    hb = threading.Thread(target=heartbeat_sender, args=(conn, stop), daemon=True)
    hb.start()
    print("Sending autopilot heartbeats (MAV_TYPE_QUADROTOR) ...")

    print("Waiting for heartbeat from Herelink ...")
    hb_msg = conn.wait_heartbeat(timeout=10)
    if hb_msg is None:
        print("ERROR: No heartbeat received.")
        sys.exit(1)
    print(f"Heartbeat OK — system={conn.target_system} component={conn.target_component}")
    print()
    print(f"Listening for {duration}s — move sticks now ...")
    print("=" * 60)

    counts = {}
    deadline = time.monotonic() + duration

    try:
        while time.monotonic() < deadline:
            msg = conn.recv_match(blocking=True, timeout=0.5)
            if msg is None:
                continue

            t = msg.get_type()
            counts[t] = counts.get(t, 0) + 1

            if t in RC_TYPES:
                print(f"\n*** RC DATA [{t}] ***")
                d = msg.to_dict()
                d.pop('mavpackettype', None)
                for k, v in d.items():
                    print(f"    {k}: {v}")
            elif t not in NOISE:
                if counts[t] == 1:
                    print(f"[new] {t}: {msg.to_dict()}")
    finally:
        stop.set()

    print()
    print("=" * 60)
    print("Empfangene Message-Typen:")
    for t, n in sorted(counts.items(), key=lambda x: -x[1]):
        flag = " *** RC ***" if t in RC_TYPES else ""
        print(f"  {t:45s} x{n:4d}{flag}")

    if not any(t in RC_TYPES for t in counts):
        print()
        print("Keine RC-Daten empfangen.")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Autopilot-Heartbeat Test — RC-Routing vom Herelink freischalten',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument('--port',     default=UART_PORT)
    parser.add_argument('--baud',     type=int, default=BAUD)
    parser.add_argument('--duration', type=int, default=20,
                        help='Lausch-Dauer in Sekunden')
    args = parser.parse_args()

    try:
        main(args.port, args.baud, args.duration)
    except KeyboardInterrupt:
        print("\nStopped.")
