#!/usr/bin/env python3
"""
exia-argus2 — MAVLink Stream Request + RC Diagnostic
Connects to a MAVLink device, sends GCS heartbeats, requests all data streams,
then prints every message received (highlighting RC-related ones).

Usage:
    python test_mavlink_stream.py                        # default /dev/ttyTHS1 @ 115200
    python test_mavlink_stream.py --port /dev/ttyTHS1 --baud 57600
    python test_mavlink_stream.py --sniff-only           # skip stream requests, just listen
    python test_mavlink_stream.py --duration 30
"""

import sys
import time
import threading
import argparse
from pymavlink import mavutil, mavlink as ml

UART_PORT  = '/dev/ttyTHS1'
BAUD       = 115200
HB_RATE_HZ = 1          # GCS heartbeat rate
RC_RATE_HZ = 10         # requested RC stream rate

RC_TYPES = {
    'MANUAL_CONTROL',
    'RC_CHANNELS',
    'RC_CHANNELS_RAW',
    'RC_CHANNELS_SCALED',
    'RC_CHANNELS_OVERRIDE',
}

# MAVLink data stream IDs and human names
STREAMS = [
    (ml.MAV_DATA_STREAM_ALL,         'ALL'),
    (ml.MAV_DATA_STREAM_RC_CHANNELS, 'RC_CHANNELS'),
    (ml.MAV_DATA_STREAM_RAW_SENSORS, 'RAW_SENSORS'),
    (ml.MAV_DATA_STREAM_EXTENDED_STATUS, 'EXTENDED_STATUS'),
    (ml.MAV_DATA_STREAM_RAW_CONTROLLER, 'RAW_CONTROLLER'),
    (ml.MAV_DATA_STREAM_POSITION,    'POSITION'),
    (ml.MAV_DATA_STREAM_EXTRA1,      'EXTRA1'),
    (ml.MAV_DATA_STREAM_EXTRA2,      'EXTRA2'),
    (ml.MAV_DATA_STREAM_EXTRA3,      'EXTRA3'),
]


# ---------------------------------------------------------------------------
# Background heartbeat sender
# ---------------------------------------------------------------------------

def heartbeat_sender(conn, stop: threading.Event):
    while not stop.is_set():
        conn.mav.heartbeat_send(
            ml.MAV_TYPE_GCS,
            ml.MAV_AUTOPILOT_INVALID,
            0, 0, 0,
        )
        stop.wait(1.0 / HB_RATE_HZ)


# ---------------------------------------------------------------------------
# Stream request
# ---------------------------------------------------------------------------

def request_streams(conn, rate_hz: int, sniff_only: bool):
    if sniff_only:
        print("[sniff-only] Skipping stream requests.")
        return

    print(f"Requesting {len(STREAMS)} data streams at {rate_hz} Hz ...")
    for stream_id, name in STREAMS:
        conn.mav.request_data_stream_send(
            conn.target_system,
            conn.target_component,
            stream_id,
            rate_hz,
            1,          # start
        )
        print(f"  → {name} (id={stream_id})")
    print()


# ---------------------------------------------------------------------------
# Main listener loop
# ---------------------------------------------------------------------------

def listen(conn, duration: int):
    print(f"Listening for {duration}s — move sticks and press buttons now ...")
    print("=" * 70)

    counts: dict[str, int] = {}
    deadline = time.monotonic() + duration

    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        msg = conn.recv_match(blocking=True, timeout=min(1.0, remaining))
        if msg is None:
            continue

        t = msg.get_type()
        counts[t] = counts.get(t, 0) + 1

        if t in RC_TYPES:
            # Print RC messages in full, highlighted
            print(f"\n*** RC DATA [{t}] ***")
            d = msg.to_dict()
            d.pop('mavpackettype', None)
            for k, v in d.items():
                print(f"    {k}: {v}")
        elif t not in ('HEARTBEAT', 'RADIO_STATUS', 'TIMESYNC', 'SCALED_PRESSURE'):
            # Print other non-noise messages once per new type
            if counts[t] == 1:
                print(f"[new] {t}: {msg.to_dict()}")

    print()
    print("=" * 70)
    print("Summary — message types received:")
    for t, n in sorted(counts.items(), key=lambda x: -x[1]):
        flag = " *** RC DATA ***" if t in RC_TYPES else ""
        print(f"  {t:45s} x{n:4d}{flag}")

    if not any(t in RC_TYPES for t in counts):
        print()
        print("! No RC data received. See configuration notes below.")
        print_config_help()


# ---------------------------------------------------------------------------
# Configuration help
# ---------------------------------------------------------------------------

def print_config_help():
    print("""
HOW TO GET RC DATA ON THIS UART
================================

Option 1 — Herelink web UI (most likely fix)
----------------------------------------------
1. Connect your laptop to the Herelink Air Unit's WiFi AP
   (SSID: "Herelink-XXXXXX", password on the unit label)
2. Open http://192.168.144.10 in a browser
3. Go to  Settings → MAVLink Routing
4. Find the serial port that your Jetson is connected to
   (UART_A = /dev/ttyS1 on Herelink, UART_B = /dev/ttyS3)
5. Enable "Forward RC" or set input source to include RC_CHANNELS
6. Also check: Settings → Serial Ports → baud rate matches 115200

Option 2 — QGroundControl stream request (temporary / test)
-------------------------------------------------------------
In QGC: Widgets → MAVLink Inspector or Vehicle Setup → Parameters
Set stream rates via MAVLink shell:
  param set MAV_EXTRA3_RATE 10   (some FW expose RC via EXTRA3)

Option 3 — Verify physical wiring
-----------------------------------
Herelink Air Unit pinout:
  FC Port   TX/RX → your flight controller (RC data goes HERE as input)
  UART_A    TX=pin5 RX=pin6 → payload/companion (needs routing config)
  UART_B    TX=pin9 RX=pin10 → second payload port

If /dev/ttyTHS1 is wired to the FC port you may be seeing telemetry
coming FROM the FC, not from the Herelink controller. In that case
the RC stream goes INTO the FC and won't be echoed back.

Option 4 — Use SBUS/PPM instead of MAVLink RC
-----------------------------------------------
Herelink also outputs SBUS on a dedicated pin. Wiring the SBUS output
to a GPIO or a SBUS-capable UART may be simpler than MAVLink RC routing.
""")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description='MAVLink stream request + RC diagnostic',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument('--port',       default=UART_PORT)
    parser.add_argument('--baud',       type=int, default=BAUD)
    parser.add_argument('--rate',       type=int, default=RC_RATE_HZ,
                        help='Requested stream rate in Hz')
    parser.add_argument('--duration',   type=int, default=20,
                        help='Listen duration in seconds')
    parser.add_argument('--sniff-only', action='store_true',
                        help='Do not send stream requests; just listen')
    args = parser.parse_args()

    print(f"Connecting to {args.port} @ {args.baud} baud ...")
    conn = mavutil.mavlink_connection(args.port, baud=args.baud, source_system=255)

    stop_hb = threading.Event()
    hb_thread = threading.Thread(target=heartbeat_sender, args=(conn, stop_hb), daemon=True)
    hb_thread.start()
    print("GCS heartbeat thread started.")

    print("Waiting for heartbeat ...")
    hb = conn.wait_heartbeat(timeout=10)
    if hb is None:
        print("ERROR: No heartbeat. Check wiring and pairing.")
        sys.exit(1)
    print(f"Heartbeat OK — system={conn.target_system} component={conn.target_component}")
    print()

    try:
        request_streams(conn, args.rate, args.sniff_only)
        # Small delay so stream requests are processed before we start counting
        time.sleep(0.5)
        listen(conn, args.duration)
    finally:
        stop_hb.set()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")
