#!/usr/bin/env python3
"""Show live SBUS channel values. Press Ctrl+C to stop."""
import time, serial, argparse

SBUS_BAUD, SBUS_FRAME_LEN = 100000, 25

def decode(frame):
    b = frame[1:23]
    raw = [
        ((b[0]       | b[1]  << 8) & 0x07FF),
        ((b[1]  >> 3 | b[2]  << 5) & 0x07FF),
        ((b[2]  >> 6 | b[3]  << 2 | b[4] << 10) & 0x07FF),
        ((b[4]  >> 1 | b[5]  << 7) & 0x07FF),
        ((b[5]  >> 4 | b[6]  << 4) & 0x07FF),
    ]
    fs = bool(frame[23] & 0x08)
    return raw, fs

parser = argparse.ArgumentParser()
parser.add_argument('--port', default='/dev/sbus')
args = parser.parse_args()

ser = serial.Serial(args.port, SBUS_BAUD, bytesize=8, parity='E', stopbits=2, timeout=0.1)
buf = bytearray()
print("Watching CH1-CH5 (move your sticks)... Ctrl+C to stop\n")

while True:
    buf.extend(ser.read(50))
    while len(buf) >= SBUS_FRAME_LEN:
        if buf[0] != 0x0F:
            buf.pop(0); continue
        frame = bytes(buf[:SBUS_FRAME_LEN])
        if frame[24] == 0x00:
            ch, fs = decode(frame)
            pos = int((max(172, min(1811, ch[1])) - 172) / (1811 - 172) * 4095)
            print(f"CH1:{ch[0]:4d} CH2:{ch[1]:4d}→pos:{pos:4d} CH3:{ch[2]:4d} CH4:{ch[3]:4d} CH5:{ch[4]:4d}  {'FAILSAFE' if fs else '        '}", end='\r')
        buf = buf[SBUS_FRAME_LEN:]
