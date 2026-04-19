"""
Jetson-side simulator for the pwm-slave STM32 firmware.

Sends 11-byte RC frames over a serial port at 50 Hz, sweeping CH1 as a sine
wave so you can watch rc.ch1 change on the STM32 while CH2..4 hold at center.

Frame format (matches STM32 parser in Core/Src/main.c):
    [0]     0xAA
    [1]     0x55
    [2:3]   CH1 uint16 LE
    [4:5]   CH2 uint16 LE
    [6:7]   CH3 uint16 LE
    [8:9]   CH4 uint16 LE
    [10]    CRC8 Dallas/Maxim over bytes [2:10)

Usage:
    pip install pyserial
    python sim_jetson.py              # uses PORT below
    python sim_jetson.py COM7         # override port on the command line
"""

import math
import struct
import sys
import time

import serial

PORT = "COM4"      # default; Device Manager -> STLink Virtual COM Port (COMx)
BAUD = 115200
RATE_HZ = 50

# Servo test positions: fully retracted → mid → fully extended → mid
SERVO_POSITIONS = [300, 1100, 1600, 600]  # matches linear servo specs
HOLD_TIME_SEC = 5.0  # seconds to hold each position


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


def build_frame(ch1: int, ch2: int, ch3: int, ch4: int) -> bytes:
    payload = struct.pack("<HHHH", ch1, ch2, ch3, ch4)
    return bytes([0xAA, 0x55]) + payload + bytes([crc8_dallas(payload)])


def main() -> None:
    port = sys.argv[1] if len(sys.argv) > 1 else PORT
    ser = serial.Serial(port, BAUD, timeout=0)
    print(f"sending to {port} @ {BAUD} baud, {RATE_HZ} Hz")
    print("servo test sequence: retracextendedted(300) → mid(600) → extended(1600) → mid(600)")
    print("Ctrl+C to stop")

    period = 1.0 / RATE_HZ
    t0 = time.time()
    next_tick = t0
    frames = 0
    position_idx = 0
    last_position_change = t0

    try:
        while True:
            t = time.time() - t0

            # Change position every HOLD_TIME_SEC seconds
            if (t - (last_position_change - t0)) >= HOLD_TIME_SEC:
                position_idx = (position_idx + 1) % len(SERVO_POSITIONS)
                last_position_change = time.time()
                print(f"t={t:6.1f}s  switching to position {SERVO_POSITIONS[position_idx]} " +
                      f"({'retracted' if SERVO_POSITIONS[position_idx] == 300 else
                          'mid' if SERVO_POSITIONS[position_idx] == 600 else 'extended'})")

            ch1 = SERVO_POSITIONS[position_idx]
            frame = build_frame(ch1, 1024, 1024, 1024)
            ser.write(frame)
            frames += 1

            next_tick += period
            sleep_for = next_tick - time.time()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_tick = time.time()   # fell behind — resync, don't spiral
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        print("closed")


if __name__ == "__main__":
    main()
