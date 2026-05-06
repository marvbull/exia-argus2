#!/usr/bin/env python3
"""
Minimal test: move Waveshare Serial Bus Servo to 3 positions.
Usage: python3 test_throttle_servo.py --port /dev/waveshare
"""
import time
import argparse
import serial

SERVO_ID = 1
BAUD     = 1000000

def checksum(servo_id, length, instruction, params):
    return (~(servo_id + length + instruction + sum(params))) & 0xFF

def write_cmd(ser, servo_id, address, data):
    params = bytes([address]) + bytes(data)
    instr  = 0x03
    length = len(params) + 2
    ck     = checksum(servo_id, length, instr, params)
    pkt    = bytes([0xFF, 0xFF, servo_id, length, instr]) + params + bytes([ck])
    print(f"  TX: {list(pkt)}")
    ser.write(pkt)
    ser.flush()
    time.sleep(0.05)

def torque_enable(ser, servo_id):
    print("→ Torque enable")
    write_cmd(ser, servo_id, 0x28, [0x01])

def move_to(ser, servo_id, position, speed=1500, acc=50):
    print(f"→ Move to position {position}")
    pos = max(0, min(4095, position))
    data = [
        acc & 0xFF,
        pos & 0xFF,
        (pos >> 8) & 0xFF,
        0x00, 0x00,          # time = 0
        speed & 0xFF,
        (speed >> 8) & 0xFF,
    ]
    write_cmd(ser, servo_id, 0x29, data)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default='/dev/waveshare')
    parser.add_argument('--id',   default=1, type=int)
    args = parser.parse_args()

    print(f"Opening {args.port} @ {BAUD} baud")
    ser = serial.Serial(args.port, BAUD, timeout=0.1)
    time.sleep(0.5)

    torque_enable(ser, args.id)
    time.sleep(0.5)

    print("\nTest sequence:")
    move_to(ser, args.id, 0)
    time.sleep(2)
    move_to(ser, args.id, 2048)
    time.sleep(2)
    move_to(ser, args.id, 4095)
    time.sleep(2)
    move_to(ser, args.id, 0)
    time.sleep(2)

    ser.close()
    print("Done.")

if __name__ == '__main__':
    main()
