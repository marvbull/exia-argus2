#!/usr/bin/env python3
"""
Gear-Servo Calibration Tool

Schickt kontinuierlich Frames an den STM32 und lässt dich den PWM-µs für
den Gear-Servo (CH3 im STM32-Frame, PA1 / TIM5_CH2) live einstellen.

Brake bleibt auf 1150µs (gezogen) und Gas auf 300 (aus) — sicher zum Testen.
Sendet mit 50 Hz im Hintergrund, sonst greift die 200-ms-Failsafe der Firmware.

Usage:
    python3 test_gear_calibrate.py

Workflow:
  1. Wert eingeben → Servo fährt → notieren bei welcher Position er physisch ist
  2. Nochmal anderen Wert → ...
  3. Wenn alle drei Gänge (NEUTRAL, DRIVE, REVERSE) identifiziert: notieren und
     in jetson_to_stm32_bridge_v3.py die Konstanten anpassen:
       GEAR_NEUTRAL_US, GEAR_DRIVE_US, GEAR_REVERSE_US, GEAR_FAILSAFE_US
"""

import serial
import struct
import threading
import time
import sys

STM32_PORT  = '/dev/stm32'
STM32_BAUD  = 115200
RATE_HZ     = 50
FRAME_SYNC  = [0xAA, 0x55]

# Firmware-Limits (Core/Src/main.c: PWM_GEAR_MIN_US / PWM_GEAR_MAX_US)
GEAR_MIN_US = 1100
GEAR_MAX_US = 1900

# Sichere Defaults für die anderen Kanäle während der Kalibrierung
SAFE_BRAKE  = 1150     # Bremse fest gezogen
SAFE_GAS    = 300      # kein Gas (legacy STM32-Pfad)
SAFE_CH4    = 600


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
    return bytes(FRAME_SYNC) + payload + bytes([crc8_dallas(payload)])


class Sender(threading.Thread):
    """Sendet Frames mit aktuellem Gear-Wert im Hintergrund — verhindert Failsafe."""
    def __init__(self, stm: serial.Serial):
        super().__init__(daemon=True)
        self.stm    = stm
        self.gear_us = 1500
        self.running = True
        self.lock   = threading.Lock()

    def set_gear(self, us: int) -> None:
        with self.lock:
            self.gear_us = us

    def get_gear(self) -> int:
        with self.lock:
            return self.gear_us

    def run(self) -> None:
        period = 1.0 / RATE_HZ
        next_t = time.monotonic()
        while self.running:
            with self.lock:
                us = self.gear_us
            frame = build_frame(SAFE_BRAKE, SAFE_GAS, us, SAFE_CH4)
            try:
                self.stm.write(frame)
            except Exception:
                pass
            next_t += period
            sleep_s = next_t - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                next_t = time.monotonic()


def main():
    print(f"Opening STM32: {STM32_PORT} @ {STM32_BAUD}")
    try:
        stm = serial.Serial(STM32_PORT, STM32_BAUD, timeout=0)
    except serial.SerialException as e:
        print(f"FEHLER: {e}")
        sys.exit(1)

    sender = Sender(stm)
    sender.start()
    time.sleep(0.1)   # erste paar Frames raussenden bevor User tippt

    print()
    print("="*60)
    print(" Gear-Servo-Kalibrierung")
    print("="*60)
    print(f"  Brake : {SAFE_BRAKE}µs (gezogen)   Gas: {SAFE_GAS} (aus)")
    print(f"  Gear  : startet bei {sender.get_gear()}µs")
    print(f"  Range : {GEAR_MIN_US}..{GEAR_MAX_US}µs (Firmware-Limits)")
    print()
    print("Befehle:")
    print("  <µs>      Gear-Wert direkt setzen     (z.B. 1500)")
    print("  +50, -100 relativ ändern              (z.B. +50, -100)")
    print("  s         aktuellen Wert zeigen")
    print("  q         Beenden")
    print()
    print("Typische Servo-Werte zum Probieren:")
    print("  1100 (volle linke Position)")
    print("  1300")
    print("  1500 (Mitte)")
    print("  1700")
    print("  1900 (volle rechte Position)")
    print()
    print("Hinweis: Während du tippst läuft der Servo-Stream im Hintergrund weiter,")
    print("d.h. Brake bleibt gezogen und der Servo hält seine letzte Position.")
    print()

    try:
        while True:
            try:
                inp = input(f"[gear={sender.get_gear()}µs] > ").strip()
            except (EOFError, KeyboardInterrupt):
                break

            if not inp:
                continue
            if inp.lower() in ('q', 'quit', 'exit'):
                break
            if inp.lower() == 's':
                print(f"  Aktuell: {sender.get_gear()}µs")
                continue

            try:
                if inp[0] in ('+', '-'):
                    delta = int(inp)
                    new_val = sender.get_gear() + delta
                else:
                    new_val = int(inp)
            except ValueError:
                print(f"  '{inp}' ist keine Zahl")
                continue

            if new_val < GEAR_MIN_US or new_val > GEAR_MAX_US:
                print(f"  ABGELEHNT: {new_val}µs außerhalb {GEAR_MIN_US}..{GEAR_MAX_US} "
                      f"(Firmware würde clampen)")
                continue

            sender.set_gear(new_val)
            print(f"  → {new_val}µs gesendet")

    finally:
        # Servo zur Sicherheit auf NEUTRAL-Default fahren bevor wir aufhören
        sender.set_gear(1500)
        time.sleep(0.2)
        sender.running = False
        sender.join(timeout=1.0)
        stm.close()
        print("\nFertig. STM32 erhält keine Frames mehr → Firmware-Failsafe greift "
              "nach 200ms (Brake ON, Gear NEUTRAL).")


if __name__ == '__main__':
    main()
