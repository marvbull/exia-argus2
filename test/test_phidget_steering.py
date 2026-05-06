#!/usr/bin/env python3
"""
Phidget Steering Test (DCC1000 + HKT22)

Testet den Lenkmotor mit hardwareseitigem PID (MotorPositionController).
Bewusst klein gehalten — kein selbstgebauter Software-PID wie im ROS2-Node.

Hardware:
    VINT Hub  → DCC1000 (Motor Controller)
              → HKT22  (Hall Encoder, am gleichen DCC1000 angeschlossen)

Übersetzung: 300 PPR × 4 (Quadratur) × 4.25 × (76/13) Getriebe = 7958.769 Counts pro Lenkrad-Grad

Usage:
    python3 test_phidget_steering.py                   # interaktives Menü
    python3 test_phidget_steering.py --sweep 500       # einmal +/-500° fahren
    python3 test_phidget_steering.py --goto 200        # einmal auf 200° fahren
    python3 test_phidget_steering.py --hub-port 1      # anderer Hub-Port

Sicherheit:
  • Standard-Sweep ist nur ±200° (sehr konservativ — voller Lenkanschlag wäre 1750°)
  • Acceleration und Velocity sind reduziert gegenüber Production-Code
  • Ctrl+C bringt den Motor sofort sauber zum Stillstand und disengaged ihn
"""

import sys
import time
import argparse
import signal
from contextlib import contextmanager

try:
    from Phidget22.Devices.MotorPositionController import MotorPositionController
    from Phidget22.PhidgetException import PhidgetException
except ImportError:
    print("FEHLER: Phidget22 Library nicht gefunden.")
    print("Installation: pip install Phidget22")
    sys.exit(1)


# ── Hardware-Kalibrierung ─────────────────────────────────────────────────────
ENCODER_PPR        = 300              # Pulses per Revolution
QUADRATURE         = 4                # x4 Decoding
GEARBOX_RATIO_1    = 4.25
GEARBOX_RATIO_2    = 76 / 13          # Kettengetriebe o.ä.
RESCALE_FACTOR     = -360.0 / (ENCODER_PPR * QUADRATURE * GEARBOX_RATIO_1 * GEARBOX_RATIO_2)
# Negativ → korrekte Drehrichtung (Stick rechts = Lenkrad rechts)

MAX_STEERING_DEG   = 1750.0           # voller Lenkanschlag (für Reference)

# ── Test-Defaults (konservativ) ───────────────────────────────────────────────
DEFAULT_HUB_PORT       = 3
DEFAULT_SWEEP_DEG      = 200.0        # nur ±200° für ersten Test
DEFAULT_VELOCITY_LIMIT = 400.0        # deg/s (Production: 1800)
DEFAULT_ACCELERATION   = 1200.0       # deg/s² (Production: 9000) — sanftes Anfahren/Bremsen
DEFAULT_CURRENT_LIMIT  = 10.0         # A (Production: 15)
DEFAULT_DEAD_BAND      = 3.0          # deg
DEFAULT_KP             = 400.0
DEFAULT_KI             = 0.0
DEFAULT_KD             = 250.0        # mehr Dämpfung gegen abruptes Stoppen
ATTACH_TIMEOUT_MS      = 5000


# ── Setup ─────────────────────────────────────────────────────────────────────

@contextmanager
def open_motor(hub_port: int, args):
    """Öffnet den MotorPositionController, garantiert sauberes Disengage am Ende."""
    m = MotorPositionController()
    m.setHubPort(hub_port)
    m.setIsHubPortDevice(False)

    print(f"Verbinde mit DCC1000 auf Hub-Port {hub_port}...")
    try:
        m.openWaitForAttachment(ATTACH_TIMEOUT_MS)
    except PhidgetException as e:
        print(f"FEHLER: Attach fehlgeschlagen: {e}")
        print(f"Check: VINT Hub eingesteckt? DCC1000 auf Port {hub_port}?")
        sys.exit(1)

    try:
        m.setRescaleFactor(RESCALE_FACTOR)
        m.setCurrentLimit(args.current_limit)
        m.setVelocityLimit(args.velocity_limit)
        m.setAcceleration(args.acceleration)
        m.setDeadBand(args.dead_band)
        m.setKp(args.kp)
        m.setKi(args.ki)
        m.setKd(args.kd)

        # Aktuelle Position als Nullpunkt setzen
        try:
            m.addPositionOffset(-m.getPosition())
        except PhidgetException:
            pass

        m.setEngaged(True)
        time.sleep(0.2)

        print(f"  Rescale-Faktor : {RESCALE_FACTOR:.6f} deg/count")
        print(f"  Velocity-Limit : {args.velocity_limit} deg/s")
        print(f"  Acceleration   : {args.acceleration} deg/s²")
        print(f"  Current-Limit  : {args.current_limit} A")
        print(f"  Dead-Band      : {args.dead_band} deg")
        print(f"  PID            : Kp={args.kp}, Ki={args.ki}, Kd={args.kd}")
        print(f"  Position@Start : {m.getPosition():.2f} deg (auf 0 referenziert)")
        print()

        yield m
    finally:
        try:
            m.setTargetPosition(0.0)
            time.sleep(0.3)
            m.setEngaged(False)
            m.close()
            print("\nMotor disengaged und geschlossen.")
        except PhidgetException:
            pass


# ── Bewegungs-Routinen ────────────────────────────────────────────────────────

def goto_with_progress(m, target_deg: float, settle_time: float = 2.0) -> float:
    """Fährt zu Ziel-Position und loggt Position alle 100 ms bis settled."""
    print(f"  Ziel: {target_deg:+.1f}°")
    m.setTargetPosition(target_deg)

    start = time.monotonic()
    last_print = 0.0
    while time.monotonic() - start < settle_time:
        now = time.monotonic() - start
        if now - last_print >= 0.1:
            try:
                pos = m.getPosition()
            except PhidgetException:
                pos = float('nan')
            err = target_deg - pos
            print(f"    t={now:4.2f}s  pos={pos:+8.2f}°  err={err:+7.2f}°", end='\r')
            last_print = now
        time.sleep(0.02)

    try:
        final_pos = m.getPosition()
    except PhidgetException:
        final_pos = float('nan')
    print(f"    final  pos={final_pos:+8.2f}°  err={target_deg - final_pos:+7.2f}°")
    return final_pos


def sweep(m, deg: float):
    """Klassischer Sweep: Mitte → +deg → -deg → Mitte."""
    print(f"\n=== Sweep ±{deg}° ===")
    goto_with_progress(m, +deg, settle_time=2.5)
    goto_with_progress(m, -deg, settle_time=2.5)
    goto_with_progress(m, 0.0,  settle_time=2.0)


def step_response(m, deg: float):
    """Sprungantwort: Mitte → deg in einem Schritt, mit feinerer Logging-Auflösung."""
    print(f"\n=== Step Response ({deg}° Sprung) ===")
    goto_with_progress(m, 0.0, settle_time=1.0)
    print(f"  Sprung auf {deg:+.1f}°...")
    m.setTargetPosition(deg)
    start = time.monotonic()
    while time.monotonic() - start < 1.5:
        try:
            pos = m.getPosition()
        except PhidgetException:
            pos = float('nan')
        t_ms = (time.monotonic() - start) * 1000
        print(f"    t={t_ms:6.1f}ms  pos={pos:+8.2f}°", end='\r')
        time.sleep(0.01)
    print()
    goto_with_progress(m, 0.0, settle_time=1.5)


def manual_mode(m):
    """Interaktiv: gib Ziel-Positionen ein bis 'q'."""
    print("\n=== Manueller Modus ===")
    print("Gib Ziel-Position in Grad ein (oder 'q' zum Beenden, 'p' für aktuelle Position)")
    while True:
        try:
            inp = input("Ziel [°]: ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if inp.lower() in ('q', 'quit', 'exit'):
            break
        if inp.lower() == 'p':
            try:
                print(f"  Aktuelle Position: {m.getPosition():+.2f}°")
            except PhidgetException as e:
                print(f"  Fehler: {e}")
            continue
        try:
            target = float(inp)
        except ValueError:
            print("  ungültige Eingabe")
            continue
        if abs(target) > MAX_STEERING_DEG:
            print(f"  ABGELEHNT: |{target}°| > {MAX_STEERING_DEG}° (Lenkanschlag)")
            continue
        goto_with_progress(m, target, settle_time=2.5)


def tuning_mode(m, sweep_deg: float = 500.0):
    """Live-Tuning: PID/Velocity/Acceleration ändern und sofort sweepen.
    Schnelle Iteration ohne Skript-Neustart."""
    setters = {
        'kp':  ('Kp',           m.setKp,            m.getKp),
        'ki':  ('Ki',           m.setKi,            m.getKi),
        'kd':  ('Kd',           m.setKd,            m.getKd),
        'acc': ('Acceleration', m.setAcceleration,  m.getAcceleration),
        'vel': ('VelocityLimit',m.setVelocityLimit, m.getVelocityLimit),
        'db':  ('DeadBand',     m.setDeadBand,      m.getDeadBand),
        'cur': ('CurrentLimit', m.setCurrentLimit,  m.getCurrentLimit),
    }

    def show():
        print("\n  Aktuelle Werte:")
        for k, (label, _, getter) in setters.items():
            try:
                print(f"    {k:5s} {label:14s} = {getter():.2f}")
            except PhidgetException as e:
                print(f"    {k:5s} {label:14s} = (Fehler: {e})")
        print(f"    sweep                = ±{sweep_deg:.0f}°")

    print(f"\n=== Tuning-Mode (Sweep ±{sweep_deg}°) ===")
    print("Befehle:")
    print("  s              Sweep ausführen")
    print("  <key> <wert>   Wert ändern (z.B. 'kd 300', 'acc 800')")
    print("  sweep <deg>    Sweep-Range ändern (z.B. 'sweep 300')")
    print("  show           aktuelle Werte zeigen")
    print("  q              zurück zum Hauptmenü")

    while True:
        show()
        try:
            inp = input("\n> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print()
            return
        if not inp:
            continue
        if inp in ('q', 'quit', 'exit', 'back'):
            return
        if inp == 's' or inp == 'sweep':
            sweep(m, sweep_deg)
            continue
        if inp == 'show':
            continue

        parts = inp.split()
        if len(parts) != 2:
            print("  Format: <key> <wert>  (z.B. 'kd 300')")
            continue
        key, val_str = parts
        try:
            val = float(val_str)
        except ValueError:
            print(f"  '{val_str}' ist keine Zahl")
            continue

        if key == 'sweep':
            sweep_deg = max(10.0, min(MAX_STEERING_DEG, val))
            print(f"  Sweep-Range jetzt ±{sweep_deg:.0f}°")
            continue
        if key not in setters:
            print(f"  unbekannter Key '{key}'. Erlaubt: {', '.join(setters.keys())}, sweep")
            continue
        label, setter, _ = setters[key]
        try:
            setter(val)
            print(f"  {label} = {val} gesetzt")
        except PhidgetException as e:
            print(f"  Fehler beim Setzen: {e}")


def menu(m):
    """Interaktives Hauptmenü."""
    while True:
        print("\n─────────────────────────────")
        print("  1) Sweep ±200°       (Standard-Test)")
        print("  2) Sweep ±500°       (mittel)")
        print("  3) Sweep ±1000°      (groß)")
        print("  4) Step-Response 200°")
        print("  5) Manueller Modus   (Position eingeben)")
        print("  6) Aktuelle Position lesen")
        print("  t) Tuning-Mode       (PID live einstellen)")
        print("  q) Beenden")
        try:
            choice = input("Auswahl: ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print()
            return

        if   choice == '1': sweep(m, 200)
        elif choice == '2': sweep(m, 500)
        elif choice == '3': sweep(m, 1000)
        elif choice == '4': step_response(m, 200)
        elif choice == '5': manual_mode(m)
        elif choice == '6':
            try:
                print(f"  Position: {m.getPosition():+.2f}°")
            except PhidgetException as e:
                print(f"  Fehler: {e}")
        elif choice == 't': tuning_mode(m, sweep_deg=500.0)
        elif choice in ('q', 'quit', 'exit'):
            return
        else:
            print("  ungültige Auswahl")


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--hub-port', type=int, default=DEFAULT_HUB_PORT,
                   help=f'VINT Hub Port (default: {DEFAULT_HUB_PORT})')
    p.add_argument('--sweep', type=float, metavar='DEG',
                   help='direkt einen Sweep ±DEG fahren und beenden')
    p.add_argument('--goto', type=float, metavar='DEG',
                   help='direkt eine Position anfahren und beenden')
    p.add_argument('--step', type=float, metavar='DEG',
                   help='direkt eine Step-Response fahren und beenden')

    p.add_argument('--velocity-limit', type=float, default=DEFAULT_VELOCITY_LIMIT)
    p.add_argument('--acceleration',   type=float, default=DEFAULT_ACCELERATION)
    p.add_argument('--current-limit',  type=float, default=DEFAULT_CURRENT_LIMIT)
    p.add_argument('--dead-band',      type=float, default=DEFAULT_DEAD_BAND)
    p.add_argument('--kp', type=float, default=DEFAULT_KP)
    p.add_argument('--ki', type=float, default=DEFAULT_KI)
    p.add_argument('--kd', type=float, default=DEFAULT_KD)

    args = p.parse_args()

    # Sauberer Ctrl+C Handler — der context manager macht den Rest
    signal.signal(signal.SIGINT, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))

    try:
        with open_motor(args.hub_port, args) as m:
            if args.sweep is not None:
                sweep(m, args.sweep)
            elif args.goto is not None:
                if abs(args.goto) > MAX_STEERING_DEG:
                    print(f"FEHLER: |{args.goto}°| > {MAX_STEERING_DEG}° (Lenkanschlag)")
                    sys.exit(1)
                goto_with_progress(m, args.goto, settle_time=3.0)
            elif args.step is not None:
                step_response(m, args.step)
            else:
                menu(m)
    except KeyboardInterrupt:
        print("\nAbbruch durch Benutzer.")


if __name__ == '__main__':
    main()
