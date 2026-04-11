#!/bin/bash
# deploy_rp2350.sh — SBUS Bridge Code auf RP2350 laden
# Kopiert src/rp2350/main.py via mpremote auf den RP2350.
#
# Voraussetzung: MicroPython bereits auf RP2350 geflasht
#
# Usage:
#   bash scripts/deploy_rp2350.sh          # kopieren + starten
#   bash scripts/deploy_rp2350.sh --repl   # REPL öffnen (Live-Output)

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
SOURCE="$SCRIPT_DIR/src/rp2350/main.py"

# mpremote installieren falls nötig
if ! command -v mpremote &>/dev/null; then
    echo "mpremote nicht gefunden. Installiere..."
    pip install mpremote
fi

if [[ ! -f "$SOURCE" ]]; then
    echo "ERROR: $SOURCE nicht gefunden."
    exit 1
fi

echo "Kopiere $SOURCE auf RP2350 ..."
mpremote connect auto cp "$SOURCE" :main.py
echo "Kopiert."
echo ""

if [[ "$1" == "--repl" ]]; then
    echo "REPL öffnen (Ctrl+X zum Beenden) ..."
    mpremote connect auto repl
else
    echo "RP2350 neu starten und Script ausführen ..."
    mpremote connect auto reset
    echo ""
    echo "Fertig! RP2350 läuft jetzt."
    echo ""
    echo "Live-Output anzeigen:"
    echo "  bash scripts/deploy_rp2350.sh --repl"
    echo ""
    echo "SBUS auf Jetson lesen:"
    echo "  python test/test_sbus.py --port /dev/ttyTHS2"
fi
