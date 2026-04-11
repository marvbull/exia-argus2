#!/bin/bash
# flash_rp2350.sh — MicroPython auf RP2350 flashen
# Erkennt automatisch das RP2350-Laufwerk und kopiert die Firmware.
#
# Voraussetzung: RP2350 im BOOTSEL-Modus (BOOTSEL halten → USB einstecken)
#
# Usage:
#   bash scripts/flash_rp2350.sh
#   bash scripts/flash_rp2350.sh /pfad/zur/firmware.uf2

set -e

FIRMWARE="${1:-}"

# Suche RP2350-Laufwerk (macOS und Linux)
find_mount() {
    if [[ "$OSTYPE" == "darwin"* ]]; then
        for vol in /Volumes/RP2350 /Volumes/RPI-RP2; do
            [[ -d "$vol" ]] && echo "$vol" && return
        done
    else
        for mnt in /media/"$USER"/RP2350 /media/"$USER"/RPI-RP2 /mnt/rp2350; do
            [[ -d "$mnt" ]] && echo "$mnt" && return
        done
    fi
    echo ""
}

# Firmware-Datei suchen falls nicht angegeben
if [[ -z "$FIRMWARE" ]]; then
    FIRMWARE=$(find . -maxdepth 2 -name "*.uf2" 2>/dev/null | head -1)
    if [[ -z "$FIRMWARE" ]]; then
        echo "Keine .uf2 Datei gefunden."
        echo ""
        echo "MicroPython für RP2350 herunterladen:"
        echo "  https://micropython.org/download/RPI_PICO2/"
        echo ""
        echo "Dann erneut ausführen:"
        echo "  bash scripts/flash_rp2350.sh /pfad/zur/firmware.uf2"
        exit 1
    fi
fi

echo "Firmware: $FIRMWARE"
echo ""
echo "RP2350 suchen..."

MOUNT=""
for i in $(seq 1 15); do
    MOUNT=$(find_mount)
    [[ -n "$MOUNT" ]] && break
    echo "  Warte auf RP2350-Laufwerk... ($i/15)"
    sleep 1
done

if [[ -z "$MOUNT" ]]; then
    echo ""
    echo "ERROR: RP2350-Laufwerk nicht gefunden."
    echo ""
    echo "Anleitung:"
    echo "  1. BOOTSEL-Knopf auf dem RP2350 gedrückt halten"
    echo "  2. USB-Kabel einstecken"
    echo "  3. Knopf loslassen"
    echo "  4. Dieses Script erneut starten"
    exit 1
fi

echo "RP2350 gefunden: $MOUNT"
echo "Flashe $FIRMWARE ..."
cp "$FIRMWARE" "$MOUNT/"
echo "Fertig! RP2350 startet automatisch neu."
