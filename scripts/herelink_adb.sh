#!/bin/bash
# herelink_adb.sh — ADB-Verbindung zum Herelink Air Unit herstellen
#
# Voraussetzung: Jetson im Herelink-WiFi (192.168.43.x)
#
# Usage:
#   bash scripts/herelink_adb.sh            # verbinden
#   bash scripts/herelink_adb.sh shell      # Shell öffnen
#   bash scripts/herelink_adb.sh sbus       # SBUS-Werte live anzeigen
#   bash scripts/herelink_adb.sh info       # Systeminfo sammeln

HERELINK_IP="192.168.43.1"
HERELINK_ADB="${HERELINK_IP}:5555"
SBUS_FILE="/tmp/sbus_values"

connect() {
    echo "Verbinde mit Herelink ($HERELINK_ADB) ..."

    # Alte Verbindungen trennen
    adb disconnect "$HERELINK_ADB" &>/dev/null || true

    adb connect "$HERELINK_ADB"

    # Prüfen ob verbunden
    if ! adb -s "$HERELINK_ADB" shell "echo ok" &>/dev/null; then
        echo ""
        echo "ERROR: Verbindung fehlgeschlagen."
        echo ""
        echo "Prüfe:"
        echo "  1. Jetson ist im Herelink-WiFi (SSID: Herelink)"
        echo "  2. ip addr show wlP1p1s0 | grep 192.168.43"
        exit 1
    fi

    echo "Verbunden!"
}

case "${1:-connect}" in
    shell)
        connect
        echo "Shell öffnen ..."
        adb -s "$HERELINK_ADB" shell
        ;;

    sbus)
        connect
        echo "SBUS-Werte live (Ctrl+C zum Beenden) ..."
        echo ""
        echo "  ch1   ch2   ch3   ch4   ch5   ch6   ch7   ch8"
        echo "------------------------------------------------------"
        while true; do
            vals=$(adb -s "$HERELINK_ADB" shell "cat $SBUS_FILE" 2>/dev/null)
            if [[ -n "$vals" ]]; then
                IFS=',' read -ra ch <<< "$vals"
                printf "  %4s  %4s  %4s  %4s  %4s  %4s  %4s  %4s\r" \
                    "${ch[0]}" "${ch[1]}" "${ch[2]}" "${ch[3]}" \
                    "${ch[4]}" "${ch[5]}" "${ch[6]}" "${ch[7]}"
            fi
            sleep 0.05
        done
        ;;

    info)
        connect
        OUT="/tmp/herelink_info.txt"
        echo "Sammle Systeminfo → $OUT ..."
        adb -s "$HERELINK_ADB" shell << 'EOF' > "$OUT" 2>&1
echo "=== BUILD ==="
getprop ro.build.version.release
getprop ro.product.model

echo "=== MAVLINK CONFIG ==="
cat /tmp/mavlink-router.conf

echo "=== SBUS VALUES ==="
cat /tmp/sbus_values

echo "=== SERIAL DEVICES ==="
ls -la /dev/ttyS*

echo "=== MAVLINK ROUTER PROCESS ==="
ps | grep mavlink
EOF
        echo "Gespeichert: $OUT"
        cat "$OUT"
        ;;

    connect|*)
        connect
        ;;
esac
