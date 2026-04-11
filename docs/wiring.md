# Wiring — exia-argus2

## Übersicht

```
Herelink Air Unit
      │
      │ SBUS-Out (invertiert, 100000 Baud 8E2)
      ▼
  RP2350 GP1  (UART0 RX, INV_RX)
  RP2350 GP4  (UART1 TX)
      │
      │ SBUS-Frames weitergeleitet (115200 Baud 8N1)
      ▼
  Jetson Pin 23  (/dev/ttyTHS2 RX)
```

---

## Herelink Air Unit → RP2350

| Herelink Pin | RP2350 Pin | Signal         |
|-------------|------------|----------------|
| SBUS-Out    | GP1        | SBUS (invertiert) |
| GND         | GND        | Masse          |
| 3.3V        | 3V3        | optional (falls kein USB) |

> SBUS ist invertiertes UART — der RP2350 invertiert das Signal per Software (`UART.INV_RX`), kein Hardware-Inverter nötig.

---

## RP2350 → Jetson Orin (40-Pin Header)

| RP2350 Pin | Jetson Pin | Signal        |
|------------|------------|---------------|
| GP4        | Pin 23     | TX → RX (ttyTHS2) |
| GND        | Pin 6      | Masse         |

### Jetson 40-Pin Header (Auszug)

```
 [1]  3.3V      [2]  5V
 [3]  I2C SDA   [4]  5V
 [5]  I2C SCL   [6]  GND   ◄── GND vom RP2350
 [7]  GPIO      [8]  ttyTHS1 TX
 [9]  GND       [10] ttyTHS1 RX  ◄── Herelink MAVLink UART
...
[23] ttyTHS2 RX ◄── RP2350 GP4 TX
[24] ttyTHS2 TX
```

---

## Herelink Air Unit → Jetson (MAVLink UART)

| Herelink Pin | Jetson Pin | Signal       |
|-------------|------------|--------------|
| UART TX     | Pin 10     | ttyTHS1 RX   |
| UART RX     | Pin 8      | ttyTHS1 TX   |
| GND         | Pin 9      | Masse        |

> Baud: 115200 — MAVLink Telemetrie (kein RC)

---

## Software

| Device          | Script                        | Beschreibung              |
|----------------|-------------------------------|---------------------------|
| RP2350          | `rp2350_sbus_bridge/main.py`  | Liest SBUS, leitet weiter |
| Jetson ttyTHS2 | `test_sbus.py`                | Liest SBUS vom RP2350     |
| Jetson ttyTHS1 | `test_herelink.py`            | MAVLink Telemetrie        |
