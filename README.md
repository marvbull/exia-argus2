# exia-argus2

NVIDIA Jetson Orin Nano platform receiving RC commands from a CubePilot Herelink V1.0 Air Unit via UART/MAVLink.

## Hardware

- NVIDIA Jetson Orin Nano
- CubePilot Herelink V1.0 Air Unit + GCS Controller

## Wiring

| Herelink Air Unit TELEM2 | Jetson Orin Nano J12 |
|--------------------------|----------------------|
| Pin 2 — TX (3.3V TTL)   | Pin 10 — RX          |
| Pin 3 — RX (3.3V TTL)   | Pin 8  — TX          |
| Pin 6 — GND             | Pin 6  — GND         |

UART: `/dev/ttyTHS1`, 115200 baud, 8N1, no flow control.

## Setup

```bash
# Add user to dialout group (once)
sudo usermod -aG dialout $USER
newgrp dialout

# Disable serial console on ttyTHS1 (if active)
sudo systemctl stop nvgetty
sudo systemctl disable nvgetty

# Install dependency
pip3 install pymavlink
```

## Test

```bash
python3 test_herelink.py
# Custom port/baud:
python3 test_herelink.py --port /dev/ttyTHS1 --baud 115200
```

Expected output when Herelink GCS is on and paired:
```
Heartbeat OK — system=1 component=1
------------------------------------------------------------
   x (pitch)   y (roll)   z (thr)   r (yaw)  buttons
------------------------------------------------------------
        +0          +0        500        +0  -
```

Move sticks → values change. Press A/B/C/D → buttons column updates.
