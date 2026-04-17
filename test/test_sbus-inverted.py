import serial
import time

s = serial.Serial('/dev/ttyTHS1', 115200, bytesize=8, parity='E', stopbits=2, timeout=0.005)

buf = bytearray()
last_rx = time.time()

def parse_sbus(frame):
    if len(frame) != 25 or frame[0] != 0x0F or frame[24] != 0x00:
        return None
    channels = [0] * 16
    data = frame[1:23]
    bits = 0
    bit_count = 0
    ch_idx = 0
    for byte in data:
        bits |= byte << bit_count
        bit_count += 8
        while bit_count >= 11 and ch_idx < 16:
            channels[ch_idx] = bits & 0x7FF
            bits >>= 11
            bit_count -= 11
            ch_idx += 1
    return channels

last_print = 0

while True:
    chunk = s.read(64)
    now = time.time()
    
    # Gap detection: >3 ms silence → frame boundary
    if chunk:
        if now - last_rx > 0.003:
            buf.clear()  # reset on gap
        buf.extend(chunk)
        last_rx = now
    
    if len(buf) >= 25:
        frame = bytes(buf[:25])
        buf.clear()
        result = parse_sbus(frame)
        if result and now - last_print > 0.1:
            ch = result
            print(f"CH1-8: {ch[0]:4d} {ch[1]:4d} {ch[2]:4d} {ch[3]:4d} "
                  f"{ch[4]:4d} {ch[5]:4d} {ch[6]:4d} {ch[7]:4d}")
            last_print = now