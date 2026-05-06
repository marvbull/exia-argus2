import serial
import time
import sys
import fcntl
import struct

PORT = '/dev/sbus'
BAUD = 100000

def force_custom_baud(fd, baud):
    """Set non-standard baud via termios2 BOTHER. pyserial silently rounds 100000→115200 on some Jetson drivers."""
    TCGETS2 = 0x802C542A
    TCSETS2 = 0x402C542B
    BOTHER  = 0o010000
    CBAUD   = 0o010017
    buf = bytearray(44)
    fcntl.ioctl(fd, TCGETS2, buf)
    cflag = struct.unpack('I', buf[8:12])[0]
    cflag = (cflag & ~CBAUD) | BOTHER
    buf[8:12]  = struct.pack('I', cflag)
    buf[36:40] = struct.pack('I', baud)   # c_ispeed
    buf[40:44] = struct.pack('I', baud)   # c_ospeed
    fcntl.ioctl(fd, TCSETS2, bytes(buf))

def parse_sbus(frame):
    channels = [0] * 16
    bits, n, ch = 0, 0, 0
    for b in frame[1:23]:
        bits |= b << n
        n += 8
        while n >= 11 and ch < 16:
            channels[ch] = bits & 0x7FF
            bits >>= 11
            n -= 11
            ch += 1
    flags = frame[23]
    return channels, bool(flags & 0x04), bool(flags & 0x08)  # frame_lost, failsafe

def main():
    s = serial.Serial(PORT, BAUD, bytesize=8, parity='E', stopbits=2, timeout=0.005)
    try:
        force_custom_baud(s.fileno(), BAUD)
    except Exception as e:
        print(f"[warn] termios2 setup failed, baud may be wrong: {e}", file=sys.stderr)

    buf = bytearray()
    ok, bad = 0, 0
    last_print = 0

    while True:
        chunk = s.read(64)
        if chunk:
            buf.extend(chunk)

        # Resynchronizing parser: slide until we find 0x0F...0x00 at [0] and [24]
        while len(buf) >= 25:
            if buf[0] != 0x0F:
                del buf[0]
                continue
            if buf[24] != 0x00:
                del buf[0]          # false positive header — shift one and retry
                bad += 1
                continue
            frame = bytes(buf[:25])
            del buf[:25]
            ch, frame_lost, failsafe = parse_sbus(frame)
            ok += 1
            now = time.time()
            if now - last_print > 0.1:
                tag = (' LOST' if frame_lost else '') + (' FS' if failsafe else '')
                print(f"CH1-8: {ch[0]:4d} {ch[1]:4d} {ch[2]:4d} {ch[3]:4d} "
                      f"{ch[4]:4d} {ch[5]:4d} {ch[6]:4d} {ch[7]:4d} "
                      f"| ok={ok} bad={bad}{tag}")
                last_print = now

        # Sanity cap
        if len(buf) > 200:
            del buf[:len(buf)-100]

if __name__ == '__main__':
    main()