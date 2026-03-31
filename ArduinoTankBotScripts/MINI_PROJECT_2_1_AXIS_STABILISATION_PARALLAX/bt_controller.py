"""
bt_controller.py — Laptop BT Terminal Controller
=================================================
Two layers:
  Layer 1: send_tank_status() — TankStatus wire format (from TankStatus.h)
  Layer 2: drive()            — PS2 differential mix motor logic

Keys: w=fwd s=back a=left d=right q=pivotL e=pivotR x=stop +=faster -=slower
      t=3min test  p=state  ESC=quit
"""

import serial
import struct
import sys
import tty
import termios
import time

PORT  = '/dev/rfcomm0'
BAUD  = 9600
SPEED = 80

def clamp(val, lo, hi):
    return max(lo, min(hi, val))

def to_byte(signed_val):
    """Signed int8 (-127..+127) → unsigned byte for TankStatus packet.
    Mega casts back with (int8_t)buf[0]."""
    return clamp(signed_val, -127, 127) & 0xFF

# ── LAYER 1: TankStatus packet (matches makeByteTankStatus exactly) ──
def send_tank_status(left, right, ex=0.0, ey=0.0, ez=90.0):
    buf = bytearray(16)
    buf[0] = to_byte(left)    # driveLeft  unsigned char
    buf[1] = to_byte(right)   # driveRight unsigned char
    buf[2] = 0                # padding
    buf[3] = 0                # padding
    struct.pack_into('<f', buf, 4,  ex)   # eulerX
    struct.pack_into('<f', buf, 8,  ey)   # eulerY
    struct.pack_into('<f', buf, 12, ez)   # eulerZ (90.0 = TankStatus default)
    ser.write(buf)

# ── LAYER 2: PS2 differential mix motor logic ──
def drive(throttle, turn):
    """Matches PS2 code: leftCmd=throttle+turn, rightCmd=throttle-turn"""
    left  = clamp(throttle + turn, -127, 127)
    right = clamp(throttle - turn, -127, 127)
    send_tank_status(left, right)
    return left, right

def stop():
    send_tank_status(0, 0)
    return 0, 0

def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

def run_test_sequence(speed):
    print("\n=== TEST SEQUENCE ===")
    print(f"FORWARD 3 min at speed={speed}...")
    drive(speed, 0)
    for i in range(180):
        time.sleep(1)
        remaining = 180 - i
        if remaining % 30 == 0:
            print(f"  {remaining}s remaining...")
    print("LEFT TURN 2s...")
    drive(speed, -speed)
    time.sleep(2)
    print("RIGHT TURN 2s...")
    drive(speed, speed)
    time.sleep(2)
    stop()
    print("=== DONE ===\n")

print(f"Connecting to {PORT}...")
try:
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
except Exception as e:
    print(f"Failed: {e}")
    print("Run: sudo rfcomm bind 0 <HC-05 MAC>")
    sys.exit(1)

time.sleep(2)
stop()
print("Connected!")
print("================================")
print("  w=fwd  s=back  a=left  d=right")
print("  q=pivotL  e=pivotR  x=stop")
print("  +=faster  -=slower")
print("  t=3min test  p=state  ESC=quit")
print(f"  Speed: {SPEED}/127")
print("================================")

L, R = 0, 0

try:
    while True:
        ch = getch()
        if   ch == 'w': L,R = drive( SPEED,  0);      print(f"FWD    L={L} R={R}")
        elif ch == 's': L,R = drive(-SPEED,  0);      print(f"BWD    L={L} R={R}")
        elif ch == 'a': L,R = drive( SPEED, -SPEED);  print(f"LEFT   L={L} R={R}")
        elif ch == 'd': L,R = drive( SPEED,  SPEED);  print(f"RIGHT  L={L} R={R}")
        elif ch == 'q': L,R = drive( 0,     -SPEED);  print(f"PIVOTL L={L} R={R}")
        elif ch == 'e': L,R = drive( 0,      SPEED);  print(f"PIVOTR L={L} R={R}")
        elif ch == 'x': L,R = stop();                  print("STOP")
        elif ch == '+': SPEED = clamp(SPEED+10, 10, 127); print(f"Speed → {SPEED}")
        elif ch == '-': SPEED = clamp(SPEED-10, 10, 127); print(f"Speed → {SPEED}")
        elif ch == 't': run_test_sequence(SPEED)
        elif ch == 'p': print(f"Speed={SPEED} L={L} R={R}")
        elif ch in ('\x1b', '\x03'): stop(); print("Quit"); break

except KeyboardInterrupt:
    pass
finally:
    stop()
    time.sleep(0.2)
    ser.close()
    print("Disconnected")