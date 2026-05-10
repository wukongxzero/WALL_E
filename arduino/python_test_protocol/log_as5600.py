#!/usr/bin/env python3
import serial, sys, time, signal

PORT = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM1'
OUT  = sys.argv[2] if len(sys.argv) > 2 else f'as5600_{int(time.time())}.csv'

ser = serial.Serial(PORT, 115200, timeout=1)
time.sleep(2)  # let UNO finish auto-reset
ser.reset_input_buffer()

print(f"logging {PORT} -> {OUT}  (Ctrl-C to stop)")
n = 0
with open(OUT, 'w') as f:
    def stop(*_):
        print(f"\nwrote {n} rows to {OUT}")
        ser.close(); f.close(); sys.exit(0)
    signal.signal(signal.SIGINT, stop)
    while True:
        line = ser.readline().decode(errors='replace').strip()
        if not line: continue
        f.write(line + '\n'); f.flush()
        n += 1
        if n % 50 == 0:
            print(f"  {n} rows  | last: {line}")
