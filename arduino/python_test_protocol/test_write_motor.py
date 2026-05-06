"""
ramp test — struct.pack for tx, wrapper for rx.
"""

import serial
import struct
import time
import tankstatus_wrapper

THROTTLE_AMPLITUDE = 50
RAMP_UP_SEC        = 3.0
HOLD_SEC           = 2.0
RAMP_DOWN_SEC      = 3.0

PORT       = '/dev/ttyACM0'
BAUD       = 115200
PKT_LEN    = 8
CMD_HZ     = 50
CMD_PERIOD = 1.0 / CMD_HZ
CENTER     = 127


def ramp_throttle(elapsed):
    if elapsed < RAMP_UP_SEC:
        return THROTTLE_AMPLITUDE * (elapsed / RAMP_UP_SEC)
    elif elapsed < RAMP_UP_SEC + HOLD_SEC:
        return THROTTLE_AMPLITUDE
    elif elapsed < RAMP_UP_SEC + HOLD_SEC + RAMP_DOWN_SEC:
        t = elapsed - RAMP_UP_SEC - HOLD_SEC
        return THROTTLE_AMPLITUDE * (1.0 - t / RAMP_DOWN_SEC)
    else:
        return 0.0


def encode_tx(throttle_signed, steering_signed=0):
    drive_right = max(0, min(255, CENTER - int(throttle_signed)))
    drive_left  = max(0, min(255, CENTER - int(steering_signed)))
    return struct.pack('<BB6x', drive_left, drive_right)


def main():
    ser = serial.Serial(PORT, BAUD, timeout=0)
    ser.reset_input_buffer()

    rx = tankstatus_wrapper.TankStatusClass()

    total_dur = RAMP_UP_SEC + HOLD_SEC + RAMP_DOWN_SEC + 0.5
    start     = time.monotonic()
    last_cmd  = 0.0

    print(f"ramp test: amplitude={THROTTLE_AMPLITUDE}, "
          f"{RAMP_UP_SEC}s up / {HOLD_SEC}s hold / {RAMP_DOWN_SEC}s down")
    print("Ctrl+C to abort\n")

    try:
        while True:
            now     = time.monotonic()
            elapsed = now - start

            if now - last_cmd >= CMD_PERIOD:
                last_cmd = now
                throttle = ramp_throttle(elapsed)
                ser.write(encode_tx(throttle))

            if ser.in_waiting >= PKT_LEN:
                raw = ser.read(PKT_LEN)
                rx.build_from_bytes(raw)
                throttle_now = ramp_throttle(elapsed)
                print(f"t={elapsed:5.2f}s  cmd={throttle_now:+6.1f}  "
                      f"L={rx.eulerXFloat:+.3f} m/s  "
                      f"R={rx.eulerYFloat:+.3f} m/s")

            if elapsed > total_dur:
                break

            time.sleep(0.0005)

    except KeyboardInterrupt:
        print("\naborted")
    finally:
        for _ in range(3):
            ser.write(encode_tx(0))
            time.sleep(0.01)
        ser.close()
        print("stopped")


if __name__ == "__main__":
    main()
