"""
ramp test — using pybind11 wrapper for both TX serialization and RX deserialization.
"""

import sys
import serial
import time
import tankstatus_wrapper

THROTTLE_AMPLITUDE = 50
RAMP_UP_SEC = 3.0
HOLD_SEC = 2.0
RAMP_DOWN_SEC = 3.0

PORT = "/dev/ttyACM0"
BAUD = 115200
PKT_LEN = 8  # Updated to 10 bytes to match the C++ compiler's struct padding!
CMD_HZ = 50
CMD_PERIOD = 1.0 / CMD_HZ
CENTER = 127


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


def main():
    print("--- Testing TankStatus Wrapper with Serial Ramp ---\n")

    # 1. Instantiate two class objects: one for Transmit, one for Receive
    tx = tankstatus_wrapper.TankStatusClass()
    rx = tankstatus_wrapper.TankStatusClass()

    # 2. Setup Serial Connection
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0)
    except Exception as e:
        print(f"Failed to open port {PORT}: {e}")
        sys.exit(1)

    print("Waiting 2 seconds for Arduino to boot...")
    time.sleep(2.0)

    # Aggressively wipe out any startup garbage bytes
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    total_dur = RAMP_UP_SEC + HOLD_SEC + RAMP_DOWN_SEC + 0.5
    start = time.monotonic()
    last_cmd = 0.0

    print(
        f"ramp test: amplitude={THROTTLE_AMPLITUDE}, "
        f"{RAMP_UP_SEC}s up / {HOLD_SEC}s hold / {RAMP_DOWN_SEC}s down"
    )
    print("Ctrl+C to abort\n")

    try:
        while True:
            now = time.monotonic()
            elapsed = now - start

            # ---------------------------------------------------------
            # TRANSMIT (Serialize to Bytes)
            # ---------------------------------------------------------
            if now - last_cmd >= CMD_PERIOD:
                last_cmd = now
                # throttle = ramp_throttle(elapsed)

                # Modify wrapper properties directly
                # Note: Adjust to `tx.drive_left` if pybind11 converted the case
                # tx.drive_left = max(0, min(255, CENTER - int(throttle)))
                # tx.drive_right = max(0, min(255, CENTER - int(0)))
                tx.drive_left = 250
                tx.drive_right = 250

                try:
                    # Let the C++ wrapper handle the packing and byte alignment!
                    tx_packet = tx.make_into_bytes()
                    ser.write(tx_packet)
                except Exception as e:
                    print(f"MakeIntoBytes failed: {e}")

            # ---------------------------------------------------------
            # RECEIVE (Deserialize from Bytes)
            # ---------------------------------------------------------
            #
            if ser.in_waiting >= 1:
                raw = ser.read(1)
                print(raw)
            # if ser.in_waiting >= PKT_LEN:
            # Read exactly 10 bytes (NO readline!)
            #    raw = ser.read(PKT_LEN)

            #   try:
            # Build from the raw byte array
            #       rx.build_from_bytes(raw)
            #   throttle_now = ramp_throttle(elapsed)

            # Read properties back to verify
            #   print(
            #       f"t={elapsed:5.2f}s  cmd={throttle_now:+6.1f}  "
            #       f"L={rx.drive_left}  "
            #       f"R={rx.drive_right}  "
            #       f"EulerX={rx.eulerXFloat:+.3f}"
            # 3   )
            # except Exception as e:
            #   print(
            #       f"BuildFromBytes failed (check pybind byte array handling): {e}"
            #   )

            if elapsed > total_dur:
                break

            time.sleep(0.0005)

    except KeyboardInterrupt:
        print("\naborted")
    finally:
        print("\nStopping motors...")
        for _ in range(3):
            # Safely return motors to center
            tx.drive_left = CENTER
            tx.drive_right = CENTER
            try:
                ser.write(tx.make_into_bytes())
            except:
                pass
            time.sleep(0.01)
        ser.close()
        print("stopped")


if __name__ == "__main__":
    main()
