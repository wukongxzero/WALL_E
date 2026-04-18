import time
import os
import sys
import socket

# 1. Dynamically find the path to '../build' relative to this script
current_dir = os.path.dirname(os.path.abspath(__file__))
build_dir = os.path.join(current_dir, "../TankStatusProtocol/", "builds")
sys.path.append(build_dir)

import tankstatus_wrapper

# Target configuration (Update to your host IP if running inside a container/VM)
TARGET_IP = "10.98.48.5"  # or "127.0.0.1"
TARGET_PORT = 8881


def main():
    # Create a UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print(f"Starting Full TankStatus UDP dump to {TARGET_IP}:{TARGET_PORT}...")
    print("Press Ctrl+C to stop.")

    try:
        # 1. Instantiate the C++ wrapper class once
        tank_status = tankstatus_wrapper.TankStatusClass()

        # Simulated data variable
        simulated_euler_x = 0.0

        while True:
            # 2. Update the properties using the Python bindings
            # The C++ wrapper will hold these floats and handle the encoding internally
            tank_status.eulerXFloat = simulated_euler_x
            tank_status.eulerYFloat = 90.0
            tank_status.eulerZFloat = -45.0

            # Assuming you mapped the pointers to these properties in Pybind11:
            tank_status.drive_left = 127
            tank_status.drive_right = 255

            # 3. Serialize to bytes using your C++ native method
            # This triggers MakeIntoBytes() -> makeByteTankStatus() -> ENCODE_SHORT()
            raw_packet = tank_status.make_into_bytes()

            # Ensure it is treated as a standard Python bytes object for the socket
            byte_data = bytes(raw_packet)

            # 4. Send the 8-byte packet
            sock.sendto(byte_data, (TARGET_IP, TARGET_PORT))

            # Print the struct data and the raw hex bytes for debugging
            print(
                f"Sent EulerX: {tank_status.eulerXFloat:6.2f} | Hex: {byte_data.hex()}"
                f"Sent EulerY: {tank_status.eulerYFloat:6.2f} | Hex: {byte_data.hex()}"
                f"Sent EulerZ: {tank_status.eulerZFloat:6.2f} | Hex: {byte_data.hex()}"
                f"Sent drive_left: {tank_status.drive_left:6.2f} | Hex: {byte_data.hex()}"
                f"Sent drive_right: {tank_status.drive_right:6.2f} | Hex: {byte_data.hex()}"
            )

            # 5. Increment simulated value and wrap around
            simulated_euler_x += 10.5
            if simulated_euler_x > 360.0:
                simulated_euler_x = 0.0

            # Wait 500ms before sending the next packet
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nUDP dump stopped by user.")
    except Exception as e:
        print(f"\nError during UDP execution: {e}")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
