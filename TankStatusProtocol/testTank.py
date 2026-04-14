import sys

try:
    # Import the module exactly as named in pybind11_add_module in CMake
    import tankstatus_wrapper
except ImportError:
    print("Error: Could not find 'my_wrapper'.")
    print(
        "Make sure the compiled .so or .pyd file is in the same directory as this script."
    )
    sys.exit(1)


def main():
    print("--- Testing TankStatus Wrapper ---\n")

    # 1. Instantiate the class
    # The name of the class here depends on what you named it inside your
    # PYBIND11_MODULE(my_wrapper, m) block in tankstatuswrapper.cpp.
    # Assuming you bound it as 'TankStatusClass':
    tank_status = my_wrapper.TankStatusClass()

    # 2. Modify properties
    print("Setting initial values...")
    tank_status.eulerXFloat = 15.5
    tank_status.eulerYFloat = -45.0
    tank_status.eulerZFloat = 90.2

    # Assuming driveRight is bound as a standard integer/property
    tank_status.driveRight = 128

    # 3. Read properties back to verify
    print(f"Euler X: {tank_status.eulerXFloat}")
    print(f"Euler Y: {tank_status.eulerYFloat}")
    print(f"Euler Z: {tank_status.eulerZFloat}")
    print(f"Drive Right: {tank_status.driveRight}")

    # 4. Test serialization (MakeIntoBytes)
    print("\nSerializing to bytes...")
    try:
        # This assumes pybind11 successfully converts unsigned char* to a Python bytes object
        packet = tank_status.MakeIntoBytes()
        print(f"Packet type: {type(packet)}")
        print(f"Packet data: {packet}")
    except Exception as e:
        print(f"MakeIntoBytes failed (check pybind11 pointer bindings): {e}")

    # 5. Test deserialization (BuildFromBytes)
    print("\nTesting BuildFromBytes...")
    try:
        # Create a dummy byte array of the correct length (16 bytes based on your headers)
        dummy_buffer = bytearray([0] * 16)

        # Populate it with some fake data if needed
        dummy_buffer[0] = 255  # Example: setting driveLeft to max

        tank_status.BuildFromBytes(dummy_buffer)
        print("Successfully built from bytes!")
    except Exception as e:
        print(
            f"BuildFromBytes failed (check how your wrapper handles byte arrays): {e}"
        )


if __name__ == "__main__":
    main()
