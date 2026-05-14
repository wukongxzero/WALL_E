import serial
import tankstatus_wrapper

PORT = "/dev/ttyACM1"


def main():
    # Connect to the Arduino (Make sure the port matches your setup)
    ser = serial.Serial(PORT, 115200)

    # Instantiate your wrapper
    rx_status = tankstatus_wrapper.TankStatusClass()

    print("Listening to Arduino...")

    try:
        while True:
            # Read the exact number of bytes required for a packet
            # Use rx_status.packetLength if exposed, otherwise hardcode your TANKSTATUS_PACKET_LENGTH
            packet_length = 8

            if ser.in_waiting >= packet_length:
                # Read the binary bytes from the serial port
                raw_bytes = ser.read(packet_length)

                # Pass the bytes to your C++ wrapper to decode
                rx_status.build_from_bytes(raw_bytes)

                # Print the decoded values
                print(
                    f"Left PWM: {rx_status.drive_left} | Right PWM: {rx_status.drive_right}"
                )

    except KeyboardInterrupt:
        print("\nExiting...")
        ser.close()


if __name__ == "__main__":
    main()
