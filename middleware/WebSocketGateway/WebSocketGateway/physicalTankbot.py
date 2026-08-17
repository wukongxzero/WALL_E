import serial
import tankstatus_wrapper

class PhysicalTankBot:
    def __init__(self, port='/dev/ttyACM0', baud=115200):
        self.ser = serial.Serial(port, baud, timeout=1)
        self.status = tankstatus_wrapper.TankStatusClass()

    def move(self, left_speed: int, right_speed: int):
        """Sends speed commands to the Arduino via bytes."""
        self.status.drive_left = left_speed
        self.status.drive_right = right_speed
        
        # Convert C++ wrapper data to byte packet
        packet = self.status.make_into_bytes()
        self.ser.write(packet)
        return f"Moved: Left={left_speed}, Right={right_speed}"
