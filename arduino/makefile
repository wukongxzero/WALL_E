FQBN_MEGA = arduino:avr:mega
FQBN_UNO = arduino:avr:uno

PORT_MEGA = /dev/ttyACM0
PORT_UNO = /dev/ttyACM1

SKETCH_FLASH_DRIVE = ~/WALL_E_ROS2/arduino/MEGA_V3/
SKETCH_FLASH_MOCK = ~/WALL_E_ROS2/arduino/MOCK_ARDUINO_STREAMIO/
SKETCH_FLASH_STANDBY = ~/WALL_E_ROS2/arduino/blink/

compile:
	arduino-cli lib install "Simple FOC"
	arduino-cli lib install "SimpleFOCDrivers"
	arduino-cli compile --fqbn $(FQBN_MEGA) $(SKETCH_FLASH_DRIVE)
	arduino-cli compile --fqbn $(FQBN_MEGA) $(SKETCH_FLASH_MOCK)
	arduino-cli compile --fqbn $(FQBN_MEGA) $(SKETCH_FLASH_STANDBY)

flash_drive_loop:
	arduino-cli compile --fqbn $(FQBN_MEGA) $(SKETCH_FLASH_DRIVE)
	arduino-cli upload -p $(PORT_MEGA) --fqbn $(FQBN_MEGA) $(SKETCH_FLASH_DRIVE)

flash_mock_loop:
	arduino-cli compile --fqbn $(FQBN_MEGA) $(SKETCH_FLASH_MOCK)
	arduino-cli upload -p $(PORT_MEGA) --fqbn $(FQBN_MEGA) $(SKETCH_FLASH_MOCK)

flash_standby:
	arduino-cli upload -p $(PORT_MEGA) --fqbn $(FQBN_MEGA) $(SKETCH_FLASH_STANDBY)

all: compile flash

read_vel:
	source python_test_protocol/venv/bin/activate && python3 python_test_protocol/test_read_encoder.py

write_duino:
	source python_test_protocol/venv/bin/activate && python3 python_test_protocol/test_write_motor.py $(PORT_MEGA)




list-ports:
	arduino-cli board list


monitor:
	arduino-cli monitor -p $(PORT_MEGA) -c baudrate=115200

