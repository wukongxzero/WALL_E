FQBN_MEGA = arduino:avr:mega
FQBN_UNO = arduino:avr:uno
PORT = /dev/ttyACM0
SKETCH_FLASH_DRIVE = ~/WALL_E_ROS2/arduino/MEGA_V3/
SKETCH_FLASH_MOCK = ~/WALL_E_ROS2/arduino/MOCK_ARDUINO_STREAMIO/

compile:
	arduino-cli compile --fqbn $(FQBN_MEGA) $(SKETCH_FLASH_DRIVE)
	arduino-cli compile --fqbn $(FQBN_MEGA) $(SKETCH_FLASH_MOCK)

flash_drive_loop:
	arduino-cli upload -p $(PORT) --fqbn $(FQBN_MEGA) $(SKETCH_FLASH_DRIVE)

flash_mock_loop:
	arduino-cli upload -p $(PORT) --fqbn $(FQBN_MEGA) $(SKETCH_FLASH_MOCK)


all: compile flash

read_vel:
	source python_test_protocol/venv/bin/activate && python3 python_test_protocol/test_read_encoder.py
monitor:
	arduino-cli monitor -p $(PORT) -c baudrate=115200

