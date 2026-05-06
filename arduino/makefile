FQBN = arduino:avr:mega
PORT = /dev/ttyACM0
SKETCH = ~/WALL_E_ROS2/arduino/MEGA_V3/

compile:
	arduino-cli compile --fqbn $(FQBN) $(SKETCH)

flash:
	arduino-cli upload -p $(PORT) --fqbn $(FQBN) $(SKETCH)

all: compile flash

read_vel:
	source python_test_protocol/venv/bin/activate && python3 python_test_protocol/test_read_encoder.py
monitor:
	arduino-cli monitor -p $(PORT) -c baudrate=115200

