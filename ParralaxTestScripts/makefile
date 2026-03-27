# --- Compiler & Paths ---
FLEXSPIN = ../ParralaxController/build/bin/flexspin
FLEXSPIN_LOADER = ../ParralaxController/build/bin/proploader
PORT = /dev/ttyUSB0
# Use abspath to ensure FlexSpin doesn't get lost in relative "../" paths
#LIB_ROOT = $(abspath ../ParralaxController/Simple-Libraries/Learn/Simple-Libraries)
# --- 1. Targeted Library Includes (-I) ---
# We only include the specific folders needed for ADC and Serial output
#INC_FLAGS += -I $(LIB_ROOT)/Convert/libabvolts
#INC_FLAGS += -I $(LIB_ROOT)/Utility/libsimpletools
#INC_FLAGS += -I $(LIB_ROOT)/TextDevices/libsimpletext
INC_FLAGS += -I$(abspath ../ParralaxController/build/include)

# --- 2. Required Source Files (.c) ---
# FlexSpin requires the implementation files to be passed in the command line
#LIB_SRCS = $(LIB_ROOT)/Convert/libabvolts/abvolts.c \
#           $(LIB_ROOT)/Utility/libsimpletools/libsimpletools.c \
#           $(LIB_ROOT)/TextDevices/libsimpletext/simpletext.c \
#           $(LIB_ROOT)/TextDevices/libsimpletext/safe_printf.c

# --- 3. Project Files ---
TARGET = TestJoyStick.elf
SRCS = TestJoyStick.c
HEADERS = ./include/


TEST1 = TestJoyStick.c
TEST2 = testSPIDisplay.c ST7796S.c 
TEST3 = TestHC05.c hco5.c

TARGET_TEST_1 = TestJoyStick.o
TARGET_TEST_2 = testSPIDisplay.o
TARGET_TEST_3 = TestHC05.o


# --- Build Rules ---
all: $(TARGET)

compile:
	$(FLEXSPIN)  $(INC_FLAGS) $(TEST1) -I$(HEADERS) -o $(TARGET_TEST_1)
	$(FLEXSPIN) $(INC_FLAGS) $(TEST2) -I ./include/ -o $(TARGET_TEST_2)
	$(FLEXSPIN) $(INC_FLAGS) $(TEST3) tankStatus.c -I $(HEADERS) -o $(TARGET_TEST_3)
	gcc -I$(INC_FLAGS) tankStatus.c BlueToothSerial.c -o bt_term.o -lbluetooth

set-port:
	$PORT = $(FLEXSPIN_LOADER) -P

upload-joystick:
	$(FLEXSPIN_LOADER) -p $(PORT) $(TARGET_TEST_1) -D baudrate=115200 -t

upload-SPI:
	$(FLEXSPIN_LOADER) -p $(PORT) $(TARGET_TEST_2) -D baudrate=115200 -t

upload-HC05:

	$(FLEXSPIN_LOADER) -p $(PORT) $(TARGET_TEST_3) -D baudrate=115200 -t

ubuntu-install-dep:
	sudo apt-get install libbluetooth-dev
open-bt-terminal:
	./bt_term.o 00:25:08:E0:1F:EF
find-bt-mac-hc05:
	bluetoothctl menu scan
	namepattern HC-05
	scan on


clean:
	rm -f *.elf *.binary *.pasm *.list

.PHONY: all clean
