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

# --- Build Rules ---
all: $(TARGET)

$(TARGET): $(SRCS)
	@echo "Linking ADC Libraries and Compiling..."
	$(FLEXSPIN) $(INC_FLAGS) $(SRCS) $(LIB_SRCS) -o $(TARGET)

set-port:
	$PORT = $(FLEXSPIN_LOADER) -P

upload:
	$(FLEXSPIN_LOADER) -p $(PORT) $(TARGET) -D baudrate=156200 -t



clean:
	rm -f *.elf *.binary *.pasm *.list

.PHONY: all clean
