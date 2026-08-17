FLEXSPIN = ../ParralaxController/build/bin/flexspin
FLEXSPIN_LOADER = ../ParralaxController/build/bin/proploader
PORT = /dev/ttyUSB0
INC_FLAGS += -I$(abspath ../ParralaxController/build/include)


TARGET = FlashSprites.elf
TEST_TARGET = ReadSprites.elf

SRCS = ./FlashGraphicsToEEPROM.c ./pixelArtSampleData.h
SRCS_TEST = ./TestGraphicsReadFromEEPROM.c

compile:
	$(FLEXSPIN)  $(INC_FLAGS)  $(SRCS) -o $(TARGET)
	$(FLEXSPIN)  $(INC_FLAGS)  $(SRCS_TEST) -o $(TEST_TARGET)

flash:
	$(FLEXSPIN_LOADER) -p $(PORT) $(TARGET) -D baudrate=115200 -t

test:
	$(FLEXSPIN_LOADER) -p $(PORT) $(TEST_TARGET) -D baudrate=115200 -t


