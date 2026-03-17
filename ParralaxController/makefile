FC="./build/bin/flexspin"
PROP_LIB= "./build/include/"
TANK_BOT_LIB= ./include/
PORT= /dev/ttyUSB0
BIN_PROP=main.o
BAUD=156200
SRCDIR = ./src
CC=clang

# Target binaries
FLAGS =
TARGET = main.binary
SIM_TARGET = simulate.o
SRC = src/
CFILES = $(wildcard $(SRCDIR)/*.c)

all: $(TARGET) $(SIM_TARGET)

$(TARGET): $(SRC)
	$(FC) -L $(TANK_BOT_LIB) $(CFILES)

load:
	./build/bin/proploader -p $(PORT) $(SRCDIR)/$(TARGET) -D baudrate=156200

run-simulation:
	$(CC) -I $(TANK_BOT_LIB) $(CFILES) -lncurses -o ./build/$(SIM_TARGET) && ./build/$(SIM_TARGET)

install-flexspin:
	git submodule update --recursive && cd ./flexprop/ && pwd && make install INSTALL=../build 

#compile: $(BIN_PROP)
#	./build/bin/flexspin main.cpp

#set-port:
#	echo ($(FLEXSPIN) -P)

#upload:
#	$(FLEXSPIN) -p $(PORT) build/bin/$(BIN_PROP) -D baudrate=156200 -t

clean:
	rm -r ./build/*;rm $(TARGET);rm *.pasm;*.





