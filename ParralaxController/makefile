FC="./build/bin/flexspin"
PROP_LOADER ="./build/bin/proploader"
PROP_LIB= "./build/include/"
SIMPLE_IDE_LIB = "./Simple-Libraries/Learn/Simple Libraries/"
TANK_BOT_LIB= ./include/
UNITY_TEST_LIB=./test/Unity/src/
PORT= /dev/ttyUSB2
BIN_PROP=main.o
BAUD=115200
SRCDIR = ./src
CC=clang
INC_FLAGS += -I$(abspath ./build/include)
SAMPLE_DATA =  -D "INCLUDE_SAMPLES" 
BAUD = 115200


# Target binaries
FLAGS =
TESTFLAGS = -lncurses -lm 
TARGET = main.binary
SIM_TARGET = simulate.o
SRC = src/
ALL_CFILES = $(wildcard $(SRCDIR)/*.c)
CFILES = $(filter-out $(SRCDIR)/main.c, $(ALL_CFILES))
UNITY_CFILE = test/Unity/src/unity.c
TEST_SOURCES = $(wildcard $(TESTDIR)/Test*.c)

all: $(TARGET) $(SIM_TARGET)

$(TARGET): $(SRC)
	$(FC) -I $(TANK_BOT_LIB)  $(CFILES) $(SRC)main.c -o $(SRC)$(TARGET)

load:
	#./build/bin/proploader -p $(PORT) $(SRCDIR)/$(TARGET) -D baudrate=156200 -t
	$(PROP_LOADER) -p $(PORT) $(SRC)$(TARGET) -D baudrate=$(BAUD) -t


run-simulation:
	$(CC) -I$(TANK_BOT_LIB) $(CFILES) $(SAMPLE_DATA) -D "SIMULATION_SCREEN" -lncurses -lm src/main.c -o ./build/$(SIM_TARGET) && ./build/$(SIM_TARGET)


install-flexspin:
	git submodule init 
	mkdir -p ./build
	git submodule update --recursive && cd ./flexprop/ && pwd && make install INSTALL=../build 

test-clang:
	#src/tankStatus.c test/unity/unity.c test/test_tankStatus.c

	$(CC) -I$(TANK_BOT_LIB) $(CFILES) $(UNITY_CFILE) -I$(UNITY_TEST_LIB)* -D "INCLUDE_SAMPLES" -D "SIMULATION_SCREEN" $(TESTFLAGS) ./test/TestTankStatus.c -o ./build/test.o
	./build/test.o
	$(CC) -I$(TANK_BOT_LIB) $(CFILES) $(UNITY_CFILE) -I$(UNITY_TEST_LIB)* -D "INCLUDE_SAMPLES" -D "SIMULATION_SCREEN" $(TESTFLAGS) ./test/TestGraphicsNoRender.c -o ./build/test.o
	./build/test.o
	$(CC) -I$(TANK_BOT_LIB) $(CFILES) $(UNITY_CFILE) -I$(UNITY_TEST_LIB)* -D "INCLUDE_SAMPLES" -D "SIMULATION_SCREEN" $(TESTFLAGS) ./test/TestRenderCurses.c -o ./build/test.o
	./build/test.o
	$(CC) -I$(TANK_BOT_LIB) $(CFILES) $(UNITY_CFILE) -I$(UNITY_TEST_LIB)* -D "INCLUDE_SAMPLES" -D "SIMULATION_SCREEN"  $(TESTFLAGS) ./test/TestPubSubTankStatus.c -o ./build/test.o
	./build/test.o
	$(CC) -I$(TANK_BOT_LIB) $(CFILES) $(UNITY_CFILE) -I$(UNITY_TEST_LIB)* -D "INCLUDE_SAMPLES" -D "SIMULATION_SCREEN" $(TESTFLAGS) ./test/TestRotateSprite.c -o ./build/test.o
	./build/test.o

#	setup $(TEST_SOURCES)
#	@echo "--- Starting Unit Tests ---"
#	@for file in $(TEST_SOURCES); do \
#		echo "Compiling and running $$file..."; \
#		$(CC) $(CFLAGS) $(SRC_FILES) $(UNITY_SRC) $$file -o $(BUILDDIR)/runner; \
#		./$(BUILDDIR)/runner; \
#	done
#	@echo "--- All Tests Complete ---"
#
#
install-simple-ide:
	add-apt-repository ppa:rock-core/qt4
	apt update
	apt install libqtgui4 libqtcore4
	wget https://archive.org/download/simpleide-0.9.66-amd64-debian-linux/simpleide-0.9.66-amd64-debian-linux.deb -P ./build/
compile: $(BIN_PROP)
	./build/bin/flexspin main.cpp

port:
	$(PROP_LOADER) -P

upload:

	$(PROP_LOADER) -p $(PORT) $(SRCDIR)/$(TARGET) -D baudrate=156200

clean:
	rm -r ./build/*;rm $(TARGET);rm *.pasm;*.





