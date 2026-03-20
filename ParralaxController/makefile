FC="./build/bin/flexspin"
PROP_LIB= "./build/include/"
TANK_BOT_LIB= ./include/
UNITY_TEST_LIB=./test/Unity/src/
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
ALL_CFILES = $(wildcard $(SRCDIR)/*.c)
CFILES = $(filter-out $(SRCDIR)/main.c, $(ALL_CFILES))
UNITY_CFILE = test/Unity/src/unity.c
TEST_SOURCES = $(wildcard $(TESTDIR)/Test*.c)

all: $(TARGET) $(SIM_TARGET)

$(TARGET): $(SRC)
	$(FC) -L $(TANK_BOT_LIB) $(CFILES)

load:
	./build/bin/proploader -p $(PORT) $(SRCDIR)/$(TARGET) -D baudrate=156200

run-simulation:
	$(CC) -I$(TANK_BOT_LIB) $(CFILES) -D "INCLUDE_SAMPLES" -D "SIMULATION_SCREEN" -lncurses src/main.c -o ./build/$(SIM_TARGET) && ./build/$(SIM_TARGET)

install-flexspin:
	git submodule update --recursive && cd ./flexprop/ && pwd && make install INSTALL=../build 

test-clang:
	#src/tankStatus.c test/unity/unity.c test/test_tankStatus.c

	$(CC) -I$(TANK_BOT_LIB) $(CFILES) $(UNITY_CFILE) -I$(UNITY_TEST_LIB)* -D "INCLUDE_SAMPLES" -D "SIMULATION_SCREEN" -lncurses ./test/TestTankStatus.c -o ./build/test.o
	./build/test.o
	$(CC) -I$(TANK_BOT_LIB) $(CFILES) $(UNITY_CFILE) -I$(UNITY_TEST_LIB)* -D "INCLUDE_SAMPLES" -D "SIMULATION_SCREEN" -lncurses ./test/TestGraphicsNoRender.c -o ./build/test.o
	./build/test.o
	$(CC) -I$(TANK_BOT_LIB) $(CFILES) $(UNITY_CFILE) -I$(UNITY_TEST_LIB)* -D "INCLUDE_SAMPLES" -D "SIMULATION_SCREEN" -lncurses ./test/TestRenderCurses.c -o ./build/test.o
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
compile: $(BIN_PROP)
	./build/bin/flexspin main.cpp

set-port:
	echo ($(FLEXSPIN) -P)

upload:
	$(FLEXSPIN) -p $(PORT) build/bin/$(BIN_PROP) -D baudrate=156200 -t

clean:
	rm -r ./build/*;rm $(TARGET);rm *.pasm;*.





