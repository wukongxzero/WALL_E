FLEXSPIN=build/bin/flexspin
PROP_LIB= ./build/include
TANK_BOT_LIB= ./include/
PORT= /dev/ttyUSB0
BIN_PROP=main.o
BAUD=156200

install-flexspin:
	git submodule update --recursive && cd ./flexprop/ && pwd && make install INSTALL=../build 

compile: $(BIN_PROP)
	$(FLEXSPIN) -I  $(PROP_LIB) -I $(TANK_BOT_LIB) main.cpp -o build/bin/$(BIN_PROP); 

set-port:
	echo ($(FLEXSPIN) -P)

upload:
	$(FLEXSPIN) -p $(PORT) build/bin/$(BIN_PROP) -D baudrate=156200 -t

clean:
	rm -r ./build/*





