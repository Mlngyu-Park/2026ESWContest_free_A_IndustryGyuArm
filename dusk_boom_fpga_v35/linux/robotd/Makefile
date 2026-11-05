# dusk_robotd / dusk_servo — static binaries for the Zynq (CC=arm-none-linux-gnueabihf-gcc)
# or native ones for the PC test (make CC=gcc STATIC= test)
CC     ?= gcc
CFLAGS ?= -O2 -Wall -Wextra -Wno-unused-parameter
STATIC ?= -static
B = build

all: $(B)/dusk_robotd $(B)/dusk_servo

$(B)/dusk_robotd: dusk_robotd.c serial_util.c serial_util.h | $(B)
	$(CC) $(CFLAGS) $(STATIC) -o $@ dusk_robotd.c serial_util.c

$(B)/dusk_servo: dusk_servo.c serial_util.c serial_util.h | $(B)
	$(CC) $(CFLAGS) $(STATIC) -o $@ dusk_servo.c serial_util.c

$(B):
	mkdir -p $(B)

test: all
	python3 test_robotd.py $(B)

clean:
	rm -rf $(B)

.PHONY: all test clean
