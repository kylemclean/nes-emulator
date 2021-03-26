CC = gcc
override CFLAGS += -std=c99 -Wall

.PHONY: all
all: nes


nes: main.c system.c
	$(CC) $(CFLAGS) main.c system.c -o nes -lSDL2

.PHONY: clean
clean:
	rm -f nes

