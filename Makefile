CC = gcc
override CFLAGS += -std=c99 -Wall

.PHONY: all
all: nes


nes:
	$(CC) $(CFLAGS) main.c -o nes

.PHONY: clean
clean:
	rm -f nes

