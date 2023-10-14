# --- Makefile for simple Adder example.

# ARM cross-compiler
ARCH	= arm64
PROC	= aarch64
TYPE	= linux-gnu
CC	= $(PROC)-$(TYPE)-gcc
CFLAGS	= # -static
STRIP	= $(PROC)-$(TYPE)-strip


all: example

example: example.c
	$(CC) example.c $(CFLAGS) -o example
	$(STRIP) $@ 

debug:	example.c
	$(CC) example.c $(CFLAGS) -g -o example

clean:
	$(RM) *.o *~
	$(RM) example 
