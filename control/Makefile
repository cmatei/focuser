CC	= gcc -Wall -O0 -g
CFLAGS	= $(shell pkg-config --cflags libusb-1.0)
LIBS	= $(shell pkg-config --libs libusb-1.0)


all: fc

.c.o:
	$(CC) $(CFLAGS) -c -o $@ $^

fc: fc.o
	$(CC) $(CFLAGS) -o $@ $^ $(LIBS)

clean:
	-rm -f *.o fc
