
AVRCC	        = avr-gcc -mmcu=attiny2313 -DF_CPU=16000000 $(DEFINES)
AVROBJCOPY	= avr-objcopy
AVRCFLAGS	= -Wall -Os -g -Wall -I.

all: main.hex

%.o: %.c
	$(AVRCC) $(AVRCFLAGS) -c -o $@ $^

%.o: %.S
	$(AVRCC) $(AVRCFLAGS) -c -o $@ $^

%.hex: %.elf
	$(AVROBJCOPY) -j .text -j .data -O ihex $^ $@

%.bin: %.elf
	$(AVROBJCOPY) -j .text -j .data -O binary $^ $@

main.elf: main.o ds1820.o usbdrv.o usbdrvasm.o
	$(AVRCC) -o $@ $^

install: main.hex
	sudo avrdude -pt2313 -cusbtiny -U flash:w:main.hex:i

clean:
	rm -f *.hex *.bin *.o *.elf
