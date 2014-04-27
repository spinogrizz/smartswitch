.PHONY: build flash

PROGDEVICE     = attiny13
FLASHDEVICE     = attiny13
CLOCK      = 9600000
PROGRAMMER = usbasp

PROGNAME = smartswitch

build:
	avr-gcc -Wall -Os -std=c99 -DF_CPU=$(CLOCK) -mmcu=$(PROGDEVICE) -c *.c 

	@mkdir -p bin
	@mv *.o bin

	avr-gcc -Wall -Os -std=c99 -DF_CPU=$(CLOCK) -mmcu=$(PROGDEVICE) -o bin/$(PROGNAME).elf bin/*.o

	avr-objcopy -j .text -j .data -O ihex bin/$(PROGNAME).elf bin/$(PROGNAME).hex
	avr-objcopy -j .eeprom -O ihex bin/$(PROGNAME).elf bin/$(PROGNAME).eep

flash:
	avrdude -c $(PROGRAMMER) -F -p $(FLASHDEVICE) -U flash:w:bin/$(PROGNAME).hex:i 

fuses:	
	avrdude -c $(PROGRAMMER) -F -p $(FLASHDEVICE) -U lfuse:w:0x7a:m -U hfuse:w:0xff:m #-U efuse:w:0xf9:m

eeprom:
	avrdude -c $(PROGRAMMER) -F -p $(FLASHDEVICE) -U eeprom:w:bin/$(PROGNAME).eep	

# 	8MHz, no divider
# 	-U lfuse:w:0xe2:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m      

clean:
	rm -f bin/$(PROGNAME).hex
	rm -f bin/$(PROGNAME).elf
	rm -f bin/*.o

all: clean build flash 
