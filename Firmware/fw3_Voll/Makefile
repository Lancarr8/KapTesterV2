MCU      = atmega328p
F_CPU    = 16000000UL
CC       = avr-gcc
OBJCOPY  = avr-objcopy
AVRDUDE  = avrdude
PROGRAMMER = usbasp
CFLAGS   = -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os -Wall -Wextra -std=c99 \
           -ffunction-sections -fdata-sections
LDFLAGS  = -mmcu=$(MCU) -Wl,--gc-sections
TARGET   = kaptester
SRC      = main.c i2c.c ssd1306.c

all: $(TARGET).hex

$(TARGET).elf: $(SRC)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@
	@avr-size --mcu=$(MCU) --format=avr $<

flash: $(TARGET).hex
	$(AVRDUDE) -c $(PROGRAMMER) -p $(MCU) -U flash:w:$<:i

clean:
	rm -f $(TARGET).elf $(TARGET).hex

.PHONY: all flash clean
