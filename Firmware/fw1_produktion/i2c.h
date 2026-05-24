/*
 * i2c.h — Hardware-TWI Master (ATmega328P)
 * KapTester V2 Firmware — Nico Schmidt
 */

#ifndef I2C_H
#define I2C_H

#include <avr/io.h>

/* I2C-Busfrequenz */
#define I2C_FREQ_HZ  400000UL

/* SSD1306 I2C-Adresse (SA0 = GND → 0x3C) */
#define SSD1306_ADDR 0x3C

void    i2c_init(void);
uint8_t i2c_start(uint8_t addr_rw);   /* 0 = OK, 1 = Fehler */
void    i2c_stop(void);
uint8_t i2c_write(uint8_t data);       /* 0 = ACK, 1 = NACK  */

#endif /* I2C_H */
