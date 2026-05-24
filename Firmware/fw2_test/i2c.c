/*
 * i2c.c — Hardware-TWI Master (ATmega328P)
 * KapTester V2 Firmware — Nico Schmidt
 *
 * Nutzt das eingebaute TWI-Modul des ATmega328P.
 * PC4 = SDA, PC5 = SCL (fest verdrahtet durch TWI-Hardware).
 */

#include "i2c.h"
#include <util/twi.h>

/* TWBR-Wert für gewünschte SCL-Frequenz
 * TWBR = (F_CPU / F_SCL - 16) / (2 * Prescaler)
 * Prescaler = 1 (TWSR = 0x00)                   */
#define TWBR_VAL  ((uint8_t)((F_CPU / I2C_FREQ_HZ - 16UL) / 2UL))

/* ── Warten bis TWI-Operation abgeschlossen ─── */
static inline void twi_wait(void)
{
    while (!(TWCR & (1 << TWINT)));
}

/* ── I2C initialisieren ─────────────────────── */
void i2c_init(void)
{
    TWSR = 0x00;          /* Prescaler = 1 */
    TWBR = TWBR_VAL;
    TWCR = (1 << TWEN);   /* TWI einschalten */
}

/* ── START + Adresse senden ─────────────────── */
/* addr_rw = (Adresse << 1) | R/W-Bit           */
/* Rückgabe: 0 = OK, 1 = Fehler                 */
uint8_t i2c_start(uint8_t addr_rw)
{
    /* START-Condition senden */
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    twi_wait();

    uint8_t status = TWSR & 0xF8;
    if (status != TW_START && status != TW_REP_START)
        return 1;

    /* Adresse + R/W senden */
    TWDR = addr_rw;
    TWCR = (1 << TWINT) | (1 << TWEN);
    twi_wait();

    status = TWSR & 0xF8;
    return (status == TW_MT_SLA_ACK) ? 0 : 1;
}

/* ── STOP-Condition senden ──────────────────── */
void i2c_stop(void)
{
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
    /* Warten bis STOP gesendet (TWSTO wird HW-seitig gelöscht) */
    while (TWCR & (1 << TWSTO));
}

/* ── 1 Byte senden ──────────────────────────── */
/* Rückgabe: 0 = ACK empfangen, 1 = NACK/Fehler */
uint8_t i2c_write(uint8_t data)
{
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    twi_wait();
    return ((TWSR & 0xF8) == TW_MT_DATA_ACK) ? 0 : 1;
}
