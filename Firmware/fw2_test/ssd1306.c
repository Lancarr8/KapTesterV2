/*
 * ssd1306.c — Minimaler SSD1306 OLED-Treiber (128×64, I2C)
 * KapTester V2 Firmware — Nico Schmidt
 *
 * Kommunikation über i2c.h/c (Hardware-TWI).
 * Nur Textausgabe via 5×7-Font (font5x7.h).
 *
 * I2C-Protokoll SSD1306:
 *   Command:  START | ADDR<<1 | WRITE | 0x00 | cmd  | STOP
 *   Data:     START | ADDR<<1 | WRITE | 0x40 | data | ... | STOP
 */

#include "ssd1306.h"
#include "i2c.h"
#include "font5x7.h"
#include <avr/pgmspace.h>
#include <string.h>

/* ── Hilfsfunktionen ────────────────────────── */

static void oled_cmd(uint8_t cmd)
{
    i2c_start((SSD1306_ADDR << 1) | 0);  /* WRITE */
    i2c_write(0x00);   /* Control-Byte: Co=0, D/C=0 → Command */
    i2c_write(cmd);
    i2c_stop();
}

static void oled_data_start(void)
{
    i2c_start((SSD1306_ADDR << 1) | 0);
    i2c_write(0x40);   /* Control-Byte: Co=0, D/C=1 → Data-Stream */
}

/* ── SSD1306 initialisieren ─────────────────── */
void ssd1306_init(void)
{
    oled_cmd(0xAE);         /* Display OFF */
    oled_cmd(0xD5); oled_cmd(0x80); /* Taktteiler */
    oled_cmd(0xA8); oled_cmd(0x3F); /* Multiplex 64 Zeilen */
    oled_cmd(0xD3); oled_cmd(0x00); /* Display-Offset = 0 */
    oled_cmd(0x40);                 /* Startzeile = 0 */
    oled_cmd(0x8D); oled_cmd(0x14); /* Ladepumpe EIN */
    oled_cmd(0x20); oled_cmd(0x00); /* Adressierungsmodus: Horizontal */
    oled_cmd(0xA1);                 /* Segment-Remap (links→rechts) */
    oled_cmd(0xC8);                 /* COM-Scan: von unten nach oben */
    oled_cmd(0xDA); oled_cmd(0x12); /* COM-Pins-Konfiguration */
    oled_cmd(0x81); oled_cmd(0xCF); /* Kontrast */
    oled_cmd(0xD9); oled_cmd(0xF1); /* Pre-Charge-Periode */
    oled_cmd(0xDB); oled_cmd(0x40); /* VCOMH-Pegel */
    oled_cmd(0xA4);                 /* Normalmodus (aus RAM) */
    oled_cmd(0xA6);                 /* Normale Polarität (nicht invertiert) */
    oled_cmd(0xAF);                 /* Display EIN */

    ssd1306_clear();
}

/* ── Display löschen (alles schwarz) ────────── */
void ssd1306_clear(void)
{
    /* Adressbereich: Spalten 0–127, Seiten 0–7 */
    oled_cmd(0x21); oled_cmd(0x00); oled_cmd(0x7F); /* Spalten 0–127 */
    oled_cmd(0x22); oled_cmd(0x00); oled_cmd(0x07); /* Seiten   0–7  */

    oled_data_start();
    for (uint16_t i = 0; i < OLED_WIDTH * OLED_PAGES; i++)
        i2c_write(0x00);
    i2c_stop();
}

/* ── Cursor setzen (col = Pixel-Spalte 0–127, page = 0–7) ── */
void ssd1306_set_cursor(uint8_t col, uint8_t page)
{
    oled_cmd(0x21); oled_cmd(col);  oled_cmd(0x7F); /* Spalte */
    oled_cmd(0x22); oled_cmd(page); oled_cmd(0x07); /* Seite  */
}

/* ── Ein Zeichen ausgeben (6×8 Pixel) ──────── */
void ssd1306_write_char(char c)
{
    if ((uint8_t)c < 0x20 || (uint8_t)c > 0x7F) c = 0x20; /* Unbekannte Zeichen → Space */

    oled_data_start();
    for (uint8_t col = 0; col < 5; col++)
        i2c_write(pgm_read_byte(&font5x7[c - 0x20][col]));
    i2c_write(0x00);  /* 1 Pixel Abstand */
    i2c_stop();
}

/* ── Text ab Position (col in Zeichen, page 0–7) ausgeben ── */
/* Beispiel: ssd1306_print(0, 3, "Messe...") → Zeile 3, Anfang */
void ssd1306_print(uint8_t col, uint8_t page, const char *str)
{
    ssd1306_set_cursor(col * OLED_COLS_PER_CHAR, page);
    while (*str)
        ssd1306_write_char(*str++);
}
