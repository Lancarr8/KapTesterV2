/*
 * ssd1306.h — Minimaler SSD1306 OLED-Treiber (128×64, I2C)
 * KapTester V2 Firmware — Nico Schmidt
 *
 * Textmodus: 6×8 Pixel pro Zeichen → 21 Zeichen × 8 Zeilen
 */

#ifndef SSD1306_H
#define SSD1306_H

#include <avr/io.h>

/* Display-Abmessungen */
#define OLED_WIDTH   128
#define OLED_PAGES     8   /* 8 Seiten à 8 Pixel = 64 Pixel Höhe */
#define OLED_COLS_PER_CHAR 6   /* 5 Pixel + 1 Pixel Abstand */
#define OLED_CHARS_PER_ROW (OLED_WIDTH / OLED_COLS_PER_CHAR)  /* 21 */

void ssd1306_init(void);
void ssd1306_clear(void);
void ssd1306_set_cursor(uint8_t col, uint8_t page);
void ssd1306_write_char(char c);
void ssd1306_print(uint8_t col, uint8_t page, const char *str);

#endif /* SSD1306_H */
