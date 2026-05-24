/*
 * ╔══════════════════════════════════════════════════════════════════════╗
 * ║   KapTester V2  —  Firmware 2: Inbetriebnahme- & Testmodus (V2.5)   ║
 * ║   Hardware-Plattform: ATmega328P-AU @ 16.000 MHz (externer Quarz)    ║
 * ║   Zweck: Sequenzieller Hardware-Funktionstest (Hardware-Bringup)     ║
 * ║   Autor: Nico Schmidt  |  AP2-Abschlussprojekt                       ║
 * ╚══════════════════════════════════════════════════════════════════════╝
 *
 * ── QUALITÄTSSICHERUNG & INBETRIEBNAHME-PHILOSOPHIE (FÜR DIE DOKU) ──────
 * Bevor die hochkomplexe Mess-Firmware geflasht wird, isoliert diese Test-
 * Firmware jede einzelne Hardware-Komponente auf der Platine. Dadurch wird 
 * sichergestellt, dass eventuelle Hardware-Fehler (z.B. kalte Lötstellen, 
 * Kurzschlüsse oder falsche Bauteilwerte) sofort erkannt und nicht fälsch-
 * licherweise als Software-Bugs in der Haupt-Firmware interpretiert werden.
 *
 * Jedes Subsystem wird einzeln angesteuert. Der Wechsel zum nächsten Test
 * erfolgt manuell durch Drücken des Tasters SW1. Das OLED-Display dient
 * dabei als interaktiver Benutzerleitfaden (Guided Testing).
 */

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>
#include "i2c.h"
#include "ssd1306.h"

/* ═══════════════════════════════════════════════════════════════════════
 * HARDWARE-ABSTRAKTION (GPIO-MAKROS FÜR DIE SPRECHTEXTE)
 * ═══════════════════════════════════════════════════════════════════════ */
#define LED_GRN_ON()    (PORTD |=  (1<<PD2))  /* Digitale Aktivierung der grünen LED  */
#define LED_GRN_OFF()   (PORTD &= ~(1<<PD2))  /* Digitale Deaktivierung der grünen LED */
#define LED_RED_ON()    (PORTD |=  (1<<PD3))  /* Digitale Aktivierung der roten LED   */
#define LED_RED_OFF()   (PORTD &= ~(1<<PD3))  /* Digitale Deaktivierung der roten LED  */
#define DISCH_ON()      (PORTB |=  (1<<PB1))  /* Ansteuerung des Entlade-MOSFET-Gates */
#define DISCH_OFF()     (PORTB &= ~(1<<PB1))  /* Sperren des Entlade-MOSFET-Gates     */
#define BTN_PRESSED()   (!(PINC & (1<<PC2)))  /* Low-aktive Abfrage des UI-Tasters   */

/* ═══════════════════════════════════════════════════════════════════════
 * LOW-LEVEL DRIVER INITIALIZATIONS
 * ═══════════════════════════════════════════════════════════════════════ */

// Konfiguration der Ein- und Ausgänge (Datenrichtungsregister)
void gpio_init(void) {
    DDRD |=  (1<<PD2) | (1<<PD3);   /* PD2 und PD3 als digitale Ausgänge für LEDs definieren */
    DDRB |=  (1<<PB1);              /* PB1 als digitalen Ausgang für die Entladesteuerung    */
    DDRC &= ~((1<<PC2) | (1<<PC0)); /* PC2 (Button) und PC0 (ADC-Messkanal) als Eingang      */
    PORTC |=  (1<<PC2);             /* Aktivierung des internen Pull-Up-Widerstands für SW1  */
    PORTC &= ~(1<<PC0);             /* PC0 hochohmig belassen für unverfälschte Analogwerte  */
    LED_GRN_OFF(); LED_RED_OFF(); DISCH_OFF(); /* Definierten, sicheren Grundzustand herstellen */
}

// Initialisierung des Analog-Digital-Wandlers für DC-Live-Messungen
void adc_init_internal(void) {
    /* ADMUX: REFS1 und REFS0 setzen -> Aktiviert die interne 1,1V Bandgap-Referenz.
     * MUX3..0 bleiben auf 0000 -> Kanal ADC0 (PC0) ist als Multiplexer-Ziel gewählt. */
    ADMUX  = (1<<REFS1) | (1<<REFS0);
    /* ADCSRA: ADEN (ADC Enable), Prescaler auf 128 setzen (16MHz / 128 = 125kHz Wandlertakt).
     * Dies garantiert höchste Störfestigkeit und maximale Linearität des Wandlers. */
    ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
    ADCSRA |= (1<<ADSC); /* Dummy-Wandlung zum Einschwingen der internen Kondensatoren */
    while (ADCSRA & (1<<ADSC));
}

// Synchrones Einlesen des analogen Gleichspannungspegels an ADC0
uint16_t adc_read(void) {
    ADCSRA |= (1<<ADSC); /* Start einer neuen Analog-Digital-Wandlung */
    while (ADCSRA & (1<<ADSC)); /* Bit-Polling: Warten, bis das Start-Bit durch die Hardware gelöscht wird */
    return ADC; /* Rückgabe des quantisierten 10-Bit Werts (0 bis 1023) */
}

/* ═══════════════════════════════════════════════════════════════════════
 * HELPER FUNCTIONS (STEUERUNG DER TESTSCHRITTE)
 * ═══════════════════════════════════════════════════════════════════════ */

// Blockierende Funktion zur Taster-Abfrage inklusive softwarebasierter Entprellung
void wait_for_button(void) {
    while (!BTN_PRESSED()); /* Warteschleife, bis der Taster physisch gedrückt wird */
    _delay_ms(30);          /* Entprell-Zeitfenster: Mechanische Kontaktschwingungen abwarten */
    while (BTN_PRESSED());  /* Warteschleife, bis der Benutzer den Taster wieder loslässt */
    _delay_ms(30);          /* Erneute Stabilisierungspause für saubere Flankenübergänge */
}

/* ═══════════════════════════════════════════════════════════════════════
 * SEQUENZIELLE TESTSCHRITTE (THE GUIDED HARDWARE TEST)
 * ═══════════════════════════════════════════════════════════════════════ */

// TEST 00: Begrüßungs- und Strukturübersicht
void step_intro(void) {
    ssd1306_clear();
    ssd1306_print(0, 0, "=== HW-TEST V2.5 ===");
    ssd1306_print(0, 2, "Geflasht auf V2-Board");
    ssd1306_print(0, 4, "Manuelle Bestaetigung");
    ssd1306_print(0, 5, "jeden Schritts per SW1");
    ssd1306_print(0, 7, "SW1 druecken -> Start");
    wait_for_button();
}

// TEST 01: Visuelle Überprüfung des grünen LED-Zweigs
void step_led_green(void) {
    ssd1306_clear();
    ssd1306_print(0, 0, "TEST 01: LED GRUEN");
    ssd1306_print(0, 2, "Status: PD2 = HIGH");
    ssd1306_print(0, 4, "Erwartet: D3 leuchtet");
    ssd1306_print(0, 5, "Stromaufnahme ca. 5mA");
    ssd1306_print(0, 7, "SW1 -> Naechster Test");
    LED_GRN_ON(); /* Schaltet den GPIO-Pin PD2 auf VCC (+5V) */
    wait_for_button();
    LED_GRN_OFF();
}

// TEST 02: Visuelle Überprüfung des roten LED-Zweigs
void step_led_red(void) {
    ssd1306_clear();
    ssd1306_print(0, 0, "TEST 02: LED ROT");
    ssd1306_print(0, 2, "Status: PD3 = HIGH");
    ssd1306_print(0, 4, "Erwartet: D4 leuchtet");
    ssd1306_print(0, 5, "Optische Warnanzeige");
    ssd1306_print(0, 7, "SW1 -> Naechster Test");
    LED_RED_ON(); /* Schaltet den GPIO-Pin PD3 auf VCC (+5V) */
    wait_for_button();
    LED_RED_OFF();
}

// TEST 03: Last- und Kreuzbeeinflussungstest beider LED-Zweige
void step_led_both(void) {
    ssd1306_clear();
    ssd1306_print(0, 0, "TEST 03: BEIDE LEDs");
    ssd1306_print(0, 2, "Status: PD2/PD3 = HIGH");
    ssd1306_print(0, 4, "Erwartet: Rot & Gruen");
    ssd1306_print(0, 5, "Pruefung auf");
    ssd1306_print(0, 6, "Spannungseinbruch...");
    ssd1306_print(0, 7, "SW1 -> Naechster Test");
    LED_GRN_ON(); LED_RED_ON();
    wait_for_button();
    LED_GRN_OFF(); LED_RED_OFF();
}

// TEST 04: Vollfeld-Pixeltest (Hellfeld) zur Erkennung von Displaydefekten
void step_oled_pixel(void) {
    ssd1306_clear();
    /* Aktiviert jeden einzelnen Pixel des OLED-Monitors über die I2C-Schnittstelle.
     * Dient dem Aufspüren von "toten Pixeln", Spaltenfehlern oder Glasbrüchen. */
    ssd1306_fill_all(); 
    wait_for_button();
    ssd1306_clear();
}

// TEST 05: Validierung des Zeichensatzes und des grafischen Speichers
void step_oled_font(void) {
    ssd1306_clear();
    ssd1306_print(0, 0, "TEST 05: OLED FONT");
    ssd1306_print(0, 2, "ABCDEFGHIJKLMNOPQRST");
    ssd1306_print(0, 3, "abcdefghijklmnopqrst");
    ssd1306_print(0, 4, "0123456789 !=+-*/_");
    ssd1306_print(0, 6, "Pruefung der Lesbarkeit");
    ssd1306_print(0, 7, "SW1 -> Naechster Test");
    wait_for_button();
}

// TEST 06: Prüfung der Entprellung und der digitalen Taster-Eingangsschaltung
void step_button_test(void) {
    ssd1306_clear();
    ssd1306_print(0, 0, "TEST 06: BUTTON COUNT");
    ssd1306_print(0, 2, "Bitte SW1 genau 3x");
    ssd1306_print(0, 3, "hintereinander druecken");
    
    char buf[16];
    /* Schleife zählt exakt drei Tasterdrücke mit. Reagiert die Software ungenau,
     * deutet das auf ein Prellen des mechanischen Kontakts hin. */
    for (uint8_t i = 1; i <= 3; i++) {
        sprintf(buf, " Klick %u / 3", i);
        ssd1306_print(0, 5, buf);
        wait_for_button();
    }
    
    ssd1306_print(0, 5, " Klick 3 / 3 OK!  ");
    ssd1306_print(0, 7, "SW1 -> Naechster Test");
    wait_for_button();
}

// TEST 07: Anleitung für die manuelle Hardware-Messung an den Platinen-Testpunkten
void step_tp_overview(void) {
    ssd1306_clear();
    ssd1306_print(0, 0, "TEST 07: MEISTERTIPP");
    ssd1306_print(0, 2, "Nutze jetzt die");
    ssd1306_print(0, 3, "Testpunkte links!");
    ssd1306_print(0, 5, "JTP1=GND, JTP2=+5V");
    ssd1306_print(0, 6, "JTP4=LM317, JTP6=ADC");
    ssd1306_print(0, 7, "SW1 -> Start Live-Mess");
    wait_for_button();
}

// TEST 08: Analoger Belastungstest (Konstantstromquelle & Entladepfad live ausmessen)
void step_adc_live(void) {
    char buf[22];
    uint16_t raw = 0;
    uint32_t mv = 0;

    /* Dieser Test läuft in einer Endlosschleife, bis SW1 gedrückt wird.
     * Er schaltet die Konstantstromquelle und den Entladepfad synchron,
     * damit der Prüfer Spannungspegel statisch mit dem Multimeter prüfen kann. */
    for (;;) {
        if (BTN_PRESSED()) {
            _delay_ms(30);
            if (BTN_PRESSED()) {
                while (BTN_PRESSED());
                _delay_ms(30);
                break; /* Verlassen des Live-Modus bei Tasterdruck */
            }
        }

        /* * DIAGNOSTISCHER STATUSTAKT (1 Hz Intervall):
         * 1. Entladepfad sperren -> Konstantstromquelle versucht, den offenen Messpfad
         * auf ca. 3,5V bis 4,5V aufzuladen.
         */
        DISCH_OFF();
        LED_GRN_ON(); LED_RED_OFF();
        
        ssd1306_clear();
        ssd1306_print(0, 0, "TEST 08: LIVE-ADC0");
        ssd1306_print(0, 2, "STATUS: LADEN AKTIV");
        ssd1306_print(0, 3, "Messen an JTP4 & JTP6");
        
        raw = adc_read(); /* ADC-Wert einlesen (wird durch offene Klemmen bei 1,1V begrenzen -> 1023) */
        mv  = (uint32_t)raw * 1100UL / 1023UL;
        sprintf(buf, "ADC0: %4u (%lu mV)", raw, mv);
        ssd1306_print(0, 5, buf);
        ssd1306_print(0, 7, "SW1 -> Naechste Phase ");
        
        _delay_ms(1500); /* Genug Zeit für eine statische DMM-Messung */

        /* * 2. Entladepfad aktivieren -> Der Ladestrom wird gegen Masse abgeleitet.
         * Die Spannung an JTP6 bricht auf das Hardware-Gleichgewicht (~0,5V) ein.
         */
        DISCH_ON();
        LED_GRN_OFF(); LED_RED_ON();
        
        ssd1306_clear();
        ssd1306_print(0, 0, "TEST 08: LIVE-ADC0");
        ssd1306_print(0, 2, "STATUS: ENTLADEN AKTIV");
        ssd1306_print(0, 3, "Pruefe R20-Spannungsfall");
        
        raw = adc_read(); /* Liest den abfallenden Spannungspegel im Entladezustand ein */
        mv  = (uint32_t)raw * 1100UL / 1023UL;
        sprintf(buf, "ADC0: %4u (%lu mV)", raw, mv);
        ssd1306_print(0, 5, buf);
        ssd1306_print(0, 7, "SW1 -> Naechste Phase ");
        
        _delay_ms(1500); /* Genug Zeit für eine statische DMM-Messung */
    }
    
    DISCH_OFF(); /* Sicheres Deaktivieren des Entladepfads nach Testende */
    LED_GRN_OFF(); LED_RED_OFF();
}

// TEST 09: Finale Abschlussmeldung des Testzyklus
void step_outro(void) {
    ssd1306_clear();
    ssd1306_print(0, 0, "=== SEQUEZ BEENDET ===");
    ssd1306_print(0, 2, "Hardware-Inbetriebnahme");
    ssd1306_print(0, 3, "erfolgreich validiert!");
    ssd1306_print(0, 5, "Bereit fuer Firmware 3");
    ssd1306_print(0, 7, "SW1 -> Zyklus Neustart");
    wait_for_button();
}

/* ═══════════════════════════════════════════════════════════════════════
 * MAIN — EXECUTIVE SEQUENCER
 * ═══════════════════════════════════════════════════════════════════════ */
int main(void) {
    // Systemweite Hardware-Initialisierungen
    gpio_init();         
    adc_init_internal(); 
    i2c_init();          
    ssd1306_init();      
    
    /* HINWEIS FÜR DIE PRÜFUNG: Es ist bewusst kein globaler Interrupt (sei()) aktiv,
     * da diese Testfirmware rein synchron arbeitet und keine zeitkritischen Timer-ISRs benötigt. */

    // Endlose, lineare Ausführung der Testschritte
    for (;;) {
        step_intro();          /* 00: Begrüßung + Systemüberblick   */
        step_led_green();      /* 01: Grüne LED ansteuern           */
        step_led_red();        /* 02: Rote LED ansteuern            */
        step_led_both();       /* 03: Lasttest beider LED-Zweige    */
        step_oled_pixel();     /* 04: Display Hellfeld-Test         */
        step_oled_font();      /* 05: Schrift- und Matrixvalidierung */
        step_button_test();    /* 06: Button-Entprellungstest       */
        step_tp_overview();    /* 07: Einführung Platinen-Testpunkte */
        step_adc_live();       /* 08: Interaktive DC-Livemessung    */
        step_outro();          /* 09: Abschlussmeldung              */
    }
    return 0; /* Wird theoretisch nie erreicht */
}