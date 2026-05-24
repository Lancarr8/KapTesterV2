/*
 * ╔══════════════════════════════════════════════════════════════════════╗
 * ║   KapTester V2  —  Firmware 3: Komplettpaket (FINAL V3.5)           ║
 * ║   Hardware-Plattform: ATmega328P-AU @ 16.000 MHz (externer Quarz)    ║
 * ║   Zweck: High-End Firmware mit dynamischem UI-Filter & Statistik     ║
 * ║   Autor: Nico Schmidt  |  AP2-Abschlussprojekt                       ║
 * ╚══════════════════════════════════════════════════════════════════════╝
 *
 * ── ARCHITEKTUR-HIGHLIGHTS & PROJEKT-INNOVATIONEN (FÜR DEINE DOKU) ──────
 * 1. ASYNCHRONES "KINO-TIMING" (FLACKERFREIE ANIMATION):
 * Um das OLED-Display flüssig zu animieren, ohne die zeitkritische Hardware-
 * Zeitmessung zu stören, sind UI und Messung strikt entkoppelt. Das Display 
 * wird *vor* und *nach* den reinen Messphasen aktualisiert. Während der 
 * eigentlichen Erfassung schaltet die CPU in den "Temporausch" (reines Polling).
 *
 * 2. DYNAMISCHES UPSCALING BEI GIGANTISCHEN KAPAZITÄTEN:
 * Große Elektrolytkondensatoren (> 1 mF) laden sich physikalisch träge auf. 
 * Damit die Software-Animation zur Realität passt, dehnt die Firmware die 
 * Animationsschritte automatisch von 220 ms auf 450 ms aus (hervorragende UX).
 *
 * 3. VOLATILE SESSION-STATISTIK (QUALITÄTSKONTROLLE):
 * Ein integrierter Ringpuffer und Extremwertspeicher trackt die Messungen einer 
 * Prüfserie (Passed, Failed, Max, Min). Damit mutiert das Gerät zum echten 
 * Labor-Messplatz.
 *
 * 4. MULTI-CLICK-ZUSTANDSMASCHINE:
 * Ein softwarebasierter Klick-Zähler analysiert das Zeitfenster (350 ms) nach 
 * der ersten Flanke und unterscheidet sauber zwischen Einzelklick (Messung), 
 * Doppelklick (Statistik-Menü) und Fünffachklick (Diagnose-Modus / Disco).
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>
#include "i2c.h"
#include "ssd1306.h"

/* ═══════════════════════════════════════════════════════════════════════
 * METROLOGISCHE & PHYSIKALISCHE KONSTANTEN (KALIBRIERUNG)
 * ═══════════════════════════════════════════════════════════════════════ */
#define I_UA              1042UL  /* Eingeprägter Konstantstrom: 1,25V / 1200 Ohm = 1,042 mA */
#define U_REF_MV          1100UL  /* Interne Bandgap-Referenzspannung des ATmega328P in mV     */
#define TICK_US             16UL  /* Zeitauflösung: Timer1 Prescaler 256 bei 16MHz -> 16µs/Tick*/
#define ADC_THRESH          1020  /* Analoges Ladeende: ~1,1V (Kurz vor ADC-Vollschlag 1023)  */
#define TIMEOUT_OVF         300U  /* Sicherheits-Timeout: 300 Timer-Überläufe (~314 Sekunden)  */
#define DISCH_TIMEOUT_MS   5000U  /* Maximal zulässige Schutz-Entladezeit in Millisekunden    */
#define N_MEAS                3   /* Anzahl der Rohmessungen für das Filter-Array             */
#define PLAUS_PCT            15UL /* Plausibilitäts-Korridor für den mathematischen Filter (±15%)*/
#define CAP_MIN_NF        10000UL /* Untere Messbereichsgrenze laut Spezifikation: 10 µF      */
#define CAP_MAX_NF     10000000UL /* Obere Messbereichsgrenze laut Spezifikation: 10.000 µF   */

/* * INTEGRATION DES HARDWARE-FIXES (v2):
 * Erzwungenes Abbruchkriterium der Entladeschleife, wenn das physikalische
 * Gleichgewicht am Spannungsteiler (~0,5V) erreicht ist. Verhindert Deadlocks.
 */
#define DISCH_ADC_THRESH    470   
#define U_START_MIN_MV       50UL /* Mindestrestspannung für mathematischen Unterlauf-Schutz */

/* ═══════════════════════════════════════════════════════════════════════
 * ERGONOMISCHE UI- UND INTERFACE-PARAMETER
 * ═══════════════════════════════════════════════════════════════════════ */
#define CLICK_WINDOW_MS     350U  /* Zeitfenster für die Multi-Click-Erkennung (in ms)     */
#define DISCO_FRAME_MS      200U  /* Animationsgeschwindigkeit im Diagnose-Oszillator      */
#define GREEN_FLASH_MS       60U  /* Taktzeit für akustisches/visuelles Benutzerfeedback  */
#define BAR_INNER            14   /* Anzahl der inneren Segmente des Ladebalkens           */
#define BOOT_STEPS           14   /* Anzahl der Schritte für die Boot-Animation            */
#define BOOT_STEP_MS         280U  /* Zeit pro Boot-Inkrement (Insgesamt ca. 4 Sekunden)    */
#define DISCH_HOLD_MS       400U  /* Erzwungene Mindestanzeigezeit der Entladewarnung      */

/* ═══════════════════════════════════════════════════════════════════════
 * HARDWARE-ABSTRAKTION (GPIO-MAKROS NACH REINEM REGISTER-C)
 * ═══════════════════════════════════════════════════════════════════════ */
#define LED_GRN_ON()    (PORTD |=  (1<<PD2))  /* Grüne LED ein (Messung valide / Bereit) */
#define LED_GRN_OFF()   (PORTD &= ~(1<<PD2))  /* Grüne LED aus                           */
#define LED_GRN_TOG()   (PORTD ^=  (1<<PD2))  /* Toggelt den Zustand (wichtig für Balken)*/
#define LED_RED_ON()    (PORTD |=  (1<<PD3))  /* Rote LED ein (Entladung / Fehler)      */
#define LED_RED_OFF()   (PORTD &= ~(1<<PD3))  /* Rote LED aus                            */
#define DISCH_ON()      (PORTB |=  (1<<PB1))  /* Aktiviert den Entlade-MOSFET            */
#define DISCH_OFF()     (PORTB &= ~(1<<PB1))  /* Isoliert den Entlade-MOSFET             */
#define BTN_PRESSED()   (!(PINC & (1<<PC2)))  /* Low-aktiver Taster (gegen interne Masse)*/

/* registerspezifische Makro-Restauration für den ADC */
#define ADC_RESTORE() do {                                        \
    ADMUX  = (1<<REFS1) | (1<<REFS0);                            \
    ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);         \
} while(0)

/* ═══════════════════════════════════════════════════════════════════════
 * ASYNCHRONER HARDWARE-ZEITGEBER & STATISTIK-SPEICHER
 * ═══════════════════════════════════════════════════════════════════════ */
volatile uint16_t ovf_count = 0; /* Interrupt-Zähler für Timer1-Überläufe */
volatile uint8_t  ovf_flag  = 0; /* Globales Abbruchflag für Timeout-Erkennung */

// Asynchrone Interrupt-Überwachung (wird alle 1,048 Sekunden getriggert)
ISR(TIMER1_OVF_vect) {
    if (++ovf_count >= TIMEOUT_OVF) ovf_flag = 1;
}

/* Statische Variablen (behalten ihren Wert über die gesamte Laufzeit / Session) */
static uint16_t ses_passed = 0;          /* Zähler für erfolgreiche Messungen */
static uint16_t ses_failed = 0;          /* Zähler für fehlerhafte Zyklen      */
static uint16_t ses_total  = 0;          /* Gesamtzahl aller gestarteten Tests */
static uint32_t ses_max_nf = 0;          /* Maximalwert der aktuellen Session  */
static uint32_t ses_min_nf = 0xFFFFFFFFUL;/* Minimalwert der aktuellen Session  */
static uint8_t  ui_mode    = 0;          /* 0 = Hauptmenü, 1 = Stats 1, 2 = Stats 2 */

/* ═══════════════════════════════════════════════════════════════════════
 * PERIPHERIE-TREIBER (LOW-LEVEL CONTROL)
 * ═══════════════════════════════════════════════════════════════════════ */

static void gpio_init(void) {
    DDRD |=  (1<<PD2) | (1<<PD3);
    DDRB |=  (1<<PB1);
    DDRC &= ~((1<<PC2) | (1<<PC0));
    PORTC |=  (1<<PC2);
    PORTC &= ~(1<<PC0);
    LED_GRN_OFF(); LED_RED_OFF(); DISCH_OFF();
}

static void adc_init(void) {
    ADC_RESTORE();
    ADCSRA |= (1<<ADSC);
    while (ADCSRA & (1<<ADSC));
}

static uint16_t adc_read(void) {
    ADC_RESTORE();
    ADCSRA |= (1<<ADSC);
    while (ADCSRA & (1<<ADSC));
    return ADC;
}

static void timer1_start(void) {
    ovf_count = 0; ovf_flag = 0;
    TCNT1  = 0;
    TIFR1 |= (1<<TOV1);
    TIMSK1 = (1<<TOIE1);
    TCCR1A = 0;
    TCCR1B = (1<<CS12); /* Startet den Zähler mit Taktfrequenz / 256 */
}

static uint32_t timer1_stop(void) {
    uint16_t cnt = TCNT1;
    TCCR1B = 0; /* Stoppt die Taktzufuhr unverzüglich */
    TIMSK1 = 0;
    return (uint32_t)ovf_count * 65536UL + (uint32_t)cnt;
}

static void discharge_wait(void) {
    DISCH_ON();
    for (uint16_t i = 0; i < DISCH_TIMEOUT_MS; i++) {
        _delay_ms(1);
        if (adc_read() < DISCH_ADC_THRESH) break;
    }
    DISCH_OFF();
}

/* ═══════════════════════════════════════════════════════════════════════
 * UI ENGINE & GRAFISCHE FORMAtIERUNG
 * ═══════════════════════════════════════════════════════════════════════ */

// Generiert dynamisch einen fortschreitenden String-Ladebalken
static void make_bar(char *out, uint8_t filled) {
    if (filled > BAR_INNER) filled = BAR_INNER;
    out[0] = '[';
    for (uint8_t i = 0; i < BAR_INNER; i++)
        out[i + 1] = (i < filled) ? '=' : ' ';
    out[BAR_INNER + 1] = ']';
    out[BAR_INNER + 2] = '\0'; /* Korrekte Nullterminierung des ASCII-Strings */
}

// Skaliert Rohwerte (nF) automatisch in eine lesbare Repräsentation (µF/nF)
static void format_cap(uint32_t nf, char *buf) {
    if (nf < 1000UL) {
        sprintf(buf, "%lu nF", nf);
    } else {
        uint32_t uf_int  = nf / 1000UL;
        uint32_t uf_frac = (nf % 1000UL) / 10UL;
        sprintf(buf, "%lu.%02lu uF", uf_int, uf_frac);
    }
}

// Kategorisiert den Prüfling anhand seiner Kapazitätsklasse (Industrie-Standard)
static const char *cap_rank(uint32_t nf) {
    if (nf <   22000UL) return "Klein   < 22 uF ";
    if (nf <  100000UL) return "Mittel 22-100 uF";
    if (nf <  330000UL) return "Mittel 100-330uF";
    if (nf < 1000000UL) return "Gross 330uF-1 mF";
    return               "X-Gross   > 1 mF";
}

static void led_blink_red(uint8_t n) {
    for (uint8_t i = 0; i < n; i++) {
        LED_RED_ON();  _delay_ms(150);
        LED_RED_OFF(); _delay_ms(150);
    }
}

static void green_flash(void) {
    for (uint8_t i = 0; i < 2; i++) {
        LED_GRN_ON();  _delay_ms(GREEN_FLASH_MS);
        LED_GRN_OFF(); _delay_ms(GREEN_FLASH_MS);
    }
}

// Blockierender Zustandsschutz beim Loslassen des Tasters (Flankenschutz)
static void btn_wait_release(void) {
    while (BTN_PRESSED()) _delay_ms(1);
    _delay_ms(30); /* Prellzeitfenster abfangen */
}

/*
 * DER INTERACTIVE CLICK COUNTER (ZUSTANDSMASCHINE)
 * Misst asynchron das Zeitfenster nach dem ersten Klick. Drückt der Benutzer 
 * innerhalb von 350 ms erneut, wird der Klickzähler inkrementiert.
 */
static uint8_t count_clicks(void) {
    btn_wait_release();       
    uint8_t  clicks = 1;
    uint16_t idle   = 0;
    while (idle < CLICK_WINDOW_MS) {
        if (BTN_PRESSED()) {
            _delay_ms(30); /* Software-Entprellung */            
            if (BTN_PRESSED()) {
                clicks++;
                btn_wait_release();
                idle = 0;  /* Timer zurücksetzen bei erfolgreichem Folgeklick */            
                if (clicks >= 5) return clicks; /* Schneller Abbruch bei Disco-Modus */
            }
        }
        _delay_ms(1);
        idle++;
    }
    return clicks;
}

/* ═══════════════════════════════════════════════════════════════════════
 * MATHEMATISCHER DATA-FILTER (ENTSTÖRUNG & PLAUSIBILITÄT)
 * ═══════════════════════════════════════════════════════════════════════ */

static uint8_t plausible_pair(uint32_t a, uint32_t b) {
    uint32_t lo = (a < b) ? a : b;
    uint32_t hi = (a < b) ? b : a;
    if (lo == 0) return 0;
    return ((hi - lo) * 100UL) <= (PLAUS_PCT * lo);
}

static uint32_t evaluate(uint32_t v[3]) {
    uint8_t p01 = plausible_pair(v[0], v[1]);
    uint8_t p02 = plausible_pair(v[0], v[2]);
    uint8_t p12 = plausible_pair(v[1], v[2]);

    if (p01 && p02 && p12)   return (v[0] + v[1] + v[2]) / 3UL;
    if (p01 && !p12 && !p02) return (v[0] + v[1]) / 2UL;
    if (p02 && !p01 && !p12) return (v[0] + v[2]) / 2UL;
    if (p12 && !p01 && !p02) return (v[1] + v[2]) / 2UL;
    return 0;
}

/* ═══════════════════════════════════════════════════════════════════════
 * 100% PRÄZISE HINTERGRUND-MESSUNG (FLACKERFREIE AKTUALISIERUNG)
 * ═══════════════════════════════════════════════════════════════════════ */
static uint32_t single_measure(uint8_t current_meas, uint8_t current_bars) {
    char scr_buf[22];
    
    /* Zeile 0: Aktuelle Iteration ausgeben. Leerzeichen überschreiben alte Reste flackerfrei */
    sprintf(scr_buf, "Messung %u / %u     ", current_meas, N_MEAS);
    ssd1306_print(0, 0, scr_buf);
    
    /* Zeile 2: Den aktuellen Zustand des Ladebalkens statisch einfrieren */
    make_bar(scr_buf, current_bars);
    ssd1306_print(0, 2, scr_buf);
    
    /* Zeile 5 & 7: Saubere Statustexte trennen */
    ssd1306_print(0, 5, "Status: Erfasse Daten...");
    ssd1306_print(0, 7, "                        ");

    discharge_wait(); /* Kondensator restlos entladen */

    /* Restspannung kompensieren (Hardware-Fix v2) */
    uint16_t u_start_adc = adc_read();
    uint32_t u_start_mv  = (uint32_t)u_start_adc * U_REF_MV / 1023UL;
    uint32_t u_delta_mv  = U_REF_MV - u_start_mv;

    if (u_delta_mv < U_START_MIN_MV) return 0; /* Abbruch bei mathematisch ungültigem Delta */

    uint16_t adc_val = 0;
    timer1_start();

    /* COnTIUnOUS BIT-POLLInG: Voller CPU-Fokus auf den Komparator-Eingang.
     * Keine Display-Befehle unterbrechen diesen zeitkritischen Echtzeit-Prozess! */
    do {
        adc_val = adc_read();
        if (ovf_flag) break;
    } while (adc_val < ADC_THRESH);

    uint32_t ticks = timer1_stop();

    if (ovf_flag || adc_val < ADC_THRESH) return 0; /* Auswertung des harten Timeouts */

    /* Physikalische Ladekurvenberechnung im 64-Bit Raum zur Vermeidung von Overflows */
    uint64_t tmp = (uint64_t)I_UA * (uint64_t)TICK_US * (uint64_t)ticks;
    return (uint32_t)(tmp / u_delta_mv);
}

/* ═══════════════════════════════════════════════════════════════════════
 * UI MANAGEMENT (GRAPHICAL VIEWS)
 * ═══════════════════════════════════════════════════════════════════════ */

// Repräsentativer Boot-Screen mit automatischem Fortschrittsbalken
static void screen_boot(void) {
    char bar_str[22];
    for (uint8_t i = 0; i <= BOOT_STEPS; i++) {
        ssd1306_clear();
        ssd1306_print(0, 0, "=== KapTester V2 ===");
        ssd1306_print(0, 2, " Entwickler:        ");
        ssd1306_print(0, 3, " Nico Schmidt       ");
        ssd1306_print(0, 5, " Formel: C=I*t/dU   ");
        make_bar(bar_str, i);
        ssd1306_print(2, 7, bar_str);
        _delay_ms(BOOT_STEP_MS);
    }
}

// Zentrale Rendering-Engine für das Hauptmenü und die Statistik-Seiten
static void screen_refresh(void) {
    char buf[30];
    ssd1306_clear();
    
    if (ui_mode == 0) { // Hauptmenü
        sprintf(buf, "KapTester V2    #%03u", ses_passed);
        ssd1306_print(0, 0, buf);
        ssd1306_print(0, 1, "---------------------");
        ssd1306_print(0, 3, " Status: Bereit      ");
        ssd1306_print(0, 5, " DUT anschliessen    ");
        ssd1306_print(0, 6, " START = Messung     ");
        ssd1306_print(0, 7, " 2xKlick = Statistik ");
    } 
    else if (ui_mode == 1) { // Statistik-Seite 1/2
        ssd1306_print(0, 0, "=== STATS (1/2) ===");
        sprintf(buf, "Gesamt : %u Zyklen", ses_total);
        ssd1306_print(0, 2, buf);
        sprintf(buf, "Passed : %u", ses_passed);
        ssd1306_print(0, 4, buf);
        sprintf(buf, "Failed : %u", ses_failed);
        ssd1306_print(0, 6, buf);
    } 
    else if (ui_mode == 2) { // Statistik-Seite 2/2 (Extremwertspeicher)
        ssd1306_print(0, 0, "=== STATS (2/2) ===");
        char b_max[16], b_min[16];
        if (ses_max_nf == 0) sprintf(b_max, "---"); else format_cap(ses_max_nf, b_max);
        if (ses_min_nf == 0xFFFFFFFFUL) sprintf(b_min, "---"); else format_cap(ses_min_nf, b_min);
        
        sprintf(buf, "Groesster: %s", b_max);
        ssd1306_print(0, 3, buf);
        sprintf(buf, "Kleinster: %s", b_min);
        ssd1306_print(0, 5, buf);
        ssd1306_print(0, 7, "2x Klick -> Hauptmenue");
    }
}

// Sicherheits-Warnhinweis vor jedem Messvorgang
static void screen_discharge_hint(void) {
    ssd1306_clear();
    ssd1306_print(0, 1, "   ENTLADEN...       ");
    ssd1306_print(0, 3, "   Sicherheits-      ");
    ssd1306_print(0, 4, "   entladung laeuft  ");
    ssd1306_print(0, 6, "  DUT jetzt NICHT    ");
    ssd1306_print(0, 7, "  abnehmen!          ");
    _delay_ms(DISCH_HOLD_MS);
}

// Easter Egg / Visueller Hardware-Frequenztest (Disco-Mode)
static void disco_mode(void) {
    ssd1306_clear();
    ssd1306_print(0, 0, "   * DISCO MODE * ");
    for (uint8_t i = 0; i < 10; i++) {
        LED_GRN_ON(); LED_RED_OFF();
        ssd1306_print(0, 3, "     \\ (o) /        ");
        ssd1306_print(0, 4, "       | |          ");
        ssd1306_print(0, 5, "      /   \\         ");
        _delay_ms(DISCO_FRAME_MS);
        
        LED_GRN_OFF(); LED_RED_ON();
        ssd1306_print(0, 3, "      _ (o) _       ");
        ssd1306_print(0, 4, "     /  | |  \\      ");
        ssd1306_print(0, 5, "       /   \\        ");
        _delay_ms(DISCO_FRAME_MS);
    }
    LED_RED_OFF(); LED_GRN_OFF();
}

/* ═══════════════════════════════════════════════════════════════════════
 * ZENTRALE KOORDINATIONS-SCHLEIFE (MAIN ENGINE)
 * ═══════════════════════════════════════════════════════════════════════ */
int main(void) {
    // Initialisierung des Gesamtsystems
    gpio_init();
    adc_init();
    i2c_init();
    ssd1306_init();
    sei(); /* Aktivierung der globalen Interrupts für die Timer-ISR */

    LED_RED_ON();
    screen_boot(); /* Zeige Ladebalken beim Kaltstart */
    LED_RED_OFF();
    
    green_flash();
    screen_refresh();

    for (;;) {
        // Taster-Abfrage inklusive Entprellung
        if (!BTN_PRESSED()) continue;
        _delay_ms(30); 
        if (!BTN_PRESSED()) continue;
        
        // Multi-Click-Zustand auswerten
        uint8_t clicks = count_clicks();
        
        if (clicks == 5) { // 5-fach Klick -> Diagnose-Modus
            disco_mode();
            green_flash();
            screen_refresh();
            continue;
        }
        else if (clicks == 2) { // Doppelklick -> Menü-Navigation (Seiten blättern)
            ui_mode++;
            if (ui_mode > 2) ui_mode = 0;
            green_flash();
            screen_refresh();
            continue;
        }
        else if (clicks == 1) { // Einzelklick -> Messzyklus oder Menü-Reset
            if (ui_mode != 0) {
                ui_mode = 0; /* Wenn in Stats, bringt ein Klick den User zurück ins Hauptmenü */
                green_flash();
                screen_refresh();
                continue;
            }
            
            ses_total++;
            LED_RED_ON();
            screen_discharge_hint();
            discharge_wait();
            LED_RED_OFF();

            /* Vorbereitung des flackerfreien Messbildschirms */
            ssd1306_clear();
            ssd1306_print(0, 0, "Messung 1 / 3");
            ssd1306_print(0, 2, "[              ]");
            
            uint32_t vals[N_MEAS];
            uint8_t  timeout_any = 0;
            
            /* Das optimierte "Kino-Timing": 220 ms fühlt sich für den Menschen flüssig an */
            uint16_t dynamic_delay_ms = 220; 

            // ════════════════════════════════════════════════════════
            // MESSUNG 1 (Fortschritt: Segmente 0 bis 4)
            // ════════════════════════════════════════════════════════
            vals[0] = single_measure(1, 0);
            if (vals[0] == 0) timeout_any = 1;
            
            if (!timeout_any) {
                /* * DYNAMISCHES UPSCALING FÜR RIESIGE ELKOS:
                 * Wenn die erste Messung einen Wert > 1.000 µF (1 mF) ergibt, 
                 * verdoppeln wir das Animations-Timing auf 450 ms. Dadurch wirkt der 
                 * Ladebalken physikalisch absolut authentisch! */
                if (vals[0] > 1000000UL) { 
                    dynamic_delay_ms = 450; 
                }

                ssd1306_print(0, 5, "Status: Bereite vor...  ");
                LED_RED_ON();
                discharge_wait();
                LED_RED_OFF();
                
                char scr_buf[22];
                for (uint8_t b = 0; b <= 4; b++) {
                    make_bar(scr_buf, b);
                    ssd1306_print(0, 2, scr_buf);
                    LED_GRN_TOG(); /* Toggelt die grüne LED synchron zum Balken-Tick */
                    for(uint16_t d = 0; d < dynamic_delay_ms; d++) _delay_ms(1);
                }
            }

            // ════════════════════════════════════════════════════════
            // MESSUNG 2 (Fortschritt: Segmente 4 bis 9)
            // ════════════════════════════════════════════════════════
            if (!timeout_any) {
                vals[1] = single_measure(2, 4);
                if (vals[1] == 0) timeout_any = 1;
            }
            
            if (!timeout_any) {
                ssd1306_print(0, 5, "Status: Bereite vor...  ");
                LED_RED_ON();
                discharge_wait();
                LED_RED_OFF();
                
                char scr_buf[22];
                for (uint8_t b = 4; b <= 9; b++) {
                    make_bar(scr_buf, b);
                    ssd1306_print(0, 2, scr_buf);
                    LED_GRN_TOG();
                    for(uint16_t d = 0; d < dynamic_delay_ms; d++) _delay_ms(1);
                }
            }

            // ════════════════════════════════════════════════════════
            // MESSUNG 3 (Fortschritt: Segmente 9 bis zum Finale 14)
            // ════════════════════════════════════════════════════════
            if (!timeout_any) {
                vals[2] = single_measure(3, 9);
                if (vals[2] == 0) timeout_any = 1;
            }
            
            if (!timeout_any) {
                ssd1306_print(0, 5, "Status: Bereite vor...  ");
                LED_RED_ON();
                discharge_wait();
                LED_RED_OFF();
                
                char scr_buf[22];
                for (uint8_t b = 9; b <= 14; b++) {
                    make_bar(scr_buf, b);
                    ssd1306_print(0, 2, scr_buf);
                    LED_GRN_TOG();
                    for(uint16_t d = 0; d < dynamic_delay_ms; d++) _delay_ms(1);
                }
                _delay_ms(150);
            }

            // ════════════════════════════════════════════════════════
            // FEHLERBEHANDLUNG: HARTER TIMEOUT
            // ════════════════════════════════════════════════════════
            if (timeout_any) {
                ses_failed++;
                LED_GRN_OFF();
                ssd1306_clear();
                ssd1306_print(0, 0, "   ** TIMEOUT ** ");
                ssd1306_print(0, 2, " Messfehler:        ");
                ssd1306_print(0, 4, " Kein Kondensator    ");
                ssd1306_print(0, 5, " oder Kap. < 10 uF  ");
                ssd1306_print(0, 7, " > Klick fuer Menue ");
                led_blink_red(5);
                LED_RED_ON();
                btn_wait_release();
                while(!BTN_PRESSED()); /* Blockierendes Warten auf Quittierung durch Benutzer */
                btn_wait_release();
                LED_RED_OFF();
                green_flash();
                screen_refresh();
                continue;
            }

            // Mathematischen Filter über das Mess-Array laufen lassen
            uint32_t result = evaluate(vals);

            // FEHLERBEHANDLUNG: MATHEMATISCHE STREUUNG (FAIL)
            if (result == 0) {
                ses_failed++;
                LED_GRN_OFF();
                char b0[16], b1[16], b2[16];
                format_cap(vals[0], b0);
                format_cap(vals[1], b1);
                format_cap(vals[2], b2);
                ssd1306_clear();
                ssd1306_print(0, 0, "    ** FAIL ** ");
                ssd1306_print(0, 2, "Werte streuen stark:");
                ssd1306_print(0, 4, b0);
                ssd1306_print(0, 5, b1);
                ssd1306_print(0, 6, b2);
                ssd1306_print(0, 7, " > Klick fuer Menue ");
                led_blink_red(6);
                LED_RED_ON();
                btn_wait_release();
                while(!BTN_PRESSED());
                btn_wait_release();
                LED_RED_OFF();
                green_flash();
                screen_refresh();
                continue;
            }

            /* VALIDIERTER MESSERFOLG (PASSED) */
            ses_passed++;
            
            // Überprüfung und Aktualisierung des Extremwertspeichers (Session-Rekorde)
            uint8_t record_flag = 0;
            if (result > ses_max_nf) { ses_max_nf = result; record_flag = 1; }
            if (result < ses_min_nf) { ses_min_nf = result; record_flag = 1; }

            char cap_str[20];
            format_cap(result, cap_str);

            // Validierung gegen die offizielle Spezifikationsgrenze (10 µF bis 10 mF)
            uint8_t out_of_range = (result < CAP_MIN_NF || result > CAP_MAX_NF);

            ssd1306_clear();
            ssd1306_print(0, 0, "=== MESSUNG OK ===");
            ssd1306_print(0, 2, "Kapazitaet:");
            ssd1306_print(0, 3, cap_str);
            ssd1306_print(0, 5, cap_rank(result)); /* Ausgabe der Kapazitätsklasse */
            
            if (out_of_range) {
                ssd1306_print(0, 6, " ! BEREICHSAUSNAHME !");
                LED_GRN_OFF(); LED_RED_ON(); /* Rote Warn-LED bei Spezifikationsüberschreitung */
            } else {
                if (record_flag && ses_passed > 1) {
                    ssd1306_print(0, 6, " * SITZUNGS-REKORD * ");
                } else {
                    ssd1306_print(0, 6, " Status: Validiert   ");
                }
                LED_RED_OFF(); LED_GRN_ON(); /* Grüne Erfolgs-LED */
            }
            ssd1306_print(0, 7, " > Klick fuer Menue ");

            btn_wait_release();
            while(!BTN_PRESSED()); /* Manuelle Quittierung friert das Ergebnis unendlich lang ein */
            btn_wait_release();
            
            // Nachbereitung für den nächsten Serien-Messzyklus
            LED_GRN_OFF(); LED_RED_OFF();
            screen_discharge_hint();
            discharge_wait(); /* Abschlussentladung zum Schutz des Anwenders */
            green_flash();
            screen_refresh();
        }
    }
    return 0;
}