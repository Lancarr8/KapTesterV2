/*
 * ╔══════════════════════════════════════════════════════════════════════╗
 * ║   KapTester V2  —  Firmware 1: Produktion / "Light" (FINAL V1.5)     ║
 * ║   Hardware-Plattform: ATmega328P-AU @ 16.000 MHz (externer Quarz)    ║
 * ║   Anwendungsbereich: Schnelle Einzelmessung mit Hardware-Kompensation║
 * ║   Autor: Nico Schmidt  |  AP2-Abschlussprojekt                       ║
 * ╚══════════════════════════════════════════════════════════════════════╝
 *
 * ── PROFESSIOnELLE PROJEKT-HISTORIE & FEHLERKORREKTUR (WICHTIG FÜR DOKU) ──
 * * PROBLEM IN FIRMWARE-VERSION 1:
 * Der Entladewiderstand R20 (470 Ohm) bildet zusammen mit den Schutzdioden 
 * und dem Innenwiderstand des Entlade-Pfads einen Spannungsteiler. Der 
 * Kondensator (DUT) entlud sich daher nicht auf 0V, sondern lief bei ca. 0,5V 
 * in ein physikalisches Gleichgewicht. Die alte Entladeschleife wartete auf 
 * einen ADC-Wert < 5, was mathematisch unerreichbar war. Folge: Ein harter 
 * 5000 ms Timeout blockierte das Gerät, und nach dem Abschalten des Entlade-
 * MOSFETs lud die Konstantstromquelle den Cap sofort wieder auf.
 *
 * SYSTEMATISCHE KORREKTUR IN DIESER VERSION (FW1):
 * 1. Anhebung der Abbruchschwelle in discharge_wait() auf ADC-Wert 470 (~0,5V).
 * Die Schleife bricht nun exakt dann ab, wenn das Gleichgewicht erreicht ist.
 * 2. Eliminierung künstlicher Delays nach der Entladung, um ein unkontrolliertes
 * Wiederaufladen vor dem Messstart zu verhindern.
 * 3. Mathematische Kompensation über die modifizierte physikalische Formel:
 * C = (I * t) / (U_ref - U_start). Die verbleibende Startspannung (U_start)
 * wird direkt nach dem Entladen gemessen und vom Ladeziel (U_ref) abgezogen.
 * Damit arbeitet das Messgerät trotz Hardware-Einschränkung hochpräzise!
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

/* * ENTLADE-SCHWELLE (HARDWARE-FIX v2):
 * Repräsentiert das physikalische Spannungs-Gleichgewicht am Entladepfad (~0,5V).
 * Verhindert den Endlos-Timeout der Entladeschaltung.
 */
#define DISCH_ADC_THRESH    470   

/* ═══════════════════════════════════════════════════════════════════════
 * HARDWARE-ABSTRAKTION (GPIO-MAKROS NACH DATENBLATT)
 * ═══════════════════════════════════════════════════════════════════════ */
#define LED_GRN_ON()    (PORTD |=  (1<<PD2))  /* Grüne LED aktivieren (High-Pegel)  */
#define LED_GRN_OFF()   (PORTD &= ~(1<<PD2))  /* Grüne LED deaktivieren (Low-Pegel) */
#define LED_RED_ON()    (PORTD |=  (1<<PD3))  /* Rote LED aktivieren (High-Pegel)   */
#define LED_RED_OFF()   (PORTD &= ~(1<<PD3))  /* Rote LED deaktivieren (Low-Pegel)  */
#define DISCH_ON()      (PORTB |=  (1<<PB1))  /* Entlade-MOSFET durchschalten       */
#define DISCH_OFF()     (PORTB &= ~(1<<PB1))  /* Entlade-MOSFET sperren             */
#define BTN_PRESSED()   (!(PINC & (1<<PC2)))  /* Taster gegen Masse abfragen (Low-aktiv) */

/*
 * MAKRO: ANALOG-DIGITAL-WANDLER RESTAURIEREN
 * Setzt die ADC-Register auf Standardkonfiguration: Interne 1,1V Referenz,
 * Aktivierung des ADC und Einstellen des Vorteilers auf 128 (16MHz / 128 = 125kHz Wandlertakt).
 * Dies garantiert maximale Wandlungspräzision im thermodynamischen Optimum.
 */
#define ADC_RESTORE() do {                                        \
    ADMUX  = (1<<REFS1) | (1<<REFS0);                            \
    ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);         \
} while(0)

/* ═══════════════════════════════════════════════════════════════════════
 * GLOBALER HARDWARE-TIMERSTATI (VOLATILE FÜR ISR-ZUGRIFF)
 * ═══════════════════════════════════════════════════════════════════════ */
volatile uint16_t ovf_count = 0; /* Zählt die Hardware-Überläufe von Timer1 (16-Bit) */
volatile uint8_t  ovf_flag  = 0; /* Globales Software-Flag für harten Timeout-Abbruch */

/*
 * TIMER1 ÜBERLAUF INTERRUPT SERVICE ROUTINE (ISR)
 * Wird automatisch aufgerufen, wenn Timer1 von 0xFFFF auf 0x0000 springt (alle 1,048 Sekunden).
 * Dient als asynchrone Zeitbasis für die Erkennung defekter oder kurzgeschlossener Kondensatoren.
 */
ISR(TIMER1_OVF_vect) {
    if (++ovf_count >= TIMEOUT_OVF) {
        ovf_flag = 1; /* Timeout-Grenze erreicht, Ladevorgang wird abgebrochen */
    }
}

/* ═══════════════════════════════════════════════════════════════════════
 * LOW-LEVEL HARDWARE-TREIBERFUNKTIONEN
 * ═══════════════════════════════════════════════════════════════════════ */

// GPIO-Richtungsregister initialisieren
void gpio_init(void) {
    DDRD |=  (1<<PD2) | (1<<PD3);  /* PD2 (LED_GRN) und PD3 (LED_RED) als Ausgang definieren */
    DDRB |=  (1<<PB1);             /* PB1 (DISCH_GATE) als digitalen Ausgang definieren       */
    DDRC &= ~((1<<PC2) | (1<<PC0));/* PC2 (Button) und PC0 (ADC-Messkanal) als Eingang       */
    PORTC |=  (1<<PC2);            /* Internen Pull-Up-Widerstand für den Taster aktivieren    */
    PORTC &= ~(1<<PC0);            /* Pull-Up für analogen Messkanal deaktivieren (hochohmig)  */
    LED_GRN_OFF(); LED_RED_OFF(); DISCH_OFF(); /* Sicheren Ausgangszustand erzwingen */
}

// Analog-Digital-Wandler initialisieren
void adc_init(void) {
    ADC_RESTORE();
    ADCSRA |= (1<<ADSC); /* Dummy-Wandlung starten, um die analoge Hardware einzuschwingen */
    while (ADCSRA & (1<<ADSC)); /* Warten, bis die Initialisierungswandlung abgeschlossen ist */
}

// Analogen Messwert synchron einlesen
uint16_t adc_read(void) {
    ADC_RESTORE();
    ADCSRA |= (1<<ADSC); /* Konvertierung auf Kanal ADC0 starten */
    while (ADCSRA & (1<<ADSC)); /* Bit-Polling: Warten, bis Wandlung fertig ist (ADSC wird 0) */
    return ADC; /* 10-Bit Digitalwert (0 bis 1023) zurückgeben */
}

// Hardware-Timer1 für präzise Zeitmessung konfigurieren und starten
void timer1_start(void) {
    ovf_count = 0; 
    ovf_flag = 0;
    TCNT1  = 0;          /* Timer-Zählerregister komplett nullen */
    TIFR1 |= (1<<TOV1);  /* Eventuell anhängige alte Überlauf-Flags löschen */
    TIMSK1 = (1<<TOIE1); /* Timer1 Overflow Interrupt lokal freischalten */
    TCCR1A = 0;          /* Normaler Modus, keine PWM-Ausgänge aktiv */
    TCCR1B = (1<<CS12);  /* Vorteiler (Prescaler) auf 256 setzen -> Startet den Timer quantisiert */
}

// Hardware-Timer1 stoppen und absolute Ticks berechnen
uint32_t timer1_stop(void) {
    uint16_t cnt = TCNT1;/* Zählerstand sofort sichern, um Latenzen zu minimieren */
    TCCR1B = 0;          /* Taktzufuhr stoppen (Timer steht still) */
    TIMSK1 = 0;          /* Interne Interrupts deaktivieren */
    /* Absolute Zeit berechnen: (Überläufe * 65536) + verbleibende Ticks */
    return (uint32_t)ovf_count * 65536UL + (uint32_t)cnt;
}

// Intelligente Sicherheitsentladung des Kondensators (Hardware-Fix v2 integriert)
void discharge_wait(void) {
    DISCH_ON(); /* Entlade-Pfad über MOSFET niederohmig schalten */
    for (uint16_t i = 0; i < DISCH_TIMEOUT_MS; i++) {
        _delay_ms(1);
        /* Abbruch, sobald die Entladeschwelle (Gleichgewicht bei ~0,5V) erreicht ist */
        if (adc_read() < DISCH_ADC_THRESH) break;
    }
    DISCH_OFF(); /* Entlade-Pfad isolieren, um die Messung nicht zu verfälschen */
}

/* ═══════════════════════════════════════════════════════════════════════
 * UI- & FORMATIERUNGS-HILFSFUNKTIONEN
 * ═══════════════════════════════════════════════════════════════════════ */

// Bereitet den statischen Begrüßungs-Bildschirm vor
void screen_ready(void) {
    ssd1306_clear();
    ssd1306_print(0, 0, " KapTester V2");
    ssd1306_print(0, 1, " -------------------");
    ssd1306_print(0, 3, " Status: BEREIT");
    ssd1306_print(0, 5, " Kondensator an-");
    ssd1306_print(0, 6, " schliessen...");
    ssd1306_print(0, 7, " [START] druecken");
}

// Formatiert den nF-Rohwert automatisch in ein lesbares µF- oder nF-Format
void format_cap(uint32_t nf, char *buf) {
    if (nf < 1000UL) {
        sprintf(buf, "%lu nF", nf); /* Werte unter 1000 nF direkt anzeigen */
    } else {
        uint32_t uf_int  = nf / 1000UL;       /* Ganzzahliger Anteil in µF */
        uint32_t uf_frac = (nf % 1000UL) / 10UL; /* Zwei Nachkommastellen extrahieren */
        sprintf(buf, "%lu.%02lu uF", uf_int, uf_frac);
    }
}

// Taktet die rote Fehler-LED für optische Warnungen
void led_blink_red(uint8_t n) {
    for (uint8_t i = 0; i < n; i++) {
        LED_RED_ON();  _delay_ms(150);
        LED_RED_OFF(); _delay_ms(150);
    }
}

/* ═══════════════════════════════════════════════════════════════════════
 * MATHEMATISCHER DATA-FILTER (ENTSTÖRUNG & PLAUSIBILITÄT)
 * ═══════════════════════════════════════════════════════════════════════ */

// Prüft, ob zwei Messwerte innerhalb des zulässigen Toleranzfensters (±15%) liegen
uint8_t plausible_pair(uint32_t a, uint32_t b) {
    uint32_t lo = (a < b) ? a : b;
    uint32_t hi = (a < b) ? b : a;
    if (lo == 0) return 0;
    /* Prozentuale Abweichung mathematisch ohne Fließkomma-Latenz prüfen */
    return ((hi - lo) * 100UL) <= (PLAUS_PCT * lo);
}

// 2-aus-3-Filter: Eliminiert statistische Ausreißer (z.B. durch Kontaktprellen)
uint32_t evaluate(uint32_t v[3]) {
    uint8_t p01 = plausible_pair(v[0], v[1]);
    uint8_t p02 = plausible_pair(v[0], v[2]);
    uint8_t p12 = plausible_pair(v[1], v[2]);

    /* Fall 1: Alle drei Messungen sind konsistent -> Arithmetischer Mittelwert */
    if (p01 && p02 && p12)   return (v[0] + v[1] + v[2]) / 3UL;
    /* Fall 2-4: Ein Wert streut massiv -> Ausreißer verwerfen, restliche 2 mitteln */
    if (p01 && !p12 && !p02) return (v[0] + v[1]) / 2UL;
    if (p02 && !p01 && !p12) return (v[0] + v[2]) / 2UL;
    if (p12 && !p01 && !p02) return (v[1] + v[2]) / 2UL;
    /* Fall 5: Totale Streuung -> Kein Wert passt zum anderen (Messung ungültig) */
    return 0; 
}

/* ═══════════════════════════════════════════════════════════════════════
 * ZENTRALE STEUERUNGSSCHLEIFE (MAIN EXECUTION LOOP)
 * ═══════════════════════════════════════════════════════════════════════ */
int main(void) {
    // Hardware-Subsysteme hochfahren
    gpio_init();
    adc_init();
    i2c_init();
    ssd1306_init();
    sei(); /* Globale Interrupt-Freigabe für die Timer-ISR */

    LED_GRN_ON();
    screen_ready();

    for (;;) {
        // Taster abfragen mit Software-Entprellung
        if (!BTN_PRESSED()) continue;
        _delay_ms(30); /* 30 ms Karenzzeit gegen mechanisches Tasterprellen */
        if (!BTN_PRESSED()) continue;

        // Visualisierung des Messstarts
        LED_GRN_OFF();
        LED_RED_ON();
        ssd1306_clear();
        ssd1306_print(0, 2, " Messung laeuft...");
        ssd1306_print(0, 4, " Bitte warten...");

        uint32_t vals[N_MEAS]; /* Array für die 3 mathematischen Einzelmessungen */
        uint8_t  timeout_any = 0;

        // Haupt-Messzyklus (3 Durchläufe für höchste Störsicherheit)
        for (uint8_t m = 0; m < N_MEAS; m++) {
            discharge_wait(); /* Kondensator kontrolliert entladen */

            /* * HARDWARE-COMPENSATION (Fix v2):
             	* Da der Kondensator bei ~0,5V verweilt, lesen wir diese verbleibende 
             	* Startspannung exakt JETZT ein, bevor die Stromquelle aktiv wird.
             	*/
            uint16_t u_start_adc = adc_read();
            /* Skalierung auf Millivolt: (ADC-Wert * 1100 mV) / 1023 Ticks */
            uint32_t u_start_mv  = (uint32_t)u_start_adc * U_REF_MV / 1023UL;
            
            /* Spannungs-Delta berechnen: Ladeziel (1,1V) minus realer Startpunkt */
            uint32_t u_delta_mv  = U_REF_MV - u_start_mv;

            // Plausibilitätsprüfung: Ist das Delta zu klein, bricht das System ab
            if (u_delta_mv < 50UL) {
                timeout_any = 1;
                break;
            }

            uint16_t adc_val = 0;
            timer1_start(); /* Präzise Zeitmessung starten */

            /* * ABSOLUTES POLLING IM TEMPORAUSCH:
             	* Die CPU fokussiert sich zu 100% auf das Einlesen des ADC-Werts. 
             	* Keine Display-Ausgaben oder Delays stören diesen zeitkritischen Prozess!
             	*/
            do {
                adc_val = adc_read();
                if (ovf_flag) break; /* Notabschaltung bei Timeout-Flag aus der ISR */
            } while (adc_val < ADC_THRESH);

            uint32_t ticks = timer1_stop(); /* Zeitmessung stoppen und Ticks sichern */

            // Validierung, ob der Zyklus regulär beendet wurde
            if (ovf_flag || adc_val < ADC_THRESH) {
                timeout_any = 1;
                break;
            }

            /*
             	* PHYSIKALISCHE FORMELBERECHNUNG (C = I * t / dU):
             	* tmp = Strom (µA) * Zeitauflösung (µs/Tick) * gemessene Ticks
             	* Das Ergebnis liefert direkt die exakte Kapazität in Nano-Farad (nF).
             	*/
            uint64_t tmp = (uint64_t)I_UA * (uint64_t)TICK_US * (uint64_t)ticks;
            vals[m] = (uint32_t)(tmp / u_delta_mv);
            
            _delay_ms(10); /* Kurze thermische Beruhigungspause für die Hardware */
        }

        // FEHLERBEHANDLUNG: Timeout-Fall (Kein Bauteil oder Kurzschluss)
        if (timeout_any) {
            ssd1306_clear();
            ssd1306_print(0, 1, "   ** TIMEOUT **");
            ssd1306_print(0, 3, " Messfehler oder");
            ssd1306_print(0, 4, " kein Kondensator");
            ssd1306_print(0, 5, " detektiert!");
            led_blink_red(5);
            discharge_wait();
            LED_GRN_ON();
            screen_ready();
            continue;
        }

        // Mathematische Ausreißer-Filterung anwenden
        uint32_t result = evaluate(vals);

        // FEHLERBEHANDLUNG: Instabile Werte (Rauschen oder Kontaktproblem)
        if (result == 0) {
            char b0[16], b1[16], b2[16];
            format_cap(vals[0], b0);
            format_cap(vals[1], b1);
            format_cap(vals[2], b2);

            ssd1306_clear();
            ssd1306_print(0, 0, "   ** FAIL **");
            ssd1306_print(0, 2, "Werte streuen:");
            ssd1306_print(0, 3, b0);
            ssd1306_print(0, 4, b1);
            ssd1306_print(0, 5, b2);
            ssd1306_print(0, 7, "Kontakt pruefen");
            led_blink_red(6);
            LED_RED_ON();
            _delay_ms(3000);
            LED_RED_OFF();
            discharge_wait();
            LED_GRN_ON();
            screen_ready();
            continue;
        }

        // ERFOLGREICHE MESSUNG: Grenzwert-Überwachung & Ausgabe
        char cap_str[20];
        format_cap(result, cap_str);

        uint8_t out_of_range = 0;
        const char *range_warn = "";

        // Plausibilitäts-Check gegen das offizielle Datenblatt
        if (result < CAP_MIN_NF) {
            out_of_range = 1;
            range_warn   = "Warnung: < 10uF!";
        } else if (result > CAP_MAX_NF) {
            out_of_range = 1;
            range_warn   = "Warnung: > 10mF!";
        }

        // Endergebnis auf dem OLED-Display ausgeben
        ssd1306_clear();
        ssd1306_print(0, 0, " KapTester V2");
        ssd1306_print(0, 2, "Ergebnis:");
        ssd1306_print(0, 3, cap_str);

        if (out_of_range) {
            ssd1306_print(0, 5, range_warn);
            ssd1306_print(0, 6, "Ausserhalb Spec!");
            LED_RED_ON(); /* Rote LED warnt vor Spezifikationsverletzung */
        } else {
            ssd1306_print(0, 5, "Messung gueltig");
            ssd1306_print(0, 6, "Toleranz geprueft");
            LED_GRN_ON(); /* Grüne LED signalisiert Erfolg */
        }
        
        // System hält das Ergebnis für 5 Sekunden statisch im Display
        _delay_ms(5000);
        
        // Nachbereitung für den nächsten Zyklus
        LED_GRN_OFF();
        LED_RED_OFF();
        discharge_wait(); /* Sicherheitsentladung nach der Messung */
        LED_GRN_ON();
        screen_ready();   /* Zurück in den Bereit-Modus */
    }
    return 0;
}