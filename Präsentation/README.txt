Infos für Vorstellung

Vorteile gegenüber DMM

Messbereich (viele DMMs messen nur bis 200 µF oder max. 2.000 µF – für 10.000 µF-Elkos reicht das nicht), 

Messgeschwindigkeit (ein DMM braucht bei großen Elkos 30+ Sekunden und gibt oft keinen stabilen Wert – dein Gerät hat einen deterministischen Ablauf mit max. 15 s Timeout),

Sicherheit (automatischer Entladepfad, Verpolschutz, dediziertes Gerät für diese Aufgabe). Der KapTester ist ein schnelles Werkstatt-Prüfmittel für grobe Pass/Fail-Entscheidungen – kein Präzisionsgerät.

Eingangsspannung Facts

Micro-USB ist weit verbreitet, jeder hat ein passendes Kabel

Durch die Bauform ist eine Verpolung beim Einstecken nicht möglich

D+/D−-Datenleitungen sind bei mir no-connect – ich nutze den USB-Anschluss ausschließlich als Spannungsversorgung.

5V USB ist ein standardisierter Pegel, der alle Komponenten meiner Schaltung versorgen kann. 
(Der ATmega328P arbeitet von 1,8V bis 5,5V, der LM317L braucht mindestens ~2,5V Eingangsspannung um seinen Dropout zu überwinden, und das OLED läuft auf 3,3–5V. USB liefert bis zu 500 mA – meine Schaltung braucht maximal ~50 mA, also mehr als ausreichend)

Nach der 5V Eingangsspannung ->  Q1 – der PMOS Verpolschutz

