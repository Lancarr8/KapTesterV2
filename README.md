# ⚡ KapTester V2

> Capacitance Tester — ATmega328P based custom PCB project  
> Built as **AP2 (Abschlussprüfung Teil 2)** — Vocational exam project 🇩🇪

![Platform](https://img.shields.io/badge/Platform-ATmega328P-blue)
![Language](https://img.shields.io/badge/Language-C%20%28AVR%29-green)
![PCB](https://img.shields.io/badge/PCB-KiCad%208-orange)
![Status](https://img.shields.io/badge/Status-Complete-brightgreen)
![License](https://img.shields.io/badge/License-MIT-lightgrey)

---

## 📷 Photos
*Coming soon – V2 build photos*

---

## 📖 About

The **KapTester V2** is a custom-built capacitance measurement device based on the ATmega328P microcontroller. It was designed and built from scratch as part of a vocational training exam (IHK AP2), covering all phases from schematic design to PCB manufacturing and firmware development.

---

## ⚙️ Hardware

| Component | Details |
|-----------|---------|
| MCU | ATmega328P @ 16MHz |
| Display | SSD1306 OLED 128x64 (I2C) |
| Voltage Reg | LM317 (1.2V – 32V adjustable) |
| PCB | Custom 2-layer, manufactured by PCBWay |
| Power | USB (5V) |
| Interface | Tactile button, screw terminals |

---

## 💾 Firmware

Three firmware versions were developed during the project:

| Version | Folder | Description |
|---------|--------|-------------|
| FW1 | `fw1_produktion` | Production firmware – stable release |
| FW2 | `fw2_test` | Test firmware – debugging & measurement testing |
| FW3 | `fw3_Voll` | Full feature firmware – all functions enabled |

### 🔧 Flashing

```bash
# Flash production firmware via avrdude
avrdude -c usbasp -p m328p -U flash:w:fw1_produktion/kaptester.hex:i
```

---

## 📁 Repository Structure

```
KapTesterV2/
├── Firmware/
│   ├── fw1_produktion/     # Production firmware (main.c, .hex, .elf)
│   ├── fw2_test/           # Test firmware
│   └── fw3_Voll/           # Full firmware
├── KiCad KapTester V2/
│   ├── KapTesterV2.kicad_sch   # Schematic
│   ├── KapTesterV2.kicad_pcb   # PCB layout
│   └── GerberV2/               # Gerber files for manufacturing
├── Dokumente/              # Project documentation (German)
│   └── Phasen/             # Phase documentation (Info/Plan/Build/Evaluate)
└── Präsentation/           # Final exam presentation (PPTX + PDF)
```

---

## 🛠️ Tools Used

- **AVR-GCC** – Firmware compilation
- **avrdude** – Flashing
- **KiCad 8** – Schematic & PCB design
- **PCBWay** – PCB manufacturing

---

## 📄 License

MIT License – feel free to use, modify and build upon this project.
