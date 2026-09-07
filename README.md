# RTL-SDR V4

Dieses Repository enthält Experimente und Software zur Nutzung des **RTL-SDR Blog V4** für den Empfang und die Analyse von Funksignalen.

## Hardware

Der RTL-SDR V4 ist ein kostengünstiger **Software Defined Radio (SDR)**-Empfänger.

| Eigenschaft | Wert |
|---|---|
| Tuner | Rafael Micro R828D |
| Demodulator | RTL2832U |
| Empfangsbereich | ca. 500 kHz – 1,766 GHz* |
| Sample Rate | bis ca. 2,4 MS/s |
| Auflösung | 8 Bit |
| Schnittstelle | USB |
| Senden | Nein |

\* Abhängig von Betriebsart und Anwendung.

## Möglichkeiten

Mit dem RTL-SDR können unter anderem folgende Anwendungen umgesetzt werden:

- 📻 UKW-Radio
- ✈️ Flugfunk
- 🛫 ADS-B
- 📡 ACARS
- 🌦️ Wetterstationen
- 📶 ISM-Signale, z. B. 433 MHz
- 📊 Spektrum- und Signalanalyse
- 💾 Aufzeichnung von I/Q-Daten
- 🤖 automatische Signalerkennung und Machine Learning

## Signalverarbeitung

Der RTL-SDR liefert digitale **I/Q-Samples**, die anschließend softwareseitig verarbeitet werden können:

```text
Antenne
   ↓
RTL-SDR V4
   ↓
I/Q-Daten
   ↓
FFT / Filter
   ↓
Demodulation
   ↓
Dekodierung / Analyse
```

## Software

Mögliche Software und Bibliotheken:

- SDR++
- SDR#
- GNU Radio
- Python
- `librtlsdr`
- NumPy / SciPy
- Matplotlib

## Eigene Projekte

Geplante bzw. mögliche Projekte:

- [x] SDR-Verbindung mit Python
- [x] Frequenzspektrum
- [x] Waterfall
- [ ] FM-Empfang
- [ ] ACARS-Empfang und Dekodierung
- [x] ADS-B
- [ ] Signal Detection
- [ ] Machine-Learning-basierte Signalklassifikation

## Rechtlicher Hinweis

Der RTL-SDR ist ausschließlich ein **Empfänger**. Die Nutzung und Weiterverarbeitung empfangener Funksignale muss den jeweils geltenden gesetzlichen Bestimmungen entsprechen.

## Lizenz
Dieses Projekt steht unter der [MIT-Lizenz](./LICENSE).
   
## Haftungsausschluss
Bitte vor der Nutzung lesen: [DISCLAIMER.md](./DISCLAIMER.md)
