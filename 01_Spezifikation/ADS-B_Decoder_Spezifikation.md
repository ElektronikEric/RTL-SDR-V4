# Spezifikation: ADS-B Decoder (Mode S / 1090 MHz)

**Version:** 1.0
**Plattform:** Windows (MinGW-w64), RTL-SDR
**Referenz-Implementierung:** dump1090 (FlightAware), GPLv2

---

## 1. Zweck und Geltungsbereich

Diese Spezifikation beschreibt einen Software-Decoder, der ADS-B-Signale
(Automatic Dependent Surveillance–Broadcast) im Mode-S-Format auf 1090 MHz
empfängt, demoduliert, dekodiert und validiert. Sie deckt sowohl den
**aktuellen Stand** (Signalerfassung bis Rohnachricht + CRC-Prüfung) als
auch die **geplante Erweiterung** um eine vollständige Nachrichten-Analyse
(Position, Höhe, Geschwindigkeit, Kennung, Flugzeug-Tracking) ab, analog
zu dump1090.

Nicht Gegenstand dieser Spezifikation: Netzwerkprotokolle Dritter (z. B.
FlightAware-Feed), Kartendarstellung im Detail (nur als Ausblick), sowie
Zertifizierung/Zulassung für sicherheitskritische Anwendungen. Der Decoder
ist ein passiver Empfänger für Hobby-/Analysezwecke.

---

## 2. Systemüberblick

```
RTL-SDR Hardware (2 MSps @ 1090 MHz)
        │
        ▼
  Capture-Thread (rtlsdr_read_async, Callback)
        │  raw IQ-Bytes [I,Q,I,Q,...]
        ▼
   Ringpuffer (64 MB, Producer/Consumer)
        │
        ▼
  Main-Loop: Chunk-Verarbeitung (100 ms Blöcke)
        │
        ├─ IQ → Magnitude (I² + Q²)
        │
        ▼
   Squelch / Burst-Erkennung (Noise-Floor-Median, Hangtime)
        │  Burst = Magnitude-Array eines vermuteten Mode-S-Frames
        ▼
   Preamble-Suche (Muster 1-0-1-0-1-1-1-0)
        │
        ▼
   PPM-Bit-Dekodierung (112 Bit / 14 Byte)
        │
        ▼
   CRC-Prüfung (24-Bit Mode-S-Polynom)
        │
        ▼
   [AKTUELL: Ende – Ergebnis wird verworfen]
        │
        ▼
   [GEPLANT: Nachrichten-Dekodierung nach DF/Type-Code]
        │
        ▼
   [GEPLANT: Aircraft-Tracking-Tabelle, CPR-Positionsauflösung]
        │
        ▼
   [GEPLANT: Ausgabe – JSONL, SBS/BaseStation, ggf. Netzwerk-Server]
```

---

## 3. Hardware- und Erfassungsschicht

### 3.1 RTL-SDR-Anbindung
- Dynamisches Laden von `rtlsdr.dll` zur Laufzeit (`LoadLibraryA` /
  `GetProcAddress`), keine Compile-Time-Abhängigkeit.
- Benötigte Funktionen: `get_device_count`, `open`, `close`,
  `set_center_freq`, `set_sample_rate`, `set_freq_correction`,
  `set_tuner_gain_mode`, `set_tuner_gain`, `reset_buffer`,
  `read_async`, `cancel_async`.
- Fehlt eine Funktion beim Laden, bricht das Programm mit Fehlermeldung ab.

### 3.2 Empfangsparameter

| Parameter | Wert | Begründung |
|---|---|---|
| Center-Frequenz | 1090,000 MHz | ADS-B/Mode-S-Standardfrequenz |
| Samplerate | 2 MSps | Nyquist-Reserve für 1 MHz breite Pulse; dump1090 nutzt ebenfalls 2 MSps |
| Frequenzkorrektur | 0 ppm (konfigurierbar) | Tuner-abhängig, manuell anzupassen |
| Gain | 40,0 dB (manuell, `MANUAL_GAIN_TENTH_DB=400`) | Fest statt AGC, für reproduzierbare Pegel |
| Chunkgröße | 0,1 s (200.000 Samples) | Kompromiss zwischen Latenz und Verarbeitungs-Overhead |

### 3.3 Ringpuffer
- Kapazität: 64 MiB, klassisches Producer/Consumer-Modell mit
  `CRITICAL_SECTION` + `CONDITION_VARIABLE`.
- Producer: `rtlsdr_callback` (läuft im Capture-Thread, vom Treiber aufgerufen).
- Consumer: Main-Loop (`ring_pop`, blockierend mit Timeout 200 ms).
- Overflow wird gezählt (`overflow_count`) und alle 0,5 s protokolliert.
- **Hinweis:** Bei Overruns gehen Rohdaten verloren – das ist der einzige
  Datenverlustpfad im System und sollte im Betrieb überwacht werden
  (siehe Abschnitt 8.3, Monitoring).

---

## 4. Signalverarbeitung

### 4.1 IQ → Magnitude
`out_mag[i] = (I-127)² + (Q-127)²`
Kein Wurzelziehen (quadratische Magnitude reicht für Schwellenvergleich
und ist günstiger als `sqrt`).

### 4.2 Squelch / Burst-Erkennung
- **Noise-Floor:** gleitender Median über die letzten 1000 geglätteten
  Powerwerte (Optimierung gegenüber vollem `qsort()` über 200.000 Werte),
  aktualisiert alle `NOISE_FLOOR_UPDATE_SEC` (0,2 s).
- **Glättung:** exponentielles Smoothing mit Zeitkonstante 5 µs.
- **Schwelle:** `noise_floor * SQUELCH_RATIO` (Faktor 50).
- **Hangtime:** 2 ms Nachlauf, bevor ein Burst als beendet gilt.
- **Burst-Grenzen:** min. 0,1 ms, max. 2 ms (danach Zwangs-Abschluss).
- Ergebnis eines abgeschlossenen Bursts wird per Callback
  (`on_burst_complete`) an den Decoder übergeben.

### 4.3 Preamble-Erkennung
- Mode-S-Preamble: 8 Chips, Muster `1-0-1-0-1-1-1-0`.
- Sample-genaue Suche (nicht nur auf Chip-Grenzen), wie in dump1090.
- Zusätzliches Amplitudenkriterium: gemittelter Peak der High-Chips muss
  über `MODES_DEBUG_NOPREAMBLE_LEVEL` (50) liegen, um Rauschtreffer
  auszuschließen.

### 4.4 Bit-Dekodierung (PPM)
- Mode S kodiert jedes Bit als Pulse-Position-Modulation innerhalb eines
  1-µs-Chips (2 Samples bei 2 MSps).
- Bitentscheidung: Vergleich der Magnitude an Chip-Anfang, -Mitte und -Ende;
  liegt das Maximum in der Chip-Mitte → Bit = 1, sonst 0.
- Es werden 112 Bit (14 Byte) nach der Preamble gelesen (Long-Message-Format).

### 4.5 CRC / Parity
- 24-Bit-CRC nach Mode-S-Polynom `0xFFFA0480`, Tabellenverfahren
  (256-Einträge-Lookup-Table, analog dump1090).
- Geprüft werden die ersten 11 Byte gegen die letzten 3 Byte (Parity-Feld).
- Ergebnis: `crc_ok` oder `crc_mismatch`.

### 4.6 Frame-Struktur (aktuell ausgewertet)

| Bits | Feld | Bemerkung |
|---|---|---|
| 0–4 | DF (Downlink Format) | z. B. 17 = ADS-B Extended Squitter |
| 8–31 | ICAO-Adresse (24 Bit) | eindeutige Flugzeug-Kennung |
| 32–87 | Payload (ME-Feld, 56 Bit) | **aktuell nicht weiter dekodiert** |
| 88–111 | Parity (24 Bit) | CRC-Prüfsumme |

---

## 5. Aktueller Funktionsumfang (Stand des vorliegenden Codes)

- Empfang, Demodulation, Preamble-Erkennung, Bit-Dekodierung, CRC-Prüfung.
- `ADSBResult` enthält DF, ICAO-Adresse, Rohpayload (7 Byte), Parity,
  Signalstärke und Statusgrund (`reason`).
- **Kein** Logging aktiv: `on_burst_complete()` verwirft das Ergebnis
  aktuell vollständig (kein JSONL-Schreiben, obwohl `LOG_FILE` definiert ist).
- **Kein** Debug-IQ-Dump aktiv (`DEBUG_DUMP_FAILED = 0`).
- **Keine** Interpretation des ME-Feldes (Payload) nach Nachrichtentyp.

> Diese Lücken sind der Ausgangspunkt für Abschnitt 6 (geplante Analyse).

---

## 6. Geplante Erweiterung: Nachrichten-Analyse (dump1090-Stil)

### 6.1 Zielbild
Nach erfolgreicher CRC-Prüfung soll das ME-Feld (56 Bit Payload) nach
**Type Code** (Bits 33–37 des Gesamtframes, also die ersten 5 Bit des
ME-Feldes) ausgewertet und in strukturierte, pro Flugzeug aggregierte
Zustände überführt werden.

### 6.2 Nachrichtentypen und geplante Dekodierung

| Type Code | Nachrichtentyp | Geplante Extraktion |
|---|---|---|
| 1–4 | Identification (Callsign) | 8 Zeichen à 6 Bit, Charset-Tabelle (analog dump1090 `ais_charset`) |
| 5–8 | Surface Position | Lat/Lon (CPR, Boden-Referenz), Bewegungsstatus |
| 9–18 | Airborne Position (Baro-Höhe) | Höhe (AC12-Format), CPR-codierte Lat/Lon |
| 19 | Airborne Velocity | Subtypen 1/2 (Ground Speed) und 3/4 (Airspeed/Heading); liefert Geschwindigkeit, Kurs/Heading, Steig-/Sinkrate |
| 20–22 | Airborne Position (GNSS-Höhe) | wie 9–18, aber GNSS- statt Baro-Höhe |
| 23–27 | reserviert / Test | niedrige Priorität |
| 28 | Aircraft Status (Notfall/ACAS) | Notfallcode, Squawk (bei Subtype 1) |
| 29 | Target State & Status | ausgewählte Höhe, Autopilot-Modi (optional, niedrige Priorität) |
| 31 | Operational Status | Fähigkeitsflags (z. B. ADS-B-Version) |

### 6.3 CPR-Positionsdekodierung (Compact Position Reporting)

- Jede Airborne-/Surface-Position-Nachricht enthält ein Even/Odd-Flag und
  17-Bit-codierte Lat/Lon-Fraktionen.
- **Global Decoding:** benötigt ein Even- und ein Odd-Framepaar
  desselben Flugzeugs innerhalb von max. 10 s → absolute Position.
- **Local (Relative) Decoding:** nutzt eine bekannte Referenzposition
  (z. B. letzte bekannte Position oder Empfängerstandort) für schnellere,
  aber weniger eindeutige Auflösung.
- Geplante Implementierung: pro ICAO-Adresse werden die letzten Even- und
  Odd-Frames mit Zeitstempel zwischengespeichert; sobald ein gültiges
  Paar vorliegt, wird die Global-CPR-Formel angewendet.

### 6.4 Höhen-Dekodierung
- **AC12-Format** (Airborne Position DF17): 12-Bit-Feld, Q-Bit bestimmt
  25-ft- oder 100-ft-Auflösung (Gillham-Code bei Q=0, sonst linear).
- Geplant: eine Dekodierfunktion `decode_ac12_altitude()` analog
  dump1090s `decodeAC12Field`.

### 6.5 Geschwindigkeits-/Kursdekodierung
- Aus Type Code 19, Subtype 1/2: East-West- und North-South-Velocity
  → resultierende Ground Speed und Heading via `atan2`/`sqrt`.
- Subtype 3/4: Airspeed (IAS/TAS) und Heading direkt codiert.
- Vertikalrate (Vertical Rate) aus separatem Feld, Vorzeichen + 64 ft/min-Schritten.

### 6.6 Aircraft-Tracking-Tabelle

Geplante In-Memory-Struktur (Hash-Map oder sortiertes Array nach ICAO-Adresse):

```c
typedef struct {
    uint32_t icao_addr;
    char callsign[9];
    int have_altitude;   int altitude_ft;
    int have_speed;      double ground_speed_kt, heading_deg, vertical_rate_fpm;
    int have_position;   double lat, lon;
    uint64_t last_even_ts, last_odd_ts;
    uint8_t even_frame[7], odd_frame[7];
    time_t last_seen;
    int messages_total, messages_crc_ok;
} AircraftState;
```

- Einträge veralten nach konfigurierbarem Timeout (dump1090-Standard: 60 s
  „aktiv“, danach Anzeige als „alt“, Löschung nach z. B. 5 Minuten ohne
  Nachricht).
- Aktualisierung erfolgt inkrementell bei jeder erfolgreich dekodierten
  Nachricht (nicht nur bei Positionsnachrichten).

### 6.7 Ausgabeformate (geplant)

| Format | Zweck |
|---|---|
| JSONL (`ads_b_messages.jsonl`) | Rohnachrichten-Log für Offline-Analyse, ein JSON-Objekt pro Zeile |
| SBS-1/BaseStation-CSV | Kompatibilität mit bestehenden ADS-B-Tools (z. B. Virtual Radar Server) |
| Aircraft-Snapshot (JSON) | periodischer Dump der Tracking-Tabelle, ähnlich dump1090s `aircraft.json` |
| Optional: TCP-Server (Port 30003 „SBS“ oder 30002 „Raw“) | Live-Feed für externe Clients/Kartendarstellung |

### 6.8 Validierung/Qualitätssicherung der Analyse
- Vergleich der dekodierten ICAO-Adressen/Positionen gegen eine bekannte
  Referenzquelle (z. B. ADS-B-Exchange oder ein zweiter Empfänger) zur
  Verifikation der CPR- und Höhendekodierung.
- Unit-Tests mit aufgezeichneten Referenz-IQ-Dateien und bekannten
  erwarteten Dekodierergebnissen (Regressionstest gegen dump1090-Ausgabe
  derselben Aufnahme).
- Statistik-Zähler: Frames/Sekunde, CRC-Erfolgsquote, Anteil DF17 vs.
  andere DF-Typen, Anzahl aktiver Flugzeuge.

---

## 7. Nicht-funktionale Anforderungen

| Kategorie | Anforderung |
|---|---|
| Echtzeitfähigkeit | Verarbeitung eines 100-ms-Chunks muss < 100 ms dauern (siehe eingebautes Timing-Debugging); sonst drohen Ringpuffer-Overruns |
| Robustheit | Fehlerhafte/unvollständige Bursts dürfen den Hauptthread nicht blockieren oder crashen |
| Portabilität der Analyseschicht | CPR-/Höhen-/Geschwindigkeitsdekodierung sollte als von der Windows-spezifischen Erfassungsschicht getrennte, plattformneutrale Bibliothek implementiert werden, um Wiederverwendung (z. B. für Offline-IQ-Dateien) zu ermöglichen |
| Konfigurierbarkeit | Frequenzkorrektur, Gain, Squelch-Ratio, Ausgabeformat und Tracking-Timeout sollten künftig per Konfigurationsdatei statt Compile-Time-Konstanten einstellbar sein |
| Lizenz | Da Architektur/Algorithmen an dump1090 (GPLv2) angelehnt sind, muss bei Veröffentlichung die GPLv2-Lizenzkompatibilität sichergestellt werden |

---

## 8. Bekannte Einschränkungen des aktuellen Stands

1. **Kein DF11/DF4/DF5/DF20/DF21-Support** – aktuell wird nur die
   generische 112-Bit-Struktur behandelt; kurze Nachrichten (56 Bit,
   z. B. DF11 „All-Call Reply“) werden nicht gesondert erkannt und
   könnten fehlinterpretiert werden.
2. **Kein Confidence-/Error-Correction-Mechanismus** – dump1090 nutzt
   Bit-Fehlerkorrektur (Single-Bit-Fix via CRC-Restklassen) für DF17;
   dieser Decoder verwirft Nachrichten bei CRC-Fehler vollständig.
3. **Feste Preamble-Schwelle** – `MODES_DEBUG_NOPREAMBLE_LEVEL = 50` ist
   nicht automatisch an den Rauschpegel gekoppelt; bei stark wechselnder
   Störumgebung ggf. adaptiv gestalten.
4. **Verarbeitung ist einfädig** – Demodulation und (künftige) Nachrichten-
   analyse laufen im selben Thread wie das Ringpuffer-Auslesen; bei
   hoher Nachrichtenrate (dichter Luftraum) sollte die Analyseschicht in
   einen eigenen Thread ausgelagert werden.

---

## 9. Nächste Implementierungsschritte (Vorschlag/Reihenfolge)

1. `on_burst_complete()` erweitern: bei `r.ok == 1` Nachricht als JSONL
   protokollieren (ICAO, DF, Rohbytes, Zeitstempel, Signalstärke).
2. Type-Code-Dispatch für DF17/18 (ME-Feld) implementieren.
3. Identification- und Airborne-Velocity-Dekodierung (einfachste Typen,
   keine CPR-Abhängigkeit) umsetzen und testen.
4. AC12-Höhendekodierung implementieren.
5. CPR-Global-Decoding inkl. Even/Odd-Zwischenspeicherung pro ICAO.
6. Aircraft-Tracking-Tabelle mit Timeout-Logik.
7. Optionaler SBS-Netzwerkserver für Kompatibilität mit vorhandenen
   ADS-B-Visualisierungstools.
