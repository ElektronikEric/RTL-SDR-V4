# Spezifikation: ADS-B Decoder (Mode S / 1090 MHz)

**Version:** 2.0
**Plattform:** Windows (MinGW-w64), RTL-SDR
**Referenz-Implementierung:** dump1090 (FlightAware), GPLv2

**Änderungen gegenüber v1.x:** Vollständige Überarbeitung, da sich die
Architektur seit v1.0 grundlegend geändert hat. Die frühere Fassung
beschrieb einen asynchronen Aufbau mit Capture-Thread, 64-MB-Ringpuffer
und Squelch-/Noise-Floor-Burst-Erkennung. **Diese Komponenten existieren
im aktuellen Code nicht mehr.** Der Decoder liest synchron, verzichtet
vollständig auf Squelch und sucht die Preamble kontinuierlich über
überlappende Chunks hinweg. Diese Spezifikation beschreibt ausschließlich
den tatsächlichen, aktuellen Implementierungsstand sowie die weiterhin
geplanten Erweiterungen.

---

## 1. Zweck und Geltungsbereich

Diese Spezifikation beschreibt einen Software-Decoder, der ADS-B-Signale
(Automatic Dependent Surveillance–Broadcast) im Mode-S-Format auf 1090 MHz
empfängt, demoduliert, dekodiert und validiert. Sie deckt den **aktuellen
Stand** (Signalerfassung bis Rohnachricht + CRC-Prüfung, nur Long-Frames
DF17/DF18) sowie die **geplante Erweiterung** um vollständige
Nachrichten-Analyse (Position, Höhe, Geschwindigkeit, Kennung,
Flugzeug-Tracking) und eine **Kartenvisualisierung** ab.

Nicht Gegenstand dieser Spezifikation: Netzwerkprotokolle Dritter,
Zertifizierung/Zulassung für sicherheitskritische Anwendungen. Der
Decoder ist ein passiver Empfänger für Hobby-/Analysezwecke.

---

## 2. Architektur-Entscheidung: Synchrones Lesen statt Ringpuffer + Squelch

Die ursprüngliche Fassung (v1.0) war strukturell von einem wM-Bus-Decoder-
Projekt übernommen worden: asynchroner Capture-Thread (`rtlsdr_read_async`)
→ Ringpuffer → Squelch mit Noise-Floor-Median und Hangtime → Burst an den
Decoder. In der praktischen Erprobung zeigte sich, dass dieser Ansatz für
ADS-B ungeeignet ist:

- Die exponentielle Glättung des Squelch (Zeitkonstante im Mikrosekunden-
  bereich) verwischte die für die Preamble-Erkennung entscheidenden,
  nur 0,5 µs breiten Einzelpulse.
- Ein fester Schwellwert-Faktor (`SQUELCH_RATIO`) ist für stark schwankende
  Signalstärken (nahe/ferne Flugzeuge) ungeeignet – er schnitt entweder
  schwache Signale ab oder ließ bei zu niedriger Schwelle zu viel Rauschen
  durch.
- dump1090 selbst verwendet kein vergleichbares Squelch-Konzept, sondern
  durchsucht den kontinuierlichen Magnitude-Strom direkt nach dem
  charakteristischen Preamble-Muster; die Struktur-Erkennung selbst wirkt
  bereits als Filter, da das Muster im Rauschen statistisch extrem
  unwahrscheinlich zufällig auftritt.

**Aktuelle Lösung:** Der Decoder liest die IQ-Daten synchron
(`rtlsdr_read_sync`) in großen Chunks (`CHUNK_SAMPLES = 262.144` Samples,
≈131 ms bei 2 MSps), wandelt sie in Magnitude um und durchsucht **jede
Sample-Position** im Chunk direkt auf das Preamble-Muster – ohne
Squelch, ohne Glättung, ohne Zwischenschritt „Burst“. Ein kleiner
Überlappungspuffer (`OVERLAP_SAMPLES`) zwischen aufeinanderfolgenden
Chunks verhindert, dass Nachrichten an Chunk-Grenzen komplett verloren
gehen.

---

## 3. Systemüberblick (aktueller Stand)

```
RTL-SDR Hardware (2 MSps @ 1090 MHz)
        │
        ▼
  Hauptthread: rtlsdr_read_sync()  (BLOCKIEREND, kein separater Capture-Thread)
        │  raw IQ-Bytes [I,Q,I,Q,...], CHUNK_SAMPLES pro Aufruf
        ▼
   IQ → Magnitude (I² + Q²)
        │
        ▼
   Overlap-Puffer voranstellen (letzte OVERLAP_SAMPLES aus vorigem Chunk)
        │
        ▼
   Kontinuierliche Preamble-Suche (sample-genau, JEDE Position im Chunk)
        │  Muster: 1-0-1-0-1-1-1-0, Mindest-Peak PREAMBLE_MIN_PEAK
        ▼
   Erstes Byte dekodieren (PPM) → DF-Vorfilter (nur DF 17/18 weiterverarbeiten)
        │
        ├─ DF ≠ 17/18  →  verwerfen, Lockout, weitersuchen
        │
        ▼ (DF = 17 oder 18)
   Vollständige 112-Bit-Nachricht dekodieren (PPM, 14 Byte)
        │
        ▼
   CRC-Prüfung (24-Bit Mode-S-Polynom)
        │
        ├─ CRC ungültig  →  verwerfen, minimaler Sprung, weitersuchen
        │
        ▼ (CRC gültig)
   Konsolen-Ausgabe: ICAO, DF, CA, Peak, Sample-Index
        │
        ▼
   [GEPLANT: JSONL-Logging, ME-Feld-Dekodierung, Tracking-Tabelle,
    Kartenvisualisierung — siehe Abschnitt 6]
```

**Wichtig:** Es gibt aktuell **keinen** separaten Capture-Thread und
**keinen** Ringpuffer. Erfassung (`read_sync`) und Verarbeitung
(Magnitude, Preamble-Suche, Dekodierung) laufen sequenziell im selben
Thread. Solange die Verarbeitung eines Chunks schneller abgeschlossen
ist, als der nächste Chunk auf USB-/Treiberebene bereitsteht, ist das
unkritisch — siehe dazu die Einschränkung in Abschnitt 8.4.

---

## 4. Hardware- und Erfassungsschicht

### 4.1 RTL-SDR-Anbindung
- Dynamisches Laden von `rtlsdr.dll` zur Laufzeit (`LoadLibraryA` /
  `GetProcAddress`), keine Compile-Time-Abhängigkeit.
- Benötigte Funktionen: `get_device_count`, `open`, `close`,
  `set_center_freq`, `set_sample_rate`, `set_freq_correction`,
  `set_tuner_gain_mode`, `set_tuner_gain`, `reset_buffer`,
  **`read_sync`** (synchron, kein `read_async`/`cancel_async` mehr nötig).
- Fehlt eine Funktion beim Laden, bricht das Programm mit Fehlermeldung ab.

### 4.2 Empfangsparameter

| Parameter | Wert | Begründung |
|---|---|---|
| Center-Frequenz | 1090,000 MHz | ADS-B/Mode-S-Standardfrequenz |
| Samplerate | 2 MSps | Nyquist-Reserve für 1 MHz breite Pulse; dump1090 nutzt ebenfalls 2 MSps |
| Frequenzkorrektur | 0 ppm (konfigurierbar) | Tuner-abhängig, manuell anzupassen |
| Gain | 40,0 dB (manuell, `MANUAL_GAIN_TENTH_DB=400`) | Fest statt AGC, für reproduzierbare Pegel |
| Chunkgröße | 262.144 Samples (≈131 ms) | Ein `read_sync`-Aufruf pro Iteration; groß genug, um Overhead pro Aufruf gering zu halten |
| Overlap | 64 Samples | Puffer der letzten Samples des Vorgänger-Chunks, siehe 8.1 (aktuell zu klein!) |

### 4.3 Kein Ringpuffer mehr
Die Producer/Consumer-Architektur mit `CRITICAL_SECTION` und
`CONDITION_VARIABLE` aus v1.0 wurde vollständig entfernt. `read_sync`
liefert die Daten direkt in einen einzigen Puffer (`iq`), aus dem im
selben Funktionsaufruf weiterverarbeitet wird. Es gibt daher auch keinen
`overflow_count`-Zähler mehr — Datenverluste können stattdessen nur noch
auf Treiber-/USB-Ebene entstehen, wenn die Verarbeitung eines Chunks zu
lange dauert (siehe 8.4).

---

## 5. Signalverarbeitung (aktueller Stand)

### 5.1 IQ → Magnitude
`mag[i] = (I-127)² + (Q-127)²` (Funktion `iq_to_mag`). Kein Wurzelziehen,
keine Glättung — die rohen Magnitude-Werte werden direkt für die
Preamble-Suche verwendet.

### 5.2 Overlap-Verwaltung
Vor jedem neuen Chunk werden die letzten `OVERLAP_SAMPLES` (64) Samples
des vorherigen Durchlaufs vor die neuen Magnitude-Werte kopiert
(`memcpy(mag, overlap, ...)`), sodass eine Preamble, die genau an der
Chunk-Grenze beginnt, nicht verloren geht. Am Ende jeder Iteration werden
die letzten 64 Samples des kombinierten Puffers wieder in `overlap`
gesichert.

### 5.3 Kontinuierliche Preamble-Suche
`is_preamble_at()` prüft für jede Sample-Position `j` im kombinierten
Puffer das feste Muster `1-0-1-0-1-1-1-0` über 10 aufeinanderfolgende
Magnitude-Werte sowie einen Mindest-Peak (`PREAMBLE_MIN_PEAK = 50`,
Mittelwert der 5 High-Chips). Die Suche läuft über die komplette
Chunklänge (`for j = 0 .. total_mag - 32`), nicht nur an vorher per
Squelch markierten Stellen.

### 5.4 Zweistufige Dekodierung (Effizienz-Optimierung)
Um nicht bei jedem Preamble-Treffer sofort die vollen 112 Bit zu
dekodieren, wird zunächst nur das **erste Byte** dekodiert
(`decode_first_byte_ppm_2m`) und daraus das Downlink Format (DF) extrahiert:

- **DF ≠ 17 und DF ≠ 18:** Nachricht wird sofort verworfen (kein
  Extended-Squitter-Long-Frame), Suchposition springt um
  `LOCKOUT_SAMPLES` (240 Samples ≈ 120 µs) weiter.
- **DF = 17 oder 18:** Erst jetzt wird die vollständige 112-Bit-Nachricht
  dekodiert (`decode_message_ppm_2m`, 14 Byte).

### 5.5 PPM-Bit-Dekodierung
Pro Bit werden zwei Samples verglichen (2 Samples/Bit bei 2 MSps):
`early = mag[s]`, `late = mag[s+1]`. Nach Mode-S-Konvention liegt der
Puls bei einer `1` in der ersten Hälfte des Chips, bei einer `0` in der
zweiten Hälfte:

```c
int bit = (early > late) ? 1 : 0;
```

Dies gilt identisch in `decode_first_byte_ppm_2m` und
`decode_message_ppm_2m`.

### 5.6 CRC / Parity
24-Bit-CRC nach dem korrekten Mode-S-Generatorpolynom:

```c
const uint32_t POLY = 0xFFF409u;
```

Berechnung: 88 Datenbits (11 Byte) einschieben, danach 24 weitere
Schiebeschritte mit Null-Bits, Ergebnis gegen das PI-Feld (Byte 11–13)
vergleichen (`check_adsb_crc`). Nur bei Übereinstimmung gilt die
Nachricht als gültig.

### 5.7 Lockout-/Weitersuch-Strategie nach einem Treffer

| Fall | Verhalten |
|---|---|
| Preamble gefunden, DF ≠ 17/18 | `j += LOCKOUT_SAMPLES` (240) |
| Preamble gefunden, DF = 17/18, aber Nachricht nicht vollständig dekodierbar (zu nah am Chunk-Ende) | `j += LOCKOUT_SAMPLES` (240), Nachricht als „truncated“ ausgegeben |
| Preamble gefunden, DF = 17/18, Nachricht dekodiert, aber CRC ungültig | `j += 1` (minimaler Sprung — erlaubt erneuten Versuch bei leicht verschobener Preamble-Position in unmittelbarer Nähe) |
| Preamble gefunden, DF = 17/18, CRC gültig | Ausgabe auf Konsole, `j += LOCKOUT_SAMPLES` (240) |

### 5.8 Frame-Struktur (aktuell ausgewertet)

| Bits | Feld | Bemerkung |
|---|---|---|
| 0–4 | DF (Downlink Format) | Vorfilter: nur 17/18 werden weiterverarbeitet |
| 5–7 | CA (Capability) | wird ausgegeben, nicht weiter interpretiert |
| 8–31 | ICAO-Adresse (24 Bit) | eindeutige Flugzeug-Kennung |
| 32–87 | Payload (ME-Feld, 56 Bit) | **aktuell nicht dekodiert** |
| 88–111 | Parity (24 Bit) | CRC-Prüfsumme |

---

## 6. Aktueller Funktionsumfang (Stand des vorliegenden Codes)

- Synchroner Empfang, Magnitude-Berechnung, kontinuierliche
  Preamble-Suche mit Chunk-Overlap, zweistufige PPM-Dekodierung
  (DF-Vorfilter + volle Nachricht), CRC-Prüfung mit korrektem Polynom.
- Nur **Long Frames** (112 Bit) mit DF 17 (ADS-B Extended Squitter) oder
  DF 18 (TIS-B/ADS-R) werden ausgewertet; alle anderen DF-Werte werden
  ohne weitere Prüfung verworfen.
- Ausgabe **ausschließlich auf der Konsole** (`printf`): Treffer-Nummer,
  Sample-Index, Peak, rohes erstes Byte, DF, CA, ICAO-Adresse (erste 3
  Payload-Bytes als Hex). Endstatistik `hits`/`df_decoded` beim Beenden.
- **Kein** Datei-/JSONL-Logging aktiv.
- **Keine** Interpretation des ME-Feldes (Payload) nach Nachrichtentyp.
- **Keine** Aircraft-Tracking-Tabelle, **keine** Kartenvisualisierung.
- **Kein** Squelch, **kein** Ringpuffer, **kein** separater Capture-Thread
  (siehe Abschnitt 2).

---

## 7. Geplante Erweiterung: Nachrichten-Analyse und Visualisierung (dump1090-Stil)

Dieser Abschnitt beschreibt weiterhin geplante, noch nicht implementierte
Funktionalität. Sie baut auf dem in Abschnitt 5 beschriebenen,
synchronen Erfassungspfad auf (nicht mehr auf Ringpuffer/Squelch).

### 7.1 Zielbild
Nach erfolgreicher CRC-Prüfung soll das ME-Feld (56 Bit Payload) nach
**Type Code** (die ersten 5 Bit des ME-Feldes) ausgewertet und in
strukturierte, pro Flugzeug aggregierte Zustände überführt werden. Diese
Zustände sollen anschließend auf einer Karte visualisiert werden (7.6).

### 7.2 Nachrichtentypen und geplante Dekodierung

| Type Code | Nachrichtentyp | Geplante Extraktion |
|---|---|---|
| 1–4 | Identification (Callsign) | 8 Zeichen à 6 Bit, Charset-Tabelle (analog dump1090 `ais_charset`) |
| 5–8 | Surface Position | Lat/Lon (CPR, Boden-Referenz), Bewegungsstatus |
| 9–18 | Airborne Position (Baro-Höhe) | Höhe (AC12-Format), CPR-codierte Lat/Lon |
| 19 | Airborne Velocity | Subtypen 1/2 (Ground Speed) und 3/4 (Airspeed/Heading); Geschwindigkeit, Kurs/Heading, Steig-/Sinkrate |
| 20–22 | Airborne Position (GNSS-Höhe) | wie 9–18, aber GNSS- statt Baro-Höhe |
| 23–27 | reserviert / Test | niedrige Priorität |
| 28 | Aircraft Status (Notfall/ACAS) | Notfallcode, Squawk (bei Subtype 1) |
| 29 | Target State & Status | ausgewählte Höhe, Autopilot-Modi (optional, niedrige Priorität) |
| 31 | Operational Status | Fähigkeitsflags (z. B. ADS-B-Version) |

### 7.3 CPR-Positionsdekodierung (Compact Position Reporting)
- Jede Airborne-/Surface-Position-Nachricht enthält ein Even/Odd-Flag und
  17-Bit-codierte Lat/Lon-Fraktionen.
- **Global Decoding:** benötigt ein Even- und ein Odd-Framepaar
  desselben Flugzeugs innerhalb von max. 10 s → absolute Position.
- **Local (Relative) Decoding:** nutzt eine bekannte Referenzposition für
  schnellere, aber weniger eindeutige Auflösung.
- Geplant: pro ICAO-Adresse werden die letzten Even- und Odd-Frames mit
  Zeitstempel zwischengespeichert; sobald ein gültiges Paar vorliegt,
  wird die Global-CPR-Formel angewendet.
- Diese Lat/Lon-Werte sind die direkte Datengrundlage für die
  Kartenvisualisierung (7.6).

### 7.4 Höhen-Dekodierung
- **AC12-Format** (Airborne Position DF17): 12-Bit-Feld, Q-Bit bestimmt
  25-ft- oder 100-ft-Auflösung (Gillham-Code bei Q=0, sonst linear).
- Geplant: `decode_ac12_altitude()` analog dump1090s `decodeAC12Field`.

### 7.5 Geschwindigkeits-/Kursdekodierung
- Type Code 19, Subtype 1/2: East-West-/North-South-Velocity →
  Ground Speed und Heading via `atan2`/`sqrt`.
- Subtype 3/4: Airspeed (IAS/TAS) und Heading direkt codiert.
- Vertikalrate: Vorzeichen + 64 ft/min-Schritte.
- Heading wird später für die Ausrichtung des Flugzeug-Icons auf der
  Karte benötigt (7.6.3).

### 7.6 Aircraft-Tracking-Tabelle

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
  „aktiv“, danach „alt“, Löschung nach z. B. 5 Minuten ohne Nachricht).
- Aktualisierung bei jeder erfolgreich dekodierten Nachricht, nicht nur
  bei Positionsnachrichten.
- Zentrale Datenquelle für Ausgabeformate (7.7) und Kartenvisualisierung
  (7.8).

### 7.7 Ausgabeformate (geplant)

| Format | Zweck |
|---|---|
| JSONL (`ads_b_messages.jsonl`) | Rohnachrichten-Log für Offline-Analyse |
| SBS-1/BaseStation-CSV | Kompatibilität mit bestehenden ADS-B-Tools (z. B. Virtual Radar Server) |
| Aircraft-Snapshot (JSON, `aircraft.json`) | periodischer Dump der Tracking-Tabelle; direkte Datenquelle für die Kartenvisualisierung |
| Optional: TCP-Server (Port 30003 „SBS“ oder 30002 „Raw“) | Live-Feed für externe Clients/Kartendarstellung |
| HTTP-Server | liefert `aircraft.json` sowie statische Web-Dateien der Kartenansicht aus |

### 7.8 Kartenvisualisierung

#### 7.8.1 Zielbild
Live-Darstellung der getrackten Flugzeuge (7.6) auf einer interaktiven
Karte, analog zur dump1090-Weboberfläche (`gmap.html` bzw. Forks wie
`tar1090`/`readsb`): Position, Höhe, Geschwindigkeit, Kurs, Kennung,
fortlaufend aktualisiert ohne manuellen Reload.

#### 7.8.2 Architektur
```
Aircraft-Tracking-Tabelle (In-Memory, C)
        │  periodischer Snapshot (z. B. alle 1 s)
        ▼
   aircraft.json
        │
        ▼
  Eingebetteter HTTP-Server (eigener Thread)
        │  statische Web-Dateien (HTML/JS/CSS) + aircraft.json
        ▼
   Browser: Leaflet.js + OpenStreetMap-Kacheln
        │  Polling auf aircraft.json
        ▼
   Flugzeug-Icons, rotiert nach Heading, mit Tooltip
```

#### 7.8.3 Funktionale Anforderungen
- Basiskarte (OpenStreetMap via Leaflet.js), zentriert auf konfigurierbare
  Empfängerposition.
- Ein Marker pro ICAO-Adresse mit `have_position == 1`, Rotation nach
  `heading_deg`.
- Live-Update, Standardintervall 1–2 s (Polling); WebSocket-Push als
  spätere Option.
- Tooltip: ICAO, Callsign, Höhe, Ground Speed, Kurs, Vertikalrate, Zeit
  seit letzter Nachricht.
- Optional (spätere Ausbaustufe): Flugspur/Trail, Reichweitenring.
- Alterung/Ausblenden gemäß Tracking-Timeout (7.6).

#### 7.8.4 Technische Umsetzungsoptionen

| Variante | Beschreibung | Vor-/Nachteile |
|---|---|---|
| **A: Eingebetteter Webserver** | Minimaler HTTP-Server direkt im C-Programm (Winsock, eigener Thread), liefert HTML/JS + `aircraft.json` | Keine externen Abhängigkeiten; mehr Implementierungsaufwand in C |
| **B: Getrenntes Anzeige-Tool** | Decoder schreibt nur `aircraft.json`/JSONL; separates Skript/Programm übernimmt Darstellung | Geringerer C-Aufwand, klare Trennung; zusätzlicher Prozess nötig |

Empfehlung für die erste Ausbaustufe: **Variante A**, minimaler HTTP-Server
(`GET /` für HTML, `GET /aircraft.json` für Daten).

#### 7.8.5 Abhängigkeiten
- Setzt funktionierende CPR-Positionsdekodierung (7.3) voraus.
- Setzt die Aircraft-Tracking-Tabelle (7.6) voraus.
- Kartenkacheln benötigen im Regelfall eine Internetverbindung im Browser;
  Offline-Kachel-Caching ist optional, nicht Teil der ersten Ausbaustufe.

#### 7.8.6 Nicht-funktionale Anforderungen
- HTTP-Server/JSON-Export laufen in einem eigenen Thread, getrennt vom
  zeitkritischen Erfassungs-/Dekodierpfad (Abschnitt 8.4).
- Funktioniert in aktuellen Desktop-Browsern (Chrome, Firefox, Edge) ohne
  Plugins.
- Bei 0 aktiven Flugzeugen: klarer „keine Ziele erfasst“-Zustand statt
  leerer Ansicht.

---

## 8. Bekannte Einschränkungen des aktuellen Stands

1. **`OVERLAP_SAMPLES = 64` ist zu klein.** Eine vollständige Nachricht
   benötigt ab Preamble-Beginn ca. 16 (Preamble) + 224 (112 Bit × 2
   Samples) = 240 Samples. Liegt der Preamble-Start näher als 240 Samples
   am Chunk-Ende, wird die Nachricht als „truncated“ verworfen und geht
   verloren, da der Overlap-Puffer nicht genug Kontext für den nächsten
   Durchlauf vorhält. **Empfehlung:** `OVERLAP_SAMPLES` auf mindestens
   300–400 erhöhen.
2. **Kein Logging.** Alle Ergebnisse landen nur auf der Konsole; es gibt
   aktuell keine Datei-Ausgabe für spätere Auswertung.
3. **Kein DF11/DF4/DF5/DF20/DF21-Support.** Es werden ausschließlich Long
   Frames mit DF 17/18 verarbeitet; kurze 56-Bit-Nachrichten (z. B. DF11
   „All-Call Reply“) werden ignoriert, da bereits der DF-Vorfilter sie
   verwirft.
4. **Kein Confidence-/Error-Correction-Mechanismus.** dump1090 nutzt
   Bit-Fehlerkorrektur (Single-Bit-Fix via CRC-Restklassen) für DF17;
   dieser Decoder verwirft Nachrichten bei CRC-Fehler vollständig.
5. **Feste Preamble-Schwelle** (`PREAMBLE_MIN_PEAK = 50`) ist nicht an
   den aktuellen Rauschpegel gekoppelt; bei schwachen/entfernten
   Flugzeugen oder wechselnder Störumgebung ggf. adaptiv gestalten.
6. **Synchrones, einfädiges Lesen (siehe 8.4).**
7. **Keine Visualisierung vorhanden** — bis zur Umsetzung von 7.8 gibt es
   keine grafische Ausgabe.

### 8.4 Risiko durch synchrones Lesen ohne Capture-Thread
Da `rtlsdr_read_sync` blockierend im Hauptthread aufgerufen wird und kein
separater Capture-Thread mehr existiert, läuft während der Verarbeitung
eines Chunks (Magnitude-Berechnung + vollständige Preamble-Suche über
den ganzen Chunk) **keine Erfassung neuer Daten**. Der RTL-SDR-Treiber
puffert intern nur eine begrenzte Menge an Samples; dauert die
Verarbeitung eines Chunks spürbar länger als die Zeit, die der nächste
Chunk zum Befüllen braucht (~131 ms bei 2 MSps und 262.144 Samples),
können auf USB-/Treiberebene **stillschweigend Samples verloren gehen**,
ohne dass das Programm dies wie beim früheren Ringpuffer-Overflow-Zähler
meldet. Dies ist aktuell unkritisch, solange die Verarbeitung pro Chunk
deutlich unter ~131 ms bleibt, sollte aber beobachtet werden, sobald
Schritt 7.6 (Tracking-Tabelle) oder 7.8 (HTTP-Server) im selben Thread
ergänzt werden.

---

## 9. Nicht-funktionale Anforderungen

| Kategorie | Anforderung |
|---|---|
| Echtzeitfähigkeit | Verarbeitung eines Chunks (Magnitude + Preamble-Suche + Dekodierung) muss deutlich unter der Chunk-Dauer (~131 ms) bleiben, da kein Capture-Thread mehr puffert (siehe 8.4) |
| Robustheit | Fehlerhafte/unvollständige Nachrichten dürfen die Verarbeitung nicht blockieren oder crashen |
| Portabilität der Analyseschicht | CPR-/Höhen-/Geschwindigkeitsdekodierung sollte als plattformneutrale Bibliothek getrennt von der Windows-spezifischen Erfassungsschicht implementiert werden |
| Konfigurierbarkeit | Frequenzkorrektur, Gain, Preamble-Schwelle, Overlap-Größe, Ausgabeformat, Tracking-Timeout und Empfängerposition sollten künftig per Konfigurationsdatei statt Compile-Time-Konstanten einstellbar sein |
| Nebenläufigkeit künftiger Erweiterungen | Sobald HTTP-Server (7.8) oder Tracking-Tabelle (7.6) ergänzt werden, sollten diese in eigenen Threads laufen, um den zeitkritischen Erfassungspfad (8.4) nicht zu verzögern |
| Lizenz | Architektur/Algorithmen sind an dump1090 (GPLv2) angelehnt; bei Veröffentlichung GPLv2-Kompatibilität sicherstellen. Bei Nutzung von Leaflet.js (BSD-2) und OpenStreetMap-Kacheln (ODbL, Attribution erforderlich) deren Lizenzbedingungen einhalten |

---

## 10. Nächste Implementierungsschritte (Vorschlag/Reihenfolge)

1. **`OVERLAP_SAMPLES` erhöhen** (auf ≥ 300–400), um Nachrichtenverluste
   an Chunk-Grenzen zu vermeiden (siehe 8.1).
2. **JSONL-Logging ergänzen**: erfolgreich dekodierte Nachrichten (ICAO,
   DF, Rohbytes, Zeitstempel, Signalstärke) in Datei schreiben statt nur
   auf Konsole auszugeben.
3. Type-Code-Dispatch für DF17/18 (ME-Feld) implementieren.
4. Identification- und Airborne-Velocity-Dekodierung (einfachste Typen,
   keine CPR-Abhängigkeit) umsetzen und testen.
5. AC12-Höhendekodierung implementieren.
6. CPR-Global-Decoding inkl. Even/Odd-Zwischenspeicherung pro ICAO.
7. Aircraft-Tracking-Tabelle mit Timeout-Logik.
8. Minimaler HTTP-Server + `aircraft.json`-Export (Grundlage für 7.8),
   zunächst nur als JSON-Endpunkt testbar.
9. Kartenvisualisierung (Frontend): statische HTML-Seite mit Leaflet.js,
   Polling auf `aircraft.json`, Flugzeug-Icons mit Rotation und Tooltip.
10. Beobachtung/Messung der Verarbeitungszeit pro Chunk (siehe 8.4),
    ggf. Rückkehr zu einem Capture-Thread mit einfachem Puffer, falls
    Schritte 7–9 die Verarbeitungszeit über die Chunk-Dauer heben.
11. Optionaler SBS-Netzwerkserver für Kompatibilität mit vorhandenen
    ADS-B-Visualisierungstools.
