# -*- coding: utf-8 -*-
"""
ACARS VHF Decoder (educational / hobby project)
=================================================

Signalkette:
  RTL-SDR IQ  ->  AM-Envelope-Demod  ->  Tiefpass + Dezimation (Audio)
             ->  Squelch (Burst-Erkennung)
             ->  Momentanfrequenz (Hilbert-Diskriminator)
             ->  Bit-Extraktion (NRZI: 1200 Hz = Bitwechsel, 2400 Hz = gleich)
             ->  Byte-Framing (Suche nach SYN SYN SOH), Paritaetspruefung
             ->  Feldparsing + BCS/CRC-Pruefung
             ->  Logging (JSONL) + optionales WAV-Dump fehlgeschlagener Bursts

WICHTIGE EINSCHRAENKUNGEN (bitte lesen):
- Die Bit-Taktrueckgewinnung ist "open loop": Nach dem Ende der Praeambel wird
  exakt alle SAMPLES_PER_BIT Samples ein Bit entnommen, ohne laufende
  Nachregelung. Bei kurzen ACARS-Bursts (< 1 s) reicht das meist aus, bei
  schwachem SNR oder starkem Sample-Clock-Offset kann es zu Bitfehlern
  fuehren. Falls viele Frames an der Praeambel oder mitten im Text
  "abreissen", ist eine Early-Late-Gate-Nachregelung der naechste sinnvolle
  Ausbauschritt.
- Die CRC-Parameter (Polynom ist bekannt: CRC-CCITT x^16+x^12+x^5+1, auch als
  "ACARS-16-ARINC" referenziert), aber Init-Wert / Bit-Reflektion / Byte-
  reihenfolge sind hier eine gaengige Annahme (reflektiert, LSB-first,
  Init 0xFFFF) und MUESSEN an einer echten, bekannten Nachricht verifiziert
  werden (siehe Funktion crc_ccitt() und die Anleitung am Dateiende).
- Fuer produktive/robuste Dekodierung ist acarsdec (Open Source, C) die
  Referenz - dieser Code ist zum Verstehen und Selbst-Bauen gedacht.

Getestet mit: RTL-SDR V4 (oder kompatibel), rtlsdr.dll (Windows).
"""

import ctypes
import json
import queue
import struct
import threading
import time
import wave
from datetime import datetime, timezone

from collections import deque

import numpy as np
from scipy.signal import butter, sosfilt, sosfilt_zi, hilbert
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


# ============================================================
# KONFIGURATION
# ============================================================

DLL_PATH = r"C:\Users\Ruber\OneDrive\Desktop\Elektronik\Radar_2\rtlsdr.dll"

CENTER_FREQ = 131_725_000       # ACARS-Kanal (z.B. 131.725 MHz)
IQ_SAMPLE_RATE = 240_000        # muss im gueltigen RTL-SDR Bereich liegen
FREQ_CORRECTION = 60            # ppm, wie in deinem bisherigen Skript

CHUNK_SECONDS = 0.1             # Groesse der pro Lesevorgang verarbeiteten IQ-Bloecke
NUM_IQ_SAMPLES = int(IQ_SAMPLE_RATE * CHUNK_SECONDS)  # 24000, teilbar durch AUDIO_DECIM

AUDIO_DECIM = 5                                  # 240000 / 5 = 48000 Hz Audiotakt
AUDIO_RATE = IQ_SAMPLE_RATE // AUDIO_DECIM
LOWPASS_CUTOFF_HZ = 3000                         # Audiobandbreite (Toene liegen bei 1200/2400 Hz)

BIT_RATE = 2400.0
SAMPLES_PER_BIT = AUDIO_RATE / BIT_RATE          # 20 bei 48 kHz

FREQ_MARK = 1200.0    # "Bitwechsel"
FREQ_SPACE = 2400.0   # "kein Wechsel" / Praeambelton
FREQ_MID = (FREQ_MARK + FREQ_SPACE) / 2.0

PREAMBLE_MIN_SEC = 0.25          # Mindestlaenge des reinen 2400-Hz-Praeambeltons
SQUELCH_RATIO = 5.0              # Faktor ueber Rauschboden, ab dem "Signal da" gilt
SQUELCH_HANGTIME_SEC = 0.05      # wie lange nach Signalende noch mitgeschnitten wird
MAX_BURST_SEC = 1.2              # Sicherheitslimit pro Burst (ACARS: 0.17-0.91 s laut Spezifikation)
MIN_BURST_SEC = 0.15             # kuerzere Bursts sind sicher kein ACARS-Frame

NOISE_FLOOR_WINDOW_SEC = 3.0     # Fenster fuer die gleitende Rauschboden-Schaetzung
NOISE_FLOOR_UPDATE_SEC = 0.5     # wie oft der Median neu berechnet wird
POWER_SMOOTHING_MS = 8.0         # Glaettungszeitkonstante fuer die Squelch-Leistung.
                                  # Ohne Glaettung reicht die statistische Streuung
                                  # einzelner Rauschsamples aus, um die Schwelle
                                  # dauernd zufaellig zu ueberschreiten.

# Manuelle Verstaerkung statt Hardware-AGC verwenden (empfohlen!). Die
# automatische AGC des Tuners aendert die Verstaerkung staendig, was die
# Pegel-basierte Squelch-Erkennung durcheinanderbringen kann. Wert in
# Zehntel-dB, z.B. 400 = 40.0 dB. None = AGC (automatisch) verwenden.
MANUAL_GAIN_TENTH_DB = 400

LOG_FILE = "acars_messages.jsonl"
DEBUG_DUMP_FAILED = True         # Bursts ohne gueltige CRC als WAV speichern (zum Tunen)
DEBUG_DIR = "acars_debug_wav"

# --- Live-Anzeige ---
SPECTRUM_FFT_SIZE = 4096         # Fenstergroesse fuer die Spektrumsanzeige (aus dem IQ-Chunk)
LEVEL_HISTORY_SEC = 12           # wie viele Sekunden Pegelverlauf im unteren Plot sichtbar sind


# ============================================================
# RTL-SDR ANBINDUNG (ctypes) - aufbauend auf deinem Skript
# ============================================================

class RtlSdr:
    def __init__(self, dll_path):
        self.lib = ctypes.CDLL(dll_path)
        self._bind()
        self.dev = ctypes.c_void_p()

    def _bind(self):
        lib = self.lib
        lib.rtlsdr_get_device_count.restype = ctypes.c_uint

        lib.rtlsdr_open.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_uint]
        lib.rtlsdr_open.restype = ctypes.c_int

        lib.rtlsdr_close.argtypes = [ctypes.c_void_p]
        lib.rtlsdr_close.restype = ctypes.c_int

        lib.rtlsdr_set_center_freq.argtypes = [ctypes.c_void_p, ctypes.c_uint]
        lib.rtlsdr_set_center_freq.restype = ctypes.c_int

        lib.rtlsdr_set_sample_rate.argtypes = [ctypes.c_void_p, ctypes.c_uint]
        lib.rtlsdr_set_sample_rate.restype = ctypes.c_int

        lib.rtlsdr_set_freq_correction.argtypes = [ctypes.c_void_p, ctypes.c_int]
        lib.rtlsdr_set_freq_correction.restype = ctypes.c_int

        lib.rtlsdr_set_tuner_gain_mode.argtypes = [ctypes.c_void_p, ctypes.c_int]
        lib.rtlsdr_set_tuner_gain_mode.restype = ctypes.c_int

        lib.rtlsdr_set_tuner_gain.argtypes = [ctypes.c_void_p, ctypes.c_int]
        lib.rtlsdr_set_tuner_gain.restype = ctypes.c_int

        lib.rtlsdr_reset_buffer.argtypes = [ctypes.c_void_p]
        lib.rtlsdr_reset_buffer.restype = ctypes.c_int

        lib.rtlsdr_read_sync.argtypes = [
            ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.c_int)
        ]
        lib.rtlsdr_read_sync.restype = ctypes.c_int

    def open(self):
        if self.lib.rtlsdr_get_device_count() == 0:
            raise RuntimeError("Kein RTL-SDR Geraet gefunden.")
        if self.lib.rtlsdr_open(ctypes.byref(self.dev), 0) != 0:
            raise RuntimeError("RTL-SDR konnte nicht geoeffnet werden.")

    def configure(self, center_freq, sample_rate, freq_correction, manual_gain_tenth_db=None):
        if self.lib.rtlsdr_set_center_freq(self.dev, center_freq) != 0:
            raise RuntimeError("Center frequency konnte nicht gesetzt werden.")
        if self.lib.rtlsdr_set_sample_rate(self.dev, sample_rate) != 0:
            raise RuntimeError("Sample rate konnte nicht gesetzt werden.")
        if self.lib.rtlsdr_set_freq_correction(self.dev, freq_correction) != 0:
            # nicht fatal auf allen Geraeten, daher nur Warnung
            print("Warnung: Frequenzkorrektur konnte nicht gesetzt werden.")

        if manual_gain_tenth_db is None:
            self.lib.rtlsdr_set_tuner_gain_mode(self.dev, 0)  # 0 = Auto-Gain (AGC)
        else:
            self.lib.rtlsdr_set_tuner_gain_mode(self.dev, 1)  # 1 = manueller Modus
            result = self.lib.rtlsdr_set_tuner_gain(self.dev, manual_gain_tenth_db)
            if result != 0:
                print(f"Warnung: manuelle Verstaerkung {manual_gain_tenth_db/10:.1f} dB "
                      f"konnte nicht gesetzt werden, bleibe bei AGC.")

        self.lib.rtlsdr_reset_buffer(self.dev)

    def read_iq(self, num_samples):
        num_bytes = num_samples * 2
        buf = (ctypes.c_ubyte * num_bytes)()
        num_read = ctypes.c_int()
        result = self.lib.rtlsdr_read_sync(self.dev, buf, num_bytes, ctypes.byref(num_read))
        if result != 0 or num_read.value < 2:
            return None
        raw = np.frombuffer(buf, dtype=np.uint8, count=num_read.value)
        i = raw[0::2].astype(np.float32) - 127.5
        q = raw[1::2].astype(np.float32) - 127.5
        return i, q

    def close(self):
        self.lib.rtlsdr_close(self.dev)


# ============================================================
# AM-DEMODULATION UND AUDIO-FILTERUNG (mit Filterzustand ueber Chunks hinweg)
# ============================================================

class AudioFrontend:
    """Envelope-Demod -> Tiefpass -> Dezimation, Filterzustand bleibt zwischen
    aufeinanderfolgenden Chunks erhalten (keine Kloeter/Truebung an Chunk-Grenzen)."""

    def __init__(self, iq_rate, decim, cutoff_hz):
        self.decim = decim
        self.sos = butter(4, cutoff_hz, btype="low", fs=iq_rate, output="sos")
        self.zi = sosfilt_zi(self.sos)[:, None] * 0.0
        # sosfilt_zi liefert Form (n_sections, 2); wir brauchen (n_sections, 2)
        self.zi = sosfilt_zi(self.sos)

    def process(self, i, q):
        envelope = np.sqrt(i * i + q * q).astype(np.float64)
        filtered, self.zi = sosfilt(self.sos, envelope, zi=self.zi)
        audio = filtered[:: self.decim]
        return audio


# ============================================================
# SQUELCH / BURST-ERKENNUNG
# ============================================================

class SquelchBurstCapture:
    """Erkennt Signalbursts anhand der Audio-Leistung und liefert vollstaendige
    Audio-Segmente (numpy arrays) zur weiteren Dekodierung.

    Der Rauschboden wird als gleitender MEDIAN ueber ein paar Sekunden
    Leistungswerte geschaetzt - unabhaengig davon, ob gerade ein Burst laeuft.
    Das ist absichtlich robuster als eine waehrend eines Bursts eingefrorene
    Mittelwertschaetzung: Solange echte Signale in der Minderheit der Zeit
    auftreten (bei ACARS der Normalfall), kann sich die Schwelle nie
    dauerhaft "festfressen" - auch nicht nach einem falschen Trigger direkt
    beim Programmstart (z.B. waehrend die Tuner-AGC noch einpendelt).
    """

    def __init__(self, audio_rate):
        self.audio_rate = audio_rate
        window_len = int(NOISE_FLOOR_WINDOW_SEC * audio_rate)
        self.power_history = deque(maxlen=window_len)
        self.noise_floor = None
        self.samples_since_update = 0
        self.update_every = int(NOISE_FLOOR_UPDATE_SEC * audio_rate)

        # Glaettung der Momentanleistung (einfaches Einpol-IIR) VOR der
        # Squelch-Entscheidung. tau = POWER_SMOOTHING_MS.
        tau_samples = max(1.0, POWER_SMOOTHING_MS / 1000.0 * audio_rate)
        self.smooth_alpha = 1.0 - np.exp(-1.0 / tau_samples)
        self.smoothed_power = None

        self.in_burst = False
        self.burst_samples = []
        self.silence_run = 0
        self.hang_samples = int(SQUELCH_HANGTIME_SEC * audio_rate)
        self.max_burst_samples = int(MAX_BURST_SEC * audio_rate)

    def _maybe_update_noise_floor(self):
        min_needed = int(0.2 * self.audio_rate)
        if len(self.power_history) < min_needed:
            return
        self.noise_floor = float(np.median(self.power_history))

    def push(self, audio_chunk):
        bursts = []
        raw_power = audio_chunk.astype(np.float64) ** 2

        for idx in range(len(raw_power)):
            raw_p = raw_power[idx]

            if self.smoothed_power is None:
                self.smoothed_power = raw_p
            else:
                self.smoothed_power = (
                    (1 - self.smooth_alpha) * self.smoothed_power + self.smooth_alpha * raw_p
                )
            p = self.smoothed_power  # ab hier: geglaettete Leistung verwenden

            self.power_history.append(p)
            self.samples_since_update += 1
            if self.noise_floor is None or self.samples_since_update >= self.update_every:
                self._maybe_update_noise_floor()
                self.samples_since_update = 0

            threshold = max((self.noise_floor or p) * SQUELCH_RATIO, 1e-9)
            above = p > threshold

            if above:
                if not self.in_burst:
                    self.in_burst = True
                    self.burst_samples = []
                self.burst_samples.append(audio_chunk[idx])
                self.silence_run = 0
            elif self.in_burst:
                self.burst_samples.append(audio_chunk[idx])  # Hangtime mitschneiden
                self.silence_run += 1
                if self.silence_run >= self.hang_samples:
                    bursts.append(np.array(self.burst_samples, dtype=np.float64))
                    self.in_burst = False
                    self.burst_samples = []
                    self.silence_run = 0

            if self.in_burst and len(self.burst_samples) >= self.max_burst_samples:
                bursts.append(np.array(self.burst_samples, dtype=np.float64))
                self.in_burst = False
                self.burst_samples = []
                self.silence_run = 0

        return bursts


# ============================================================
# FREQUENZ-DISKRIMINATOR + BIT-EXTRAKTION
# ============================================================

def instantaneous_frequency(audio, fs):
    analytic = hilbert(audio)
    phase = np.unwrap(np.angle(analytic))
    freq = np.diff(phase) * fs / (2.0 * np.pi)
    return freq


def find_preamble_end(freq, fs):
    """Sucht das Ende des durchgehenden 2400-Hz-Praeambeltons.
    Rueckgabe: Sample-Index, ab dem das Bit-Sampling beginnen soll, oder None."""

    near_space = np.abs(freq - FREQ_SPACE) < (FREQ_SPACE - FREQ_MID) * 0.6
    min_run = int(PREAMBLE_MIN_SEC * fs)

    run_start = None
    run_len = 0
    for idx, ok in enumerate(near_space):
        if ok:
            if run_start is None:
                run_start = idx
            run_len += 1
        else:
            if run_len >= min_run:
                return idx  # erstes Sample NACH der stabilen Praeambel
            run_start = None
            run_len = 0

    if run_len >= min_run:
        return len(near_space)  # Praeambel geht bis Ende (unwahrscheinlich nuetzlich)

    return None


def bits_from_frequency(freq, start_index):
    """Tastet die Momentanfrequenz alle SAMPLES_PER_BIT Samples ab und
    wendet die NRZI-Regel an: nahe FREQ_MARK -> Bit wechselt, sonst gleich."""

    bits = []
    bit = 1  # Praeambel bestand aus lauter 1en, direkt davor war das letzte Bit "1"
    pos = float(start_index)

    while int(round(pos)) < len(freq):
        sample_idx = int(round(pos))
        f = freq[sample_idx]
        if abs(f - FREQ_MARK) < abs(f - FREQ_SPACE):
            bit = 1 - bit  # Wechsel
        # sonst: Bit bleibt gleich
        bits.append(bit)
        pos += SAMPLES_PER_BIT

    return bits


# ============================================================
# BYTE-FRAMING, PARITAET, CRC, FELD-PARSING
# ============================================================

SYN = 0x16
SOH = 0x01
STX = 0x02
ETX = 0x03
ETB = 0x17
DEL = 0x7F


def bits_to_bytes_lsb_first(bits):
    """Gruppiert Bits zu 8er-Bloecken (LSB zuerst) -> Liste von rohen Bytes
    (inklusive Paritaetsbit, noch ungeprueft)."""
    bytes_out = []
    for i in range(0, len(bits) - 7, 8):
        chunk = bits[i : i + 8]
        value = 0
        for bit_index, bit in enumerate(chunk):  # LSB zuerst
            value |= (bit & 1) << bit_index
        bytes_out.append(value)
    return bytes_out


def strip_parity_odd(byte_val):
    """Prueft ungerade Paritaet (Bit 7) und liefert die unteren 7 Bit zurueck.
    Gibt None zurueck, wenn die Paritaet nicht passt."""
    data7 = byte_val & 0x7F
    parity_bit = (byte_val >> 7) & 1
    ones = bin(data7).count("1") + parity_bit
    if ones % 2 == 1:  # ungerade Paritaet korrekt
        return data7
    return None


def find_sync_offset(raw_bytes):
    """Sucht das Muster SYN SYN SOH in der (paritaetsbereinigten) Byte-Folge.
    Gibt den Index NACH dem SOH zurueck, oder None."""
    for i in range(len(raw_bytes) - 2):
        a = strip_parity_odd(raw_bytes[i])
        b = strip_parity_odd(raw_bytes[i + 1])
        c = strip_parity_odd(raw_bytes[i + 2])
        if a == SYN and b == SYN and c == SOH:
            return i + 3
    return None


def crc_ccitt(data: bytes) -> int:
    """CRC-CCITT (x^16+x^12+x^5+1), reflektierte Variante (Poly 0x8408),
    LSB-first, Init 0xFFFF.

    ACHTUNG: Init-Wert / Reflektion / evtl. Byte-Reihenfolge des BCS-Felds
    sind hier eine gaengige Annahme und MUESSEN gegen eine echte Nachricht
    mit bekanntem BCS verifiziert werden (siehe Anleitung am Dateiende).
    """
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0x8408
            else:
                crc >>= 1
    return crc & 0xFFFF


def parse_acars_frame(raw_bytes, sync_offset):
    """Extrahiert Zeichen bis ETX/ETB, dann 2 Byte BCS, prueft CRC.
    Gibt ein dict mit den Rohdaten zurueck (auch bei fehlgeschlagener CRC,
    damit man zum Tunen nachschauen kann)."""

    chars = []
    i = sync_offset
    end_marker = None

    while i < len(raw_bytes):
        val = strip_parity_odd(raw_bytes[i])
        i += 1
        if val is None:
            return {"ok": False, "reason": "parity_error", "partial_text": "".join(chars)}
        if val in (ETX, ETB):
            end_marker = val
            break
        chars.append(chr(val) if 32 <= val < 127 or val in (13, 10) else f"\\x{val:02x}")

    if end_marker is None:
        return {"ok": False, "reason": "no_end_marker", "partial_text": "".join(chars)}

    if i + 1 >= len(raw_bytes):
        return {"ok": False, "reason": "truncated_before_bcs", "partial_text": "".join(chars)}

    bcs_bytes = bytes([raw_bytes[i] & 0xFF, raw_bytes[i + 1] & 0xFF])
    received_bcs = bcs_bytes[0] | (bcs_bytes[1] << 8)

    # BCS wird ueber Nachricht INKLUSIVE Endezeichen, ohne die fuehrenden
    # SYN/SYN/SOH berechnet (ARINC 618) - ggf. beim Tunen anpassen.
    payload = bytes(
        (strip_parity_odd(b) if strip_parity_odd(b) is not None else 0)
        for b in raw_bytes[sync_offset - 1 : i]  # inkl. SOH bis inkl. ETX/ETB
    )
    calculated = crc_ccitt(payload)

    return {
        "ok": calculated == received_bcs,
        "reason": "crc_ok" if calculated == received_bcs else "crc_mismatch",
        "text": "".join(chars),
        "received_bcs": received_bcs,
        "calculated_bcs": calculated,
    }


# ============================================================
# BURST -> VOLLSTAENDIGE DEKODIERUNG
# ============================================================

def decode_burst(audio_burst, fs):
    if len(audio_burst) < int(MIN_BURST_SEC * fs):
        return None

    freq = instantaneous_frequency(audio_burst, fs)
    start = find_preamble_end(freq, fs)
    if start is None:
        return {"ok": False, "reason": "no_preamble"}

    bits = bits_from_frequency(freq, start)
    raw_bytes = bits_to_bytes_lsb_first(bits)

    sync_offset = find_sync_offset(raw_bytes)
    if sync_offset is None:
        return {"ok": False, "reason": "no_sync_pattern"}

    result = parse_acars_frame(raw_bytes, sync_offset)
    return result


# ============================================================
# LOGGING
# ============================================================

def log_message(result):
    entry = dict(result)
    entry["timestamp_utc"] = datetime.now(timezone.utc).isoformat()
    with open(LOG_FILE, "a", encoding="utf-8") as f:
        f.write(json.dumps(entry, ensure_ascii=False) + "\n")
    if entry.get("ok"):
        print(f"[{entry['timestamp_utc']}] ACARS OK: {entry.get('text','')!r}")
    else:
        print(f"[{entry['timestamp_utc']}] Dekodierung fehlgeschlagen: {entry.get('reason')}")


def dump_debug_wav(audio_burst, fs, reason):
    import os

    os.makedirs(DEBUG_DIR, exist_ok=True)
    fname = os.path.join(
        DEBUG_DIR, f"burst_{reason}_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}.wav"
    )
    audio_i16 = np.clip(audio_burst / (np.max(np.abs(audio_burst)) + 1e-9) * 32767, -32768, 32767).astype(np.int16)
    with wave.open(fname, "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(int(fs))
        wf.writeframes(audio_i16.tobytes())


# ============================================================
# HAUPTPROGRAMM
# ============================================================

def main():
    sdr = RtlSdr(DLL_PATH)
    sdr.open()
    sdr.configure(CENTER_FREQ, IQ_SAMPLE_RATE, FREQ_CORRECTION,
                  manual_gain_tenth_db=MANUAL_GAIN_TENTH_DB)

    print(f"Empfange auf {CENTER_FREQ/1e6:.3f} MHz, {IQ_SAMPLE_RATE/1e6:.3f} MSps IQ, "
          f"{AUDIO_RATE} Hz Audio, {SAMPLES_PER_BIT:.1f} Samples/Bit.")
    print("Fenster schliessen oder Strg+C zum Beenden.\n")

    frontend = AudioFrontend(IQ_SAMPLE_RATE, AUDIO_DECIM, LOWPASS_CUTOFF_HZ)
    squelch = SquelchBurstCapture(AUDIO_RATE)

    # ---------------- Live-Diagramme vorbereiten ----------------
    spectrum_window = np.hanning(SPECTRUM_FFT_SIZE)
    freq_axis_mhz = (
        CENTER_FREQ + np.linspace(-IQ_SAMPLE_RATE / 2, IQ_SAMPLE_RATE / 2, SPECTRUM_FFT_SIZE)
    ) / 1e6

    max_points = max(2, int(LEVEL_HISTORY_SEC / CHUNK_SECONDS))
    level_times = deque(maxlen=max_points)
    level_values = deque(maxlen=max_points)
    threshold_values = deque(maxlen=max_points)

    fig, (ax_spec, ax_level) = plt.subplots(2, 1, figsize=(11, 7))
    fig.tight_layout(pad=3.0)

    (spec_line,) = ax_spec.plot(freq_axis_mhz, np.zeros(SPECTRUM_FFT_SIZE))
    ax_spec.set_xlabel("Frequenz (MHz)")
    ax_spec.set_ylabel("Pegel (dB)")
    ax_spec.set_title(f"Abgetastetes Spektrum um {CENTER_FREQ/1e6:.3f} MHz")

    (level_line,) = ax_level.plot([], [], label="Audiopegel (geglaettet)")
    (thresh_line,) = ax_level.plot([], [], "--", label="Squelch-Schwelle")
    ax_level.set_xlabel("Zeit (s)")
    ax_level.set_ylabel("Leistung (log)")
    ax_level.set_yscale("log")
    ax_level.legend(loc="upper right")
    status_text = ax_level.text(
        0.01, 0.95, "", transform=ax_level.transAxes, va="top",
        fontfamily="monospace", fontsize=9,
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.8),
    )

    t0 = time.time()

    def update(frame):
        iq = sdr.read_iq(NUM_IQ_SAMPLES)
        if iq is None:
            return spec_line, level_line, thresh_line, status_text
        i, q = iq

        # --- Spektrum aktualisieren ---
        iq_complex = (i[-SPECTRUM_FFT_SIZE:] + 1j * q[-SPECTRUM_FFT_SIZE:]) * spectrum_window
        spectrum = np.fft.fftshift(np.fft.fft(iq_complex, SPECTRUM_FFT_SIZE))
        power_db = 20 * np.log10(np.abs(spectrum) + 1e-9)
        spec_line.set_ydata(power_db)
        # Y-Achse dynamisch nachfuehren, statt sie wie im urspruenglichen
        # Wasserfall-Skript ein fuer alle Mal einzufrieren.
        lo, hi = np.percentile(power_db, [1, 99])
        ax_spec.set_ylim(lo - 5, hi + 10)

        # --- ACARS-Pipeline (wie bisher) ---
        audio_chunk = frontend.process(i, q)
        bursts = squelch.push(audio_chunk)

        for burst in bursts:
            result = decode_burst(burst, AUDIO_RATE)
            if result is None:
                continue
            log_message(result)
            if DEBUG_DUMP_FAILED and not result.get("ok"):
                dump_debug_wav(burst, AUDIO_RATE, result.get("reason", "unknown"))

        # --- Pegel-Diagramm aktualisieren ---
        t = time.time() - t0
        noise_floor = squelch.noise_floor or 1e-9
        smoothed = squelch.smoothed_power or 1e-9

        level_times.append(t)
        level_values.append(smoothed)
        threshold_values.append(noise_floor * SQUELCH_RATIO)

        level_line.set_data(level_times, level_values)
        thresh_line.set_data(level_times, threshold_values)

        if len(level_times) > 1:
            ax_level.set_xlim(level_times[0], level_times[-1])
            all_vals = list(level_values) + list(threshold_values)
            ax_level.set_ylim(max(1e-12, min(all_vals) * 0.5), max(all_vals) * 3)

        state = "SIGNAL - Burst wird aufgezeichnet" if squelch.in_burst else "ruhig"
        if squelch.noise_floor is None:
            status_text.set_text("Status: Rauschboden wird kalibriert...")
        else:
            status_text.set_text(
                f"Status:        {state}\n"
                f"Rauschboden:   {noise_floor:.3e}\n"
                f"Schwelle:      {noise_floor*SQUELCH_RATIO:.3e}\n"
                f"Pegel jetzt:   {smoothed:.3e}"
            )

        return spec_line, level_line, thresh_line, status_text

    ani = FuncAnimation(
        fig, update, interval=int(CHUNK_SECONDS * 1000), blit=False, cache_frame_data=False
    )

    try:
        plt.show()
    except KeyboardInterrupt:
        print("\nBeende...")
    finally:
        sdr.close()
        print("RTL-SDR geschlossen.")


if __name__ == "__main__":
    main()


# ============================================================
# ANLEITUNG: CRC/BCS-PARAMETER VERIFIZIEREN
# ============================================================
#
# 1. Sammle mit DEBUG_DUMP_FAILED=True ein paar WAV-Schnipsel echter Bursts,
#    bei denen "text" plausibel aussieht (lesbarer ACARS-Inhalt), aber
#    reason == "crc_mismatch" ist.
# 2. Nimm eine dieser Nachrichten und vergleiche mit einer Referenz
#    (z.B. Ausgabe von acarsdec fuer denselben Burst, oder ein bekanntes
#    Beispiel aus der Literatur) um Klartext + tatsaechliche BCS-Bytes
#    zu bekommen.
# 3. Probiere in crc_ccitt() systematisch:
#       - Polynom 0x1021 (nicht reflektiert) statt 0x8408 (reflektiert)
#       - Init-Wert 0x0000 statt 0xFFFF
#       - Payload-Grenzen in parse_acars_frame() (mit/ohne SOH, mit/ohne
#         Endezeichen, ab STX statt ab SOH)
#    bis calculated_bcs == received_bcs fuer mehrere Nachrichten passt.
# 4. Erst danach ist die CRC-Pruefung zuverlaessig - bis dahin kannst du
#    "ok" ignorieren und dich am reinen Klartext ("text"-Feld) orientieren,
#    um zu sehen, ob die Bit-Extraktion grundsaetzlich funktioniert.