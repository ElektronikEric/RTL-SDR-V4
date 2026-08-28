#!/usr/bin/env python3
"""
Modularer ACARS-Viewer (Startversion) für RTL-SDR v4 unter Windows via ctypes/rtlsdr.dll.

Funktionen:
- Liest IQ-Samples direkt aus rtlsdr.dll (synchron)
- Tunt auf ACARS-Frequenzen (standardmäßig 131.725 MHz)
- Demoduliert AM und filtert den Audiobereich
- Versucht eine einfache ACARS-Bit/Frame-Rekonstruktion (2400 bps BPSK-ähnlicher Baseband-Ansatz)
- Gibt decodierte ASCII-Fragmente und Metadaten im Terminal aus
- Optional: Live-Audio-Spektrum

Wichtig:
- Diese Version ist eine modulare Grundlage.
- Für maximale Decode-Rate: Timing-Recovery, besseres Carrier/Clock-Recovery und robustere Sync-Logik erweitern.
"""

from __future__ import annotations

import argparse
import ctypes
import math
import queue
import signal
import sys
import threading
import time
from dataclasses import dataclass
from typing import Iterable, Optional, List, Tuple

import numpy as np

try:
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    HAS_MPL = True
except Exception:
    HAS_MPL = False


# ============================================================================
# Konfiguration / Datenmodelle
# ============================================================================

@dataclass(slots=True)
class RadioConfig:
    dll_path: str
    device_index: int = 0
    center_freq_hz: int = 131_725_000
    sample_rate_hz: int = 1_024_000
    ppm: int = 0
    gain_mode_auto: bool = True
    agc_on: bool = True
    block_iq_samples: int = 16384  # complex samples per read


@dataclass(slots=True)
class AcarsConfig:
    # ACARS: 2400 bit/s
    bitrate: int = 2400
    audio_rate_hz: int = 48_000
    bp_low_hz: float = 1000.0
    bp_high_hz: float = 4000.0
    squelch_db: float = -45.0

    # Vereinfachte Frame-Erkennung
    min_frame_bytes: int = 12
    max_frame_bytes: int = 300


@dataclass(slots=True)
class DecodeMessage:
    timestamp: float
    freq_hz: int
    text: str
    raw_bytes_hex: str
    confidence: float


# ============================================================================
# RTL-SDR ctypes Wrapper
# ============================================================================

class RtlSdrLib:
    def __init__(self, dll_path: str):
        self.lib = ctypes.CDLL(dll_path)
        self._setup_signatures()

    def _setup_signatures(self):
        p_dev = ctypes.c_void_p

        self.lib.rtlsdr_get_device_count.restype = ctypes.c_uint
        self.lib.rtlsdr_get_device_count.argtypes = []

        self.lib.rtlsdr_open.restype = ctypes.c_int
        self.lib.rtlsdr_open.argtypes = [ctypes.POINTER(p_dev), ctypes.c_uint]

        self.lib.rtlsdr_close.restype = ctypes.c_int
        self.lib.rtlsdr_close.argtypes = [p_dev]

        self.lib.rtlsdr_set_sample_rate.restype = ctypes.c_int
        self.lib.rtlsdr_set_sample_rate.argtypes = [p_dev, ctypes.c_uint]

        self.lib.rtlsdr_set_center_freq.restype = ctypes.c_int
        self.lib.rtlsdr_set_center_freq.argtypes = [p_dev, ctypes.c_uint]

        self.lib.rtlsdr_set_freq_correction.restype = ctypes.c_int
        self.lib.rtlsdr_set_freq_correction.argtypes = [p_dev, ctypes.c_int]

        self.lib.rtlsdr_set_tuner_gain_mode.restype = ctypes.c_int
        self.lib.rtlsdr_set_tuner_gain_mode.argtypes = [p_dev, ctypes.c_int]

        self.lib.rtlsdr_set_agc_mode.restype = ctypes.c_int
        self.lib.rtlsdr_set_agc_mode.argtypes = [p_dev, ctypes.c_int]

        self.lib.rtlsdr_reset_buffer.restype = ctypes.c_int
        self.lib.rtlsdr_reset_buffer.argtypes = [p_dev]

        self.lib.rtlsdr_read_sync.restype = ctypes.c_int
        self.lib.rtlsdr_read_sync.argtypes = [
            p_dev,
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.POINTER(ctypes.c_int),
        ]


class RtlSdrSource:
    def __init__(self, cfg: RadioConfig):
        self.cfg = cfg
        self._lib = RtlSdrLib(cfg.dll_path).lib
        self._dev = ctypes.c_void_p(None)
        self._opened = False

        self._num_bytes = cfg.block_iq_samples * 2
        self._buf = (ctypes.c_ubyte * self._num_bytes)()
        self._n_read = ctypes.c_int(0)

    def open(self):
        if self._opened:
            return

        if self._lib.rtlsdr_get_device_count() == 0:
            raise RuntimeError("Kein RTL-SDR gefunden.")

        rc = self._lib.rtlsdr_open(ctypes.byref(self._dev), self.cfg.device_index)
        if rc != 0:
            raise RuntimeError(f"rtlsdr_open fehlgeschlagen: rc={rc}")

        self._check(self._lib.rtlsdr_set_sample_rate(self._dev, self.cfg.sample_rate_hz), "set_sample_rate")
        self._check(self._lib.rtlsdr_set_center_freq(self._dev, self.cfg.center_freq_hz), "set_center_freq")
        self._check(self._lib.rtlsdr_set_freq_correction(self._dev, self.cfg.ppm), "set_freq_correction")
        self._check(self._lib.rtlsdr_set_tuner_gain_mode(self._dev, 0 if self.cfg.gain_mode_auto else 1), "set_tuner_gain_mode")
        self._check(self._lib.rtlsdr_set_agc_mode(self._dev, 1 if self.cfg.agc_on else 0), "set_agc_mode")
        self._check(self._lib.rtlsdr_reset_buffer(self._dev), "reset_buffer")

        # First block verwerfen
        self._lib.rtlsdr_read_sync(self._dev, self._buf, self._num_bytes, ctypes.byref(self._n_read))
        self._opened = True

    def _check(self, rc: int, name: str):
        if rc != 0:
            raise RuntimeError(f"{name} fehlgeschlagen: rc={rc}")

    def read_iq(self) -> np.ndarray:
        if not self._opened:
            raise RuntimeError("SDR nicht geöffnet.")

        rc = self._lib.rtlsdr_read_sync(self._dev, self._buf, self._num_bytes, ctypes.byref(self._n_read))
        if rc != 0:
            raise RuntimeError(f"rtlsdr_read_sync fehlgeschlagen: rc={rc}")

        n = int(self._n_read.value)
        if n <= 0:
            return np.zeros(self.cfg.block_iq_samples, dtype=np.complex64)

        raw = np.frombuffer(self._buf, dtype=np.uint8, count=n).astype(np.float32)
        # interleaved I,Q unsigned byte -> complex centered at 0
        i = raw[0::2] / 127.5 - 1.0
        q = raw[1::2] / 127.5 - 1.0
        iq = (i + 1j * q).astype(np.complex64)
        return iq

    def close(self):
        if self._opened:
            self._lib.rtlsdr_close(self._dev)
            self._opened = False

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()


# ============================================================================
# DSP Utilities
# ============================================================================

def lowpass_fir(cutoff_hz: float, fs_hz: float, num_taps: int = 129) -> np.ndarray:
    """Fenster-Sinc Lowpass."""
    if num_taps % 2 == 0:
        num_taps += 1
    n = np.arange(num_taps) - (num_taps - 1) / 2
    fc = cutoff_hz / fs_hz
    h = 2 * fc * np.sinc(2 * fc * n)
    h *= np.hamming(num_taps)
    h /= np.sum(h)
    return h.astype(np.float32)


def bandpass_fir(low_hz: float, high_hz: float, fs_hz: float, num_taps: int = 257) -> np.ndarray:
    """Bandpass via LP(high)-LP(low)."""
    h_high = lowpass_fir(high_hz, fs_hz, num_taps)
    h_low = lowpass_fir(low_hz, fs_hz, num_taps)
    h = h_high - h_low
    return h.astype(np.float32)


class AcarsDemodulator:
    """
    Einfache Pipeline:
      IQ -> AM envelope -> DC remove -> decimate -> bandpass(1..4kHz)
    """
    def __init__(self, radio_cfg: RadioConfig, acars_cfg: AcarsConfig):
        self.radio_cfg = radio_cfg
        self.acars_cfg = acars_cfg

        self.decim = max(1, int(round(radio_cfg.sample_rate_hz / acars_cfg.audio_rate_hz)))
        self.out_fs = radio_cfg.sample_rate_hz / self.decim

        # Vor-Filter für Decimation
        anti_alias_cut = min(0.45 * self.out_fs, 0.45 * radio_cfg.sample_rate_hz / self.decim)
        self.h_lpf = lowpass_fir(anti_alias_cut, radio_cfg.sample_rate_hz, 129)
        self.h_bp = bandpass_fir(acars_cfg.bp_low_hz, acars_cfg.bp_high_hz, self.out_fs, 257)

        self._zi_lpf = np.zeros(len(self.h_lpf) - 1, dtype=np.float32)
        self._zi_bp = np.zeros(len(self.h_bp) - 1, dtype=np.float32)

    @staticmethod
    def _fir_filter(x: np.ndarray, h: np.ndarray, zi: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        # simple overlap-save style stateful FIR via convolution on prepended state
        x_ext = np.concatenate([zi, x.astype(np.float32)])
        y = np.convolve(x_ext, h, mode="valid")
        new_zi = x_ext[-(len(h) - 1):]
        return y.astype(np.float32), new_zi.astype(np.float32)

    def process(self, iq: np.ndarray) -> np.ndarray:
        # AM envelope
        env = np.abs(iq).astype(np.float32)

        # DC removal
        env -= np.mean(env)

        # Anti-alias + decimate
        y_lpf, self._zi_lpf = self._fir_filter(env, self.h_lpf, self._zi_lpf)
        audio = y_lpf[::self.decim]

        # Bandpass ACARS audio band
        y_bp, self._zi_bp = self._fir_filter(audio, self.h_bp, self._zi_bp)

        # normalize
        std = float(np.std(y_bp))
        if std > 1e-9:
            y_bp = y_bp / std

        return y_bp


# ============================================================================
# ACARS Decoder (Start/heuristisch)
# ============================================================================

class AcarsDecoder:
    """
    Vereinfachter, modularer Startdecoder.
    Strategie:
    - Symbolentscheidung via integrate-and-dump bei 2400 bps
    - Framing heuristisch über druckbaren ASCII-Anteil
    """

    def __init__(self, cfg: AcarsConfig):
        self.cfg = cfg
        self.sps = int(round(cfg.audio_rate_hz / cfg.bitrate))
        self._bit_buffer: List[int] = []

    def _audio_power_db(self, x: np.ndarray) -> float:
        p = float(np.mean(x * x)) + 1e-12
        return 10.0 * math.log10(p)

    def demod_bits(self, audio: np.ndarray) -> List[int]:
        # Sehr einfache binäre Symbolentscheidung (Vorzeichen nach Integrator)
        bits = []
        n = len(audio)
        step = self.sps
        for i in range(0, n - step, step):
            sym = audio[i:i + step]
            v = float(np.sum(sym))
            bits.append(1 if v >= 0 else 0)
        return bits

    @staticmethod
    def bits_to_bytes(bits: List[int]) -> bytes:
        out = bytearray()
        for i in range(0, len(bits) - 7, 8):
            b = 0
            for k in range(8):
                b = (b << 1) | (bits[i + k] & 1)
            out.append(b)
        return bytes(out)

    @staticmethod
    def printable_ratio(data: bytes) -> float:
        if not data:
            return 0.0
        good = 0
        for c in data:
            if c in (0x0A, 0x0D, 0x09) or (32 <= c <= 126):
                good += 1
        return good / len(data)

    def _extract_ascii_candidates(self, data: bytes) -> List[Tuple[str, bytes, float]]:
        """
        Sucht Fenster mit hohem Printable-Anteil als Frame-Kandidat.
        """
        results = []
        min_b = self.cfg.min_frame_bytes
        max_b = self.cfg.max_frame_bytes

        L = len(data)
        if L < min_b:
            return results

        # Sliding windows grob
        for start in range(0, max(1, L - min_b), 8):
            for size in (min_b, 24, 40, 64, 96, 128, 180, 240):
                if size > max_b:
                    continue
                end = start + size
                if end > L:
                    continue
                chunk = data[start:end]
                ratio = self.printable_ratio(chunk)
                if ratio >= 0.70:
                    text = chunk.decode("ascii", errors="ignore").strip()
                    if len(text) >= 8:
                        results.append((text, chunk, ratio))
        return results

    def feed(self, audio: np.ndarray, freq_hz: int) -> List[DecodeMessage]:
        msgs: List[DecodeMessage] = []

        if self._audio_power_db(audio) < self.cfg.squelch_db:
            return msgs

        new_bits = self.demod_bits(audio)
        if not new_bits:
            return msgs

        self._bit_buffer.extend(new_bits)

        # Buffer begrenzen
        max_bits = self.cfg.max_frame_bytes * 8 * 8
        if len(self._bit_buffer) > max_bits:
            self._bit_buffer = self._bit_buffer[-max_bits:]

        data = self.bits_to_bytes(self._bit_buffer)
        cands = self._extract_ascii_candidates(data)

        # Nur die besten 2 Kandidaten pro Feed
        cands.sort(key=lambda x: x[2], reverse=True)
        for text, raw, conf in cands[:2]:
            msgs.append(
                DecodeMessage(
                    timestamp=time.time(),
                    freq_hz=freq_hz,
                    text=text,
                    raw_bytes_hex=raw[:80].hex(),
                    confidence=float(conf),
                )
            )

        # Optional: bei Treffer etwas Puffer abschneiden
        if msgs and len(self._bit_buffer) > 2000:
            self._bit_buffer = self._bit_buffer[-1200:]

        return msgs


# ============================================================================
# Viewer / Orchestrierung
# ============================================================================

class AcarsViewerApp:
    def __init__(self, radio_cfg: RadioConfig, acars_cfg: AcarsConfig, show_plot: bool):
        self.radio_cfg = radio_cfg
        self.acars_cfg = acars_cfg
        self.show_plot = show_plot and HAS_MPL
        self.running = True

        self.demod = AcarsDemodulator(radio_cfg, acars_cfg)
        self.decoder = AcarsDecoder(acars_cfg)

        self.msg_queue: "queue.Queue[DecodeMessage]" = queue.Queue()
        self.audio_queue: "queue.Queue[np.ndarray]" = queue.Queue(maxsize=10)

    def stop(self, *_):
        self.running = False

    def run(self):
        signal.signal(signal.SIGINT, self.stop)

        worker = threading.Thread(target=self._worker_loop, daemon=True)
        worker.start()

        if self.show_plot:
            self._run_plot_loop()
        else:
            self._run_terminal_loop()

        self.running = False
        worker.join(timeout=1.0)

    def _worker_loop(self):
        try:
            with RtlSdrSource(self.radio_cfg) as src:
                print(
                    f"[INFO] SDR offen | f={self.radio_cfg.center_freq_hz/1e6:.3f} MHz, "
                    f"sr={self.radio_cfg.sample_rate_hz/1e6:.3f} MS/s"
                )
                while self.running:
                    iq = src.read_iq()
                    audio = self.demod.process(iq)

                    # Decoder
                    msgs = self.decoder.feed(audio, self.radio_cfg.center_freq_hz)
                    for m in msgs:
                        self.msg_queue.put(m)

                    # Für Plot
                    if self.show_plot:
                        if self.audio_queue.full():
                            try:
                                self.audio_queue.get_nowait()
                            except queue.Empty:
                                pass
                        self.audio_queue.put(audio)

        except Exception as e:
            print(f"[ERROR] Worker beendet: {e}", file=sys.stderr)
            self.running = False

    def _run_terminal_loop(self):
        print("[INFO] ACARS-Viewer läuft. Strg+C zum Beenden.\n")
        while self.running:
            try:
                msg = self.msg_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            ts = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(msg.timestamp))
            print(
                f"[{ts}] {msg.freq_hz/1e6:.3f} MHz | conf={msg.confidence:.2f}\n"
                f"  TEXT: {msg.text}\n"
                f"  RAW : {msg.raw_bytes_hex}\n"
            )

    def _run_plot_loop(self):
        print("[INFO] ACARS-Viewer + Plot läuft. Fenster schließen oder Strg+C.")

        fig, ax = plt.subplots(figsize=(12, 4))
        x = np.linspace(0, self.acars_cfg.audio_rate_hz / 2, 1024)
        y0 = np.full_like(x, -80.0)
        line, = ax.plot(x, y0, color="#00d5ff", lw=1.0)
        ax.set_title("Demoduliertes Audio-Spektrum (ACARS-Band)")
        ax.set_xlabel("Hz")
        ax.set_ylabel("dB")
        ax.set_ylim(-90, 40)
        ax.grid(True, alpha=0.3)

        text_box = ax.text(0.01, 0.96, "", transform=ax.transAxes, va="top", fontsize=9)

        def update(_):
            if not self.running:
                return line, text_box

            latest_audio = None
            while True:
                try:
                    latest_audio = self.audio_queue.get_nowait()
                except queue.Empty:
                    break

            if latest_audio is not None and len(latest_audio) >= 2048:
                N = 2048
                w = np.hanning(N)
                spec = np.fft.rfft(latest_audio[:N] * w)
                psd = 20.0 * np.log10(np.abs(spec) + 1e-9)
                fx = np.fft.rfftfreq(N, d=1.0 / self.acars_cfg.audio_rate_hz)
                # x-Achse evtl. neu setzen
                line.set_data(fx, psd)
                ax.set_xlim(0, self.acars_cfg.audio_rate_hz / 2)

            # Letzte Nachricht anzeigen
            preview = ""
            got = []
            for _i in range(3):
                try:
                    got.append(self.msg_queue.get_nowait())
                except queue.Empty:
                    break
            if got:
                m = got[-1]
                preview = f"Last: conf={m.confidence:.2f} | {m.text[:80]}"
                # restliche auch drucken
                for mm in got:
                    ts = time.strftime("%H:%M:%S", time.localtime(mm.timestamp))
                    print(f"[{ts}] {mm.freq_hz/1e6:.3f} MHz | conf={mm.confidence:.2f} | {mm.text}")
            text_box.set_text(preview)

            return line, text_box

        ani = animation.FuncAnimation(fig, update, interval=100, blit=False, cache_frame_data=False)
        plt.tight_layout()
        plt.show()
        self.running = False


# ============================================================================
# CLI
# ============================================================================

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Modularer ACARS-Viewer für RTL-SDR v4")
    p.add_argument("--dll", required=True, help="Pfad zu rtlsdr.dll")
    p.add_argument("--freq", type=int, default=131_725_000, help="ACARS Frequenz in Hz (z.B. 131725000)")
    p.add_argument("--sr", type=int, default=1_024_000, help="Sample Rate in Hz")
    p.add_argument("--ppm", type=int, default=0, help="Frequenzkorrektur in ppm")
    p.add_argument("--plot", action="store_true", help="Live-Plot des demodulierten Audio-Spektrums")
    p.add_argument("--squelch", type=float, default=-45.0, help="Audio-Squelch in dB")
    return p.parse_args()


def main() -> int:
    args = parse_args()

    radio_cfg = RadioConfig(
        dll_path=args.dll,
        center_freq_hz=args.freq,
        sample_rate_hz=args.sr,
        ppm=args.ppm,
    )
    acars_cfg = AcarsConfig(squelch_db=args.squelch)

    app = AcarsViewerApp(radio_cfg, acars_cfg, show_plot=args.plot)
    app.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())