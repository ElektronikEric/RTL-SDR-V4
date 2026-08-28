from __future__ import annotations

import math
import time
from typing import List, Tuple
import numpy as np

from acars_viewer.config.AcarsConfig import AcarsConfig
from acars_viewer.config.DecodeMessage import DecodeMessage


class AcarsDecoder:
    def __init__(self, cfg: AcarsConfig):
        self.cfg = cfg
        self.sps = int(round(cfg.audio_rate_hz / cfg.bitrate))
        self._bit_buffer: List[int] = []

    def _audio_power_db(self, x: np.ndarray) -> float:
        p = float(np.mean(x * x)) + 1e-12
        return 10.0 * math.log10(p)

    def demod_bits(self, audio: np.ndarray) -> List[int]:
        bits = []
        step = self.sps
        for i in range(0, len(audio) - step, step):
            v = float(np.sum(audio[i:i + step]))
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
        results = []
        min_b = self.cfg.min_frame_bytes
        max_b = self.cfg.max_frame_bytes
        L = len(data)
        if L < min_b:
            return results

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

        max_bits = self.cfg.max_frame_bytes * 8 * 8
        if len(self._bit_buffer) > max_bits:
            self._bit_buffer = self._bit_buffer[-max_bits:]

        data = self.bits_to_bytes(self._bit_buffer)
        cands = self._extract_ascii_candidates(data)
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

        if msgs and len(self._bit_buffer) > 2000:
            self._bit_buffer = self._bit_buffer[-1200:]

        return msgs