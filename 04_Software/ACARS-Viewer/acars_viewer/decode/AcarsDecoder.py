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
        self._bits: List[int] = []

    def _pwr_db(self, x: np.ndarray) -> float:
        return 10.0 * math.log10(float(np.mean(x * x)) + 1e-12)

    def _demod_bits(self, audio: np.ndarray) -> List[int]:
        out = []
        for i in range(0, len(audio) - self.sps, self.sps):
            out.append(1 if float(np.sum(audio[i:i + self.sps])) >= 0 else 0)
        return out

    @staticmethod
    def _bits_to_bytes(bits: List[int]) -> bytes:
        b = bytearray()
        for i in range(0, len(bits) - 7, 8):
            v = 0
            for k in range(8):
                v = (v << 1) | (bits[i + k] & 1)
            b.append(v)
        return bytes(b)

    @staticmethod
    def _printable_ratio(data: bytes) -> float:
        if not data:
            return 0.0
        ok = 0
        for c in data:
            if c in (9, 10, 13) or (32 <= c <= 126):
                ok += 1
        return ok / len(data)

    def _candidates(self, data: bytes) -> List[Tuple[str, bytes, float]]:
        c = []
        if len(data) < self.cfg.min_frame_bytes:
            return c
        for s in range(0, max(1, len(data) - self.cfg.min_frame_bytes), 8):
            for n in (12, 24, 40, 64, 96, 128, 180, 240):
                if n > self.cfg.max_frame_bytes or s + n > len(data):
                    continue
                chunk = data[s:s + n]
                r = self._printable_ratio(chunk)
                if r >= 0.70:
                    txt = chunk.decode("ascii", errors="ignore").strip()
                    if len(txt) >= 8:
                        c.append((txt, chunk, r))
        return c

    def feed(self, audio: np.ndarray, freq_hz: int) -> List[DecodeMessage]:
        if self._pwr_db(audio) < self.cfg.squelch_db:
            return []

        self._bits.extend(self._demod_bits(audio))
        if len(self._bits) > self.cfg.max_frame_bytes * 8 * 8:
            self._bits = self._bits[-self.cfg.max_frame_bytes * 8 * 8:]

        data = self._bits_to_bytes(self._bits)
        cand = sorted(self._candidates(data), key=lambda x: x[2], reverse=True)[:2]

        out = []
        for txt, raw, conf in cand:
            out.append(
                DecodeMessage(
                    timestamp=time.time(),
                    freq_hz=freq_hz,
                    text=txt,
                    raw_bytes_hex=raw[:80].hex(),
                    confidence=float(conf),
                )
            )
        return out