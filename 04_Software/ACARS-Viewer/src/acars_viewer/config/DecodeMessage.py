from __future__ import annotations
from dataclasses import dataclass


@dataclass(slots=True)
class DecodeMessage:
    timestamp: float
    freq_hz: int
    text: str
    raw_bytes_hex: str
    confidence: float