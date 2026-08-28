from __future__ import annotations
from dataclasses import dataclass


@dataclass(slots=True)
class AcarsConfig:
    bitrate: int = 2400
    audio_rate_hz: int = 48_000
    bp_low_hz: float = 1000.0
    bp_high_hz: float = 4000.0
    squelch_db: float = -45.0
    min_frame_bytes: int = 12
    max_frame_bytes: int = 300