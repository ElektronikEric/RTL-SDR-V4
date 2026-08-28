from dataclasses import dataclass


@dataclass(slots=True)
class RadioConfig:
    dll_path: str
    device_index: int = 0
    center_freq_hz: int = 131_725_000
    sample_rate_hz: int = 1_024_000
    ppm: int = 0
    gain_mode_auto: bool = True
    agc_on: bool = True
    block_iq_samples: int = 16384