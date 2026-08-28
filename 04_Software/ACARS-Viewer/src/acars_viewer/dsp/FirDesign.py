from __future__ import annotations
import numpy as np


class FirDesign:
    @staticmethod
    def lowpass_fir(cutoff_hz: float, fs_hz: float, num_taps: int = 129) -> np.ndarray:
        if num_taps % 2 == 0:
            num_taps += 1
        n = np.arange(num_taps) - (num_taps - 1) / 2
        fc = cutoff_hz / fs_hz
        h = 2 * fc * np.sinc(2 * fc * n)
        h *= np.hamming(num_taps)
        h /= np.sum(h)
        return h.astype(np.float32)

    @staticmethod
    def bandpass_fir(low_hz: float, high_hz: float, fs_hz: float, num_taps: int = 257) -> np.ndarray:
        h_high = FirDesign.lowpass_fir(high_hz, fs_hz, num_taps)
        h_low = FirDesign.lowpass_fir(low_hz, fs_hz, num_taps)
        return (h_high - h_low).astype(np.float32)