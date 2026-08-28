from __future__ import annotations
import numpy as np
from acars_viewer.config.RadioConfig import RadioConfig
from acars_viewer.config.AcarsConfig import AcarsConfig
from acars_viewer.dsp.FirDesign import FirDesign


class AcarsDemodulator:
    def __init__(self, radio_cfg: RadioConfig, acars_cfg: AcarsConfig):
        self.radio_cfg = radio_cfg
        self.acars_cfg = acars_cfg

        self.decim = max(1, int(round(radio_cfg.sample_rate_hz / acars_cfg.audio_rate_hz)))
        self.out_fs = radio_cfg.sample_rate_hz / self.decim

        anti_alias_cut = min(0.45 * self.out_fs, 0.45 * radio_cfg.sample_rate_hz / self.decim)
        self.h_lpf = FirDesign.lowpass_fir(anti_alias_cut, radio_cfg.sample_rate_hz, 129)
        self.h_bp = FirDesign.bandpass_fir(acars_cfg.bp_low_hz, acars_cfg.bp_high_hz, self.out_fs, 257)

        self._zi_lpf = np.zeros(len(self.h_lpf) - 1, dtype=np.float32)
        self._zi_bp = np.zeros(len(self.h_bp) - 1, dtype=np.float32)

    def _fir_filter(self, x: np.ndarray, h: np.ndarray, zi: np.ndarray):
        x_ext = np.concatenate([zi, x.astype(np.float32)])
        y = np.convolve(x_ext, h, mode="valid")
        new_zi = x_ext[-(len(h) - 1):]
        return y.astype(np.float32), new_zi.astype(np.float32)

    def process(self, iq: np.ndarray) -> np.ndarray:
        env = np.abs(iq).astype(np.float32)
        env -= np.mean(env)

        y_lpf, self._zi_lpf = self._fir_filter(env, self.h_lpf, self._zi_lpf)
        audio = y_lpf[::self.decim]

        y_bp, self._zi_bp = self._fir_filter(audio, self.h_bp, self._zi_bp)

        std = float(np.std(y_bp))
        if std > 1e-9:
            y_bp = y_bp / std
        return y_bp