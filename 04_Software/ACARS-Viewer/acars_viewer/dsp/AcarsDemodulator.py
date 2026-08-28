import numpy as np

from acars_viewer.config.RadioConfig import RadioConfig
from acars_viewer.config.AcarsConfig import AcarsConfig
from acars_viewer.dsp.FirDesign import FirDesign


class AcarsDemodulator:
    def __init__(self, radio_cfg: RadioConfig, acars_cfg: AcarsConfig):
        self.decim = max(1, int(round(radio_cfg.sample_rate_hz / acars_cfg.audio_rate_hz)))
        self.out_fs = radio_cfg.sample_rate_hz / self.decim

        self.h_lpf = FirDesign.lowpass_fir(0.45 * self.out_fs, radio_cfg.sample_rate_hz, 129)
        self.h_bp = FirDesign.bandpass_fir(acars_cfg.bp_low_hz, acars_cfg.bp_high_hz, self.out_fs, 257)

        self._zi_lpf = np.zeros(len(self.h_lpf) - 1, dtype=np.float32)
        self._zi_bp = np.zeros(len(self.h_bp) - 1, dtype=np.float32)

    def _fir(self, x: np.ndarray, h: np.ndarray, zi: np.ndarray):
        x_ext = np.concatenate([zi, x.astype(np.float32)])
        y = np.convolve(x_ext, h, mode="valid")
        return y.astype(np.float32), x_ext[-(len(h) - 1):].astype(np.float32)

    def process(self, iq: np.ndarray) -> np.ndarray:
        env = np.abs(iq).astype(np.float32)
        env -= np.mean(env)

        y, self._zi_lpf = self._fir(env, self.h_lpf, self._zi_lpf)
        audio = y[::self.decim]

        audio, self._zi_bp = self._fir(audio, self.h_bp, self._zi_bp)
        s = float(np.std(audio))
        if s > 1e-9:
            audio = audio / s
        return audio