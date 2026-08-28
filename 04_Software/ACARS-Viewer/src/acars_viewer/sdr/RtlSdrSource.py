from __future__ import annotations

import ctypes
import numpy as np
from acars_viewer.config.RadioConfig import RadioConfig
from acars_viewer.sdr.RtlSdrLib import RtlSdrLib


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
        raw = np.frombuffer(self._buf, dtype=np.uint8, count=n).astype(np.float32)

        i = raw[0::2] / 127.5 - 1.0
        q = raw[1::2] / 127.5 - 1.0
        return (i + 1j * q).astype(np.complex64)

    def close(self):
        if self._opened:
            self._lib.rtlsdr_close(self._dev)
            self._opened = False

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()