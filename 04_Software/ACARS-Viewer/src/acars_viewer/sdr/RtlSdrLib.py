from __future__ import annotations
import ctypes


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
            p_dev, ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.c_int)
        ]