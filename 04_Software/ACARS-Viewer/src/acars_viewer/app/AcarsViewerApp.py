from __future__ import annotations

import queue
import signal
import threading
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from acars_viewer.config.RadioConfig import RadioConfig
from acars_viewer.config.AcarsConfig import AcarsConfig
from acars_viewer.sdr.RtlSdrSource import RtlSdrSource
from acars_viewer.dsp.AcarsDemodulator import AcarsDemodulator
from acars_viewer.decode.AcarsDecoder import AcarsDecoder


class AcarsViewerApp:
    def __init__(self, radio_cfg: RadioConfig, acars_cfg: AcarsConfig, show_plot: bool):
        self.radio_cfg = radio_cfg
        self.acars_cfg = acars_cfg
        self.show_plot = show_plot
        self.running = True

        self.demod = AcarsDemodulator(radio_cfg, acars_cfg)
        self.decoder = AcarsDecoder(acars_cfg)

        self.msg_queue: "queue.Queue" = queue.Queue()
        self.audio_queue: "queue.Queue[np.ndarray]" = queue.Queue(maxsize=10)

    def stop(self, *_):
        self.running = False

    def run(self):
        signal.signal(signal.SIGINT, self.stop)
        worker = threading.Thread(target=self._worker_loop, daemon=True)
        worker.start()

        if self.show_plot:
            self._run_plot_loop()
        else:
            self._run_terminal_loop()

        self.running = False
        worker.join(timeout=1.0)

    def _worker_loop(self):
        try:
            with RtlSdrSource(self.radio_cfg) as src:
                print(
                    f"[INFO] SDR offen | f={self.radio_cfg.center_freq_hz/1e6:.3f} MHz, "
                    f"sr={self.radio_cfg.sample_rate_hz/1e6:.3f} MS/s"
                )
                while self.running:
                    iq = src.read_iq()
                    audio = self.demod.process(iq)

                    for msg in self.decoder.feed(audio, self.radio_cfg.center_freq_hz):
                        self.msg_queue.put(msg)

                    if self.show_plot:
                        if self.audio_queue.full():
                            try:
                                self.audio_queue.get_nowait()
                            except queue.Empty:
                                pass
                        self.audio_queue.put(audio)
        except Exception as e:
            print(f"[ERROR] Worker beendet: {e}")
            self.running = False

    def _run_terminal_loop(self):
        print("[INFO] ACARS-Viewer läuft. Strg+C zum Beenden.\n")
        while self.running:
            try:
                msg = self.msg_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            ts = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(msg.timestamp))
            print(
                f"[{ts}] {msg.freq_hz/1e6:.3f} MHz | conf={msg.confidence:.2f}\n"
                f"  TEXT: {msg.text}\n"
                f"  RAW : {msg.raw_bytes_hex}\n"
            )

    def _run_plot_loop(self):
        print("[INFO] ACARS-Viewer + Plot läuft.")
        fig, ax = plt.subplots(figsize=(12, 4))
        x = np.linspace(0, self.acars_cfg.audio_rate_hz / 2, 1024)
        y = np.full_like(x, -80.0)
        line, = ax.plot(x, y, lw=1.0)
        ax.set_title("Demoduliertes Audio-Spektrum")
        ax.set_xlabel("Hz")
        ax.set_ylabel("dB")
        ax.set_ylim(-90, 40)
        ax.grid(True, alpha=0.3)
        txt = ax.text(0.01, 0.96, "", transform=ax.transAxes, va="top", fontsize=9)

        def update(_):
            if not self.running:
                return line, txt

            latest = None
            while True:
                try:
                    latest = self.audio_queue.get_nowait()
                except queue.Empty:
                    break

            if latest is not None and len(latest) >= 2048:
                N = 2048
                w = np.hanning(N)
                spec = np.fft.rfft(latest[:N] * w)
                psd = 20.0 * np.log10(np.abs(spec) + 1e-9)
                fx = np.fft.rfftfreq(N, d=1.0 / self.acars_cfg.audio_rate_hz)
                line.set_data(fx, psd)
                ax.set_xlim(0, self.acars_cfg.audio_rate_hz / 2)

            msgs = []
            for _i in range(3):
                try:
                    msgs.append(self.msg_queue.get_nowait())
                except queue.Empty:
                    break
            if msgs:
                m = msgs[-1]
                txt.set_text(f"Last: conf={m.confidence:.2f} | {m.text[:80]}")
                for mm in msgs:
                    t = time.strftime("%H:%M:%S", time.localtime(mm.timestamp))
                    print(f"[{t}] {mm.freq_hz/1e6:.3f} MHz | conf={mm.confidence:.2f} | {mm.text}")

            return line, txt

        animation.FuncAnimation(fig, update, interval=100, blit=False, cache_frame_data=False)
        plt.tight_layout()
        plt.show()
        self.running = False