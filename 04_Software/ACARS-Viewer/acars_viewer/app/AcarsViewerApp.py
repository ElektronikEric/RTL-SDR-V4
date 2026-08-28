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

        self.msg_q = queue.Queue()
        self.audio_q = queue.Queue(maxsize=10)

    def stop(self, *_):
        self.running = False

    def run(self):
        signal.signal(signal.SIGINT, self.stop)
        t = threading.Thread(target=self._worker, daemon=True)
        t.start()

        if self.show_plot:
            self._plot_loop()
        else:
            self._text_loop()

        self.running = False
        t.join(timeout=1.0)

    def _worker(self):
        try:
            with RtlSdrSource(self.radio_cfg) as sdr:
                print(f"[INFO] SDR offen @ {self.radio_cfg.center_freq_hz/1e6:.3f} MHz")
                while self.running:
                    iq = sdr.read_iq()
                    audio = self.demod.process(iq)

                    for m in self.decoder.feed(audio, self.radio_cfg.center_freq_hz):
                        self.msg_q.put(m)

                    if self.show_plot:
                        if self.audio_q.full():
                            try:
                                self.audio_q.get_nowait()
                            except queue.Empty:
                                pass
                        self.audio_q.put(audio)
        except Exception as e:
            print(f"[ERROR] {e}")
            self.running = False

    def _text_loop(self):
        print("[INFO] ACARS läuft. Strg+C beendet.")
        while self.running:
            try:
                m = self.msg_q.get(timeout=0.2)
            except queue.Empty:
                continue
            ts = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(m.timestamp))
            print(f"[{ts}] {m.freq_hz/1e6:.3f} MHz conf={m.confidence:.2f}")
            print(f"  TEXT: {m.text}")
            print(f"  RAW : {m.raw_bytes_hex}")

    def _plot_loop(self):
        fig, ax = plt.subplots(figsize=(11, 4))
        x = np.linspace(0, self.acars_cfg.audio_rate_hz / 2, 1025)
        y = np.full_like(x, -80.0)
        line, = ax.plot(x, y)
        ax.set_ylim(-90, 40)
        ax.set_xlim(0, self.acars_cfg.audio_rate_hz / 2)
        ax.set_title("ACARS Audio-Spektrum")
        ax.grid(True, alpha=0.3)
        text = ax.text(0.01, 0.96, "", transform=ax.transAxes, va="top")

        def update(_):
            latest = None
            while True:
                try:
                    latest = self.audio_q.get_nowait()
                except queue.Empty:
                    break

            if latest is not None and len(latest) >= 2048:
                N = 2048
                w = np.hanning(N)
                spec = np.fft.rfft(latest[:N] * w)
                psd = 20 * np.log10(np.abs(spec) + 1e-9)
                fx = np.fft.rfftfreq(N, d=1.0 / self.acars_cfg.audio_rate_hz)
                line.set_data(fx, psd)

            last = None
            while True:
                try:
                    last = self.msg_q.get_nowait()
                except queue.Empty:
                    break
            if last is not None:
                text.set_text(f"Last: conf={last.confidence:.2f} | {last.text[:70]}")
                t = time.strftime("%H:%M:%S", time.localtime(last.timestamp))
                print(f"[{t}] {last.text}")

            return line, text

        animation.FuncAnimation(fig, update, interval=100, blit=False, cache_frame_data=False)
        plt.tight_layout()
        plt.show()
        self.running = False