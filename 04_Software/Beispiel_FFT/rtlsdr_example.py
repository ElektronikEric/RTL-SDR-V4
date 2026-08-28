import ctypes
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ── 1. DLL laden ──────────────────────────────────────────────────────────
dll_path = r"C:\Users\Ruber\OneDrive\Desktop\Elektronik\Radar_2\rtlsdr.dll"
lib   = ctypes.CDLL(dll_path)
p_dev = ctypes.c_void_p

lib.rtlsdr_get_device_count.restype  = ctypes.c_uint
lib.rtlsdr_get_device_count.argtypes = []
lib.rtlsdr_open.restype              = ctypes.c_int
lib.rtlsdr_open.argtypes             = [ctypes.POINTER(p_dev), ctypes.c_uint]
lib.rtlsdr_close.restype             = ctypes.c_int
lib.rtlsdr_close.argtypes            = [p_dev]
lib.rtlsdr_set_sample_rate.restype   = ctypes.c_int
lib.rtlsdr_set_sample_rate.argtypes  = [p_dev, ctypes.c_uint]
lib.rtlsdr_set_center_freq.restype   = ctypes.c_int
lib.rtlsdr_set_center_freq.argtypes  = [p_dev, ctypes.c_uint]
lib.rtlsdr_set_freq_correction.restype  = ctypes.c_int
lib.rtlsdr_set_freq_correction.argtypes = [p_dev, ctypes.c_int]
lib.rtlsdr_set_tuner_gain_mode.restype  = ctypes.c_int
lib.rtlsdr_set_tuner_gain_mode.argtypes = [p_dev, ctypes.c_int]
lib.rtlsdr_set_agc_mode.restype      = ctypes.c_int
lib.rtlsdr_set_agc_mode.argtypes     = [p_dev, ctypes.c_int]
lib.rtlsdr_reset_buffer.restype      = ctypes.c_int
lib.rtlsdr_reset_buffer.argtypes     = [p_dev]
lib.rtlsdr_read_sync.restype         = ctypes.c_int
lib.rtlsdr_read_sync.argtypes        = [p_dev, ctypes.c_void_p, ctypes.c_int,
                                         ctypes.POINTER(ctypes.c_int)]

# ── 2. Parameter ──────────────────────────────────────────────────────────
CENTER_FREQ     = 95_400_000
SAMPLE_RATE     = 2_048_000
FREQ_CORRECTION = 60
FFT_SIZE        = 2048
NUM_SAMPLES     = FFT_SIZE

# ── 3. Gerät öffnen ───────────────────────────────────────────────────────
dev = p_dev(None)
assert lib.rtlsdr_open(ctypes.byref(dev), 0) == 0, "Gerät konnte nicht geöffnet werden"
lib.rtlsdr_set_sample_rate(dev, SAMPLE_RATE)
lib.rtlsdr_set_center_freq(dev, CENTER_FREQ)
lib.rtlsdr_set_freq_correction(dev, FREQ_CORRECTION)
lib.rtlsdr_set_tuner_gain_mode(dev, 0)
lib.rtlsdr_set_agc_mode(dev, 1)
lib.rtlsdr_reset_buffer(dev)

num_bytes = NUM_SAMPLES * 2
buf    = (ctypes.c_ubyte * num_bytes)()
n_read = ctypes.c_int(0)
lib.rtlsdr_read_sync(dev, buf, num_bytes, ctypes.byref(n_read))  # verwerfen

# ── 4. Frequenzachse ──────────────────────────────────────────────────────
freq_axis = np.fft.fftshift(np.fft.fftfreq(FFT_SIZE, d=1.0 / SAMPLE_RATE))
freq_mhz  = (freq_axis + CENTER_FREQ) / 1e6
window    = np.hanning(FFT_SIZE)

# ── 5. Plot vorbereiten ───────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(13, 5))
fig.patch.set_facecolor('#0e0e0e')
ax.set_facecolor('#0e0e0e')

psd_init = np.full(FFT_SIZE, -90.0)

line, = ax.plot(freq_mhz, psd_init, linewidth=0.9, color='#00ccff')

# ── fill in einer Liste – so kann update() sie ersetzen ───────────────────
fill_ref = [ax.fill_between(freq_mhz, -90, psd_init, alpha=0.15, color='#00ccff')]

ax.axvline(CENTER_FREQ / 1e6, color='yellow', linestyle='--',
           linewidth=0.8, alpha=0.5, label=f"Mitte: {CENTER_FREQ/1e6:.1f} MHz")

ax.set_xlim(freq_mhz[0], freq_mhz[-1])
ax.set_ylim(-90, 50)
ax.set_xlabel("Frequenz (MHz)", color='white', fontsize=11)
ax.set_ylabel("PSD (dB)", color='white', fontsize=11)
ax.set_title(
    f"Live-Spektrum  |  Mitte: {CENTER_FREQ/1e6:.1f} MHz  |  "
    f"Bandbreite: {SAMPLE_RATE/1e6:.1f} MHz  |  "
    f"Auflösung: {SAMPLE_RATE/FFT_SIZE:.0f} Hz/Bin",
    color='white', fontsize=11
)
ax.tick_params(colors='white')
ax.xaxis.set_major_locator(plt.MultipleLocator(0.2))
ax.xaxis.set_minor_locator(plt.MultipleLocator(0.05))
ax.grid(True, which='major', color='#333333', alpha=0.8)
ax.grid(True, which='minor', color='#222222', alpha=0.5)
for spine in ax.spines.values():
    spine.set_edgecolor('#444444')

frame_text = ax.text(0.01, 0.97, '', transform=ax.transAxes,
                     color='#aaaaaa', fontsize=8, va='top')
frame_count = [0]

# ── 6. Update-Funktion ────────────────────────────────────────────────────
def update(_):
    lib.rtlsdr_read_sync(dev, buf, num_bytes, ctypes.byref(n_read))

    data   = np.frombuffer(buf, dtype=np.uint8).astype(np.float64).copy()
    iq     = data.view(np.complex128)
    iq     = iq / 127.5 - (1.0 + 1.0j)
    iq    -= np.mean(iq)

    psd    = np.abs(np.fft.fft(iq * window)) ** 2
    psd_db = 10.0 * np.log10(np.fft.fftshift(psd) + 1e-12)

    # Linie aktualisieren
    line.set_ydata(psd_db)

    # ── Fill ersetzen: alte entfernen, neue hinzufügen ────────────────────
    fill_ref[0].remove()
    fill_ref[0] = ax.fill_between(freq_mhz, -90, psd_db, alpha=0.15, color='#00ccff')

    frame_count[0] += 1
    peak_idx = int(np.argmax(psd_db))
    frame_text.set_text(
        f"Frame #{frame_count[0]}  |  "
        f"Peak: {psd_db[peak_idx]:.1f} dB @ {freq_mhz[peak_idx]:.3f} MHz"
    )
    return line, fill_ref[0], frame_text

# ── 7. Animation starten ──────────────────────────────────────────────────
ani = animation.FuncAnimation(fig, update, interval=50,
                               blit=False, cache_frame_data=False)
plt.tight_layout()
plt.show()

# ── 8. Gerät schließen ────────────────────────────────────────────────────
lib.rtlsdr_close(dev)
print("Gerät geschlossen.")