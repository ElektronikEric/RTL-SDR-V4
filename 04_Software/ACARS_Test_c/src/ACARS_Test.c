/*
 * ============================================================
 * ACARS VHF Decoder - C-Portierung (Windows)
 * ============================================================
 *
 * Portierung der Python-Signalkette:
 *   RTL-SDR IQ (asynchron, eigener Thread)
 *     -> Ringpuffer
 *     -> AM-Envelope-Demod
 *     -> Butterworth-Tiefpass (2x Biquad, feste Koeffizienten) + Dezimation
 *     -> Squelch (gleitender Median-Rauschboden, geglaettete Leistung)
 *     -> Burst-Erfassung (Hangtime + Notbremse)
 *     -> Momentanfrequenz via Hilbert-FIR + Quadratur-Diskriminator
 *     -> NRZI-Bit-Extraktion -> Byte-Framing -> Paritaet -> CRC/BCS
 *     -> Logging (JSONL) + WAV-Dump fehlgeschlagener Bursts
 *
 * Build (MinGW-w64):
 *   gcc -O2 -Wall -o acars_decoder.exe acars_decoder.c -lm -lwinmm
 *
 * Build (MSVC, "x64 Native Tools Command Prompt"):
 *   cl /O2 /W3 acars_decoder.c
 *
 * rtlsdr.dll wird zur LAUFZEIT dynamisch geladen (wie im Python-Original
 * per ctypes) - es wird KEINE rtlsdr.lib zum Linken benoetigt, nur die
 * DLL muss neben der .exe liegen oder im PATH sein.
 *
 * WICHTIGE EINSCHRAENKUNGEN (wie schon in der Python-Version):
 * - Die Bit-Taktrueckgewinnung ist "open loop" (kein Nachregeln waehrend
 *   eines Bursts). Bei kurzen ACARS-Bursts reicht das meist, bei starkem
 *   Sample-Clock-Offset koennen Bitfehler auftreten.
 * - Die BCS/CRC-Parameter (Poly 0x8408, Init 0xFFFF, LSB-first) sind eine
 *   gaengige Annahme und sollten an einer echten Nachricht verifiziert
 *   werden (siehe Kommentar am Dateiende).
 * - Es gibt KEINE grafische Spektrumsanzeige wie in der Python-Version
 *   (das wuerde eine zusaetzliche GUI-Bibliothek wie SDL2 erfordern).
 *   Stattdessen gibt es eine textbasierte Pegelanzeige in der Konsole.
 */

#define _CRT_SECURE_NO_WARNINGS
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <time.h>

/* ============================================================
 * KONFIGURATION (Werte wie im Python-Original)
 * ============================================================ */

static const char *DLL_PATH = "rtlsdr.dll"; /* ggf. vollen Pfad eintragen */

#define CENTER_FREQ        131725000u
#define IQ_SAMPLE_RATE     240000u
#define FREQ_CORRECTION    60

#define CHUNK_SECONDS      0.1
#define NUM_IQ_SAMPLES     ((int)(IQ_SAMPLE_RATE * CHUNK_SECONDS))  /* 24000 */

#define AUDIO_DECIM        5
#define AUDIO_RATE         (IQ_SAMPLE_RATE / AUDIO_DECIM)  /* 48000 */

#define BIT_RATE           2400.0
#define SAMPLES_PER_BIT    (AUDIO_RATE / BIT_RATE)         /* 20.0 */

#define FREQ_MARK          1200.0
#define FREQ_SPACE         2400.0
#define FREQ_MID           ((FREQ_MARK + FREQ_SPACE) / 2.0)

#define PREAMBLE_MIN_SEC   0.25
#define SQUELCH_RATIO      5.0
#define SQUELCH_HANGTIME_SEC 0.05
#define MAX_BURST_SEC      1.2
#define MIN_BURST_SEC      0.15

#define NOISE_FLOOR_WINDOW_SEC 3.0
#define NOISE_FLOOR_UPDATE_SEC 0.5
#define POWER_SMOOTHING_MS 8.0

#define MANUAL_GAIN_TENTH_DB 400  /* 40.0 dB; -1 = AGC verwenden */

/* MAX_BURST_SEC * AUDIO_RATE als ganzzahlige Konstante (fuer Array-Groessen).
   Muss von Hand synchron zu MAX_BURST_SEC/AUDIO_RATE gehalten werden, falls
   diese geaendert werden - ein Fliesskomma-Ausdruck ist an dieser Stelle
   (Groesse eines 'static'-Arrays) auf manchen Compilern kein gueltiger
   konstanter Ausdruck. */
#define MAX_BURST_SAMPLES_CONST 57600  /* = 1.2 * 48000 */

static const char *LOG_FILE = "acars_messages.jsonl";
static const char *DEBUG_DIR = "acars_debug_wav";
#define DEBUG_DUMP_FAILED 1

/* Ringpuffer fuer rohe IQ-Bytes zwischen Capture-Thread und Verarbeitung */
#define RINGBUF_BYTES (4 * 1024 * 1024)

/* ============================================================
 * FILTERKOEFFIZIENTEN (offline mit scipy berechnet, siehe Kommentare)
 * ============================================================ */

/* Butterworth Tiefpass Ordnung 4, cutoff=3000 Hz, fs=240000 Hz, als 2 SOS */
typedef struct { double b0, b1, b2, a1, a2; double z1, z2; } Biquad;

static const double LOWPASS_SOS[2][5] = {
    /* b0, b1, b2, a1, a2  (a0 ist bei scipy immer 1, hier weggelassen) */
    {2.150568737288e-06, 4.301137474576e-06, 2.150568737288e-06, -1.859076265958e+00, 8.648248987673e-01},
    {1.000000000000e+00, 2.000000000000e+00, 1.000000000000e+00, -1.935714837121e+00, 9.417004516037e-01},
};

/* Hilbert-FIR (Fenstermethode, Hamming, 65 Taps) fuer die FSK-Frequenzschaetzung */
#define HILBERT_TAPS 65
#define HILBERT_DELAY (HILBERT_TAPS / 2) /* 32 */
static const double HILBERT_H[HILBERT_TAPS] = {
    0.0000000000e+00, -1.6883777731e-03, 0.0000000000e+00, -2.1910135612e-03, 0.0000000000e+00,
    -3.1669763105e-03, 0.0000000000e+00, -4.6960942882e-03, 0.0000000000e+00, -6.8693789273e-03,
    0.0000000000e+00, -9.7965930840e-03, 0.0000000000e+00, -1.3619275427e-02, 0.0000000000e+00,
    -1.8533578722e-02, 0.0000000000e+00, -2.4831901059e-02, 0.0000000000e+00, -3.2983317004e-02,
    0.0000000000e+00, -4.3801899945e-02, 0.0000000000e+00, -5.8839293203e-02, 0.0000000000e+00,
    -8.1449570993e-02, 0.0000000000e+00, -1.2040819875e-01, 0.0000000000e+00, -2.0800332029e-01,
    0.0000000000e+00, -6.3520964319e-01, 0.0000000000e+00, 6.3520964319e-01, 0.0000000000e+00,
    2.0800332029e-01, 0.0000000000e+00, 1.2040819875e-01, 0.0000000000e+00, 8.1449570993e-02,
    0.0000000000e+00, 5.8839293203e-02, 0.0000000000e+00, 4.3801899945e-02, 0.0000000000e+00,
    3.2983317004e-02, 0.0000000000e+00, 2.4831901059e-02, 0.0000000000e+00, 1.8533578722e-02,
    0.0000000000e+00, 1.3619275427e-02, 0.0000000000e+00, 9.7965930840e-03, 0.0000000000e+00,
    6.8693789273e-03, 0.0000000000e+00, 4.6960942882e-03, 0.0000000000e+00, 3.1669763105e-03,
    0.0000000000e+00, 2.1910135612e-03, 0.0000000000e+00, 1.6883777731e-03, 0.0000000000e+00
};

/* ============================================================
 * RTL-SDR: dynamisches Laden der DLL-Funktionen (wie ctypes in Python)
 * ============================================================ */

typedef uint32_t (__cdecl *rtlsdr_get_device_count_t)(void);
typedef int (__cdecl *rtlsdr_open_t)(void **dev, uint32_t index);
typedef int (__cdecl *rtlsdr_close_t)(void *dev);
typedef int (__cdecl *rtlsdr_set_center_freq_t)(void *dev, uint32_t freq);
typedef int (__cdecl *rtlsdr_set_sample_rate_t)(void *dev, uint32_t rate);
typedef int (__cdecl *rtlsdr_set_freq_correction_t)(void *dev, int ppm);
typedef int (__cdecl *rtlsdr_set_tuner_gain_mode_t)(void *dev, int manual);
typedef int (__cdecl *rtlsdr_set_tuner_gain_t)(void *dev, int gain_tenth_db);
typedef int (__cdecl *rtlsdr_reset_buffer_t)(void *dev);
typedef void (__cdecl *rtlsdr_read_async_cb_t)(unsigned char *buf, uint32_t len, void *ctx);
typedef int (__cdecl *rtlsdr_read_async_t)(void *dev, rtlsdr_read_async_cb_t cb, void *ctx,
                                            uint32_t buf_num, uint32_t buf_len);
typedef int (__cdecl *rtlsdr_cancel_async_t)(void *dev);

typedef struct {
    HMODULE handle;
    rtlsdr_get_device_count_t get_device_count;
    rtlsdr_open_t open_dev;
    rtlsdr_close_t close_dev;
    rtlsdr_set_center_freq_t set_center_freq;
    rtlsdr_set_sample_rate_t set_sample_rate;
    rtlsdr_set_freq_correction_t set_freq_correction;
    rtlsdr_set_tuner_gain_mode_t set_tuner_gain_mode;
    rtlsdr_set_tuner_gain_t set_tuner_gain;
    rtlsdr_reset_buffer_t reset_buffer;
    rtlsdr_read_async_t read_async;
    rtlsdr_cancel_async_t cancel_async;
    void *dev;
} RtlSdr;

#define LOAD_FN(var, name) do { \
        sdr->var = (void *)GetProcAddress(sdr->handle, name); \
        if (!sdr->var) { fprintf(stderr, "rtlsdr.dll: Funktion %s fehlt.\n", name); return 0; } \
    } while (0)

static int rtlsdr_load(RtlSdr *sdr, const char *dll_path) {
    memset(sdr, 0, sizeof(*sdr));
    sdr->handle = LoadLibraryA(dll_path);
    if (!sdr->handle) {
        fprintf(stderr, "Konnte %s nicht laden (Fehlercode %lu).\n", dll_path, GetLastError());
        return 0;
    }
    LOAD_FN(get_device_count, "rtlsdr_get_device_count");
    LOAD_FN(open_dev, "rtlsdr_open");
    LOAD_FN(close_dev, "rtlsdr_close");
    LOAD_FN(set_center_freq, "rtlsdr_set_center_freq");
    LOAD_FN(set_sample_rate, "rtlsdr_set_sample_rate");
    LOAD_FN(set_freq_correction, "rtlsdr_set_freq_correction");
    LOAD_FN(set_tuner_gain_mode, "rtlsdr_set_tuner_gain_mode");
    LOAD_FN(set_tuner_gain, "rtlsdr_set_tuner_gain");
    LOAD_FN(reset_buffer, "rtlsdr_reset_buffer");
    LOAD_FN(read_async, "rtlsdr_read_async");
    LOAD_FN(cancel_async, "rtlsdr_cancel_async");
    return 1;
}

/* ============================================================
 * RINGPUFFER (Producer: Capture-Thread, Consumer: Verarbeitungs-Thread)
 * ============================================================ */

typedef struct {
    unsigned char *buf;
    size_t capacity;
    size_t head, tail, count;
    CRITICAL_SECTION lock;
    CONDITION_VARIABLE not_empty;
    volatile LONG overflow_count;
} RingBuffer;

static void ring_init(RingBuffer *r, size_t capacity) {
    r->buf = malloc(capacity);
    r->capacity = capacity;
    r->head = r->tail = r->count = 0;
    r->overflow_count = 0;
    InitializeCriticalSection(&r->lock);
    InitializeConditionVariable(&r->not_empty);
}

/* Wird aus dem rtlsdr-Callback-Thread aufgerufen */
static void ring_push(RingBuffer *r, const unsigned char *data, size_t len) {
    EnterCriticalSection(&r->lock);
    if (len > r->capacity) len = r->capacity; /* Sicherheitsnetz, sollte nie passieren */
    for (size_t i = 0; i < len; i++) {
        if (r->count == r->capacity) {
            /* Puffer voll: aeltestes Byte verwerfen (Overrun) */
            r->tail = (r->tail + 1) % r->capacity;
            r->count--;
            InterlockedIncrement(&r->overflow_count);
        }
        r->buf[r->head] = data[i];
        r->head = (r->head + 1) % r->capacity;
        r->count++;
    }
    LeaveCriticalSection(&r->lock);
    WakeConditionVariable(&r->not_empty);
}

/* Blockiert, bis mindestens need Bytes verfuegbar sind oder stop gesetzt wird */
static int ring_pop(RingBuffer *r, unsigned char *out, size_t need, volatile LONG *stop) {
    EnterCriticalSection(&r->lock);
    while (r->count < need && !*stop) {
        SleepConditionVariableCS(&r->not_empty, &r->lock, 200);
    }
    if (*stop && r->count < need) {
        LeaveCriticalSection(&r->lock);
        return 0;
    }
    for (size_t i = 0; i < need; i++) {
        out[i] = r->buf[r->tail];
        r->tail = (r->tail + 1) % r->capacity;
    }
    r->count -= need;
    LeaveCriticalSection(&r->lock);
    return 1;
}

/* ============================================================
 * AM-DEMODULATION + TIEFPASS + DEZIMATION
 * ============================================================ */

static Biquad g_biquad[2];

static void biquad_init(void) {
    for (int s = 0; s < 2; s++) {
        g_biquad[s].b0 = LOWPASS_SOS[s][0];
        g_biquad[s].b1 = LOWPASS_SOS[s][1];
        g_biquad[s].b2 = LOWPASS_SOS[s][2];
        g_biquad[s].a1 = LOWPASS_SOS[s][3];
        g_biquad[s].a2 = LOWPASS_SOS[s][4];
        g_biquad[s].z1 = g_biquad[s].z2 = 0.0;
    }
}

/* Direct Form II Transposed, ein Sample */
static inline double biquad_process(Biquad *bq, double x) {
    double y = bq->b0 * x + bq->z1;
    bq->z1 = bq->b1 * x - bq->a1 * y + bq->z2;
    bq->z2 = bq->b2 * x - bq->a2 * y;
    return y;
}

/* envelope: NUM_IQ_SAMPLES Werte rein, audio_out: NUM_IQ_SAMPLES/AUDIO_DECIM raus */
static void am_envelope_filter_decimate(const unsigned char *iq_bytes, int n_iq_samples,
                                         double *audio_out, int *n_audio_out) {
    int count = 0;
    for (int idx = 0; idx < n_iq_samples; idx++) {
        double i_val = (double)iq_bytes[idx * 2] - 127.5;
        double q_val = (double)iq_bytes[idx * 2 + 1] - 127.5;
        double envelope = sqrt(i_val * i_val + q_val * q_val);

        double filtered = envelope;
        filtered = biquad_process(&g_biquad[0], filtered);
        filtered = biquad_process(&g_biquad[1], filtered);

        if (idx % AUDIO_DECIM == 0) {
            audio_out[count++] = filtered;
        }
    }
    *n_audio_out = count;
}

/* ============================================================
 * SQUELCH / BURST-ERKENNUNG
 * ============================================================ */

typedef void (*BurstCallback)(const double *samples, int n);

typedef struct {
    double *power_history;
    int history_capacity, history_count, history_pos;
    double *history_scratch; /* fuer Median-Berechnung wiederverwendet */

    double noise_floor;
    int have_noise_floor;
    int samples_since_update, update_every;

    double smooth_alpha;
    double smoothed_power;
    int have_smoothed_power;

    int in_burst;
    double *burst_buf;
    int burst_len, burst_capacity;
    int silence_run, hang_samples, max_burst_samples;

    BurstCallback on_burst;
} Squelch;

static int cmp_double(const void *a, const void *b) {
    double da = *(const double *)a, db = *(const double *)b;
    return (da > db) - (da < db);
}

static void squelch_init(Squelch *sq, int audio_rate, BurstCallback cb) {
    memset(sq, 0, sizeof(*sq));
    sq->history_capacity = (int)(NOISE_FLOOR_WINDOW_SEC * audio_rate);
    sq->power_history = malloc(sizeof(double) * sq->history_capacity);
    sq->history_scratch = malloc(sizeof(double) * sq->history_capacity);
    sq->update_every = (int)(NOISE_FLOOR_UPDATE_SEC * audio_rate);

    double tau_samples = (POWER_SMOOTHING_MS / 1000.0) * audio_rate;
    if (tau_samples < 1.0) tau_samples = 1.0;
    sq->smooth_alpha = 1.0 - exp(-1.0 / tau_samples);

    sq->hang_samples = (int)(SQUELCH_HANGTIME_SEC * audio_rate);
    sq->max_burst_samples = (int)(MAX_BURST_SEC * audio_rate);
    sq->burst_capacity = sq->max_burst_samples + sq->hang_samples + 16;
    sq->burst_buf = malloc(sizeof(double) * sq->burst_capacity);
    sq->on_burst = cb;
}

static void squelch_update_noise_floor(Squelch *sq) {
    int min_needed = 0; /* entspricht Python: 0.2s Mindestfuellstand */
    /* min_needed wird vom Aufrufer ueber history_count sichergestellt */
    (void)min_needed;
    memcpy(sq->history_scratch, sq->power_history, sizeof(double) * sq->history_count);
    qsort(sq->history_scratch, sq->history_count, sizeof(double), cmp_double);
    sq->noise_floor = sq->history_scratch[sq->history_count / 2];
    sq->have_noise_floor = 1;
}

static void squelch_push(Squelch *sq, const double *audio, int n) {
    for (int idx = 0; idx < n; idx++) {
        double raw_p = audio[idx] * audio[idx];

        if (!sq->have_smoothed_power) {
            sq->smoothed_power = raw_p;
            sq->have_smoothed_power = 1;
        } else {
            sq->smoothed_power = (1 - sq->smooth_alpha) * sq->smoothed_power + sq->smooth_alpha * raw_p;
        }
        double p = sq->smoothed_power;

        /* Ringpuffer fuer die Median-Schaetzung */
        if (sq->history_count < sq->history_capacity) {
            sq->power_history[sq->history_count++] = p;
        } else {
            sq->power_history[sq->history_pos] = p;
        }
        sq->history_pos = (sq->history_pos + 1) % sq->history_capacity;

        sq->samples_since_update++;
        int min_needed_samples = (int)(0.2 * sq->history_capacity / NOISE_FLOOR_WINDOW_SEC); /* ~0.2s */
        if ((!sq->have_noise_floor || sq->samples_since_update >= sq->update_every)
            && sq->history_count >= min_needed_samples) {
            squelch_update_noise_floor(sq);
            sq->samples_since_update = 0;
        }

        double threshold = sq->have_noise_floor ? sq->noise_floor * SQUELCH_RATIO : p;
        if (threshold < 1e-9) threshold = 1e-9;
        int above = p > threshold;

        if (above) {
            if (!sq->in_burst) {
                sq->in_burst = 1;
                sq->burst_len = 0;
            }
            if (sq->burst_len < sq->burst_capacity) sq->burst_buf[sq->burst_len++] = audio[idx];
            sq->silence_run = 0;
        } else if (sq->in_burst) {
            if (sq->burst_len < sq->burst_capacity) sq->burst_buf[sq->burst_len++] = audio[idx];
            sq->silence_run++;
            if (sq->silence_run >= sq->hang_samples) {
                sq->on_burst(sq->burst_buf, sq->burst_len);
                sq->in_burst = 0;
                sq->burst_len = 0;
                sq->silence_run = 0;
            }
        }

        if (sq->in_burst && sq->burst_len >= sq->max_burst_samples) {
            sq->on_burst(sq->burst_buf, sq->burst_len);
            sq->in_burst = 0;
            sq->burst_len = 0;
            sq->silence_run = 0;
        }
    }
}

/* ============================================================
 * FSK-DEMODULATION (Hilbert-FIR + Quadratur-Diskriminator)
 * ============================================================ */

/* Rueckgabe: Anzahl gueltiger Frequenzwerte in freq_out */
static int instantaneous_frequency(const double *audio, int n, double *freq_out) {
    if (n <= 2 * HILBERT_DELAY + 1) return 0;

    double prev_re = 0, prev_im = 0;
    int have_prev = 0;
    int out_count = 0;

    for (int idx = HILBERT_DELAY; idx < n - HILBERT_DELAY; idx++) {
        double im = 0.0;
        for (int k = 0; k < HILBERT_TAPS; k++) {
            im += HILBERT_H[k] * audio[idx - HILBERT_DELAY + k];
        }
        double re = audio[idx];

        if (have_prev) {
            double cross_re = re * prev_re + im * prev_im;
            double cross_im = im * prev_re - re * prev_im;
            double angle = atan2(cross_im, cross_re);
            /* Vorzeichen: siehe Kommentar - der Diskriminator liefert
               ohne dieses Minus die negierte Frequenz. */
            freq_out[out_count++] = -angle * AUDIO_RATE / (2.0 * M_PI);
        }
        prev_re = re;
        prev_im = im;
        have_prev = 1;
    }
    return out_count;
}

static int find_preamble_end(const double *freq, int len) {
    double band = (FREQ_SPACE - FREQ_MID) * 0.6;
    int min_run = (int)(PREAMBLE_MIN_SEC * AUDIO_RATE);
    int run_len = 0;

    for (int idx = 0; idx < len; idx++) {
        int near_space = fabs(freq[idx] - FREQ_SPACE) < band;
        if (near_space) {
            run_len++;
        } else {
            if (run_len >= min_run) return idx;
            run_len = 0;
        }
    }
    if (run_len >= min_run) return len;
    return -1;
}

static int bits_from_frequency(const double *freq, int len, int start_index, int *bits_out, int max_bits) {
    int bit = 1;
    double pos = (double)start_index;
    int count = 0;
    while ((int)llround(pos) < len && count < max_bits) {
        int idx = (int)llround(pos);
        double f = freq[idx];
        if (fabs(f - FREQ_MARK) < fabs(f - FREQ_SPACE)) {
            bit = 1 - bit;
        }
        bits_out[count++] = bit;
        pos += SAMPLES_PER_BIT;
    }
    return count;
}

/* ============================================================
 * BYTE-FRAMING, PARITAET, CRC/BCS, FELD-PARSING
 * ============================================================ */

#define SYN 0x16
#define SOH 0x01
#define ETX 0x03
#define ETB 0x17

static int bits_to_bytes_lsb_first(const int *bits, int n_bits, uint8_t *bytes_out, int max_bytes) {
    int n_bytes = 0;
    for (int i = 0; i + 8 <= n_bits && n_bytes < max_bytes; i += 8) {
        uint8_t value = 0;
        for (int b = 0; b < 8; b++) value |= (bits[i + b] & 1) << b;
        bytes_out[n_bytes++] = value;
    }
    return n_bytes;
}

static int strip_parity_odd(uint8_t byte_val) {
    int data7 = byte_val & 0x7F;
    int parity_bit = (byte_val >> 7) & 1;
    int ones = parity_bit;
    for (int i = 0; i < 7; i++) if (data7 & (1 << i)) ones++;
    return (ones % 2 == 1) ? data7 : -1;
}

static int find_sync_offset(const uint8_t *raw_bytes, int n) {
    for (int i = 0; i + 2 < n; i++) {
        if (strip_parity_odd(raw_bytes[i]) == SYN &&
            strip_parity_odd(raw_bytes[i + 1]) == SYN &&
            strip_parity_odd(raw_bytes[i + 2]) == SOH) {
            return i + 3;
        }
    }
    return -1;
}

static uint16_t crc_ccitt(const uint8_t *data, int n) {
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < n; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 1) ? (crc >> 1) ^ 0x8408 : crc >> 1;
        }
    }
    return crc;
}

typedef struct {
    int ok;
    char reason[32];
    char text[300];
    uint16_t received_bcs;
    uint16_t calculated_bcs;
} AcarsResult;

static AcarsResult parse_acars_frame(const uint8_t *raw_bytes, int n, int sync_offset) {
    AcarsResult r; memset(&r, 0, sizeof(r));
    int i = sync_offset, text_len = 0, end_marker = -1;

    while (i < n) {
        int val = strip_parity_odd(raw_bytes[i]);
        i++;
        if (val < 0) { strcpy(r.reason, "parity_error"); r.text[text_len] = 0; return r; }
        if (val == ETX || val == ETB) { end_marker = val; break; }
        if (text_len < (int)sizeof(r.text) - 1) r.text[text_len++] = (char)val;
    }
    r.text[text_len] = 0;

    if (end_marker < 0) { strcpy(r.reason, "no_end_marker"); return r; }
    if (i + 1 >= n) { strcpy(r.reason, "truncated_before_bcs"); return r; }

    uint16_t received_bcs = raw_bytes[i] | ((uint16_t)raw_bytes[i + 1] << 8);

    uint8_t payload[300];
    int payload_len = 0;
    for (int k = sync_offset - 1; k < i && payload_len < (int)sizeof(payload); k++) {
        int v = strip_parity_odd(raw_bytes[k]);
        payload[payload_len++] = (v >= 0) ? (uint8_t)v : 0;
    }
    uint16_t calculated = crc_ccitt(payload, payload_len);

    r.received_bcs = received_bcs;
    r.calculated_bcs = calculated;
    r.ok = (calculated == received_bcs);
    strcpy(r.reason, r.ok ? "crc_ok" : "crc_mismatch");
    return r;
}

static AcarsResult decode_burst(const double *audio, int n) {
    AcarsResult r; memset(&r, 0, sizeof(r));
    if (n < (int)(MIN_BURST_SEC * AUDIO_RATE)) { strcpy(r.reason, "too_short"); return r; }

    static double freq[MAX_BURST_SAMPLES_CONST + 64];
    int freq_len = instantaneous_frequency(audio, n, freq);
    if (freq_len <= 0) { strcpy(r.reason, "too_short"); return r; }

    int start = find_preamble_end(freq, freq_len);
    if (start < 0) { strcpy(r.reason, "no_preamble"); return r; }

    static int bits[8192];
    int n_bits = bits_from_frequency(freq, freq_len, start, bits, 8192);

    static uint8_t raw_bytes[1200];
    int n_bytes = bits_to_bytes_lsb_first(bits, n_bits, raw_bytes, 1200);

    int sync_offset = find_sync_offset(raw_bytes, n_bytes);
    if (sync_offset < 0) { strcpy(r.reason, "no_sync_pattern"); return r; }

    return parse_acars_frame(raw_bytes, n_bytes, sync_offset);
}

/* ============================================================
 * LOGGING (JSONL) + WAV-DUMP
 * ============================================================ */

static void json_escape_append(char *dst, size_t dst_size, const char *src) {
    size_t di = strlen(dst);
    for (size_t si = 0; src[si] && di + 2 < dst_size; si++) {
        unsigned char c = (unsigned char)src[si];
        if (c == '"' || c == '\\') { dst[di++] = '\\'; dst[di++] = c; }
        else if (c == '\n') { dst[di++] = '\\'; dst[di++] = 'n'; }
        else if (c >= 32 && c < 127) { dst[di++] = c; }
        else { di += snprintf(dst + di, dst_size - di, "\\u%04x", c); }
    }
    dst[di] = 0;
}

static void iso_timestamp_utc(char *out, size_t out_size) {
    SYSTEMTIME st;
    GetSystemTime(&st);
    snprintf(out, out_size, "%04d-%02d-%02dT%02d:%02d:%02d.%03dZ",
              st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
}

static void log_message(const AcarsResult *r) {
    char ts[64];
    iso_timestamp_utc(ts, sizeof(ts));

    char text_escaped[700] = {0};
    json_escape_append(text_escaped, sizeof(text_escaped), r->text);

    FILE *f = fopen(LOG_FILE, "a");
    if (f) {
        fprintf(f, "{\"ok\": %s, \"reason\": \"%s\", \"text\": \"%s\", "
                    "\"received_bcs\": %u, \"calculated_bcs\": %u, \"timestamp_utc\": \"%s\"}\n",
                r->ok ? "true" : "false", r->reason, text_escaped,
                r->received_bcs, r->calculated_bcs, ts);
        fclose(f);
    }

    if (r->ok) {
        printf("\n[%s] ACARS OK: \"%s\"\n", ts, r->text);
    } else {
        printf("\n[%s] Dekodierung fehlgeschlagen: %s\n", ts, r->reason);
    }
}

static void dump_debug_wav(const double *audio, int n, const char *reason) {
    CreateDirectoryA(DEBUG_DIR, NULL); /* ignoriert Fehler, falls Ordner schon existiert */

    SYSTEMTIME st;
    GetSystemTime(&st);
    char fname[512];
    snprintf(fname, sizeof(fname), "%s\\burst_%s_%04d%02d%02d_%02d%02d%02d_%03d.wav",
             DEBUG_DIR, reason, st.wYear, st.wMonth, st.wDay,
             st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);

    double max_abs = 1e-12;
    for (int i = 0; i < n; i++) if (fabs(audio[i]) > max_abs) max_abs = fabs(audio[i]);

    int16_t *pcm = malloc(sizeof(int16_t) * n);
    for (int i = 0; i < n; i++) {
        double v = audio[i] / max_abs * 32767.0;
        if (v > 32767) v = 32767;
        if (v < -32768) v = -32768;
        pcm[i] = (int16_t)v;
    }

    FILE *f = fopen(fname, "wb");
    if (f) {
        uint32_t data_bytes = (uint32_t)(n * sizeof(int16_t));
        uint32_t byte_rate = AUDIO_RATE * 2;
        uint16_t block_align = 2;
        uint16_t bits_per_sample = 16;
        uint16_t audio_format = 1; /* PCM */
        uint16_t num_channels = 1;
        uint32_t sample_rate = AUDIO_RATE;
        uint32_t fmt_size = 16;
        uint32_t riff_size = 36 + data_bytes;

        fwrite("RIFF", 1, 4, f); fwrite(&riff_size, 4, 1, f); fwrite("WAVE", 1, 4, f);
        fwrite("fmt ", 1, 4, f); fwrite(&fmt_size, 4, 1, f);
        fwrite(&audio_format, 2, 1, f); fwrite(&num_channels, 2, 1, f);
        fwrite(&sample_rate, 4, 1, f); fwrite(&byte_rate, 4, 1, f);
        fwrite(&block_align, 2, 1, f); fwrite(&bits_per_sample, 2, 1, f);
        fwrite("data", 1, 4, f); fwrite(&data_bytes, 4, 1, f);
        fwrite(pcm, sizeof(int16_t), n, f);
        fclose(f);
    }
    free(pcm);
}

/* ============================================================
 * VERARBEITUNGS-PIPELINE (globaler Zustand fuer den Burst-Callback)
 * ============================================================ */

static volatile LONG g_stop = 0;
static RingBuffer g_ring;

static void on_burst_complete(const double *samples, int n) {
    AcarsResult r = decode_burst(samples, n);
    log_message(&r);
    if (DEBUG_DUMP_FAILED && !r.ok) {
        dump_debug_wav(samples, n, r.reason);
    }
}

static BOOL WINAPI console_handler(DWORD signal) {
    if (signal == CTRL_C_EVENT || signal == CTRL_CLOSE_EVENT) {
        InterlockedExchange(&g_stop, 1);
        return TRUE;
    }
    return FALSE;
}

typedef struct { RtlSdr *sdr; } CaptureThreadArgs;

static void __cdecl rtlsdr_callback(unsigned char *buf, uint32_t len, void *ctx) {
    (void)ctx;
    if (g_stop) return;
    ring_push(&g_ring, buf, len);
}

static DWORD WINAPI capture_thread_fn(LPVOID arg) {
    CaptureThreadArgs *args = (CaptureThreadArgs *)arg;
    /* Puffergroesse/-anzahl fuer rtlsdr_read_async: 16 Puffer a 16384 Bytes
       ist ein bewaehrter Standardwert (siehe librtlsdr-Beispiele). */
    args->sdr->read_async(args->sdr->dev, rtlsdr_callback, NULL, 16, 16384);
    return 0;
}

int main(void) {
    RtlSdr sdr;
    if (!rtlsdr_load(&sdr, DLL_PATH)) return 1;

    if (sdr.get_device_count() == 0) {
        fprintf(stderr, "Kein RTL-SDR Geraet gefunden.\n");
        return 1;
    }
    if (sdr.open_dev(&sdr.dev, 0) != 0) {
        fprintf(stderr, "RTL-SDR konnte nicht geoeffnet werden.\n");
        return 1;
    }

    sdr.set_center_freq(sdr.dev, CENTER_FREQ);
    sdr.set_sample_rate(sdr.dev, IQ_SAMPLE_RATE);
    sdr.set_freq_correction(sdr.dev, FREQ_CORRECTION);

    if (MANUAL_GAIN_TENTH_DB >= 0) {
        sdr.set_tuner_gain_mode(sdr.dev, 1);
        sdr.set_tuner_gain(sdr.dev, MANUAL_GAIN_TENTH_DB);
    } else {
        sdr.set_tuner_gain_mode(sdr.dev, 0);
    }
    sdr.reset_buffer(sdr.dev);

    printf("Empfange auf %.3f MHz, %.3f MSps IQ, %d Hz Audio, %.1f Samples/Bit.\n",
           CENTER_FREQ / 1e6, IQ_SAMPLE_RATE / 1e6, AUDIO_RATE, SAMPLES_PER_BIT);
    printf("Strg+C zum Beenden.\n\n");

    SetConsoleCtrlHandler(console_handler, TRUE);

    ring_init(&g_ring, RINGBUF_BYTES);
    biquad_init();
    Squelch squelch;
    squelch_init(&squelch, AUDIO_RATE, on_burst_complete);

    CaptureThreadArgs cap_args = { &sdr };
    HANDLE cap_thread = CreateThread(NULL, 0, capture_thread_fn, &cap_args, 0, NULL);

    unsigned char *iq_bytes = malloc(NUM_IQ_SAMPLES * 2);
    double *audio_chunk = malloc(sizeof(double) * NUM_IQ_SAMPLES); /* Obergrenze */

    LARGE_INTEGER status_last, status_now, freq_perf;
    QueryPerformanceFrequency(&freq_perf);
    QueryPerformanceCounter(&status_last);

    while (!g_stop) {
        if (!ring_pop(&g_ring, iq_bytes, (size_t)NUM_IQ_SAMPLES * 2, &g_stop)) break;

        int n_audio = 0;
        am_envelope_filter_decimate(iq_bytes, NUM_IQ_SAMPLES, audio_chunk, &n_audio);
        squelch_push(&squelch, audio_chunk, n_audio);

        /* Text-Statuszeile ca. 2x pro Sekunde aktualisieren */
        QueryPerformanceCounter(&status_now);
        double elapsed = (double)(status_now.QuadPart - status_last.QuadPart) / freq_perf.QuadPart;
        if (elapsed > 0.5) {
            status_last = status_now;
            const char *state = squelch.in_burst ? "SIGNAL" : "ruhig ";
            if (squelch.have_noise_floor) {
                double thr = squelch.noise_floor * SQUELCH_RATIO;
                double ratio = squelch.smoothed_power / (thr > 0 ? thr : 1e-12);
                int bar_len = (int)(ratio * 20);
                if (bar_len > 40) bar_len = 40;
                if (bar_len < 0) bar_len = 0;
                char bar[41];
                memset(bar, '#', bar_len);
                bar[bar_len] = 0;
                printf("\r[%s] Pegel/Schwelle=%.2f |%-40s|   ", state, ratio, bar);
            } else {
                printf("\r[%s] Rauschboden wird kalibriert...   ", state);
            }
            fflush(stdout);
        }

        if (g_ring.overflow_count > 0) {
            fprintf(stderr, "\nWarnung: Ringpuffer-Overrun (%ld Bytes verloren) - "
                             "Verarbeitung ist zu langsam fuer die Datenrate.\n",
                    g_ring.overflow_count);
            g_ring.overflow_count = 0;
        }
    }

    printf("\nBeende...\n");
    sdr.cancel_async(sdr.dev);
    WaitForSingleObject(cap_thread, 2000);
    CloseHandle(cap_thread);
    sdr.close_dev(sdr.dev);
    FreeLibrary(sdr.handle);

    free(iq_bytes);
    free(audio_chunk);
    free(g_ring.buf);
    free(squelch.power_history);
    free(squelch.history_scratch);
    free(squelch.burst_buf);

    printf("RTL-SDR geschlossen.\n");
    return 0;
}

/* ============================================================
 * ANLEITUNG: CRC/BCS-PARAMETER VERIFIZIEREN (wie in der Python-Version)
 * ============================================================
 *
 * 1. Sammle mit DEBUG_DUMP_FAILED WAV-Schnipsel echter Bursts mit
 *    plausiblem Klartext ("text" im JSONL-Log) aber reason=crc_mismatch.
 * 2. Vergleiche mit einer Referenzdekodierung (z.B. acarsdec) fuer denselben
 *    Burst, um die tatsaechlichen BCS-Bytes zu bekommen.
 * 3. Probiere in crc_ccitt() alternative Parameter (Poly 0x1021 statt
 *    0x8408, Init 0x0000 statt 0xFFFF) sowie andere Payload-Grenzen in
 *    parse_acars_frame(), bis calculated_bcs == received_bcs passt.
 */
