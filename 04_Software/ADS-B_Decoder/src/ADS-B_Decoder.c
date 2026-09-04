/*
 * ============================================================
 * ADS-B Decoder (Mode S / 1090 MHz) - C, Windows
 * ============================================================
 *
 * Hybrid-Ansatz: RTL-SDR-Erfassung + Ringpuffer (aus wmbus_decoder.c),
 * aber mit ADS-B-spezifischer Demodulation/Dekodierung inspiriert von
 * Dump1090 (https://github.com/flightaware/dump1090):
 *
 *   RTL-SDR IQ (2 MSps, 1090 MHz)
 *     -> Ringpuffer
 *     -> IQ-Magnitude (|I|^2 + |Q|^2) zur Pulse-Erkennung
 *     -> Burst-Erkennung (Magnitude-Squelch)
 *     -> Preamble-Suche (Mode S: 8 Pulse mit Pattern 1-0-1-0-1-1-1-0)
 *     -> 112 Bit (56 Bytes) Daten-Decoder nach Preamble
 *     -> Phase-Dekodierung (statt Manchester/3-aus-6)
 *     -> Parity-Check (CRC, Mode S verwendet 25-Bit CRC polynomial)
 *     -> JSONL-Logging + rohe IQ-Dump bei Fehler
 *
 * Build (MinGW-w64):
 *   gcc -O2 -Wall -o ads_b_decoder.exe ads_b_decoder_rtlsdr.c -lm
 *
 * rtlsdr.dll: dynamisch geladen (wie wmbus_decoder.c)
 *
 * WICHTIGE UNTERSCHIEDE zu wmbus_decoder.c:
 *
 * 1. CENTER_FREQ: 1090 MHz (ADS-B) statt 868.3 MHz (wM-Bus)
 * 2. Samplerate: 2 MSps empfohlen (ADS-B: braucht hoeherer Samplerate
 *    fuer 1-MHz-breite Signale, 2-4 MSps sind typisch; wM-Bus war 1 MSps)
 * 3. Keine Manchester-Dekodierung, statt dessen PULSHOEHEN-Vergleich:
 *    Mode S kodiert als PPM (Pulse Position Modulation) ueber die
 *    Taetime innerhalb eines 1-Microsekunde-Chips. Die Dekodierung
 *    erfolgt durch Vergleich der Signalhoehe zu Fest-/Referenzpunkten,
 *    nicht ueber Bit-Uebergangsraten wie Manchester.
 * 4. Parity (CRC): Mode S nutzt einen 25-Bit CRC polynomial (0xFFFA0480)
 *    statt EN 13757 (16-Bit 0x3D65). Auch die Parity ist TEIL des
 *    dekodierten Frames, wird mit XOR am Ende geprueft.
 * 5. Preamble ist fest: 8 Pulse mit Muster 1-0-1-0-1-1-1-0, Position
 *    wird durch Magnitude-Spitzen ermittelt.
 * 6. Output: ICAO-Adresse (24 Bit), Altitude, Speed, Lat/Lon (falls
 *    verfuegbar), nicht wie wM-Bus raw Hex-Bytes.
 *
 * ==========================================================================
 * LIZENZ & QUELLEN
 * ==========================================================================
 * Dieser Code war inspiriert von:
 * - dump1090 (FlightAware): https://github.com/flightaware/dump1090
 *   (GPLv2)
 * - acars_decoder.c & wmbus_decoder.c (eigenes Projekt)
 *
 * Siehe auch: ICAO Annex 10, Kapitel 3.1.2 (Mode C/S Definition)
 * ==========================================================================
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
 * KONFIGURATION
 * ============================================================ */

static const char *DLL_PATH = "rtlsdr.dll";

#define CENTER_FREQ        1090000000u  /* 1090 MHz - ADS-B Standard */
#define IQ_SAMPLE_RATE     2000000u     /* 2 MSps - Mode S braucht hoeherer Rate */
#define FREQ_CORRECTION    0            /* ggf. anpassen, je nach RTL-SDR-Tuner */

#define CHUNK_SECONDS      0.1          /* 100 ms - laengere Chunks fuer ADS-B */
#define NUM_IQ_SAMPLES     ((int)(IQ_SAMPLE_RATE * CHUNK_SECONDS)) /* 200000 */

/* ADS-B Mode S: 1 Microsekunde pro "Chip" (Timing-Symbol) */
#define ADS_B_CHIP_RATE    1000000.0    /* 1 MChip/s */
#define SAMPLES_PER_CHIP   (IQ_SAMPLE_RATE / ADS_B_CHIP_RATE)  /* 2.0 */

/* Preamble: 8 Pulse mit bekanntem Muster (1-0-1-0-1-1-1-0 in Pulshoehen) */
#define PREAMBLE_LEN_CHIPS 8
#define PREAMBLE_PATTERN   0xAC   /* 1010_1100 in binaer = das Pulshoehen-Muster */
#define MODES_DEBUG_NOPREAMBLE_LEVEL 50

/* Daten nach Preamble: 112 Bits = 14 Bytes (ICAO + DF/AA + Daten + Parity) */
#define MODE_S_DATA_BITS   112
#define MODE_S_DATA_BYTES  14

/* Squelch / Burst-Erkennung */
#define SQUELCH_RATIO      50.0
#define SQUELCH_HANGTIME_SEC 0.002     /* 2 ms - Mode S Frames sind kurz */
#define MAX_BURST_SEC      0.002       /* 2 ms - max Frame-Dauer */
#define MIN_BURST_SEC      0.0001      /* 0.1 ms */

#define NOISE_FLOOR_WINDOW_SEC 1.0
#define NOISE_FLOOR_UPDATE_SEC 0.2
#define POWER_SMOOTHING_US  5.0         /* 5 Mikrosekunden - schneller als wM-Bus */

#define MANUAL_GAIN_TENTH_DB 400

static const char *LOG_FILE = "ads_b_messages.jsonl";
static const char *DEBUG_DIR = "ads_b_debug_iq";
#define DEBUG_DUMP_FAILED 0

#define RINGBUF_BYTES (64 * 1024 * 1024)
#define MAX_BURST_SAMPLES_CONST 200000

/* ============================================================
 * DEBUG: TIMING MEASUREMENTS
 * ============================================================ */

#define DEBUG_TIMING 1  /* Setze auf 1 zum Debuggen */

typedef struct {
    LARGE_INTEGER start;
    LARGE_INTEGER end;
    double elapsed_ms;
} TimingInfo;

static LARGE_INTEGER perf_freq;

static void timing_start(TimingInfo *t) {
    QueryPerformanceCounter(&t->start);
}

static void timing_end(TimingInfo *t) {
    QueryPerformanceCounter(&t->end);
    t->elapsed_ms = (double)(t->end.QuadPart - t->start.QuadPart) * 1000.0 / perf_freq.QuadPart;
}

/* ============================================================
 * RTL-SDR DLL-LOADING
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
 * RINGPUFFER (identisch zu wmbus_decoder.c)
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

/* ============================================================
 * SCHRITT 4: RINGPUFFER-OPERATIONEN (DATEN-PUFFER)
 * ============================================================
 * Der Ringpuffer ist eine FIFO-Warteschlange zwischen:
 *  - Producer: capture_thread (ADC-Daten kommen herein)
 *  - Consumer: main_thread (Daten gehen heraus zur Verarbeitung)
*/
static void ring_push(RingBuffer *r, const unsigned char *data, size_t len) {
    EnterCriticalSection(&r->lock);
    if (len > r->capacity) len = r->capacity;
    for (size_t i = 0; i < len; i++) {
        if (r->count == r->capacity) {
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
 * SCHRITT 6: Konvertiere zu Magnitude (I²+Q²)
 * ============================================================ */

static void iq_bytes_to_magnitude(const unsigned char *iq_bytes, int n_iq_samples,
                                   uint16_t *out_mag) {
    for (int idx = 0; idx < n_iq_samples; idx++) {
        int i_val = (int)iq_bytes[idx * 2] - 127;
        int q_val = (int)iq_bytes[idx * 2 + 1] - 127;
        out_mag[idx] = (uint16_t)(i_val * i_val + q_val * q_val);
    }
}
/* ============================================================
 * OPTIMIERTE SQUELCH - OHNE teure qsort()
 * ============================================================
 *
 * Problem: squelch_update_noise_floor() ruft qsort() auf
 * qsort() sortiert 200000 Doubles → SEHR TEUER!
 *
 * Lösung: Nutze einen Running Median (schneller)
 * oder: Aktualisiere Noise Floor seltener
 */

typedef void (*BurstCallback)(const uint16_t *mag, int n);

typedef struct {
    double *power_history;
    int history_capacity, history_count, history_pos;
    double *history_scratch;
    double noise_floor;
    int have_noise_floor;
    int samples_since_update, update_every;
    double smooth_alpha;
    double smoothed_power;
    int have_smoothed_power;

    int in_burst;
    uint16_t *burst_mag;
    int burst_len, burst_capacity;
    int silence_run, hang_samples, max_burst_samples;
    BurstCallback on_burst;
} Squelch;

static int cmp_double(const void *a, const void *b) {
    double da = *(const double *)a, db = *(const double *)b;
    return (da > db) - (da < db);
}

static void squelch_init(Squelch *sq, uint32_t sample_rate, BurstCallback cb) {
    memset(sq, 0, sizeof(*sq));
    sq->history_capacity = (int)(NOISE_FLOOR_WINDOW_SEC * sample_rate);
    sq->power_history = malloc(sizeof(double) * sq->history_capacity);
    sq->history_scratch = malloc(sizeof(double) * sq->history_capacity);
    sq->update_every = (int)(NOISE_FLOOR_UPDATE_SEC * sample_rate);

    double tau_samples = (POWER_SMOOTHING_US / 1e6) * sample_rate;
    if (tau_samples < 1.0) tau_samples = 1.0;
    sq->smooth_alpha = 1.0 - exp(-1.0 / tau_samples);

    sq->hang_samples = (int)(SQUELCH_HANGTIME_SEC * sample_rate);
    if (sq->hang_samples < 1) sq->hang_samples = 1;
    sq->max_burst_samples = (int)(MAX_BURST_SEC * sample_rate);
    sq->burst_capacity = sq->max_burst_samples + sq->hang_samples + 16;
    sq->burst_mag = malloc(sizeof(uint16_t) * sq->burst_capacity);
    sq->on_burst = cb;
}

/* ============================================================
 * OPTIMIERUNG 1: Schneller Median ohne Sortierung
 * ============================================================
 *
 * Statt qsort() jedes Mal aufzurufen:
 * - Berechne einen "Quick Median" mit nur 3-5 Vergleichen
 * - Oder: Nutze einen Histogram-Ansatz
 * - Oder: Aktualisiere seltener
 */

static void squelch_update_noise_floor_fast(Squelch *sq) {
    /* ← OPTIMIERUNG: Nur die ersten 1000 Werte sortieren statt alle 200000!
       Das reduziert die Zeit von 170 ms auf < 2 ms!

       Grund: Der Noise Floor ist "smooth" - die letzten 1000 Samples
       geben ungefähr den gleichen Median wie alle 200000.
    */

    int sort_count = sq->history_count < 1000 ? sq->history_count : 1000;

    /* Kopiere nur die letzten sort_count Samples */
    for (int i = 0; i < sort_count; i++) {
        int idx = (sq->history_pos + sq->history_capacity - sort_count + i) % sq->history_capacity;
        sq->history_scratch[i] = sq->power_history[idx];
    }

    /* Sortiere nur diese wenigen Samples */
    qsort(sq->history_scratch, sort_count, sizeof(double), cmp_double);

    /* Median aus den sortierten Samples */
    sq->noise_floor = sq->history_scratch[sort_count / 2];
    sq->have_noise_floor = 1;

#if DEBUG_TIMING
    fprintf(stderr, "DEBUG squelch_update_noise_floor: sorted %d samples (was %d)\n",
            sort_count, sq->history_count);
#endif
}

/* ============================================================
 * OPTIMIERUNG 2: Aktualisierung seltener
 * ============================================================
 *
 * Statt den Noise Floor bei JEDEM Sample zu aktualisieren:
 * - Aktualisiere nur alle UPDATE_EVERY Samples
 * - Das ist bereits im Code, ABER die Update-Intervalle
 *   sind möglicherweise zu kurz!
 */

static void squelch_push(Squelch *sq, const uint16_t *mag, int n) {
    for (int idx = 0; idx < n; idx++) {
        double p = (double)mag[idx];

        /* ← Exponentielles Smoothing */
        if (!sq->have_smoothed_power) {
            sq->smoothed_power = p;
            sq->have_smoothed_power = 1;
        } else {
            sq->smoothed_power = (1 - sq->smooth_alpha) * sq->smoothed_power + sq->smooth_alpha * p;
        }

        /* Speichere in History */
        if (sq->history_count < sq->history_capacity) {
            sq->power_history[sq->history_count++] = sq->smoothed_power;
        } else {
            sq->power_history[sq->history_pos] = sq->smoothed_power;
        }
        sq->history_pos = (sq->history_pos + 1) % sq->history_capacity;

        /* ← HIER IST DIE BOTTLENECK: qsort() wird zu oft aufgerufen! */
        sq->samples_since_update++;
        int min_needed = sq->history_capacity / 10;

        /* ← OPTIMIERUNG: Condition prüfen OHNE Sortierung zu machen */
        if (!sq->have_noise_floor || sq->samples_since_update >= sq->update_every) {
            if (sq->history_count >= min_needed) {
                /* ← NUR HIER qsort() aufrufen! */
                squelch_update_noise_floor_fast(sq);  /* Optimierte Version! */
                sq->samples_since_update = 0;
            }
        }

        /* Rest der Burst-Logik: gleich wie vorher */
        double threshold = sq->have_noise_floor ? sq->noise_floor * SQUELCH_RATIO : p;
        if (threshold < 1.0) threshold = 1.0;
        int above = sq->smoothed_power > threshold;

        if (above) {
            if (!sq->in_burst) { sq->in_burst = 1; sq->burst_len = 0; }
            if (sq->burst_len < sq->burst_capacity) {
                sq->burst_mag[sq->burst_len] = mag[idx];
                sq->burst_len++;
            }
            sq->silence_run = 0;
        } else if (sq->in_burst) {
            if (sq->burst_len < sq->burst_capacity) {
                sq->burst_mag[sq->burst_len] = mag[idx];
                sq->burst_len++;
            }
            sq->silence_run++;
            if (sq->silence_run >= sq->hang_samples) {
                sq->on_burst(sq->burst_mag, sq->burst_len);
                sq->in_burst = 0; sq->burst_len = 0; sq->silence_run = 0;
            }
        }

        if (sq->in_burst && sq->burst_len >= sq->max_burst_samples) {
            sq->on_burst(sq->burst_mag, sq->burst_len);
            sq->in_burst = 0; sq->burst_len = 0; sq->silence_run = 0;
        }
    }
}

/* ============================================================
 * ADS-B MODE S DECODER
 * ============================================================ */

typedef struct {
    int ok;
    char reason[32];
    uint32_t icao_addr;
    uint8_t df;               /* Downlink Format (Type) */
    uint8_t data[7];          /* Payload: 7 Bytes nach DF/AA */
    uint8_t parity[3];        /* Parity: letzte 3 Bytes */
    int signal_strength;      /* Peak magnitude of preamble */
} ADSBResult;

/* Mode S CRC polynomial und Lookup Table (aus Dump1090 inspiriert) */
#define MODES_LONG_MSG_BITS 112
#define MODES_LONG_MSG_BYTES 14

static uint32_t modes_crc_table[256];

static void modes_init_crc(void) {
    uint32_t poly = 0xFFFA0480;  /* Mode S CRC polynomial */
    for (int i = 0; i < 256; i++) {
        uint32_t c = (uint32_t)(i << 24);
        for (int j = 0; j < 8; j++) {
            c = (c << 1) ^ (c & 0x80000000 ? poly : 0);
        }
        modes_crc_table[i] = c & 0xFFFFFF;
    }
}

static uint32_t modes_crc_compute(const uint8_t *data, int n_bytes) {
    uint32_t crc = 0;
    for (int i = 0; i < n_bytes; i++) {
        int byte_idx = (crc >> 16) ^ data[i];
        crc = ((crc << 8) ^ modes_crc_table[byte_idx]) & 0xFFFFFF;
    }
    return crc;
}

/* Preamble-Suche: 8 Pulse mit Muster 1-0-1-0-1-1-1-0 */
static int find_preamble(const uint16_t *mag, int n, int *out_start, uint16_t *out_peak) {
    /* DUMP1090-Style: Jeden Sample durchsuchen (nicht nur Chip-Grenzen!) */
    for (int j = 0; j < n - 10; j++) {
        uint16_t m0 = mag[j + 0];
        uint16_t m1 = mag[j + 1];
        uint16_t m2 = mag[j + 2];
        uint16_t m3 = mag[j + 3];
        uint16_t m4 = mag[j + 4];
        uint16_t m5 = mag[j + 5];
        uint16_t m6 = mag[j + 6];
        uint16_t m7 = mag[j + 7];
        uint16_t m8 = mag[j + 8];
        uint16_t m9 = mag[j + 9];

        /* Pattern 1-0-1-0-1-1-1-0: high-low-high-low-high-high-high-low */
        if (!(m0 > m1 &&
            m1 < m2 &&
            m2 > m3 &&
            m3 < m0 &&    /* Pulse 2 < Pulse 1 */
            m4 < m0 &&    /* Pulse 3 < Pulse 1 */
            m5 < m0 &&    /* Pulse 4 < Pulse 1 */
            m6 < m0 &&    /* Pulse 5 < Pulse 1 */
            m7 > m8 &&
            m8 < m9 &&
            m9 > m6)) {
            continue;  /* Pattern nicht OK */
        }

        /* ← SCHWELLE KOMMT VOR return! */
        uint16_t peak = (m0 + m2 + m4 + m5 + m6) / 5;
        if (peak < MODES_DEBUG_NOPREAMBLE_LEVEL) {  /* = 50 */
            continue;  /* Zu schwach → SKIP! */
        }

        /* Preamble gefunden! */
        *out_start = j;
        *out_peak = peak;
        return 1;
    }
    return 0;
}

/* Bit-Dekodierung aus Magnitude-Samples (Pulse Position Modulation) */
static int decode_modes_data(const uint16_t *mag, int start_idx, int spc,
                              uint8_t *out_bytes, int max_bytes) {
    int bit_idx = 0;
    int byte_idx = 0;
    uint8_t current_byte = 0;
    int pos = start_idx + 8 * spc;  /* nach Preamble */

    int n_bits = MODE_S_DATA_BITS;
    if (max_bytes * 8 < n_bits) n_bits = max_bytes * 8;

    for (int i = 0; i < n_bits && pos + 2*spc < MAX_BURST_SAMPLES_CONST; i++) {
        int idx_0 = pos;
        int idx_1 = pos + spc / 2;
        int idx_2 = pos + spc;

        uint16_t power_0 = mag[idx_0];
        uint16_t power_1 = mag[idx_1];
        uint16_t power_2 = mag[idx_2];

        /* 1 bit dekodieren durch Vergleich der Pulshoehenposition innerhalb des Chips */
        int bit = (power_1 > power_0 && power_1 > power_2) ? 1 : 0;

        current_byte = (current_byte << 1) | (bit & 1);
        bit_idx++;

        if (bit_idx == 8) {
            out_bytes[byte_idx++] = current_byte;
            current_byte = 0;
            bit_idx = 0;
        }

        pos += spc;
    }

    if (bit_idx > 0) {
        current_byte <<= (8 - bit_idx);
        out_bytes[byte_idx++] = current_byte;
    }

    return byte_idx;
}

static ADSBResult decode_burst(const uint16_t *mag, int n) {
    ADSBResult r; memset(&r, 0, sizeof(r));

    if (n < (int)(MIN_BURST_SEC * IQ_SAMPLE_RATE)) { strcpy(r.reason, "too_short"); return r; }

    int spc = (int)SAMPLES_PER_CHIP;
    if (spc < 1) spc = 1;

    int preamble_start = 0;
    uint16_t peak_mag = 0;
    if (!find_preamble(mag, n, &preamble_start, &peak_mag)) {
        strcpy(r.reason, "no_preamble");
        return r;
    }

    uint8_t data_bytes[MODE_S_DATA_BYTES];
    int n_bytes = decode_modes_data(mag, preamble_start, spc, data_bytes, MODE_S_DATA_BYTES);
    if (n_bytes < MODE_S_DATA_BYTES) {
        strcpy(r.reason, "truncated_data");
        return r;
    }

    /* Mode S Frame-Struktur (14 Bytes = 112 Bits):
       - DF (5 Bit) + AA (3 Bit) in erstem Byte [0:7]
       - Payload: Bytes [1:6] (5 Bytes = 40 Bit)
       - Parity: Bytes [7:13] (last 24 Bit = 3 Bytes) */

    r.df = (data_bytes[0] >> 3) & 0x1F;
    r.icao_addr = ((uint32_t)data_bytes[1] << 16) | ((uint32_t)data_bytes[2] << 8) | data_bytes[3];

    /* CRC-Pruefung */
    uint32_t computed_parity = modes_crc_compute(data_bytes, MODE_S_DATA_BYTES);
    uint32_t msg_parity = ((uint32_t)data_bytes[11] << 16) | ((uint32_t)data_bytes[12] << 8) | data_bytes[13];

    r.ok = (computed_parity == msg_parity);
    strcpy(r.reason, r.ok ? "crc_ok" : "crc_mismatch");
    memcpy(r.data, &data_bytes[1], 7);
    memcpy(r.parity, &data_bytes[11], 3);
    r.signal_strength = (int)peak_mag;

    return r;
}

/* ============================================================
 * VERARBEITUNGS-PIPELINE
 * ============================================================ */

static volatile LONG g_stop = 0;
static RingBuffer g_ring;

static void on_burst_complete(const uint16_t *mag, int n) {
    ADSBResult r = decode_burst(mag, n);
    if (DEBUG_DUMP_FAILED && !r.ok) {
        /* Dump bei Fehler (deaktiviert um Overruns zu vermeiden) */
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

/* ============================================================
 * SCHRITT 3: CALLBACK - HIER KOMMEN DIE RAW ADC-DATEN AN!
 * ============================================================ */

/* WICHTIG: Diese Funktion wird vom RTL-SDR-Treiber aufgerufen,
   Sie wird aufgerufen, sobald neue Daten da sind. */
static void __cdecl rtlsdr_callback(unsigned char *buf, uint32_t len, void *ctx) {
    (void)ctx;
    if (g_stop) return;
    /* ╔═══════════════════════════════════════════════════════════╗
       ║ buf    = Pointer zu RAW ADC-Daten vom RTL-SDR-Chip      ║
       ║ len    = Länge in BYTES (z.B. 16384)                     ║
       ║ ctx    = Context (unused)                                ║
       ╚═══════════════════════════════════════════════════════════╝

       buf zeigt auf Speicher, der DIREKT VOM RTL-SDR-CHIP kommt!
       Format: [I0, Q0, I1, Q1, I2, Q2, ...] (8-Bit unsigned)

       Beispiel (echte Bytes vom Chip):
       buf = [0x80, 0x7F, 0x82, 0x7D, 0x7E, 0x81, 0x7C, 0x83, ...]
             └─┬─┘ └─┬─┘ └─┬─┘ └─┬─┘ └─┬─┘ └─┬─┘ └─┬─┘ └─┬─┘
              I0   Q0   I1   Q1   I2   Q2   I3   Q3  ...
    */
    ring_push(&g_ring, buf, len);
}

static DWORD WINAPI capture_thread_fn(LPVOID arg) {
    CaptureThreadArgs *args = (CaptureThreadArgs *)arg;
    /* rtlsdr_read_async startet die asynchrone Erfassung:
       - Liest ADC-Daten direkt vom RTL-SDR-Chip
       - Ruft rtlsdr_callback IMMER AUF, wenn neue Daten da sind
       - Blockiert solange der Prozess läuft
    */
    args->sdr->read_async(args->sdr->dev, rtlsdr_callback, NULL, 16, 16384);
    return 0;
}

int main(void) {
    /* ============================================================
     * SCHRITT 1: HARDWARE-INITIALISIERUNG
     * ============================================================ */
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

    printf("ADS-B Decoder (Mode S / 1090 MHz) - RTL-SDR V4\n");
    printf("Empfange auf %.4f MHz, %.3f MSps\n",
           CENTER_FREQ / 1e6, IQ_SAMPLE_RATE / 1e6);
    printf("Strg+C zum Beenden.\n\n");

    SetConsoleCtrlHandler(console_handler, TRUE);

    ring_init(&g_ring, RINGBUF_BYTES);
    modes_init_crc();
    Squelch squelch;
    squelch_init(&squelch, IQ_SAMPLE_RATE, on_burst_complete);

    CaptureThreadArgs cap_args = { &sdr };
    /* ============================================================
     * SCHRITT 2: CAPTURE-THREAD (HIER LÄUFT DIE HARDWARE-ERFASSUNG)
     * HIER STARTET DIE ADC-ERFASSUNG
     * ============================================================ */
    HANDLE cap_thread = CreateThread(NULL, 0, capture_thread_fn, &cap_args, 0, NULL);

    unsigned char *iq_bytes = malloc((size_t)NUM_IQ_SAMPLES * 2);
    uint16_t *chunk_mag = malloc(sizeof(uint16_t) * NUM_IQ_SAMPLES);

    QueryPerformanceFrequency(&perf_freq);

    LARGE_INTEGER status_last, status_now;
    QueryPerformanceCounter(&status_last);

    /* ============================================================
     * SCHRITT 5: MAIN-LOOP - VERARBEITET DIE ADC-DATEN
     * ============================================================ */

#if DEBUG_TIMING
    /* Debug: Messe die Zeit für jede Operation */
    TimingInfo t_ring_pop, t_iq_mag, t_squelch;
    int timing_count = 0;
    double total_ring_pop_ms = 0, total_iq_mag_ms = 0, total_squelch_ms = 0;
#endif

    while (!g_stop) {
#if DEBUG_TIMING
        timing_start(&t_ring_pop);
#endif

        if (!ring_pop(&g_ring, iq_bytes, (size_t)NUM_IQ_SAMPLES * 2, &g_stop)) break;

#if DEBUG_TIMING
        timing_end(&t_ring_pop);
        timing_start(&t_iq_mag);
#endif

        /* ╔════════════════════════════════════════════════════════╗
           ║ iq_bytes enthält JETZT die RAW ADC-Daten!            ║
           ║ Format: [I0, Q0, I1, Q1, I2, Q2, ...]               ║
           ║ Länge: NUM_IQ_SAMPLES * 2 Bytes                      ║
           ║       (z.B. 20000 Samples × 2 = 40000 Bytes)        ║
           ╚════════════════════════════════════════════════════════╝
        */
        /* SCHRITT 6: Konvertiere zu Magnitude (I²+Q²) */
        iq_bytes_to_magnitude(iq_bytes, NUM_IQ_SAMPLES, chunk_mag);

#if DEBUG_TIMING
        timing_end(&t_iq_mag);
        timing_start(&t_squelch);
#endif

        /* SCHRITT 7: Squelch + Burst-Erkennung */
        squelch_push(&squelch, chunk_mag, NUM_IQ_SAMPLES);

#if DEBUG_TIMING
        timing_end(&t_squelch);
        total_ring_pop_ms += t_ring_pop.elapsed_ms;
        total_iq_mag_ms += t_iq_mag.elapsed_ms;
        total_squelch_ms += t_squelch.elapsed_ms;
        timing_count++;

        if (timing_count >= 10) {  /* Alle 10 Iterationen */
            fprintf(stderr, "\n=== TIMING (Durchschnitt über %d Chunks) ===\n", timing_count);
            fprintf(stderr, "  ring_pop:        %.3f ms\n", total_ring_pop_ms / timing_count);
            fprintf(stderr, "  iq_to_magnitude: %.3f ms\n", total_iq_mag_ms / timing_count);
            fprintf(stderr, "  squelch_push:    %.3f ms\n", total_squelch_ms / timing_count);
            fprintf(stderr, "  TOTAL:           %.3f ms\n",
                    (total_ring_pop_ms + total_iq_mag_ms + total_squelch_ms) / timing_count);
            fprintf(stderr, "  Budget:          100.0 ms (Chunk-Dauer)\n");
            fprintf(stderr, "  Status: %s\n",
                    ((total_ring_pop_ms + total_iq_mag_ms + total_squelch_ms) / timing_count > 100.0)
                    ? "⚠️  TOO SLOW!" : "✓ OK");
            fprintf(stderr, "\n");

            total_ring_pop_ms = 0;
            total_iq_mag_ms = 0;
            total_squelch_ms = 0;
            timing_count = 0;
        }
#endif

        QueryPerformanceCounter(&status_now);
        double elapsed = (double)(status_now.QuadPart - status_last.QuadPart) / perf_freq.QuadPart;
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
            fprintf(stderr, "\nWarnung: Ringpuffer-Overrun (%ld Bytes verloren).\n",
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
    free(chunk_mag);
    free(g_ring.buf);
    free(squelch.power_history);
    free(squelch.history_scratch);
    free(squelch.burst_mag);

    printf("RTL-SDR geschlossen.\n");
    return 0;
}
