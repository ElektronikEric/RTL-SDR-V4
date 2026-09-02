/*
 * ============================================================
 * Wireless M-Bus Decoder (T-Mode, 433/868 MHz) - C-Portierung (Windows)
 * ============================================================
 *
 * Basiert auf der Struktur des ACARS-Decoders, aber komplett andere
 * Demodulation/Framing fuer Heizkostenverteiler & Zaehler nach
 * EN 13757-4 (Wireless M-Bus), T-Mode:
 *
 *   RTL-SDR IQ (asynchron)
 *     -> Ringpuffer
 *     -> FSK-Diskriminator direkt auf IQ (kein AM-Envelope!)
 *     -> Tiefpass + Dezimation auf ~400 kHz Zwischenrate
 *     -> Bit-Slicing (Schwelle bei 0 Hz Frequenzabweichung)
 *     -> Manchester-Decode (S-Mode) bzw. 3-aus-6-Decode (T-Mode)
 *     -> CRC-16/EN13757 pro Block
 *     -> Logging (JSONL)
 *
 * WICHTIG: Dies ist ein Geruest, kein vollstaendig validierter Stack.
 * Wireless M-Bus hat viele Modi (S1, S2, T1, T2, C1, C2) mit
 * unterschiedlicher Modulation (FSK vs. OOK) und Bitrate. Diese Datei
 * implementiert T1 (100 kBaud, Manchester auf 3-aus-6-Basis, one-way).
 * Fuer produktiven Einsatz empfiehlt sich der Abgleich mit einer
 * Referenzimplementierung wie "wmbusmeters" oder "rtl_wmbus".
 *
 * Build (MinGW-w64):
 *   gcc -O2 -Wall -o wmbus_decoder.exe wmbus_decoder.c -lm -lwinmm
 */

//#define achtSechsAcht
#define vierDreiDrei
#define _CRT_SECURE_NO_WARNINGS
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

/* ============================================================
 * KONFIGURATION
 * ============================================================ */

static const char *DLL_PATH = "rtlsdr.dll";

/* 433,82 MHz (T-Mode, viele HKV in DE/AT) oder 868,3/868,95 MHz (S-Mode). */
#ifdef vierDreiDrei
#define CENTER_FREQ        433820000u
#endif
#ifdef achtSechsAcht
#define CENTER_FREQ        868300000u
#endif
#define IQ_SAMPLE_RATE     1000000u   /* 1 MSps - genug fuer 100 kBaud FSK */
#define FREQ_CORRECTION    0

#define CHUNK_SECONDS      0.02
#define NUM_IQ_SAMPLES     ((int)(IQ_SAMPLE_RATE * CHUNK_SECONDS)) /* 20000 */

/* Zwischenrate nach Dezimation: 1 MSps / 4 = 250 kSps, reicht fuer 100 kBaud */
#define DECIM              4
#define WORK_RATE          (IQ_SAMPLE_RATE / DECIM)   /* 250000 */

#define BIT_RATE_T_MODE    100000.0
//#define BIT_RATE_T_MODE    32768.0   /* eigentlich S-Mode, Name ggf. anpassen */
#define SAMPLES_PER_BIT    (WORK_RATE / BIT_RATE_T_MODE) /* 2.5 */

/* T-Mode Deviation typ. +-25..50 kHz */
#define FSK_DEVIATION      25000.0

#define SQUELCH_RATIO      6.0
#define SQUELCH_HANGTIME_SEC 0.002
#define MAX_TELEGRAM_SEC   0.02
#define MIN_TELEGRAM_SEC   0.0005

#define NOISE_FLOOR_WINDOW_SEC 2.0
#define NOISE_FLOOR_UPDATE_SEC 0.25
#define POWER_SMOOTHING_MS 0.5

#define MANUAL_GAIN_TENTH_DB 400

#define MAX_TELEGRAM_SAMPLES_CONST 5000 /* = 0.02 * 250000 */

static const char *LOG_FILE = "wmbus_messages.jsonl";

#define RINGBUF_BYTES (4 * 1024 * 1024)

/* ============================================================
 * RTL-SDR: dynamisches Laden (identisch zum ACARS-Decoder)
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
 * RINGPUFFER (unveraendert uebernommen)
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
 * FSK-DISKRIMINATOR DIREKT AUF IQ + TIEFPASS/DEZIMATION
 * ============================================================
 * Anders als bei ACARS gibt es hier KEINE AM-Demodulation. Die
 * Nutzinformation steckt in der momentanen Frequenz des IQ-Signals
 * selbst (FSK). Wir berechnen die Phasendifferenz aufeinanderfolgender
 * IQ-Samples (Quadratur-Diskriminator) und filtern/dezimieren danach.
 */

typedef struct { double re, im; } Complex;

static double g_lp_z1 = 0.0, g_lp_z2 = 0.0;
/* einfacher IIR-Tiefpass 2. Ordnung, Cutoff ~80 kHz bei fs=1 MHz */
static const double LP_B0 = 0.0233, LP_B1 = 0.0466, LP_B2 = 0.0233;
static const double LP_A1 = -1.4190, LP_A2 = 0.5533;

static inline double lp_process(double x) {
    double y = LP_B0 * x + g_lp_z1;
    g_lp_z1 = LP_B1 * x - LP_A1 * y + g_lp_z2;
    g_lp_z2 = LP_B2 * x - LP_A2 * y;
    return y;
}

static Complex g_prev_iq = {0, 0};
static int g_have_prev_iq = 0;

/* freq_out: WORK_RATE-Sample-Frequenzwerte (Hz relativ zur Mittenfrequenz) */
static void fm_discriminate_filter_decimate(const unsigned char *iq_bytes, int n_iq_samples,
                                             double *freq_out, int *n_freq_out) {
    int count = 0;
    double acc = 0.0;
    int acc_n = 0;

    for (int idx = 0; idx < n_iq_samples; idx++) {
        double i_val = ((double)iq_bytes[idx * 2] - 127.5) / 127.5;
        double q_val = ((double)iq_bytes[idx * 2 + 1] - 127.5) / 127.5;

        double freq_hz = 0.0;
        if (g_have_prev_iq) {
            double cross_re = i_val * g_prev_iq.re + q_val * g_prev_iq.im;
            double cross_im = q_val * g_prev_iq.re - i_val * g_prev_iq.im;
            double angle = atan2(cross_im, cross_re);
            freq_hz = angle * IQ_SAMPLE_RATE / (2.0 * M_PI);
        }
        g_prev_iq.re = i_val; g_prev_iq.im = q_val; g_have_prev_iq = 1;

        double filtered = lp_process(freq_hz);

        acc += filtered;
        acc_n++;
        if (acc_n == DECIM) {
            freq_out[count++] = acc / DECIM;
            acc = 0.0; acc_n = 0;
        }
    }
    *n_freq_out = count;
}

/* ============================================================
 * SQUELCH / TELEGRAMM-ERFASSUNG (Struktur wie ACARS, andere Zeitkonstanten)
 * ============================================================ */

typedef void (*BurstCallback)(const double *freq_samples, int n);

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
    double *burst_buf;
    int burst_len, burst_capacity;
    int silence_run, hang_samples, max_burst_samples;

    BurstCallback on_burst;
} Squelch;

static int cmp_double(const void *a, const void *b) {
    double da = *(const double *)a, db = *(const double *)b;
    return (da > db) - (da < db);
}

static void squelch_init(Squelch *sq, int work_rate, BurstCallback cb) {
    memset(sq, 0, sizeof(*sq));
    sq->history_capacity = (int)(NOISE_FLOOR_WINDOW_SEC * work_rate);
    sq->power_history = malloc(sizeof(double) * sq->history_capacity);
    sq->history_scratch = malloc(sizeof(double) * sq->history_capacity);
    sq->update_every = (int)(NOISE_FLOOR_UPDATE_SEC * work_rate);

    double tau_samples = (POWER_SMOOTHING_MS / 1000.0) * work_rate;
    if (tau_samples < 1.0) tau_samples = 1.0;
    sq->smooth_alpha = 1.0 - exp(-1.0 / tau_samples);

    sq->hang_samples = (int)(SQUELCH_HANGTIME_SEC * work_rate);
    sq->max_burst_samples = (int)(MAX_TELEGRAM_SEC * work_rate);
    sq->burst_capacity = sq->max_burst_samples + sq->hang_samples + 16;
    sq->burst_buf = malloc(sizeof(double) * sq->burst_capacity);
    sq->on_burst = cb;
}

static void squelch_update_noise_floor(Squelch *sq) {
    memcpy(sq->history_scratch, sq->power_history, sizeof(double) * sq->history_count);
    qsort(sq->history_scratch, sq->history_count, sizeof(double), cmp_double);
    sq->noise_floor = sq->history_scratch[sq->history_count / 2];
    sq->have_noise_floor = 1;
}

/* Squelch arbeitet hier auf |Frequenzabweichung| statt auf Amplitude,
   da FSK-Traeger auch bei "Ruhe" (kein Sender) nur Rauschen liefert,
   das energiereicher als 0 Hz sein kann. */
static void squelch_push(Squelch *sq, const double *freq, int n) {
    for (int idx = 0; idx < n; idx++) {
        double raw_p = freq[idx] * freq[idx];

        if (!sq->have_smoothed_power) {
            sq->smoothed_power = raw_p;
            sq->have_smoothed_power = 1;
        } else {
            sq->smoothed_power = (1 - sq->smooth_alpha) * sq->smoothed_power + sq->smooth_alpha * raw_p;
        }
        double p = sq->smoothed_power;

        if (sq->history_count < sq->history_capacity) {
            sq->power_history[sq->history_count++] = p;
        } else {
            sq->power_history[sq->history_pos] = p;
        }
        sq->history_pos = (sq->history_pos + 1) % sq->history_capacity;

        sq->samples_since_update++;
        int min_needed_samples = (int)(0.2 * sq->history_capacity / NOISE_FLOOR_WINDOW_SEC);
        if ((!sq->have_noise_floor || sq->samples_since_update >= sq->update_every)
            && sq->history_count >= min_needed_samples) {
            squelch_update_noise_floor(sq);
            sq->samples_since_update = 0;
        }

        double threshold = sq->have_noise_floor ? sq->noise_floor * SQUELCH_RATIO : p;
        if (threshold < 1e-6) threshold = 1e-6;
        int above = p > threshold;

        if (above) {
            if (!sq->in_burst) { sq->in_burst = 1; sq->burst_len = 0; }
            if (sq->burst_len < sq->burst_capacity) sq->burst_buf[sq->burst_len++] = freq[idx];
            sq->silence_run = 0;
        } else if (sq->in_burst) {
            if (sq->burst_len < sq->burst_capacity) sq->burst_buf[sq->burst_len++] = freq[idx];
            sq->silence_run++;
            if (sq->silence_run >= sq->hang_samples) {
                sq->on_burst(sq->burst_buf, sq->burst_len);
                sq->in_burst = 0; sq->burst_len = 0; sq->silence_run = 0;
            }
        }

        if (sq->in_burst && sq->burst_len >= sq->max_burst_samples) {
            sq->on_burst(sq->burst_buf, sq->burst_len);
            sq->in_burst = 0; sq->burst_len = 0; sq->silence_run = 0;
        }
    }
}

/* ============================================================
 * BIT-SLICING + 3-AUS-6-DECODE (T-MODE) + CRC-16/EN13757
 * ============================================================ */

/* Bit = 1, wenn Frequenzabweichung positiv (Richtung +Deviation) */
static int bits_from_freq(const double *freq, int len, int *bits_out, int max_bits) {
    double pos = 0.0;
    int count = 0;
    (void)FSK_DEVIATION;
    while ((int)llround(pos) < len && count < max_bits) {
        int idx = (int)llround(pos);
        bits_out[count++] = (freq[idx] > 0.0) ? 1 : 0;
        pos += SAMPLES_PER_BIT;
    }
    return count;
}

/* T-Mode Sync-Wort nach Manchester-Decodierung: 0x54 0x3D (16 Bit) */
static int find_bit_sync(const int *bits, int n_bits, const uint16_t sync_word, int sync_bits) {
    for (int i = 0; i + sync_bits <= n_bits; i++) {
        uint32_t v = 0;
        for (int b = 0; b < sync_bits; b++) v = (v << 1) | (bits[i + b] & 1);
        if (v == sync_word) return i + sync_bits;
    }
    return -1;
}

/* 3-aus-6 Decodiertabelle (T-Mode, EN 13757-4). Nibble -> 6-Bit-Codewort. */
static const uint8_t CODE_3OF6[16] = {
    0x16, 0x0D, 0x0E, 0x0B, 0x1C, 0x19, 0x1A, 0x13,
    0x2C, 0x25, 0x26, 0x23, 0x34, 0x31, 0x32, 0x29
};

static int decode_3of6_symbol(uint8_t code6, uint8_t *nibble_out) {
    for (int n = 0; n < 16; n++) {
        if (CODE_3OF6[n] == code6) { *nibble_out = (uint8_t)n; return 1; }
    }
    return 0;
}

#ifdef vierDreiDrei
static int bits_to_bytes_3of6(const int *bits, int n_bits, uint8_t *bytes_out, int max_bytes) {
    int n_bytes = 0;
    for (int i = 0; i + 12 <= n_bits && n_bytes < max_bytes; i += 12) {
        uint8_t code_hi = 0, code_lo = 0;
        for (int b = 0; b < 6; b++) code_hi = (code_hi << 1) | (bits[i + b] & 1);
        for (int b = 0; b < 6; b++) code_lo = (code_lo << 1) | (bits[i + 6 + b] & 1);
        uint8_t nib_hi, nib_lo;
        if (!decode_3of6_symbol(code_hi, &nib_hi)) return n_bytes;
        if (!decode_3of6_symbol(code_lo, &nib_lo)) return n_bytes;
        bytes_out[n_bytes++] = (uint8_t)((nib_hi << 4) | nib_lo);
    }
    return n_bytes;
}
#endif
#ifdef achtSechsAcht
static int bits_to_bytes_manchester(const int *bits, int n_bits, uint8_t *bytes_out, int max_bytes) {
    int n_bytes = 0;
    for (int i = 0; i + 16 <= n_bits && n_bytes < max_bytes; i += 16) {
        uint8_t value = 0;
        for (int b = 0; b < 8; b++) {
            int b0 = bits[i + b * 2];
            int b1 = bits[i + b * 2 + 1];
            /* Manchester: 10 = 1, 01 = 0 (IEEE 802.3-Konvention, ggf. invertiert je nach Chip) */
            if (b0 == 1 && b1 == 0) value = (uint8_t)((value << 1) | 1);
            else if (b0 == 0 && b1 == 1) value = (uint8_t)(value << 1);
            else return n_bytes; /* Manchester-Verletzung -> ungültig */
        }
        bytes_out[n_bytes++] = value;
    }
    return n_bytes;
}
#endif
/* CRC-16/EN13757: Poly 0x3D65, Init 0x0000, kein Reflect, XOR-out 0xFFFF */
static uint16_t crc16_en13757(const uint8_t *data, int n) {
    uint16_t crc = 0x0000;
    for (int i = 0; i < n; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x3D65) : (uint16_t)(crc << 1);
        }
    }
    return (uint16_t)(crc ^ 0xFFFF);
}

typedef struct {
    int ok;
    char reason[32];
    uint8_t data[260];
    int data_len;
    uint16_t received_crc;
    uint16_t calculated_crc;
} WmbusResult;

/* T-Mode Telegrammstruktur (vereinfacht): L-Feld (1 Byte) + Block1 (9 Byte
   inkl. C/M/A) + CRC (2 Byte), danach weitere 16-Byte-Bloecke + je CRC.
   Hier nur Block 1 zur Demonstration ausgewertet - fuer vollstaendige
   Telegramme muessen weitere Bloecke gemaess L-Feld verarbeitet werden. */
static WmbusResult parse_wmbus_block1(const uint8_t *bytes, int n) {
    WmbusResult r; memset(&r, 0, sizeof(r));
    if (n < 12) { strcpy(r.reason, "too_short"); return r; }

    int payload_len = 10; /* L,C,M,M,A,A,A,A,V,Ty */
    uint16_t received = (uint16_t)((bytes[payload_len] << 8) | bytes[payload_len + 1]);
    uint16_t calculated = crc16_en13757(bytes, payload_len);

    r.data_len = payload_len;
    memcpy(r.data, bytes, payload_len);
    r.received_crc = received;
    r.calculated_crc = calculated;
    r.ok = (received == calculated);
    strcpy(r.reason, r.ok ? "crc_ok" : "crc_mismatch");
    return r;
}

static WmbusResult decode_telegram(const double *freq, int n) {
    WmbusResult r; memset(&r, 0, sizeof(r));
    if (n < (int)(MIN_TELEGRAM_SEC * WORK_RATE)) { strcpy(r.reason, "too_short"); return r; }

    static int bits[4096];
	#ifdef vierDreiDrei
    	int n_bits = bits_from_freq(freq, n, bits, 4096);
	#endif
	#ifdef achtSechsAcht
    	int start = find_bit_sync(bits, n_bits, 0xF68D, 16);
	#endif

    /* T-Mode Sync 0x54 0x3D nach Manchester-aehnlicher Bit-Erkennung.
       Hinweis: echte Implementierungen decodieren zuerst Manchester,
       hier vereinfachend direkt auf Rohbits gesucht. */
    int start = find_bit_sync(bits, n_bits, 0x543D, 16);
    if (start < 0) { strcpy(r.reason, "no_sync"); return r; }

    static uint8_t raw_bytes[300];
	#ifdef vierDreiDrei
    	int n_bytes = bits_to_bytes_3of6(bits + start, n_bits - start, raw_bytes, 300);
	#endif
	#ifdef achtSechsAcht
    	int n_bytes = bits_to_bytes_manchester(bits + start, n_bits - start, raw_bytes, 300);
	#endif
    if (n_bytes < 12) { strcpy(r.reason, "decode_3of6_failed"); return r; }

    return parse_wmbus_block1(raw_bytes, n_bytes);
}

/* ============================================================
 * LOGGING (JSONL)
 * ============================================================ */

static void iso_timestamp_utc(char *out, size_t out_size) {
    SYSTEMTIME st;
    GetSystemTime(&st);
    snprintf(out, out_size, "%04d-%02d-%02dT%02d:%02d:%02d.%03dZ",
              st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
}

static void log_message(const WmbusResult *r) {
    char ts[64];
    iso_timestamp_utc(ts, sizeof(ts));

    char hex[600] = {0};
    int hi = 0;
    for (int i = 0; i < r->data_len && hi + 3 < (int)sizeof(hex); i++) {
        hi += snprintf(hex + hi, sizeof(hex) - hi, "%02X", r->data[i]);
    }

    FILE *f = fopen(LOG_FILE, "a");
    if (f) {
        fprintf(f, "{\"ok\": %s, \"reason\": \"%s\", \"data_hex\": \"%s\", "
                    "\"received_crc\": %u, \"calculated_crc\": %u, \"timestamp_utc\": \"%s\"}\n",
                r->ok ? "true" : "false", r->reason, hex,
                r->received_crc, r->calculated_crc, ts);
        fclose(f);
    }

    if (r->ok) {
        printf("\n[%s] wM-Bus OK: %s\n", ts, hex);
    } else {
        printf("\n[%s] wM-Bus Dekodierung fehlgeschlagen: %s\n", ts, r->reason);
    }
}

/* ============================================================
 * PIPELINE
 * ============================================================ */

static volatile LONG g_stop = 0;
static RingBuffer g_ring;

static void on_burst_complete(const double *freq_samples, int n) {
    WmbusResult r = decode_telegram(freq_samples, n);
    log_message(&r);
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

    printf("Empfange auf %.3f MHz, %.3f MSps IQ, %.0f Hz Arbeitsrate, %.2f Samples/Bit.\n",
           CENTER_FREQ / 1e6, IQ_SAMPLE_RATE / 1e6, (double)WORK_RATE, SAMPLES_PER_BIT);
    printf("Strg+C zum Beenden.\n\n");

    SetConsoleCtrlHandler(console_handler, TRUE);

    ring_init(&g_ring, RINGBUF_BYTES);
    Squelch squelch;
    squelch_init(&squelch, WORK_RATE, on_burst_complete);

    CaptureThreadArgs cap_args = { &sdr };
    HANDLE cap_thread = CreateThread(NULL, 0, capture_thread_fn, &cap_args, 0, NULL);

    unsigned char *iq_bytes = malloc(NUM_IQ_SAMPLES * 2);
    double *freq_chunk = malloc(sizeof(double) * NUM_IQ_SAMPLES);

    LARGE_INTEGER status_last, status_now, freq_perf;
    QueryPerformanceFrequency(&freq_perf);
    QueryPerformanceCounter(&status_last);

    while (!g_stop) {
        if (!ring_pop(&g_ring, iq_bytes, (size_t)NUM_IQ_SAMPLES * 2, &g_stop)) break;

        int n_freq = 0;
        fm_discriminate_filter_decimate(iq_bytes, NUM_IQ_SAMPLES, freq_chunk, &n_freq);
        squelch_push(&squelch, freq_chunk, n_freq);

        QueryPerformanceCounter(&status_now);
        double elapsed = (double)(status_now.QuadPart - status_last.QuadPart) / freq_perf.QuadPart;
        if (elapsed > 0.5) {
            status_last = status_now;
            const char *state = squelch.in_burst ? "SIGNAL" : "ruhig ";
            printf("\r[%s] Rauschboden=%.1f Hz^2   ", state, squelch.noise_floor);
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
    free(freq_chunk);
    free(g_ring.buf);
    free(squelch.power_history);
    free(squelch.history_scratch);
    free(squelch.burst_buf);

    printf("RTL-SDR geschlossen.\n");
    return 0;
}

/* ============================================================
 * WICHTIGE HINWEISE / EINSCHRAENKUNGEN
 * ============================================================
 *
 * 1. Dies deckt nur den T-Mode (100 kBaud) grob ab. Fuer S-Mode
 *    (32.768 kBaud, echtes Manchester, viele HKV/Wasserzaehler)
 *    braucht es eine eigene Manchester-Decodierfunktion statt der
 *    3-aus-6-Tabelle sowie andere Sync-Woerter (0xF6 0x8D fuer S-Mode).
 *
 * 2. Manche Hersteller (Techem, Kamstrup, Diehl, Qundis, ista) nutzen
 *    proprietaere Varianten des Standards oder verschluesseln die
 *    Nutzdaten (AES-128 CTR gemaess EN 13757-4/5) - dafuer ist ein
 *    Schluessel je Zaehler noetig, den man i. d. R. vom Messdienstleister
 *    anfordern muss.
 *
 * 3. Diese Datei behandelt nur Block 1 eines Telegramms. Laengere
 *    Telegramme bestehen aus mehreren 16-Byte-Datenbloecken, jeweils
 *    mit eigenem CRC - das L-Feld (erstes Byte) gibt die Gesamtlaenge
 *    vor und muss ausgewertet werden, um alle Bloecke zu lesen.
 *
 * 4. Vor dem produktiven Einsatz unbedingt mit bekannten Tools wie
 *    "rtl_wmbus" oder "wmbusmeters" gegenpruefen (gleiche Bytes, CRC ok).
 */
