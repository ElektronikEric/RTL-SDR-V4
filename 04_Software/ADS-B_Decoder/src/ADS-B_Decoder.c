#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

/* ============================================================
 * Minimal ADS-B Detector: PREAMBLE + DF only (Windows, RTL-SDR)
 * ============================================================ */

static const char *DLL_PATH = "rtlsdr.dll";

#define CENTER_FREQ     1090000000u
#define SAMPLE_RATE     2000000u
#define FREQ_CORRECTION 0
#define MANUAL_GAIN_TENTH_DB 400

#define CHUNK_SAMPLES   262144
#define OVERLAP_SAMPLES 64

#define PREAMBLE_MIN_PEAK 50
#define LOCKOUT_SAMPLES   240   /* 120us @ 2MSps */

/* Wir dekodieren nur 8 Bits (= erstes Byte) */
#define BITS_TO_DECODE    8

typedef uint32_t (__cdecl *rtlsdr_get_device_count_t)(void);
typedef int (__cdecl *rtlsdr_open_t)(void **dev, uint32_t index);
typedef int (__cdecl *rtlsdr_close_t)(void *dev);
typedef int (__cdecl *rtlsdr_set_center_freq_t)(void *dev, uint32_t freq);
typedef int (__cdecl *rtlsdr_set_sample_rate_t)(void *dev, uint32_t rate);
typedef int (__cdecl *rtlsdr_set_freq_correction_t)(void *dev, int ppm);
typedef int (__cdecl *rtlsdr_set_tuner_gain_mode_t)(void *dev, int manual);
typedef int (__cdecl *rtlsdr_set_tuner_gain_t)(void *dev, int gain_tenth_db);
typedef int (__cdecl *rtlsdr_reset_buffer_t)(void *dev);
typedef int (__cdecl *rtlsdr_read_sync_t)(void *dev, void *buf, int len, int *n_read);

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
    rtlsdr_read_sync_t read_sync;
    void *dev;
} RtlSdr;

#define LOAD_FN(var, name) do { \
    sdr->var = (void*)GetProcAddress(sdr->handle, name); \
    if (!sdr->var) { \
        fprintf(stderr, "Fehlende Funktion in rtlsdr.dll: %s\n", name); \
        return 0; \
    } \
} while(0)

static int rtlsdr_load(RtlSdr *sdr, const char *dll_path) {
    memset(sdr, 0, sizeof(*sdr));
    sdr->handle = LoadLibraryA(dll_path);
    if (!sdr->handle) {
        fprintf(stderr, "Konnte %s nicht laden (WinErr=%lu)\n", dll_path, GetLastError());
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
    LOAD_FN(read_sync, "rtlsdr_read_sync");
    return 1;
}

static volatile LONG g_stop = 0;
static BOOL WINAPI console_handler(DWORD sig) {
    if (sig == CTRL_C_EVENT || sig == CTRL_CLOSE_EVENT) {
        InterlockedExchange(&g_stop, 1);
        return TRUE;
    }
    return FALSE;
}

static void iq_to_mag(const uint8_t *iq, int n_samples, uint16_t *mag) {
    for (int i = 0; i < n_samples; i++) {
        int iv = (int)iq[2*i]   - 127;
        int qv = (int)iq[2*i+1] - 127;
        mag[i] = (uint16_t)(iv*iv + qv*qv);
    }
}

/* Dump1090-inspirierter Preamble-Test */
static int is_preamble_at(const uint16_t *m, int j, int n, uint16_t *out_peak) {
    if (j + 9 >= n) return 0;

    uint16_t m0 = m[j+0], m1 = m[j+1], m2 = m[j+2], m3 = m[j+3], m4 = m[j+4];
    uint16_t m5 = m[j+5], m6 = m[j+6], m7 = m[j+7], m8 = m[j+8], m9 = m[j+9];

    if (!(m0 > m1 &&
          m1 < m2 &&
          m2 > m3 &&
          m3 < m0 &&
          m4 < m0 &&
          m5 < m0 &&
          m6 < m0 &&
          m7 > m8 &&
          m8 < m9 &&
          m9 > m6)) {
        return 0;
    }

    uint16_t peak = (uint16_t)((m0 + m2 + m4 + m5 + m6) / 5);
    if (peak < PREAMBLE_MIN_PEAK) return 0;

    *out_peak = peak;
    return 1;
}

/* Dekodiere nur 8 Bits nach Preamble in ein Byte.
   Bei 2MSps: 1us = 2 Samples, daher pro Bit Vergleich früh/spät. */
static int decode_first_byte_ppm_2m(const uint16_t *m, int n, int preamble_start, uint8_t *out_b0) {
    int bit_start = preamble_start + 16; /* Preamble dauert 8us => 16 Samples @2MSps */
    if (bit_start + (BITS_TO_DECODE * 2) + 1 >= n) return 0;

    uint8_t b0 = 0;
    for (int b = 0; b < 8; b++) {
        int s = bit_start + b * 2;  /* 2 Samples pro Datenbit */
        uint16_t early = m[s];
        uint16_t late  = m[s + 1];

        /* einfache harte Entscheidung */
        int bit = (late > early) ? 1 : 0;
        b0 = (uint8_t)((b0 << 1) | (bit & 1));
    }

    *out_b0 = b0;
    return 1;
}

int main(void) {
    RtlSdr sdr;
    if (!rtlsdr_load(&sdr, DLL_PATH)) return 1;

    if (sdr.get_device_count() == 0) {
        fprintf(stderr, "Kein RTL-SDR gefunden.\n");
        return 1;
    }
    if (sdr.open_dev(&sdr.dev, 0) != 0) {
        fprintf(stderr, "rtlsdr_open fehlgeschlagen.\n");
        return 1;
    }

    sdr.set_center_freq(sdr.dev, CENTER_FREQ);
    sdr.set_sample_rate(sdr.dev, SAMPLE_RATE);
    sdr.set_freq_correction(sdr.dev, FREQ_CORRECTION);
    if (MANUAL_GAIN_TENTH_DB >= 0) {
        sdr.set_tuner_gain_mode(sdr.dev, 1);
        sdr.set_tuner_gain(sdr.dev, MANUAL_GAIN_TENTH_DB);
    } else {
        sdr.set_tuner_gain_mode(sdr.dev, 0);
    }
    sdr.reset_buffer(sdr.dev);
    SetConsoleCtrlHandler(console_handler, TRUE);

    printf("ADS-B PREAMBLE+DF Detector (minimal)\n");
    printf("Freq=%.3f MHz, Rate=%.3f MSps\n", CENTER_FREQ/1e6, SAMPLE_RATE/1e6);
    printf("Strg+C zum Beenden\n\n");

    const int iq_bytes_len = CHUNK_SAMPLES * 2;
    uint8_t  *iq = (uint8_t*)malloc(iq_bytes_len);
    uint16_t *mag = (uint16_t*)malloc(sizeof(uint16_t) * (CHUNK_SAMPLES + OVERLAP_SAMPLES));
    uint16_t overlap[OVERLAP_SAMPLES];
    memset(overlap, 0, sizeof(overlap));

    uint64_t global_sample_base = 0;
    int hits = 0, df_ok = 0;

    while (!g_stop) {
        int n_read = 0;
        int rc = sdr.read_sync(sdr.dev, iq, iq_bytes_len, &n_read);
        if (rc != 0 || n_read <= 0) {
            fprintf(stderr, "read_sync Fehler rc=%d n_read=%d\n", rc, n_read);
            break;
        }

        int n_iq = n_read / 2;
        if (n_iq <= 0) continue;

        memcpy(mag, overlap, sizeof(uint16_t) * OVERLAP_SAMPLES);
        iq_to_mag(iq, n_iq, mag + OVERLAP_SAMPLES);

        int total_mag = OVERLAP_SAMPLES + n_iq;

        for (int j = 0; j < total_mag - 32; j++) {
            uint16_t peak = 0;
            if (!is_preamble_at(mag, j, total_mag, &peak)) continue;

            int64_t rel = (int64_t)j - OVERLAP_SAMPLES;
            uint64_t sample_index = (rel >= 0)
                ? (global_sample_base + (uint64_t)rel)
                : (global_sample_base - (uint64_t)(-rel));

            hits++;

            uint8_t b0 = 0;
            if (decode_first_byte_ppm_2m(mag, total_mag, j, &b0)) {
                uint8_t df = (uint8_t)((b0 >> 3) & 0x1F);
                if (df != 17 && df != 18) {
                    j += LOCKOUT_SAMPLES;
                    continue;
                }

                df_ok++;   // zählt jetzt nur DF17/18
                printf("hit #%d @ sample=%llu peak=%u  b0=0x%02X  DF=%u\n",
                       hits, (unsigned long long)sample_index, peak, b0, df);
            } else {
                printf("hit #%d @ sample=%llu peak=%u  b0=<truncated>\n",
                       hits, (unsigned long long)sample_index, peak);
            }

            j += LOCKOUT_SAMPLES;
        }

        if (n_iq >= OVERLAP_SAMPLES) {
            memcpy(overlap, mag + OVERLAP_SAMPLES + (n_iq - OVERLAP_SAMPLES),
                   sizeof(uint16_t) * OVERLAP_SAMPLES);
        } else {
            int keep_old = OVERLAP_SAMPLES - n_iq;
            memmove(overlap, overlap + n_iq, sizeof(uint16_t) * keep_old);
            memcpy(overlap + keep_old, mag + OVERLAP_SAMPLES, sizeof(uint16_t) * n_iq);
        }

        global_sample_base += (uint64_t)n_iq;
    }

    printf("\nBeende. hits=%d, df_decoded=%d\n", hits, df_ok);

    free(iq);
    free(mag);
    sdr.close_dev(sdr.dev);
    FreeLibrary(sdr.handle);
    return 0;
}
