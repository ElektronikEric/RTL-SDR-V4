#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

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
#define MAX_AIRCRAFT 128

#define MIN(a,b) (((a)<(b))?(a):(b))

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
    uint32_t icao;
    int hat_even;
    int hat_odd;
    uint32_t lat_even;
    uint32_t lon_even;
    uint32_t lat_odd;
    uint32_t lon_odd;
} Aircraft;

// Unsere kleine Flugzeug-Datenbank im Speicher
static Aircraft g_aircrafts[MAX_AIRCRAFT];
static int g_aircraft_count = 0;


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
/* Dekodiere nur 8 Bits nach Preamble in ein Byte.
   Bei 2MSps: 1us = 2 Samples, daher pro Bit Vergleich frueh/spaet.
   Mode-S PPM: Puls in ERSTER Haelfte = 1, Puls in ZWEITER Haelfte = 0 */
static int decode_first_byte_ppm_2m(const uint16_t *m, int n, int preamble_start, uint8_t *out_b0) {
    int bit_start = preamble_start + 16; /* Preamble dauert 8us => 16 Samples @2MSps */
    if (bit_start + (BITS_TO_DECODE * 2) + 1 >= n) return 0;

    uint8_t b0 = 0;
    for (int b = 0; b < 8; b++) {
        int s = bit_start + b * 2;  /* 2 Samples pro Datenbit */
        uint16_t early = m[s];
        uint16_t late  = m[s + 1];

        /* early > late => Puls in erster Haelfte => Bit = 1 */
        int bit = (early > late) ? 1 : 0;
        b0 = (uint8_t)((b0 << 1) | (bit & 1));
    }

    *out_b0 = b0;
    return 1;
}

static int decode_message_ppm_2m(const uint16_t *m, int n, int preamble_start,
                                 uint8_t *out, int out_bytes) {
    int bit_start = preamble_start + 16;                 // nach 8us preamble
    int needed_samples = out_bytes * 8 * 2;              // 2 samples pro bit @2MSps
    if (bit_start + needed_samples >= n) return 0;

    memset(out, 0, out_bytes);

    for (int byte_i = 0; byte_i < out_bytes; byte_i++) {
        uint8_t b = 0;
        for (int bit = 0; bit < 8; bit++) {
            int s = bit_start + (byte_i * 8 + bit) * 2;
            uint16_t early = m[s];
            uint16_t late  = m[s + 1];

            /* early > late => Puls in erster Haelfte => Bit = 1 */
            int bitval = (early > late) ? 1 : 0;
            b = (uint8_t)((b << 1) | bitval);
        }
        out[byte_i] = b;
    }
    return 1;
}

static void print_hex_message(const uint8_t *msg, int len) {
    for (int i = 0; i < len; i++) {
        printf("%02X", msg[i]);
    }
}

/**
 * Überprüft den CRC-Checksum einer 112-Bit (14 Bytes) ADS-B Nachricht.
 * Das offizielle ADS-B Generatorpolynom ist: 0xFFFA04 (24 Bits)
 *
 * @param msg Zeiger auf das 14-Byte Array der Nachricht
 * @return 1 wenn der CRC korrekt ist (keine Fehler), 0 wenn die Nachricht beschädigt ist.
 */
int check_adsb_crc(const uint8_t *msg) {
    uint32_t crc = 0;

    /* Korrektes Mode-S Generatorpolynom (24 Bit, ohne fuehrendes x^24-Bit) */
    const uint32_t POLY = 0xFFF409u;

    for (int i = 0; i < 11; i++) {
        for (int bit = 7; bit >= 0; bit--) {
            uint32_t msg_bit = (msg[i] >> bit) & 1;
            uint32_t crc_msb = (crc >> 23) & 1;
            crc = ((crc << 1) & 0xFFFFFFu) | msg_bit;
            if (crc_msb) {
                crc ^= POLY;
            }
        }
    }

    for (int i = 0; i < 24; i++) {
        uint32_t crc_msb = (crc >> 23) & 1;
        crc = (crc << 1) & 0xFFFFFFu;
        if (crc_msb) {
            crc ^= POLY;
        }
    }

    uint32_t received_pi = ((uint32_t)msg[11] << 16) |
                           ((uint32_t)msg[12] << 8)  |
                            (uint32_t)msg[13];

    return (crc == received_pi) ? 1 : 0;
}

// Hilfsfunktion für den mathematischen Modulo (C's %-Operator verhält sich bei negativen Zahlen anders)
static double cpr_mod(double a, double b) {
    double res = fmod(a, b);
    if (res < 0) res += b;
    return res;
}

// Berechnet die Anzahl der Längengrad-Zonen basierend auf der Breite
static int cpr_nl(double lat) {
    if (fabs(lat) >= 87.0) return 1;
    double c = 1.0 - (1.0 - cos(M_PI / 15.0)) / (cos(M_PI / 180.0 * lat) * cos(M_PI / 180.0 * lat));
    if (c < 0) return 1;
    double alpha = acos(c);
    int nl = (int)(2.0 * M_PI / alpha);
    return nl;
}

/*
 * Dekodiert CPR-Koordinaten global
 * cpr_lat_even / cpr_lon_even: Rohwerte aus der EVEN-Nachricht (17-bit integer)
 * cpr_lat_odd  / cpr_lon_odd:  Rohwerte aus der ODD-Nachricht  (17-bit integer)
 * out_lat / out_lon: Zeiger auf die Variablen, in die das Ergebnis (Grad) geschrieben wird
 */
int decode_cpr_global(uint32_t cpr_lat_even, uint32_t cpr_lon_even,
                      uint32_t cpr_lat_odd,  uint32_t cpr_lon_odd,
                      double *out_lat, double *out_lon)
{
    // Die Rohwerte sind 17-Bit-Ganzzahlen. Wir normieren sie auf den Bereich 0.0 bis 1.0
    double xz_even = (double)cpr_lon_even / 131072.0;
    double yz_even = (double)cpr_lat_even / 131072.0;
    double xz_odd  = (double)cpr_lon_odd  / 131072.0;
    double yz_odd  = (double)cpr_lat_odd  / 131072.0;

    // 1. Zonenindex für den Breitengrad (Latitude) berechnen
    double j = floor(59.0 * yz_even - 60.0 * yz_odd + 0.5);

    // 2. Echten Breitengrad (Latitude) für beide Fälle berechnen
    double d_lat_even = 360.0 / 60.0;
    double d_lat_odd  = 360.0 / 59.0;

    double lat_even = d_lat_even * (cpr_mod(j, 60.0) + yz_even);
    double lat_odd  = d_lat_odd  * (cpr_mod(j, 59.0) + yz_odd);

    // Südliche Hemisphäre korrigieren
    if (lat_even >= 270.0) lat_even -= 360.0;
    if (lat_odd  >= 270.0) lat_odd  -= 360.0;

    // Plausibilitätsprüfung: Die Breiten müssen in der gleichen Zone liegen
    if (cpr_nl(lat_even) != cpr_nl(lat_odd)) {
        return 0; // Ungültige Kombination (Flugzeug ist in der Zwischenzeit zu weit geflogen)
    }

    // Wir nutzen lat_even als finale Breite
    *out_lat = lat_even;

    // 3. Echten Längengrad (Longitude) berechnen
    int nl = cpr_nl(lat_even);

    // Zonenindex für den Längengrad berechnen
    if (nl > 1) {
        double m = floor(xz_even * (nl - 1) - xz_odd * nl + 0.5);
        double d_lon_even = 360.0 / (double)nl;
        *out_lon = d_lon_even * (cpr_mod(m, (double)nl) + xz_even);
    } else {
        // Sonderfall in der Nähe der Pole
        *out_lon = 360.0 * xz_even;
    }

    // Westen korrigieren (Werte über 180 Grad sind negativ auf der Erde)
    if (*out_lon >= 180.0) *out_lon -= 360.0;

    return 1; // Erfolgreich dekodiert
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

                df_ok++;

                uint8_t msg[14]; // 112-bit long frame raw
                int ok_msg = decode_message_ppm_2m(mag, total_mag, j, msg, 14);

                if (ok_msg) {
                	// Zuerst den CRC prüfen! Rauschen erzeugt viele Fehlalarme.
					if (!check_adsb_crc(msg)) {
						// Nachricht ist beschädigt (oder war nur Rauschen).
						// Wir springen ein Stück weiter und ignorieren den Hit.
						j += 1; // Normales Weiterspringen (oder + LOCKOUT_SAMPLES)
						continue;
					}
                    uint8_t ca = (uint8_t)(msg[0] & 0x07);

                    // 1. ICAO-Adresse (Bytes 1, 2, 3)
					uint32_t icao = ((uint32_t)msg[1] << 16) | ((uint32_t)msg[2] << 8) | msg[3];

					// 2. ME-Feld extrahieren (Bytes 4 bis 10)
					// Der "Type Code" (TC) bestimmt, was in der Nachricht steht (erste 5 Bits von Byte 4)
					uint8_t tc = (uint8_t)((msg[4] >> 3) & 0x1F);

					// 3. PI-Feld / Prüfsumme extrahieren (die letzten 3 Bytes: 11, 12, 13)
					uint32_t parity = ((uint32_t)msg[11] << 16) | ((uint32_t)msg[12] << 8) | msg[13];

					// Standard-Ausgabe für jedes gefundene Signal
					printf("hit #%d | DF:%d CA:%d | ICAO:%06X | TC:%2d | PI:%06X | Raw: ",
						   hits, df, ca, icao, tc, parity);
					print_hex_message(msg, 14);
					printf("\n");

					// ========================================================
					// DEKODIERUNG DER ME-NUTZDATEN NACH TYPE CODE (TC)
					// ========================================================

					// TC 1 bis 4: Flugzeugidentifikation und Rufzeichen (Callsign)
					if (tc >= 1 && tc <= 4) {
						char callsign[9];
						// Ein Callsign besteht aus 8 Zeichen, jedes Zeichen ist genau 6 Bits lang
						// Wir müssen die 48 Bits aus den Bytes 5 bis 10 bitweise zerlegen
						uint64_t c_bits = ((uint64_t)msg[5] << 40) | ((uint64_t)msg[6] << 32) |
										  ((uint64_t)msg[7] << 24) | ((uint64_t)msg[8] << 16) |
										  ((uint64_t)msg[9] << 8)  | msg[10];

						const char *lookup = "#ABCDEFGHIJKLMNOPQRSTUVWXYZ################# 0123456789######";

						for (int i = 0; i < 8; i++) {
							// Extrahiere 6 Bits von links nach rechts
							int char_code = (int)((c_bits >> (42 - i * 6)) & 0x3F);
							callsign[i] = lookup[char_code];
						}
						callsign[8] = '\0'; // String-Ende setzen

						printf("  --> [Rufzeichen / Callsign]: %s\n", callsign);
					}

					// TC 9 bis 18: Flugposition in der Luft (Airborne Position)
					else if (tc >= 9 && tc <= 18) {
					    // 1. Höhe dekodieren (12 Bits aus Byte 5 und 6)
					    uint16_t alt_code = (uint16_t)(((msg[5] << 4) | (msg[6] >> 4)) & 0x0FFF);
					    int q_bit = (alt_code >> 4) & 1;
					    int altitude = 0;
					    if (q_bit) {
					        int raw_alt = ((alt_code & 0x0F80) >> 1) | (alt_code & 0x000F);
					        altitude = raw_alt * 25 - 1000;
					    }

					    // 2. ECHTE CPR-DATEN EXTRAHIEREN (Exakte Bitmanipulation der 17-Bit Werte)
					    int cpr_format = (msg[6] >> 2) & 1; // 0 = Even, 1 = Odd

					    // CPR Latitude (17 Bits): Besteht aus den letzten 2 Bits von msg[6], ganz msg[7] und den ersten 7 Bits von msg[8]
					    uint32_t cpr_lat = (((uint32_t)(msg[6] & 0x03) << 15) |
					                        ((uint32_t)msg[7] << 7) |
					                        ((uint32_t)msg[8] >> 1)) & 0x1FFFF;

					    // CPR Longitude (17 Bits): Besteht aus dem letzten Bit von msg[8], ganz msg[9] und ganz msg[10]
					    uint32_t cpr_lon = (((uint32_t)(msg[8] & 0x01) << 16) |
					                        ((uint32_t)msg[9] << 8) |
					                        (uint32_t)msg[10]) & 0x1FFFF;

					    // 3. FLUGZEUG IN DATENBANK SUCHEN ODER ANLEGEN
					    int index = -1;
					    for (int k = 0; k < g_aircraft_count; k++) {
					        if (g_aircrafts[k].icao == icao) {
					            index = k;
					            break;
					        }
					    }

					    // Falls neu und noch Platz in der Liste, füge es hinzu
					    if (index == -1 && g_aircraft_count < MAX_AIRCRAFT) {
					        index = g_aircraft_count;
					        g_aircrafts[index].icao = icao;
					        g_aircrafts[index].hat_even = 0;
					        g_aircrafts[index].hat_odd = 0;
					        g_aircraft_count++;
					    }

					    // 4. DATEN IM SPEICHER AKTUALISIEREN
					    if (index != -1) {
					        if (cpr_format == 0) {
					            g_aircrafts[index].hat_even = 1;
					            g_aircrafts[index].lat_even = cpr_lat;
					            g_aircrafts[index].lon_even = cpr_lon;
					        } else {
					            g_aircrafts[index].hat_odd = 1;
					            g_aircrafts[index].lat_odd = cpr_lat;
					            g_aircrafts[index].lon_odd = cpr_lon;
					        }

					        // 5. GLOBAL DEKODIEREN, WENN BEIDE DATENSÄTZE DA SIND
					        if (g_aircrafts[index].hat_even && g_aircrafts[index].hat_odd) {
					            double finale_lat = 0.0;
					            double finale_lon = 0.0;

					            if (decode_cpr_global(g_aircrafts[index].lat_even, g_aircrafts[index].lon_even,
					                                  g_aircrafts[index].lat_odd,  g_aircrafts[index].lon_odd,
					                                  &finale_lat, &finale_lon))
					            {
					                printf("  --> [GEOPOSITION] ICAO:%06X | Hoehe: %d ft | Lat: %.5f | Lon: %.5f\n",
					                       icao, altitude, finale_lat, finale_lon);
					            }
					        } else {
					            // Es fehlt noch eine Nachricht für die genaue Position
					            printf("  --> [Position Luft] ICAO:%06X | Hoehe: %d ft | Warte auf %s Nachricht...\n",
					                   icao, altitude, cpr_format ? "EVEN" : "ODD");
					        }
					    }
					}


					// TC 19: Geschwindigkeit in der Luft (Airborne Velocity)
					else if (tc == 19) {
						uint8_t subtype = msg[4] & 0x07; // Bits 6-8 von ME

						if (subtype == 1 || subtype == 2) { // Ground Speed (Geschwindigkeit über Grund)
							int directional_n_s = (msg[5] >> 2) & 1; // 0 = Nord, 1 = Süd
							int v_n_s = ((msg[5] & 0x03) << 8) | msg[6]; // Nord-Süd Geschwindigkeit

							int directional_e_w = (msg[7] >> 7) & 1; // 0 = Ost, 1 = West
							int v_e_w = ((msg[7] & 0x7F) << 3) | (msg[8] >> 5); // Ost-West Geschwindigkeit

							// Umrechnung in echte Knoten (Knots) mittels Vektorrechnung (Satz des Pythagoras)
							// (Hinweis: Werte müssen im echten Decoder noch um -1 korrigiert werden)
							printf("  --> [Geschwindigkeit]: Subtyp: %d | N-S Vektor: %d (Richtung: %d) | E-W Vektor: %d (Richtung: %d)\n",
								   subtype, v_n_s, directional_n_s, v_e_w, directional_e_w);
						}
					}
                } else {
                    printf(" MSG_HEX=<truncated>");
                }
                printf("\n");

                j += LOCKOUT_SAMPLES;
                continue;
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
