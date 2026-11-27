/* Host tests for the ISO15693 codec + model (gcc, no hardware). */
#include "hf15_codec.h"
#include "hf15_model.h"
#include "frame.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

static int failures = 0, checks = 0;
#define CHECK(c, ...) do { checks++; if (!(c)) { failures++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static uint32_t rng = 12345u;
static int rnd(int lo, int hi) { rng = rng * 1103515245u + 12345u; return lo + (int)((rng >> 16) % (unsigned)(hi - lo + 1)); }
static double frnd(void) { rng = rng * 1103515245u + 12345u; return ((rng >> 8) & 0xffff) / 65536.0; }

/* ---- Flipper reference: poller encode (VCD frame) ---- */
static size_t flipper_encode(const uint8_t *tx, size_t len, uint8_t *buf) {
    static const uint8_t pat[] = {0x02, 0x08, 0x20, 0x80};
    size_t p = 0;
    buf[p++] = 0x21;
    for (size_t i = 0; i < len; i++)
        for (size_t j = 0; j < 8; j += 2) buf[p++] = pat[(tx[i] >> j) & 3];
    buf[p++] = 0x04;
    return p;
}
/* ---- Flipper reference: poller decode (VICC response bitstream, LSB-first) ---- */
static int flipper_decode(const uint8_t *buf, size_t buf_bits, uint8_t *dec, size_t dec_size, size_t *dec_bits) {
    size_t bit_pos = 0;
    memset(dec, 0, dec_size);
    if ((buf[0] & 0x1F) != 0x17) return -1;
    for (size_t i = 5; i < buf_bits - 5; i += 2) {
        size_t bi = i / 8, off = i % 8;
        uint8_t rb = (uint8_t)((buf[bi] >> off) | (buf[bi + 1] << (8 - off)));
        if (rb == 0x1D) { *dec_bits = bit_pos; return 0; }
        uint8_t bp = rb & 3;
        if (bp == 1) bit_pos++;
        else if (bp == 2) { dec[bit_pos / 8] |= 1 << (bit_pos % 8); bit_pos++; }
        else return -2;
    }
    return -3;
}

static void test_tag_encoder_vs_flipper(void) {
    uint8_t resp[12] = {0x00, 0x00, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x07, 0xE0};
    iso15693_append_crc(resp, 10);
    unsigned nh = hf15_tag_halfbit_count(12);
    uint8_t stream[64] = {0};
    for (unsigned i = 0; i < nh; i++) if (hf15_tag_halfbit(resp, 12, i)) stream[i / 8] |= 1 << (i % 8);
    uint8_t dec[32]; size_t dbits = 0;
    int r = flipper_decode(stream, nh + 8, dec, sizeof dec, &dbits);   /* +8: decoder reads one byte ahead */
    CHECK(r == 0, "flipper decoder rejected our response stream (r=%d)", r);
    CHECK(dbits == 96, "flipper decoded %zu bits, expected 96", dbits);
    CHECK(memcmp(dec, resp, 12) == 0, "flipper decoded bytes differ");
    CHECK(hf15_tag_halfbit(resp, 12, 0) && hf15_tag_halfbit(resp, 12, 1) && hf15_tag_halfbit(resp, 12, 2) &&
          !hf15_tag_halfbit(resp, 12, 3) && hf15_tag_halfbit(resp, 12, 4), "SOF pattern wrong");
}

static void test_reader_encoder_vs_flipper(void) {
    uint8_t req[5] = {0x26, 0x01, 0x00};
    iso15693_append_crc(req, 3);
    uint8_t fl[64]; size_t fb = flipper_encode(req, 5, fl);
    unsigned np = hf15_reader_period_count(5);
    CHECK(np == fb * 8 - 4, "period count %u vs flipper %zu", np, fb * 8 - 4);
    for (unsigned i = 0; i < np; i++) {
        unsigned want = (fl[i / 8] >> (i % 8)) & 1;
        if (hf15_reader_period(req, 5, i) != want) { CHECK(0, "period %u differs from flipper", i); break; }
    }
}

/* ---- synthesise RC522 field-off ADC samples of a VCD request (emulation side) ---- */
typedef struct { double amp, phase, beat_hz, noise; int ramp; } field_t;

static uint8_t pack(int i, int q) {
    if (i > 7) i = 7; if (i < -8) i = -8; if (q > 7) q = 7; if (q < -8) q = -8;
    return (uint8_t)(((i & 15) << 4) | (q & 15));
}

/* periods[] (1 = pause) laid out after `lead_us` of carrier; returns sample count. */
static unsigned synth_request(const uint8_t *per, unsigned np, double lead_us, double tail_us, const field_t *f,
                              uint8_t *out, unsigned cap, double *eof_rise_us) {
    double P = 128.0 / 13.56;
    unsigned n = (unsigned)(lead_us + np * P + tail_us);
    if (n > cap) n = cap;
    double last_rise = 0;
    for (unsigned k = 0; k < n; k++) {
        double t = k + 0.5;
        double rel = t - lead_us;
        int paused = 0;
        if (rel >= 0) {
            unsigned idx = (unsigned)(rel / P);
            if (idx < np && per[idx]) paused = 1;
        }
        double a = paused ? 0.0 : f->amp;
        if (!paused && f->ramp && rel >= 0) {                 /* slow recovery right after a pause */
            unsigned idx = (unsigned)(rel / P);
            if (idx > 0 && idx < np && per[idx - 1] && rel - idx * P < 1.0) a *= 0.5;
        }
        double ph = f->phase + 2 * M_PI * f->beat_hz * t * 1e-6;
        int I = (int)lround(a * cos(ph) + (frnd() - 0.5) * 2 * f->noise);
        int Q = (int)lround(a * sin(ph) + (frnd() - 0.5) * 2 * f->noise);
        out[k] = pack(I, Q);
    }
    /* time of the rising edge ending the EOF pause */
    for (unsigned i = np; i-- > 0;) if (per[i]) { last_rise = lead_us + (i + 1) * P; break; }
    *eof_rise_us = last_rise;
    return n;
}

static hf15_rx_event_t feed_batched(hf15_rx_t *rx, const uint8_t *s, unsigned n, uint32_t *eof_end, unsigned *at) {
    unsigned pos = 0;
    while (pos < n) {
        unsigned chunk = (unsigned)rnd(1, 300);
        if (pos + chunk > n) chunk = n - pos;
        unsigned used = 0;
        hf15_rx_event_t ev = hf15_rx_feed(rx, s + pos, chunk, pos, &used, eof_end);
        pos += used;
        if (ev != HF15_RX_NONE) { *at = pos; return ev; }
    }
    *at = pos;
    return HF15_RX_NONE;
}

static void test_emulation_rx(void) {
    static uint8_t s[40000];
    const uint8_t frames[][16] = {
        {0x26, 0x01, 0x00},                                 /* inventory */
        {0x02, 0x2B},                                       /* get system info */
        {0x02, 0x20, 0x05},                                 /* read block 5 */
        {0x22, 0x20, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x07, 0xE0, 0x03},  /* addressed read */
    };
    const unsigned lens[] = {3, 2, 3, 11};
    int ok = 0, total = 0;
    for (unsigned rep = 0; rep < 40; rep++) {
        for (unsigned f = 0; f < 4; f++) {
            uint8_t req[24];
            memcpy(req, frames[f], lens[f]);
            iso15693_append_crc(req, (uint16_t)lens[f]);
            unsigned L = lens[f] + 2;
            uint8_t per[1200];
            unsigned np = hf15_reader_period_count(L);
            for (unsigned i = 0; i < np; i++) per[i] = hf15_reader_period(req, L, i);
            field_t fd = {.amp = 3.5 + frnd() * 4.0, .phase = frnd() * 6.283, .beat_hz = 200 + frnd() * 3000,
                          .noise = 0.8, .ramp = (int)(rep & 1)};
            double rise;
            unsigned n = synth_request(per, np, 60 + rnd(0, 40), 400, &fd, s, sizeof s, &rise);
            hf15_rx_t rx;
            hf15_rx_reset(&rx);
            uint32_t eof_end = 0; unsigned at = 0;
            hf15_rx_event_t ev = feed_batched(&rx, s, n, &eof_end, &at);
            total++;
            if (ev == HF15_RX_FRAME && rx.len == L && memcmp(rx.data, req, L) == 0 &&
                fabs((double)eof_end - rise) <= 3.0) ok++;
            else if (failures < 20)
                printf("  emu-rx miss: frame %u rep %u ev=%d len=%u/%u err=%u amp=%.1f eof=%u rise=%.1f\n",
                       f, rep, ev, rx.len, L, rx.last_error, fd.amp, eof_end, rise);
        }
    }
    CHECK(ok == total, "emulation RX decoded %d/%d requests", ok, total);
}

static void test_emulation_rx_256_and_eof_only(void) {
    static uint8_t s[60000];
    /* 1-out-of-256: SOF pauses at 0 and 7, byte v -> pause at 2v+1 of a 512-period slot, EOF at 2 */
    uint8_t req[6] = {0x02, 0x20, 0x07};
    iso15693_append_crc(req, 3);
    unsigned L = 5;
    static uint8_t per[8 + 5 * 512 + 4];
    memset(per, 0, sizeof per);
    per[0] = per[7] = 1;
    for (unsigned b = 0; b < L; b++) per[8 + b * 512 + 2 * req[b] + 1] = 1;
    per[8 + L * 512 + 2] = 1;
    unsigned np = 8 + L * 512 + 4;
    field_t fd = {.amp = 5, .phase = 1.0, .beat_hz = 900, .noise = 0.7, .ramp = 0};
    double rise;
    unsigned n = synth_request(per, np, 50, 400, &fd, s, sizeof s, &rise);
    hf15_rx_t rx; hf15_rx_reset(&rx);
    uint32_t eof_end = 0; unsigned at = 0;
    hf15_rx_event_t ev = feed_batched(&rx, s, n, &eof_end, &at);
    CHECK(ev == HF15_RX_FRAME && rx.len == L && memcmp(rx.data, req, L) == 0, "1/256 decode failed (ev=%d len=%u err=%u)", ev, rx.len, rx.last_error);

    /* EOF-only: a lone pause */
    uint8_t lone[4] = {0, 0, 1, 0};
    n = synth_request(lone, 4, 80, 300, &fd, s, sizeof s, &rise);
    hf15_rx_reset(&rx);
    ev = feed_batched(&rx, s, n, &eof_end, &at);
    CHECK(ev == HF15_RX_EOF_ONLY && fabs((double)eof_end - rise) <= 3.0, "EOF-only not detected (ev=%d eof=%u rise=%.1f)", ev, eof_end, rise);

    /* a 1-sample glitch inside the carrier must not create a pause */
    uint8_t one[8] = {0};
    n = synth_request(one, 8, 50, 300, &fd, s, sizeof s, &rise);
    s[120] = pack(0, 0);
    hf15_rx_reset(&rx);
    ev = feed_batched(&rx, s, n, &eof_end, &at);
    CHECK(ev == HF15_RX_NONE && rx.pauses == 0, "glitch produced %u pauses", rx.pauses);
}

/* ---- synthesise reader-side samples of a VICC response (subcarrier load modulation) ---- */
static unsigned synth_response(const uint8_t *resp, unsigned len, double us_per_sample, double delay_us,
                               double amp, double alpha, double noise, uint8_t *out, unsigned cap) {
    double H = 256.0 / 13.56, fsc = 13.56e6 / 32;
    unsigned nh = hf15_tag_halfbit_count(len);
    double total = delay_us + nh * H + 200;
    unsigned n = (unsigned)(total / us_per_sample);
    if (n > cap) n = cap;
    int dcI = rnd(-3, 3), dcQ = rnd(-3, 3);
    double sc_phase = frnd() * 6.283;
    for (unsigned k = 0; k < n; k++) {
        double t = k * us_per_sample;
        double rel = t - delay_us;
        double m = 0;
        if (rel >= 0) {
            unsigned h = (unsigned)(rel / H);
            if (h < nh && hf15_tag_halfbit(resp, len, h))
                m = sin(2 * M_PI * fsc * t * 1e-6 + sc_phase) >= 0 ? 1.0 : -1.0;
        }
        int I = (int)lround(dcI + amp * cos(alpha) * m + (frnd() - 0.5) * 2 * noise);
        int Q = (int)lround(dcQ + amp * sin(alpha) * m + (frnd() - 0.5) * 2 * noise);
        out[k] = pack(I, Q);
    }
    return n;
}

static void test_reader_rx(void) {
    static uint8_t s[12000];
    int ok = 0, total = 0;
    for (unsigned rep = 0; rep < 60; rep++) {
        uint8_t resp[16] = {0x00, 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x07, 0xE0};
        resp[2] = (uint8_t)rep; resp[5] = (uint8_t)(rep * 7);
        iso15693_append_crc(resp, 10);
        double real_us = 1.0025 * (1.0 + (frnd() - 0.5) * 0.004);   /* +-0.2 % calibration error */
        unsigned n = synth_response(resp, 12, real_us, 370 + rnd(-10, 20), 1.5 + frnd() * 3.0,
                                    frnd() * 6.283, 0.6, s, sizeof s);
        uint8_t out[32]; unsigned olen = 0; hf15_tag_diag_t d;
        total++;
        if (hf15_tag_decode(s, n, 65700, out, sizeof out, &olen, &d) && olen == 12 && memcmp(out, resp, 12) == 0) ok++;
        else if (failures < 30) printf("  reader-rx miss rep %u (noise=%u peak=%u)\n", rep, d.noise, d.peak);
    }
    CHECK(ok == total, "reader RX decoded %d/%d responses", ok, total);
    /* silence only: must not invent a response */
    for (unsigned k = 0; k < 4000; k++) s[k] = pack(rnd(-1, 1), rnd(-1, 1));
    uint8_t out[32]; unsigned olen = 0;
    CHECK(!hf15_tag_decode(s, 4000, 65700, out, sizeof out, &olen, NULL), "decoded a response from noise");
}

/* ---- full loop: request -> emulator RX -> model -> response -> reader RX ---- */
static void test_full_loop(void) {
    uint8_t img[16 + 8 * 4 + 8];
    unsigned isz = 0;
    {
        static const uint8_t uid[8] = {0xE0, 0x07, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
        uint8_t blocks[8 * 4];
        for (unsigned i = 0; i < sizeof blocks; i++) blocks[i] = (uint8_t)(0xA0 + i);
        isz = iso15693_model_build_image(img, sizeof img, uid, 0x00, 0x00, 0x01, 4, 8, blocks, NULL);
    }
    CHECK(isz > 0 && iso15693_model_load(img, isz), "model load failed");
    static uint8_t s[40000];
    const uint8_t cmds[][4] = {{0x26, 0x01, 0x00}, {0x02, 0x2B}, {0x02, 0x20, 0x03}, {0x02, 0x2C, 0x00, 0x07}};
    const unsigned cl[] = {3, 2, 3, 4};
    for (unsigned c = 0; c < 4; c++) {
        uint8_t req[16]; memcpy(req, cmds[c], cl[c]); iso15693_append_crc(req, (uint16_t)cl[c]);
        unsigned L = cl[c] + 2;
        uint8_t per[800]; unsigned np = hf15_reader_period_count(L);
        for (unsigned i = 0; i < np; i++) per[i] = hf15_reader_period(req, L, i);
        field_t fd = {.amp = 5, .phase = 0.3 * c, .beat_hz = 1100, .noise = 0.8, .ramp = 1};
        double rise; unsigned n = synth_request(per, np, 70, 400, &fd, s, sizeof s, &rise);
        hf15_rx_t rx; hf15_rx_reset(&rx); uint32_t eof_end; unsigned at;
        hf15_rx_event_t ev = feed_batched(&rx, s, n, &eof_end, &at);
        CHECK(ev == HF15_RX_FRAME, "loop %u: emulator did not decode the request", c);
        uint8_t reply[ISO15693_RESPONSE_MAX];
        unsigned rl = iso15693_model_reply(rx.data, rx.len, reply);
        CHECK(rl >= 3, "loop %u: model gave no reply", c);
        unsigned rn = synth_response(reply, rl, 1.0025, 377, 3.0, 0.7 + c, 0.6, s, 12000);
        uint8_t out[96]; unsigned olen = 0;
        bool dec = hf15_tag_decode(s, rn, 65700, out, sizeof out, &olen, NULL);
        CHECK(dec && olen == rl && memcmp(out, reply, rl) == 0, "loop %u: reader did not decode the reply (len %u)", c, rl);
    }
}

int main(void) {
    test_tag_encoder_vs_flipper();
    test_reader_encoder_vs_flipper();
    test_emulation_rx();
    test_emulation_rx_256_and_eof_only();
    test_reader_rx();
    test_full_loop();
    printf("%s: %d/%d checks passed\n", failures ? "FAILED" : "ALL CODEC TESTS PASSED", checks - failures, checks);
    return failures ? 1 : 0;
}
