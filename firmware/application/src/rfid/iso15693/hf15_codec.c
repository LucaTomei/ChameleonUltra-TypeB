/* SPDX-License-Identifier: GPL-3.0-only */
/* ISO/IEC 15693 (NFC-V) air-interface codec. See hf15_codec.h for the framing references. */
#include "hf15_codec.h"
#include "frame.h"
#include <string.h>

/* ================================================================ VICC -> VCD encoder (emulation TX) */

static const uint8_t k_sof[5] = {1, 1, 1, 0, 1};   /* S S S U S */
static const uint8_t k_eof[5] = {1, 0, 1, 1, 1};   /* S U S S S */

unsigned hf15_tag_halfbit_count(unsigned len) { return 5u + len * 16u + 5u; }

uint8_t hf15_tag_halfbit(const uint8_t *data, unsigned len, unsigned index) {
    if (index < 5u) return k_sof[index];
    index -= 5u;
    if (index < len * 16u) {
        unsigned bit = index >> 1;                       /* payload bit, LSB-first within each byte */
        unsigned v = (data[bit >> 3] >> (bit & 7u)) & 1u;
        unsigned second = index & 1u;                    /* 0 = first half of the bit */
        return (uint8_t)(v ? second : !second);          /* 0 = S U, 1 = U S */
    }
    index -= len * 16u;
    return index < 5u ? k_eof[index] : 0u;
}

/* ================================================================ VCD -> VICC decoder (emulation RX) */

enum { RX_IDLE, RX_SOF2, RX_DATA4, RX_DATA256 };

#define RX_MIN_LEVEL     (3 * 16)                 /* carrier magnitude below this: no usable field */
#define RX_PAUSE_MIN_US  4u                       /* shorter dips are glitches */
#define RX_PAUSE_MAX_US  22u                      /* nominal pause 9.44 us */
#define RX_GAP_US        40u                      /* longer field-off: the reader stopped, abort */
#define RX_TOL_Q8        ((int32_t)(HF15_P_Q8 * 2u / 5u))   /* +-0.4 P on a pause position */
#define RX_ENTER_BELOW   2u                       /* consecutive low samples to enter a pause */
#define RX_EXIT_ABOVE    2u                       /* consecutive high samples to leave it */
#define P_Q8             ((int32_t)HF15_P_Q8)

static inline unsigned mag(uint8_t s) {
    int i = (int8_t)s >> 4;                       /* signed I nibble (bits 7..4) */
    int q = (int8_t)(uint8_t)(s << 4) >> 4;       /* signed Q nibble (bits 3..0) */
    return (unsigned)((i < 0 ? -i : i) + (q < 0 ? -q : q));
}

void hf15_rx_reset(hf15_rx_t *rx) { memset(rx, 0, sizeof(*rx)); }

void hf15_rx_restart(hf15_rx_t *rx) {
    hf15_rx_t keep = *rx;
    memset(rx, 0, sizeof(*rx));
    rx->pauses = keep.pauses; rx->frames = keep.frames; rx->errors = keep.errors;
    rx->eof_only = keep.eof_only; rx->last_error = keep.last_error; rx->peak_level = keep.peak_level;
    rx->last_pause_us = keep.last_pause_us; rx->last_pause_len = keep.last_pause_len;
}

void hf15_rx_settle(hf15_rx_t *rx, uint32_t samples) { rx->settle = samples; }

bool hf15_rx_idle(const hf15_rx_t *rx) { return rx->state == RX_IDLE && !rx->in_pause; }

static void rx_begin(hf15_rx_t *rx, uint32_t start, uint32_t end) {
    rx->state = RX_SOF2;
    rx->t0 = start;
    rx->t0_end = end;
}

static hf15_rx_event_t rx_error(hf15_rx_t *rx, uint8_t code, uint32_t start, uint32_t end) {
    rx->errors++;
    rx->last_error = code;
    rx_begin(rx, start, end);                     /* this pause may be the start of a new SOF */
    return HF15_RX_NONE;
}

static hf15_rx_event_t rx_finish(hf15_rx_t *rx) {
    rx->state = RX_IDLE;
    if ((rx->nbits & 7u) || rx->len < 4u) {       /* flags + command + CRC at minimum */
        rx->errors++;
        rx->last_error = 4;
        return HF15_RX_NONE;
    }
    rx->frames++;
    return HF15_RX_FRAME;
}

/* Nearest period index of an offset w (>= 0, us x256) and the residual distance to it. */
static unsigned nearest_period(int32_t w_q8, int32_t *resid) {
    int32_t pos = (w_q8 + P_Q8 / 2) / P_Q8;
    int32_t r = w_q8 - pos * P_Q8;
    *resid = r < 0 ? -r : r;
    return (unsigned)pos;
}

static inline int32_t absdiff(int32_t a) { return a < 0 ? -a : a; }

hf15_rx_event_t hf15_rx_pause(hf15_rx_t *rx, uint32_t start, uint32_t end) {
    rx->pauses++;
    rx->last_pause_us = start;
    rx->last_pause_len = end - start;
    int32_t s_q8 = (int32_t)(start * 256u);
    switch (rx->state) {
    case RX_IDLE:
        rx_begin(rx, start, end);
        return HF15_RX_NONE;
    case RX_SOF2: {
        int32_t d = s_q8 - (int32_t)(rx->t0 * 256u);
        if (absdiff(d - 5 * P_Q8) <= RX_TOL_Q8 || absdiff(d - 7 * P_Q8) <= RX_TOL_Q8) {
            rx->state = absdiff(d - 5 * P_Q8) <= RX_TOL_Q8 ? RX_DATA4 : RX_DATA256;
            rx->grid_q8 = (int32_t)(rx->t0 * 256u) + 8 * P_Q8;    /* first symbol/byte slot */
            rx->nbits = 0;
            rx->cur = 0;
            rx->len = 0;
        } else {
            rx_begin(rx, start, end);             /* not a SOF: resynchronise on this pause */
        }
        return HF15_RX_NONE;
    }
    case RX_DATA4:
    case RX_DATA256: {
        bool quad = rx->state == RX_DATA4;
        int32_t slot = (quad ? 8 : 512) * P_Q8;
        int32_t rel = s_q8 - rx->grid_q8;
        if (rel < 0) return rx_error(rx, 1, start, end);
        if (rel >= slot) return rx_error(rx, 2, start, end);   /* a whole slot without a pause */
        int32_t resid;
        unsigned pos = nearest_period(rel, &resid);
        if (resid > RX_TOL_Q8) return rx_error(rx, 1, start, end);
        if (pos & 1u) {                                        /* data: pause at 2v+1 */
            unsigned v = (pos - 1u) >> 1;
            if (quad) {
                if (v > 3u) return rx_error(rx, 1, start, end);
                rx->cur |= (uint8_t)(v << (rx->nbits & 7u));
                rx->nbits += 2;
                if ((rx->nbits & 7u) == 0) {
                    if (rx->len >= HF15_RX_MAX) return rx_error(rx, 3, start, end);
                    rx->data[rx->len++] = rx->cur;
                    rx->cur = 0;
                }
            } else {
                if (rx->len >= HF15_RX_MAX) return rx_error(rx, 3, start, end);
                rx->data[rx->len++] = (uint8_t)v;
                rx->nbits += 8;
            }
            rx->grid_q8 = s_q8 - (int32_t)pos * P_Q8 + slot;   /* re-anchor on the measured pause */
            return HF15_RX_NONE;
        }
        if (pos == 2u) return rx_finish(rx);                  /* EOF */
        return rx_error(rx, 1, start, end);
    }
    default:
        rx->state = RX_IDLE;
        return HF15_RX_NONE;
    }
}

hf15_rx_event_t hf15_rx_tick(hf15_rx_t *rx, uint32_t now) {
    int32_t now_q8 = (int32_t)(now * 256u);
    switch (rx->state) {
    case RX_SOF2:
        /* A SOF's second pause starts 5 or 7 periods after the first (and is processed at its end, ~8P
         * plus detection lag): with nothing by 10P the lone pause was an EOF-only frame. */
        if (now_q8 - (int32_t)(rx->t0 * 256u) > 10 * P_Q8) {
            rx->state = RX_IDLE;
            rx->eof_only++;
            return HF15_RX_EOF_ONLY;
        }
        break;
    case RX_DATA4:
        if (now_q8 - rx->grid_q8 > 9 * P_Q8) {           /* no pause in the symbol slot: aborted */
            rx->state = RX_IDLE;
            rx->errors++;
            rx->last_error = 5;
        }
        break;
    case RX_DATA256:
        if (now_q8 - rx->grid_q8 > 513 * P_Q8) {
            rx->state = RX_IDLE;
            rx->errors++;
            rx->last_error = 5;
        }
        break;
    default:
        break;
    }
    return HF15_RX_NONE;
}

hf15_rx_event_t hf15_rx_feed(hf15_rx_t *rx, const uint8_t *samples, unsigned n, uint32_t t_us,
                             unsigned *consumed, uint32_t *eof_end_us) {
    for (unsigned i = 0; i < n; i++) {
        uint32_t t = t_us + i;
        if (rx->settle) {
            rx->settle--;
            continue;
        }
        unsigned m = mag(samples[i]);
        int32_t m16 = (int32_t)m * 16;
        if (m > rx->peak_level) rx->peak_level = (uint8_t)m;
        if (!rx->in_pause) {
            if (rx->level >= RX_MIN_LEVEL && m16 * 2 < rx->level) {
                if (rx->below++ == 0) rx->cand_start = t;
                if (rx->below >= RX_ENTER_BELOW) {
                    rx->in_pause = true;
                    rx->pause_start = rx->cand_start;
                    rx->above = 0;
                }
            } else {
                rx->below = 0;
                /* A carrier sample: track the MEAN carrier magnitude (EMA, 16-sample time constant). A
                 * peak-hold would be pulled up by noise spikes at low field strength and push the pause
                 * threshold (level/2) into the carrier itself. Snap on first acquisition. */
                if (rx->level < RX_MIN_LEVEL) rx->level = m16;
                else rx->level += (m16 - rx->level) / 16;
                if (hf15_rx_tick(rx, t) == HF15_RX_EOF_ONLY) {
                    *consumed = i + 1;
                    *eof_end_us = rx->t0_end;
                    return HF15_RX_EOF_ONLY;
                }
            }
        } else if (m16 * 8 > rx->level * 5) {
            if (rx->above++ == 0) rx->cand_end = t;
            if (rx->above >= RX_EXIT_ABOVE) {
                rx->in_pause = false;
                rx->below = 0;
                uint32_t dur = rx->cand_end - rx->pause_start;
                if (dur >= RX_PAUSE_MIN_US && dur <= RX_PAUSE_MAX_US) {
                    if (hf15_rx_pause(rx, rx->pause_start, rx->cand_end) == HF15_RX_FRAME) {
                        *consumed = i + 1;
                        *eof_end_us = rx->cand_end;
                        return HF15_RX_FRAME;
                    }
                } else if (dur > RX_PAUSE_MAX_US) {
                    if (rx->state != RX_IDLE) {
                        rx->errors++;
                        rx->last_error = 6;
                    }
                    rx->state = RX_IDLE;
                }                                          /* dur < MIN: a glitch, ignored */
            }
        } else {
            rx->above = 0;
            if (t - rx->pause_start > RX_GAP_US) {         /* field gone: forget it and re-acquire */
                if (rx->state != RX_IDLE) {
                    rx->errors++;
                    rx->last_error = 6;
                    rx->state = RX_IDLE;
                }
                rx->level -= rx->level >> 4;
                if (rx->level < RX_MIN_LEVEL) {
                    rx->in_pause = false;
                    rx->below = 0;
                }
            }
        }
    }
    *consumed = n;
    return HF15_RX_NONE;
}

/* ================================================================ VCD -> VICC encoder (reader TX) */

unsigned hf15_reader_period_count(unsigned len) { return 8u + len * 32u + 4u; }

uint8_t hf15_reader_period(const uint8_t *data, unsigned len, unsigned index) {
    if (index < 8u) return (uint8_t)(index == 0u || index == 5u);      /* SOF 1/4 */
    index -= 8u;
    if (index < len * 32u) {
        unsigned sym = index >> 3, slot = index & 7u;                 /* 4 symbols per byte */
        unsigned v = (data[sym >> 2] >> ((sym & 3u) * 2u)) & 3u;      /* LSB-first */
        return (uint8_t)(slot == 2u * v + 1u);
    }
    index -= len * 32u;
    return (uint8_t)(index == 2u);                                    /* EOF: pause at period 2 */
}

/* ================================================================ VICC -> VCD decoder (reader RX) */

#define TD_MIN_ENERGY     24u      /* peak window energy below this: no response */
#define TD_NOISE_FROM     16u      /* skip the receiver switch-on transient */
#define TD_NOISE_TO       300u     /* the response starts ~377 us after our EOF: silence before */

static inline int sx_i(uint8_t s) { return (int8_t)s >> 4; }
static inline int sx_q(uint8_t s) { return (int8_t)(uint8_t)(s << 4) >> 4; }

static inline unsigned dx(const uint8_t *s, unsigned k, unsigned ch) {
    int di = sx_i(s[k]) - sx_i(s[k - 1]);
    int dq = sx_q(s[k]) - sx_q(s[k - 1]);
    di = di < 0 ? -di : di;
    dq = dq < 0 ? -dq : dq;
    return ch == 0u ? (unsigned)dq : ch == 1u ? (unsigned)di : (unsigned)(di + dq);
}

/* Energy (sum of |dx|) of channel ch over [a, a+len) samples, positions in samples x256. */
static unsigned win(const uint8_t *s, unsigned n, unsigned ch, int32_t a_q8, int32_t len_q8) {
    int32_t a = a_q8 >> 8, b = (a_q8 + len_q8) >> 8;
    if (a < 1) a = 1;
    if (b > (int32_t)n) b = (int32_t)n;
    unsigned e = 0;
    for (int32_t k = a; k < b; k++) e += dx(s, (unsigned)k, ch);
    return e;
}

static bool td_try(const uint8_t *s, unsigned n, unsigned ch, int32_t h_q8,
                   uint8_t *out, unsigned cap, unsigned *out_len, hf15_tag_diag_t *d) {
    if (n < 64u) return false;
    /* Noise floor: quietest window before the response; peak: loudest window anywhere. */
    unsigned noise = 0xffffu, peak = 0;
    unsigned noise_to = n < TD_NOISE_TO ? n / 2u : TD_NOISE_TO;
    for (int32_t a = (int32_t)TD_NOISE_FROM * 256; (a + h_q8) >> 8 <= (int32_t)noise_to; a += h_q8 / 4) {
        unsigned e = win(s, n, ch, a, h_q8);
        if (e < noise) noise = e;
    }
    for (int32_t a = (int32_t)TD_NOISE_FROM * 256; (a + h_q8) >> 8 <= (int32_t)n; a += h_q8 / 4) {
        unsigned e = win(s, n, ch, a, h_q8);
        if (e > peak) peak = e;
    }
    if (noise == 0xffffu) noise = 0;
    if (peak > d->peak) {                         /* keep the strongest channel's levels for diagnostics */
        d->noise = (uint16_t)noise;
        d->peak = (uint16_t)peak;
    }
    if (peak < TD_MIN_ENERGY || peak < 2u * noise + 8u) return false;
    unsigned thr = (noise + peak) / 2u;

    /* SOF: three subcarrier halves in a row. First coarse hit, then align on maximum contrast of
     * S S S U S (the U half and the following S edge are the sharpest timing marks). */
    int32_t sof = -1;
    for (int32_t a = (int32_t)TD_NOISE_FROM * 256; (a + 5 * h_q8) >> 8 <= (int32_t)n; a += 256) {
        if (win(s, n, ch, a, h_q8) > thr && win(s, n, ch, a + h_q8, h_q8) > thr &&
                win(s, n, ch, a + 2 * h_q8, h_q8) > thr) {
            sof = a;
            break;
        }
    }
    if (sof < 0) return false;
    int32_t best = sof, best_score = -0x7fffffff;
    for (int32_t a = sof - h_q8 / 2; a <= sof + h_q8; a += 128) {
        if (a < 256) continue;
        int32_t score = (int32_t)win(s, n, ch, a, h_q8) + (int32_t)win(s, n, ch, a + h_q8, h_q8) +
                        (int32_t)win(s, n, ch, a + 2 * h_q8, h_q8) - 3 * (int32_t)win(s, n, ch, a + 3 * h_q8, h_q8) +
                        (int32_t)win(s, n, ch, a + 4 * h_q8, h_q8);
        if (score > best_score) {
            best_score = score;
            best = a;
        }
    }
    sof = best;
    if (win(s, n, ch, sof + 3 * h_q8, h_q8) > thr || win(s, n, ch, sof + 4 * h_q8, h_q8) <= thr) return false;

    /* Data: Manchester pairs from half 5; the louder half carries the subcarrier. EOF = S U S S S, so
     * the first S S pair means the previous "0" was the EOF's leading S U. */
    unsigned bits = 0;
    bool eof = false;
    int32_t adj = 0;                                  /* timing correction from the Manchester PLL, x256 */
    memset(out, 0, cap);
    for (unsigned h = 5u;; h += 2u) {
        int32_t a = sof + (int32_t)h * h_q8 + adj;
        if ((a + 2 * h_q8 + 512) >> 8 > (int32_t)n) break;
        /* Every bit has a guaranteed transition at its centre: the half/half contrast is maximal when
         * the grid is aligned. Probe +-1 and +-2 samples and pull the grid a quarter of the way. */
        int32_t best_o = 0;
        unsigned c0 = win(s, n, ch, a, h_q8), c1 = win(s, n, ch, a + h_q8, h_q8);
        unsigned best_c = c0 > c1 ? c0 - c1 : c1 - c0;
        static const int16_t probe[4] = {-512, -256, 256, 512};
        for (unsigned q = 0; q < 4u; q++) {
            unsigned p0 = win(s, n, ch, a + probe[q], h_q8), p1 = win(s, n, ch, a + probe[q] + h_q8, h_q8);
            unsigned c = p0 > p1 ? p0 - p1 : p1 - p0;
            if (c > best_c) {
                best_c = c;
                best_o = probe[q];
            }
        }
        adj += best_o / 4;
        unsigned w0 = win(s, n, ch, a + best_o, h_q8), w1 = win(s, n, ch, a + best_o + h_q8, h_q8);
        if (w0 > thr && w1 > thr) {
            if (bits == 0u) return false;
            bits--;                                   /* drop the EOF's leading S U */
            eof = true;
            break;
        }
        if (w0 <= thr && w1 <= thr) return false;     /* signal lost before an EOF */
        if (bits >= cap * 8u) return false;
        if (w1 > w0) out[bits >> 3] |= (uint8_t)(1u << (bits & 7u));
        bits++;
    }
    if (!eof || (bits & 7u) || bits < 24u) return false;
    unsigned len = bits >> 3;
    if (!iso15693_check_crc(out, (uint16_t)len)) return false;
    /* the EOF-leading bit was written as a 0 into out[]: it lies beyond len, nothing to clear */
    *out_len = len;
    d->noise = (uint16_t)noise;
    d->peak = (uint16_t)peak;
    d->sof_sample = (uint16_t)(sof >> 8);
    d->bits = (uint16_t)bits;
    return true;
}

bool hf15_tag_decode(const uint8_t *samples, unsigned n, uint32_t us_per_sample_q16,
                     uint8_t *out, unsigned cap, unsigned *out_len, hf15_tag_diag_t *diag) {
    hf15_tag_diag_t local;
    hf15_tag_diag_t *d = diag ? diag : &local;
    memset(d, 0, sizeof(*d));
    *out_len = 0;
    if (!samples || !out || cap < 3u || us_per_sample_q16 == 0u) return false;
    /* half-bit length in samples x256 = (H_us x256) / (us per sample) */
    int32_t h_nom = (int32_t)(((uint32_t)HF15_H_Q8 << 16) / us_per_sample_q16);   /* 4833<<16 fits 32 bits */
    static const int16_t scale_permille[3] = {0, -4, 4};
    static const uint8_t channels[3] = {2, 0, 1};       /* combined first: phase independent */
    for (unsigned c = 0; c < 3u; c++) {
        for (unsigned k = 0; k < 3u; k++) {
            int32_t h_q8 = h_nom + (h_nom * scale_permille[k]) / 1000;
            if (td_try(samples, n, channels[c], h_q8, out, cap, out_len, d)) {
                d->channel = channels[c];
                d->scale = (uint8_t)k;
                return true;
            }
        }
    }
    memset(out, 0, cap);
    return false;
}
