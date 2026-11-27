/* SPDX-License-Identifier: GPL-3.0-only */
/* ISO/IEC 15693 (NFC-V) air-interface codec — pure logic, no hardware access, host-testable.
 *
 * Timing (high data rate, one subcarrier), fc = 13.56 MHz:
 *   P = 128/fc = 9.4395 us  reader (VCD) pulse-position period
 *   H = 256/fc = 18.879 us  tag (VICC) half-bit = 8 subcarrier cycles of fc/32 (423.75 kHz)
 *
 * VCD -> VICC (as decoded by the Flipper Zero parser and sent by its poller, ISO15693-3 §7.2):
 *   "1 out of 4": 8 periods per 2-bit symbol, ONE pause at period 2v+1 (v = 0..3), symbols LSB-first.
 *   SOF(1/4) = pauses at periods 0 and 5;  SOF(1/256) = pauses at periods 0 and 7.
 *   "1 out of 256": one pause per 512-period byte, at period 2v+1 (v = 0..255).
 *   EOF = a pause at period 2 of the slot that follows the last symbol/byte.
 *   An EOF-only frame (a lone pause) advances the slot in a 16-slot inventory.
 * VICC -> VCD (Flipper iso15693_signal.c, poller decoder SOF=0x17 / EOF=0x1D):
 *   SOF = S S S U S, bit 0 = S U, bit 1 = U S (bytes LSB-first), EOF = S U S S S,
 *   S = 8 subcarrier cycles, U = unmodulated, each exactly one H.
 *
 * The decoders here consume raw RC522 TestADCReg samples: one byte per sample, signed I nibble in
 * bits 7..4 and signed Q nibble in bits 3..0 (same format the ST25TB receiver uses). */
#pragma once
#include <stdint.h>
#include <stdbool.h>

#define HF15_P_Q8       2417u    /* 9.4395 us x 256 */
#define HF15_H_Q8       4833u    /* 18.879 us x 256 */
#define HF15_RX_MAX     64u      /* longest VCD request we accept (bytes, CRC included) */

/* ---------------------------------------------------------------- VICC -> VCD encoder (emulation TX) */

/* Number of half-bits (each H long) of the response for `len` payload bytes: SOF + 16/byte + EOF. */
unsigned hf15_tag_halfbit_count(unsigned len);
/* The same half-bit stream, bit-packed LSB-first into out[] (bit h = half-bit h), table driven: fast
 * enough to run between a request's EOF and the reply. Returns the half-bit count, 0 if it does not fit. */
unsigned hf15_tag_pack(const uint8_t *data, unsigned len, uint8_t *out, unsigned out_bytes);
/* ISO15693 CRC-16 (same result as iso15693_crc16 in frame.c), nibble-table driven for the reply path. */
uint16_t hf15_crc16(const uint8_t *data, unsigned len);
void hf15_append_crc(uint8_t *data, unsigned len);          /* writes 2 CRC bytes (LSB, MSB) at data[len] */
bool hf15_check_crc(const uint8_t *data, unsigned len);     /* len includes the 2 CRC bytes */
/* Half-bit `index` of the response: 1 = subcarrier (S), 0 = unmodulated (U). The 56.64 us unmodulated
 * lead of the SOF is NOT included (it is part of the response delay). */
uint8_t hf15_tag_halfbit(const uint8_t *data, unsigned len, unsigned index);

/* ---------------------------------------------------------------- VCD -> VICC decoder (emulation RX) */

typedef enum {
    HF15_RX_NONE = 0,
    HF15_RX_FRAME,      /* a complete request is in rx->data[0..rx->len) */
    HF15_RX_EOF_ONLY,   /* a lone pause: 16-slot inventory slot marker */
} hf15_rx_event_t;

typedef struct {
    /* pause detector (magnitude |I|+|Q| against a peak-held carrier level) */
    int32_t  level;          /* carrier level, magnitude x16 */
    uint8_t  below, above;
    bool     in_pause;
    uint32_t cand_start, cand_end, pause_start;
    uint32_t settle;         /* samples to ignore (e.g. right after our own load modulation) */
    /* pulse-position decoder */
    uint8_t  state;
    uint32_t t0, t0_end;     /* first SOF pause (or lone pause) start / end, us */
    int32_t  grid_q8;        /* start of the current symbol (1/4) or byte (1/256) slot, us x256 */
    unsigned nbits;
    uint8_t  cur;
    unsigned len;
    uint8_t  data[HF15_RX_MAX];
    /* diagnostics */
    uint16_t pauses, frames, errors, eof_only;
    uint8_t  last_error;     /* 1 bad position, 2 missed symbol, 3 overflow, 4 short frame, 5 timeout, 6 gap */
    uint8_t  peak_level;     /* highest carrier magnitude seen */
    uint32_t last_pause_us, last_pause_len;
    /* ADC zero (the no-carrier reading), LSB x256, learnt inside deep 100% OOK pauses: magnitudes are
     * measured from it, so a DC offset of the RC522 ADC does not hide the pauses. 0 = ideal ADC. */
    int32_t  zero_i, zero_q;
    bool     sof_fresh;      /* the pending lone pause came from idle (not from a decode error) */
} hf15_rx_t;

void hf15_rx_reset(hf15_rx_t *rx);              /* full reset (including diagnostics) */
void hf15_rx_restart(hf15_rx_t *rx);            /* new window: keep diagnostics, drop decode state */
void hf15_rx_settle(hf15_rx_t *rx, uint32_t samples);
/* True when no request is being received (safe moment to pause sampling, e.g. to probe the field). */
bool hf15_rx_idle(const hf15_rx_t *rx);
/* Feed `n` samples, samples[0] taken at `t_us` (1 sample per us). Stops at the first event and returns
 * it; `*consumed` = samples used (up to and including the one that completed the event) and
 * `*eof_end_us` = time of the rising edge that ended the EOF (the reference for the reply delay). */
hf15_rx_event_t hf15_rx_feed(hf15_rx_t *rx, const uint8_t *samples, unsigned n, uint32_t t_us,
                             unsigned *consumed, uint32_t *eof_end_us);
/* Lower layer, exposed for tests: process one detected pause [start, end) in us. */
hf15_rx_event_t hf15_rx_pause(hf15_rx_t *rx, uint32_t start_us, uint32_t end_us);
/* Lower layer, exposed for tests: let time advance to `now_us` without a pause (timeouts). */
hf15_rx_event_t hf15_rx_tick(hf15_rx_t *rx, uint32_t now_us);

/* ---------------------------------------------------------------- VCD -> VICC encoder (reader TX) */

/* Number of P periods of a "1 out of 4" request of `len` bytes: SOF(8) + 32/byte + EOF(4). */
unsigned hf15_reader_period_count(unsigned len);
/* Period `index`: 1 = pause (carrier modulated), 0 = carrier. */
uint8_t hf15_reader_period(const uint8_t *data, unsigned len, unsigned index);

/* ---------------------------------------------------------------- VICC -> VCD decoder (reader RX) */

typedef struct {
    uint8_t  channel;        /* 0 = Q, 1 = I, 2 = |dI|+|dQ| — the one that decoded */
    uint8_t  scale;          /* timing scale index that decoded (0 = nominal) */
    uint16_t sof_sample;     /* sample index where the SOF subcarrier starts */
    uint16_t noise, peak;    /* window-energy noise floor and peak (sum of |dx| over one H) */
    uint16_t bits;           /* decoded payload bits */
} hf15_tag_diag_t;

/* Decode a VICC response from raw TestADCReg samples captured right after the request, one sample
 * every `us_per_sample_q16` us (x65536; the burst path measures 1.0025 us -> 65700). Tries the Q, I and
 * combined channels and a few timing scales, returning the first decode whose CRC is valid. On success
 * out[0..*out_len) holds the response INCLUDING its 2 CRC bytes. */
bool hf15_tag_decode(const uint8_t *samples, unsigned n, uint32_t us_per_sample_q16,
                     uint8_t *out, unsigned cap, unsigned *out_len, hf15_tag_diag_t *diag);
