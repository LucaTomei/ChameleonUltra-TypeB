#ifndef HF15_READER_H
#define HF15_READER_H

/* ISO15693 (NFC-V) reader on the RC522, same technique as the validated ISO14443-B bit-bang reader:
 * the request is sent as a waveform (one modulation-register value per SPI byte, one EasyDMA burst),
 * then TestADCReg is sampled in a second burst and the tag's 423.75 kHz subcarrier reply is decoded
 * ON DEVICE by the hf15 codec (no raw samples over BLE unless asked for). The RF workspace holds the
 * waveform and the samples: reader mode and tag emulation never run at the same time.
 * DA VALIDARE su device: modulation candidates and timing come from the host. */

#include <stdint.h>
#include <stdbool.h>
#include "iso15693/hf15_codec.h"

#define HF15_READER_MAX_REQUEST  24u     /* request bytes INCLUDING the CRC the reader appends */
#define HF15_READER_MAX_SAMPLES  9000u   /* ~9 ms of reply at ~1 us per sample */

typedef struct {
    uint8_t  mod_reg;            /* RC522 register rewritten once per SPI byte (e.g. 0x14 TxControlReg) */
    uint8_t  hi, lo;             /* carrier / pause values of that register */
    uint16_t period_q8;          /* SPI bytes per 9.44 us period, x256 (2411 at 1.0025 us per byte) */
    uint16_t n_samples;          /* ADC samples captured after the request */
    uint32_t us_per_sample_q16;  /* sample period, x65536 (65700 = 1.0025 us) */
} hf15_reader_params_t;

typedef struct {
    uint16_t wave_len;
    uint16_t n_samples;
    bool decoded;
    hf15_tag_diag_t diag;
} hf15_reader_result_t;

/* Sends `req` (the firmware appends the CRC), samples the reply and decodes it. The RC522 must already
 * be in reader mode with the field on. On success resp[0..*resp_len) is the reply INCLUDING its CRC.
 * `*samples_out` (optional) points to the raw capture, valid until the next RF operation.
 * Returns STATUS_HF_TAG_OK when the burst ran (check res->decoded) or STATUS_PAR_ERR. */
uint8_t hf15_reader_transceive(const hf15_reader_params_t *p, const uint8_t *req, unsigned req_len,
                               uint8_t *resp, unsigned cap, unsigned *resp_len,
                               hf15_reader_result_t *res, const uint8_t **samples_out);

/* After a transceive the waveform area of the RF workspace is free and does not overlap the raw
 * samples: the caller may build its reply there (up to HF15_READER_SCRATCH_BYTES). */
#define HF15_READER_SCRATCH_BYTES 7000u
uint8_t *hf15_reader_scratch(void);

#endif
