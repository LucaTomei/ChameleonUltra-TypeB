/* SPDX-License-Identifier: GPL-3.0-only */
/* ISO15693 (NFC-V) reader. See hf15_reader.h. The SPIM0 burst below is the same sequence as rc522.c's
 * spim0_burst() (ISO14443-B bit-bang, validated on device), kept local so rc522.c stays untouched. */
#include "hf15_reader.h"
#include "rc522.h"
#include "iso15693/frame.h"
#include "../rf_workspace.h"
#include "hw_connect.h"
#include "app_status.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include <string.h>

#define HF15_RD_CS_LOW  nrf_gpio_pin_clear(HF_SPI_SELECT)
#define HF15_RD_CS_HIGH nrf_gpio_pin_set(HF_SPI_SELECT)

/* Workspace layout: [waveform][samples]. Worst case: a 24-byte request = (8 + 24*32 + 4) periods of at
 * most 10 SPI bytes (period_q8 is capped at 2560) + address/settle/tail bytes, then up to 9000 samples
 * (+1 leading byte). Sampling needs no TX buffer: the SPIM sends its ORC byte once TXD is exhausted. */
#define HF15_RD_PERIOD_Q8_MAX 2560u
#define HF15_RD_WAVE_MAX   ((8u + HF15_READER_MAX_REQUEST * 32u + 4u) * (HF15_RD_PERIOD_Q8_MAX / 256u) + 4u)
#define HF15_RD_RX_MAX     (HF15_READER_MAX_SAMPLES + 1u)
_Static_assert(HF15_RD_WAVE_MAX + HF15_RD_RX_MAX <= RF_HF_SNIFF_BYTES, "NFC-V reader exceeds RF workspace");
_Static_assert(HF15_READER_SCRATCH_BYTES <= HF15_RD_WAVE_MAX, "scratch must stay inside the waveform area");

uint8_t *hf15_reader_scratch(void) { return (uint8_t *)rf_hf_sniff_workspace(); }

/* One SPIM0 transaction: `tx_len` bytes from tx (0 = send only the ORC byte), `rx_len` bytes into rx. */
static void hf15_spim0_burst(const uint8_t *tx, uint16_t tx_len, uint8_t *rx, uint16_t rx_len, uint8_t orc) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    NRF_SPI0->ENABLE = SPI_ENABLE_ENABLE_Disabled;
    NRF_SPIM0->TXD.PTR = (uint32_t)tx;
    NRF_SPIM0->TXD.MAXCNT = tx_len;
    NRF_SPIM0->RXD.PTR = (uint32_t)rx;
    NRF_SPIM0->RXD.MAXCNT = rx_len;
    NRF_SPIM0->ORC = orc;
    NRF_SPIM0->EVENTS_END = 0;
    NRF_SPIM0->ENABLE = SPIM_ENABLE_ENABLE_Enabled;

    HF15_RD_CS_LOW;
    NRF_SPIM0->TASKS_START = 1;
    while (NRF_SPIM0->EVENTS_END == 0);
    HF15_RD_CS_HIGH;

    NRF_SPIM0->EVENTS_END = 0;
    NRF_SPIM0->EVENTS_ENDRX = 0;
    NRF_SPIM0->EVENTS_ENDTX = 0;
    NRF_SPIM0->EVENTS_STARTED = 0;
    NRF_SPIM0->ENABLE = SPIM_ENABLE_ENABLE_Disabled;
    NRF_SPI0->ENABLE = SPI_ENABLE_ENABLE_Enabled;
    // The legacy SPI is double-buffered and event driven: drop any stale READY/RXD left by the burst.
    NRF_SPI0->EVENTS_READY = 0;
    (void)NRF_SPI0->RXD;
    (void)NRF_SPI0->RXD;
    NRF_SPI0->EVENTS_READY = 0;
}

uint8_t hf15_reader_transceive(const hf15_reader_params_t *p, const uint8_t *req, unsigned req_len,
                               uint8_t *resp, unsigned cap, unsigned *resp_len,
                               hf15_reader_result_t *res, const uint8_t **samples_out) {
    memset(res, 0, sizeof(*res));
    *resp_len = 0;
    if (samples_out) *samples_out = NULL;
    if (!p || !req || req_len == 0u || req_len + 2u > HF15_READER_MAX_REQUEST || p->period_q8 < 256u ||
            p->period_q8 > HF15_RD_PERIOD_Q8_MAX ||
            p->n_samples == 0u || p->n_samples > HF15_READER_MAX_SAMPLES || p->us_per_sample_q16 == 0u ||
            p->mod_reg < ModeReg || p->mod_reg > TCounterValueRegL) {
        return STATUS_PAR_ERR;
    }
    uint8_t frame[HF15_READER_MAX_REQUEST];
    memcpy(frame, req, req_len);
    iso15693_append_crc(frame, (uint16_t)req_len);
    unsigned flen = req_len + 2u;

    uint8_t *ws = (uint8_t *)rf_hf_sniff_workspace();
    uint8_t *wave = ws;
    uint8_t *rx = ws + HF15_RD_WAVE_MAX;                    /* samples */

    /* Waveform: address byte (every following byte is written to mod_reg), then one value per SPI byte. */
    unsigned pos = 0;
    wave[pos++] = (uint8_t)((p->mod_reg << 1) & 0x7E);
    wave[pos++] = p->hi;                                    /* settle on the unmodulated carrier */
    uint32_t acc = 0;
    unsigned periods = hf15_reader_period_count(flen);
    for (unsigned i = 0; i < periods; i++) {
        uint8_t v = hf15_reader_period(frame, flen, i) ? p->lo : p->hi;
        acc += p->period_q8;
        while (acc >= 256u) {
            acc -= 256u;
            if (pos >= HF15_RD_WAVE_MAX - 1u) return STATUS_PAR_ERR;
            wave[pos++] = v;
        }
    }
    wave[pos++] = p->hi;                                    /* always end on full carrier */
    res->wave_len = (uint16_t)pos;

    write_register_single(CommandReg, PCD_IDLE);
    write_register_single(ComIrqReg, 0x7F);
    set_register_mask(FIFOLevelReg, 0x80);
    hf15_spim0_burst(wave, (uint16_t)pos, NULL, 0, 0x00);
    write_register_single(p->mod_reg, p->hi);               /* belt and braces: carrier on */
    write_register_single(CommandReg, PCD_RECEIVE);

    /* Continuous read of TestADCReg: the SPIM sends the read-address ORC byte for every byte, and each
     * one clocks out the previous value (byte 0 is the reply to nothing and is dropped). */
    unsigned n = p->n_samples;
    uint8_t addr = (uint8_t)(((TestADCReg << 1) & 0x7E) | 0x80);
    hf15_spim0_burst(NULL, 0, rx, (uint16_t)(n + 1u), addr);
    write_register_single(CommandReg, PCD_IDLE);
    res->n_samples = (uint16_t)n;
    const uint8_t *samples = rx + 1;
    if (samples_out) *samples_out = samples;

    res->decoded = hf15_tag_decode(samples, n, p->us_per_sample_q16, resp, cap, resp_len, &res->diag);
    return STATUS_HF_TAG_OK;
}
