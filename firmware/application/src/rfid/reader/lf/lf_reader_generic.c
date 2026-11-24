#include "lf_reader_generic.h"
#include "lf_reader_data.h"

#include "bsp_delay.h"
#include "bsp_wdt.h"
#include "bsp_time.h"
#include "circular_buffer.h"
#include "lf_125khz_radio.h"
#include "lf_reader_data.h"
#include "protocols/protocols.h"

#define NRF_LOG_MODULE_NAME lfgeneric
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
NRF_LOG_MODULE_REGISTER();

/*
 * Circular buffer for SAADC samples.
 * Increased from 128 to 512 to reduce overrun risk during USB transfer.
 * The main loop drains it as fast as possible into the output buffer.
 */
#define CIRCULAR_BUFFER_SIZE (512)
static circular_buffer cb;

static void saadc_cb(nrf_saadc_value_t *vals, size_t size) {
    for (int i = 0; i < size; i++) {
        nrf_saadc_value_t val = vals[i];
        if (!cb_push_back(&cb, &val)) {
            return;  /* buffer full — oldest samples dropped */
        }
    }
}

static void init_saadc_hw(void) {
    lf_125khz_radio_saadc_enable(saadc_cb);
}

static void uninit_saadc_hw(void) {
    lf_125khz_radio_saadc_disable();
}

/* Variante con decimazione: tiene 1 campione ogni `decimation` (>=1). Serve ai lettori
 * a banda lunga (PCF7931, T55xx) dove un blocco dura troppo per entrare nei 4096 byte di
 * frame a piena frequenza (~125 ksps): decimando si copre più tempo con lo stesso buffer,
 * mantenendo abbastanza campioni per simbolo per la demodulazione a soglia sul host. */
bool raw_read_to_buffer_decim(uint8_t *data, size_t maxlen, uint32_t timeout_ms, size_t *outlen, uint8_t decimation) {
    *outlen = 0;
    if (decimation == 0) decimation = 1;

    cb_init(&cb, CIRCULAR_BUFFER_SIZE, sizeof(uint16_t));
    init_saadc_hw();
    start_lf_125khz_radio();

    /* Wait for antenna to settle before capturing.
     * The LC circuit rings for ~400µs on field startup, then takes
     * another ~800µs to reach steady state. Skip 2ms to be safe. */
    bsp_delay_ms(2);

    uint32_t seen = 0;
    autotimer *p_at = bsp_obtain_timer(0);
    while (NO_TIMEOUT_1MS(p_at, timeout_ms) && *outlen < maxlen) {
        uint16_t val = 0;
        while (cb_pop_front(&cb, &val) && *outlen < maxlen) {
            if ((seen++ % decimation) != 0) continue;  /* decima */
            val = val >> 5;  /* 14-bit ADC → 9-bit, then >>5 gives 8-bit */
            data[*outlen] = val > 0xff ? 0xff : (uint8_t)val;
            ++(*outlen);
        }
        bsp_wdt_feed();  /* prevent watchdog reset during long captures */
    }

    bsp_return_timer(p_at);
    stop_lf_125khz_radio();
    uninit_saadc_hw();
    cb_free(&cb);

    return true;
}

bool raw_read_to_buffer(uint8_t *data, size_t maxlen, uint32_t timeout_ms, size_t *outlen) {
    return raw_read_to_buffer_decim(data, maxlen, timeout_ms, outlen, 1);
}
