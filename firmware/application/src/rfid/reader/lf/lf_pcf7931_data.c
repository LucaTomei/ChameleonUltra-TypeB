#include "lf_reader_data.h"

#include "bsp_time.h"
#include "circular_buffer.h"
#include "lf_125khz_radio.h"

#define NRF_LOG_MODULE_NAME pcf7931
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
NRF_LOG_MODULE_REGISTER();

// PCF7930/7931 read: the tag, when powered in READ MODE, continuously streams its
// blocks (128 bits each) delimited by a PMC marker, ASK-modulated. Rather than
// decode on-device, we capture the raw falling-edge intervals (in 125 kHz carrier
// cycles, one byte each) exactly like the EM410x reader and hand them to the host,
// which runs the PCF7931 demodulator (bit rule 64/32·T0 + PMC + block assembly).
// Keeping the intricate demod on the host makes it unit-testable.

#define PCF7931_CB_SIZE (256)

static circular_buffer cb_pcf7931;

// GPIO falling-edge callback: record the carrier-cycle count since the last edge.
static void pcf7931_gpio_cb(void) {
    uint32_t cntr = get_lf_counter_value();
    uint16_t val = (cntr > 0xff) ? 0xff : (cntr & 0xff);
    cb_push_back(&cb_pcf7931, &val);
    clear_lf_counter_value();
}

static void init_pcf7931_hw(void) {
    register_rio_callback(pcf7931_gpio_cb);
    lf_125khz_radio_gpiote_enable();
}

static void uninit_pcf7931_hw(void) {
    lf_125khz_radio_gpiote_disable();
    unregister_rio_callback();
}

// Capture up to `maxlen` edge intervals into `data` (one byte each) over `timeout_ms`.
// Returns true and sets *outlen when at least one interval was captured.
bool pcf7931_read_raw(uint8_t *data, size_t maxlen, uint32_t timeout_ms, size_t *outlen) {
    cb_init(&cb_pcf7931, PCF7931_CB_SIZE, sizeof(uint16_t));
    init_pcf7931_hw();
    start_lf_125khz_radio();

    size_t n = 0;
    autotimer *p_at = bsp_obtain_timer(0);
    while (n < maxlen && NO_TIMEOUT_1MS(p_at, timeout_ms)) {
        uint16_t val = 0;
        while (n < maxlen && cb_pop_front(&cb_pcf7931, &val)) {
            data[n++] = (uint8_t)(val & 0xff);
        }
    }

    bsp_return_timer(p_at);
    stop_lf_125khz_radio();
    uninit_pcf7931_hw();
    cb_free(&cb_pcf7931);

    *outlen = n;
    return n > 0;
}
