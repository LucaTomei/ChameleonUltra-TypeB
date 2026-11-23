#include "lf_reader_data.h"

#include "bsp_time.h"
#include "bsp_delay.h"
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

// ---------------------------------------------------------------------------------------
// PCF7931 WRITE (downlink). The tag is programmed by interrupting the RF field with
// precisely-timed gaps (pulse-position bit encoding). The exact PCF7931 programming
// timings MUST BE CALIBRATED ON REAL HARDWARE against the proxmark reference — the
// constants below are the correct STRUCTURE but a starting point, not device-verified.
// (field off/on via stop/start_lf_125khz_radio adds restart overhead that also needs
// trimming.) The host sends: address + optional 56-bit password + 128-bit block data.
// ---------------------------------------------------------------------------------------
#define PCF_T0_US        8                    // one 125 kHz carrier period
#define PCF_INIT_GAP_US  (52 * PCF_T0_US)     // init gap → enter programming mode
#define PCF_BIT_US       (16 * PCF_T0_US)     // one bit window
#define PCF_GAP_US       (6  * PCF_T0_US)     // modulation gap width

// Encode one bit by the gap position within the bit window (1 = late, 0 = early).
static void pcf7931_send_bit(uint8_t bit) {
    if (bit) {
        bsp_delay_us(PCF_BIT_US - PCF_GAP_US);
        stop_lf_125khz_radio(); bsp_delay_us(PCF_GAP_US); start_lf_125khz_radio();
    } else {
        stop_lf_125khz_radio(); bsp_delay_us(PCF_GAP_US); start_lf_125khz_radio();
        bsp_delay_us(PCF_BIT_US - PCF_GAP_US);
    }
}

// Write one 16-byte (128-bit) block at `address` (0-based, 7-bit). If `use_pw`, `pw`
// holds the 7-byte (56-bit) password. Best-effort: no on-device read-back confirmation.
bool pcf7931_write_block(uint8_t address, const uint8_t *data, bool use_pw, const uint8_t *pw) {
    start_lf_125khz_radio();
    bsp_delay_ms(6);                                                  // charge the tag
    stop_lf_125khz_radio(); bsp_delay_us(PCF_INIT_GAP_US); start_lf_125khz_radio();
    bsp_delay_us(PCF_BIT_US);
    if (use_pw) {
        for (int i = 0; i < 7; i++)
            for (int b = 7; b >= 0; b--) pcf7931_send_bit((pw[i] >> b) & 1);
    }
    for (int b = 6; b >= 0; b--) pcf7931_send_bit((address >> b) & 1);          // 7-bit address
    for (int i = 0; i < 16; i++)
        for (int b = 7; b >= 0; b--) pcf7931_send_bit((data[i] >> b) & 1);      // 128-bit data
    bsp_delay_ms(10);                                                // programming time
    stop_lf_125khz_radio();
    return true;
}
