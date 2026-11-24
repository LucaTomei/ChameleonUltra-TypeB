#include "lf_reader_data.h"

#include "bsp_time.h"
#include "bsp_delay.h"
#include "lf_125khz_radio.h"

#define NRF_LOG_MODULE_NAME pcf7931
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
NRF_LOG_MODULE_REGISTER();

// PCF7930/7931 READ: la lettura non vive più qui. Il tag, alimentato in read mode,
// streama ciclicamente i suoi blocchi (128 bit) in ASK. Invece della cattura a fronti
// del comparatore (inadeguata per la modulazione debole del PCF), la read usa ora il
// percorso SAADC generico `raw_read_to_buffer` (vedi lf_reader_generic.c): campiona
// l'inviluppo analogico a ~125 ksps con settle antenna, e la demodulazione gira sul host
// (Pcf7931.decodeEnvelope). Il comando 3022 in app_cmd.c chiama raw_read_to_buffer.

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
