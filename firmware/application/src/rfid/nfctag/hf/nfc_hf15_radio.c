/* SPDX-License-Identifier: GPL-3.0-only */
/* ISO15693 (NFC-V) emulation RF.
 *
 * RX: the reader's "1 out of 4 / 1 out of 256" PPM is recovered from the nRF52840 NFCT carrier-lock
 * detector (FIELDPRESENT.LOCKDETECT), NOT from the RC522 ADC. The RC522 field-off ADC path never
 * coupled the reader carrier reliably (carrier coverage ~0%); the hardware lock detector drops cleanly
 * during each reader pause. LOCKDETECT edges are timestamped with DWT->CYCCNT (64 MHz) and decoded in
 * software. Proven on device 2026-11-27: a real Flipper Inventory (26 01 00 + CRC) decodes with the
 * CRC valid, falling-edge gaps landing on exact multiples of the 604.13-cycle PPM tick. Approach from
 * SecLabz/ChameleonUltra-Tonie (same nRF52840 + RC522 board).
 *
 * TX: the reply is load-modulated at fc/32 (423.75 kHz) by bit-banging HF_ANT_SEL (a P1 GPIO), with the
 * ISO15693 SOF/EOF + Manchester half-bits produced by the host-tested hf15 codec. NFCT is disabled for
 * the duration of the reply (an active NFCT suppresses this board's GPIO load modulation), then
 * re-sensed/activated so LOCKDETECT is live again for the next command.
 *
 * ST25TB and NFC-V never run together (one HF type per slot), so they share the HF antenna / RF
 * workspace / timeslot machinery exactly like ST25TB alone does. RC522 is kept powered with its field
 * OFF (same as the qualified ST25TB emulation) so it does not drive the antenna. */
#include "nfc_hf15.h"
#include "iso15693/hf15_model.h"
#include "iso15693/hf15_codec.h"
#include "rc522.h"
#include "rf_timeslot.h"
#include "nrfx_power.h"
#include "usb_main.h"
#include "rfid_main.h"
#include "bsp_delay.h"
#include "bsp_wdt.h"
#include "ble_main.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include <string.h>

#if defined(PROJECT_CHAMELEON_ULTRA)

static inline bool hfv_advertising_pause(bool *resume) { *resume = false; return true; }   /* as ST25TB */
static inline bool hfv_usb_hfclk_active(void) { return true; }

enum {
    HFV_CPU_CYCLES_PER_US = 64,
    HFV_NFCT_READY_TIMEOUT_CYCLES = 100 * HFV_CPU_CYCLES_PER_US,
    HFV_WINDOW_CLEANUP_CYCLES = 2000 * HFV_CPU_CYCLES_PER_US,
    HFV_WINDOW_WORK_GUARD_CYCLES = 2000 * HFV_CPU_CYCLES_PER_US,
    HFV_RESPONSE_WAIT_TIMEOUT_CYCLES = 1000 * HFV_CPU_CYCLES_PER_US,
    HFV_FIELD_LOSS_CYCLES = 3000 * HFV_CPU_CYCLES_PER_US,
    HFV_ACTIVE_WINDOW_US = 2000000,
    HFV_IDLE_WINDOW_US = 500000,
    HFV_SERVICE_WINDOW_US = 25000,
    HFV_USB_WINDOW_US = 500000,
    HFV_POWER_SETTLE_MS = 1,
    HFV_RESET_SETTLE_US = 500,
    HFV_RESET_SETTLE_MS = 10,
};

/* RC522 held with the field off (identical to the qualified ST25TB settings), only so it does not drive
 * the HF antenna while NFCT senses the field and HF_ANT_SEL does the load modulation. */
enum {
    RC522_SPI_ADDRESS_MASK = 0x7e,
    RC522_SPI_READ = 0x80,
    RC522_FIELD_OFF = 0x80,
    RC522_FIELD_ENABLE_MASK = 3,
    HFV_TX_TYPE_B = 3,
    HFV_RX_MODE = 0x14,
    HFV_TX_AUTO = 0,
    HFV_TX_CONSTANT_CARRIER = 0x30,
    HFV_RX_SELECT = 0x84,
    HFV_RX_THRESHOLD = 0x84,
    HFV_DEMOD = 0x4d,
    HFV_INITIAL_RF_GAIN = 0x48,
    HFV_RF_GAIN = 0x78,
    HFV_N_CONDUCTANCE = 0x88,
    HFV_CW_CONDUCTANCE = 32,
    HFV_MOD_CONDUCTANCE = 16,
    HFV_NFC_A_TX_AUTO = 0x40,
    HFV_NFC_A_MODE = 0x3d,
};

/* Load modulation at fc/32: 16 GPIO half-periods per ISO15693 half-bit (8 subcarrier cycles).
 * 64 MHz / (2 * 423.75 kHz) = 75 + 175/339 cycles (exactly twice the ST25TB fc/16 period). */
enum {
    HFV_TX_HALF_PERIODS = 16,
    HFV_TX_START_MARGIN_CYCLES = 128,
    HFV_TX_HALF_PERIOD_BASE_CYCLES = 75,
    HFV_TX_HALF_PERIOD_FRACTION = 175,
    HFV_TX_HALF_PERIOD_DENOMINATOR = 339,
    HFV_TX_HALVES_MAX = 5 + 16 * ISO15693_RESPONSE_MAX + 5,
};

/* LOCKDETECT PPM decode (values measured on device / from the Tonie reference). A PPM tick is 604.13
 * CPU cycles (= 128/fc at 64 MHz). Half-denominator rounding is intentional. */
enum {
    HFV_LOCK_TICK_NUM = 60413,
    HFV_LOCK_TICK_DEN = 100,
    HFV_LOCK_TICK_ROUND = 30206,
    HFV_LOCK_TICK_TOL = 160,
    HFV_LOCK_SOF_MIN_1OF4 = 2830,
    HFV_LOCK_SOF_MAX_1OF4 = 3220,
    HFV_LOCK_SOF_MIN_1OF256 = 4037,
    HFV_LOCK_SOF_MAX_1OF256 = 4420,
    HFV_LOCK_SOF_SPLIT = 3500,
    HFV_LOCK_MERGED_MIN = 1993,
    HFV_LOCK_MERGED_MAX = 2780,
    HFV_LOCK_GAP_1OF4 = 19200,
    HFV_LOCK_GAP_1OF256 = 704000,
    HFV_LOCK_SOF_TIMEOUT_CYCLES = 640000,       /* ~10 ms with no SOF -> bail, re-check the field */
    HFV_LOCK_FRAME_TIMEOUT_CYCLES = 1920000,    /* ~30 ms: a real request completes in a few ms */
    /* Detector timestamp (EOF pause falling edge) to the SOF start at 64 MHz. ISO15693 t1 ~= 320.9 us;
     * the SOF begins with 56.64 us of silence. DA VALIDARE / tune this one constant with a scope. */
    HFV_LOCK_RESPONSE_WAIT_CYCLES = 20508,
    HFV_LOCK_EOF_POSITION = 2,
    HFV_LOCK_MIN_FRAME_BYTES = 4,
};

#define HFV_TICK_FROM_CYC(e)   (((uint32_t)(e) * HFV_LOCK_TICK_DEN + HFV_LOCK_TICK_ROUND) / HFV_LOCK_TICK_NUM)
#define HFV_CYC_FROM_TICK(t)   ((uint32_t)(t) * HFV_LOCK_TICK_NUM / HFV_LOCK_TICK_DEN)
#define HFV_SYM_IDX(t, c)      (((c) && (t) >= 8u) ? (((t) - 8u) / 512u + 1u) : ((t) / 8u))
#define HFV_PULSE_POS(t, c)    (((c) && (t) >= 8u) ? (((t) - 8u) % 512u) : ((t) % 8u))
#define HFV_LOCK_MIN_SYM(c)    ((c) ? HFV_LOCK_MIN_FRAME_BYTES : (HFV_LOCK_MIN_FRAME_BYTES * 4u))
#define HFV_LOCK_MAX_SYM(c)    ((c) ? (unsigned)HF15_RX_MAX : ((unsigned)HF15_RX_MAX * 4u))

static bool radio_ok;
static bool reader_open;
static bool hfv_responded;
extern bool g_is_reader_antenna_on;

static hf15_rx_t hfv_rx;        /* kept only for the diagnostics counters reported over USB */

/* DEBUG bring-up: LOCKDETECT edge log, dumped via HF15_EMU_CAPTURE to verify the on-device decode
 * matches the host decoder. 4-byte entries: 31-bit DWT-cycle delta (from receive start) | top bit =
 * lock state. offset==HFV_CAP_REARM re-arms (unfreeze + reset); the ring freezes once a frame lands. */
#define HFV_CAP_SIZE 4096u
#define HFV_CAP_REARM 0xFFFFu
static uint8_t hfv_cap[HFV_CAP_SIZE];
static volatile unsigned hfv_cap_wr;
static volatile bool hfv_cap_frozen;

unsigned nfc_hf15_radio_capture(uint8_t *out, unsigned offset, unsigned max, unsigned *wr_out) {
    if (wr_out) { *wr_out = hfv_cap_wr; }
    if (offset == HFV_CAP_REARM) {
        hfv_cap_wr = 0;
        hfv_cap_frozen = false;
        return 0;
    }
    if (offset >= HFV_CAP_SIZE) { return 0; }
    unsigned k = HFV_CAP_SIZE - offset;
    if (k > max) { k = max; }
    memcpy(out, hfv_cap + offset, k);
    return k;
}

static uint8_t hfv_reply[ISO15693_RESPONSE_MAX];
static uint8_t hfv_halves[(HFV_TX_HALVES_MAX + 7) / 8];     /* bit-packed reply, filled before the timed loop */
static uint16_t hfv_windows, hfv_replies, hfv_late;

static void delay_us(uint32_t us) {
    uint32_t start = DWT->CYCCNT;
    while (DWT->CYCCNT - start < us * HFV_CPU_CYCLES_PER_US) {
    }
}

static uint8_t spi_transfer(uint8_t value) {
    if (!radio_ok) {
        return 0;
    }
    NRF_SPI0->EVENTS_READY = 0;
    NRF_SPI0->TXD = value;
    uint32_t start = DWT->CYCCNT;
    while (!NRF_SPI0->EVENTS_READY) {
        if (DWT->CYCCNT - start > 1000u * HFV_CPU_CYCLES_PER_US) {
            radio_ok = false;
            return 0;
        }
    }
    return NRF_SPI0->RXD;
}

static void write_register(uint8_t reg, uint8_t value) {
    nrf_gpio_pin_clear(HF_SPI_SELECT);
    spi_transfer((reg << 1) & RC522_SPI_ADDRESS_MASK);
    spi_transfer(value);
    nrf_gpio_pin_set(HF_SPI_SELECT);
}

static uint8_t read_register(uint8_t reg) {
    uint8_t address = ((reg << 1) & RC522_SPI_ADDRESS_MASK) | RC522_SPI_READ;
    nrf_gpio_pin_clear(HF_SPI_SELECT);
    spi_transfer(address);
    uint8_t value = spi_transfer(address);
    nrf_gpio_pin_set(HF_SPI_SELECT);
    return value;
}

static void open_reader(void) {
    nrf_gpio_pin_set(READER_POWER);
    bsp_delay_ms(HFV_POWER_SETTLE_MS);
    NRF_SPI0->ENABLE = 0;
    NRF_SPI0->PSELSCK = HF_SPI_SCK;
    NRF_SPI0->PSELMOSI = HF_SPI_MOSI;
    NRF_SPI0->PSELMISO = HF_SPI_MISO;
    nrf_gpio_cfg_output(HF_SPI_SELECT);
    nrf_gpio_pin_set(HF_SPI_SELECT);
    nrf_gpio_cfg_output(HF_SPI_SCK);
    nrf_gpio_pin_clear(HF_SPI_SCK);
    nrf_gpio_cfg_output(HF_SPI_MOSI);
    nrf_gpio_cfg_input(HF_SPI_MISO, NRF_GPIO_PIN_NOPULL);
    NRF_SPI0->FREQUENCY = SPI_FREQUENCY_FREQUENCY_M8;
    NRF_SPI0->CONFIG = 0;
    NRF_SPI0->ENABLE = SPI_ENABLE_ENABLE_Enabled;
    radio_ok = true;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    write_register(CommandReg, PCD_RESET);
    delay_us(HFV_RESET_SETTLE_US);
    write_register(TxModeReg, HFV_TX_TYPE_B);
    write_register(RxModeReg, HFV_RX_MODE);
    write_register(TxAutoReg, HFV_TX_AUTO);
    write_register(TxSelReg, HFV_TX_CONSTANT_CARRIER);
    write_register(RxSelReg, HFV_RX_SELECT);
    write_register(RxThresholdReg, HFV_RX_THRESHOLD);
    write_register(DemodReg, HFV_DEMOD);
    write_register(RFCfgReg, HFV_INITIAL_RF_GAIN);
    write_register(GsNReg, HFV_N_CONDUCTANCE);
    write_register(CWGsCfgReg, HFV_CW_CONDUCTANCE);
    write_register(ModGsCfgReg, HFV_MOD_CONDUCTANCE);
    write_register(TxControlReg, RC522_FIELD_OFF);
    g_is_reader_antenna_on = false;
    delay_us(HFV_RESET_SETTLE_US);
}

static void close_reader(void) {
    write_register(CommandReg, PCD_IDLE);
    write_register(TxControlReg, RC522_FIELD_OFF);
    write_register(CommandReg, PCD_RESET);
    bsp_delay_ms(HFV_RESET_SETTLE_MS);
    write_register(TxControlReg, RC522_FIELD_OFF);
    write_register(TModeReg, 0);
    write_register(TxAutoReg, HFV_NFC_A_TX_AUTO);
    write_register(ModeReg, HFV_NFC_A_MODE);
    nrf_gpio_pin_set(HF_SPI_SELECT);
    NRF_SPIM0->ENABLE = 0;
    NRF_SPI0->ENABLE = 0;
    g_is_reader_antenna_on = false;
    nrf_gpio_pin_set(HF_ANT_SEL);
}

/* Reply bit stream (1 = subcarrier half-bit), prepared BEFORE the timed loop: computing it inside would
 * eat into the 75-cycle budget between two GPIO toggles. Table driven, runs inside the ~300 us t1 gap. */
static unsigned hfv_prepare(const uint8_t *data, unsigned length) {
    return hf15_tag_pack(data, length, hfv_halves, sizeof hfv_halves);
}

/* Load-modulate the packed reply, starting at DWT cycle `reply_at`. RAM function + IRQ disabled by the
 * caller so the 423.75 kHz subcarrier stays glitch-free. */
__attribute__((section(".data.hf15"), noinline, aligned(4))) static void
hfv_transmit(unsigned total, uint32_t reply_at) {
    NRF_NFCT->TASKS_DISABLE = 1;   /* an active NFCT suppresses this board's GPIO load modulation */
    __DSB();
    uint32_t mask = 1u << (HF_ANT_SEL & 31u);
    uint32_t wait_start = DWT->CYCCNT;
    while ((int32_t)(DWT->CYCCNT - reply_at) < 0) {
        if (DWT->CYCCNT - wait_start > HFV_RESPONSE_WAIT_TIMEOUT_CYCLES) {
            return;
        }
    }
    uint32_t next = DWT->CYCCNT + HFV_TX_START_MARGIN_CYCLES;
    uint32_t fraction = 0;
    for (unsigned h = 0; h < total; h++) {
        bool subcarrier = (hfv_halves[h >> 3] >> (h & 7u)) & 1u;
        for (unsigned k = 0; k < HFV_TX_HALF_PERIODS; k++) {
            volatile uint32_t *out = (subcarrier && !(k & 1u)) ? &NRF_P1->OUTSET : &NRF_P1->OUTCLR;
            while ((int32_t)(DWT->CYCCNT - next) < 0) {
            }
            *out = mask;
            next += HFV_TX_HALF_PERIOD_BASE_CYCLES;
            fraction += HFV_TX_HALF_PERIOD_FRACTION;
            if (fraction >= HFV_TX_HALF_PERIOD_DENOMINATOR) {
                fraction -= HFV_TX_HALF_PERIOD_DENOMINATOR;
                next++;
            }
        }
    }
    while ((int32_t)(DWT->CYCCNT - next) < 0) {
    }
    NRF_P1->OUTCLR = mask;
    hfv_responded = true;
}

static uint32_t hfv_absence_started;
static bool hfv_field_known;
static bool hfv_absence_pending;

static void hfv_nfct_sense_activate(void) {
    NRF_NFCT->EVENTS_READY = 0;
    /* No HW auto-collision: the NFCT must never auto-answer a Type-A REQA while it is in SENSE only to
     * read FIELDPRESENT (otherwise a reader latches a phantom ISO14443-3A tag), as on the ST25TB path. */
    NRF_NFCT->AUTOCOLRESCONFIG |= NFCT_AUTOCOLRESCONFIG_MODE_Msk;
    NRF_NFCT->TASKS_SENSE = 1;
    NRF_NFCT->TASKS_ACTIVATE = 1;
    uint32_t start = DWT->CYCCNT;
    while (!NRF_NFCT->EVENTS_READY && DWT->CYCCNT - start < HFV_NFCT_READY_TIMEOUT_CYCLES) {
    }
}

/* Decode one reader frame from the NFCT LOCKDETECT edges. Returns 1 with the frame (CRC bytes included)
 * in frame[0..*len_out) and the EOF falling-edge timestamp in *eof_cyc, or 0 if no frame completed
 * before the timeout / the field dropped. RAM function so the polling stays tight. IRQ stays on (proven
 * clean on device); a rare preemption only makes one frame malformed and the reader retries. */
__attribute__((section(".data.hf15"), noinline, aligned(4))) static int
hfv_lock_receive(uint8_t *frame, unsigned *len_out, uint32_t *eof_cyc, uint32_t deadline) {
    uint32_t rx_start = DWT->CYCCNT;
    uint32_t last_fall = 0, sof_ref = 0;
    unsigned sym_count = 0;
    bool prev = (NRF_NFCT->FIELDPRESENT & NFCT_FIELDPRESENT_LOCKDETECT_Msk) != 0;
    bool sof = false, c256 = false;
    unsigned w = hfv_cap_frozen ? HFV_CAP_SIZE : 0u;
    memset(frame, 0, HF15_RX_MAX);
    while (DWT->CYCCNT - rx_start <
           (uint32_t)(sof ? HFV_LOCK_FRAME_TIMEOUT_CYCLES : HFV_LOCK_SOF_TIMEOUT_CYCLES)) {
        /* Never run past the granted timeslot: overrunning it asserts the SoftDevice (reboot). */
        if ((int32_t)(deadline - DWT->CYCCNT) < (int32_t)HFV_WINDOW_WORK_GUARD_CYCLES) {
            return 0;
        }
        uint32_t now = DWT->CYCCNT;
        uint32_t fp = NRF_NFCT->FIELDPRESENT;
        bool cur = (fp & NFCT_FIELDPRESENT_LOCKDETECT_Msk) != 0;
        if (!(fp & NFCT_FIELDPRESENT_FIELDPRESENT_Msk)) {
            return 0;   /* field gone */
        }
        if (prev != cur) {
            if (w + 4u <= HFV_CAP_SIZE) {
                uint32_t d = now - rx_start;
                hfv_cap[w++] = (uint8_t)d;
                hfv_cap[w++] = (uint8_t)(d >> 8);
                hfv_cap[w++] = (uint8_t)(d >> 16);
                hfv_cap[w++] = (uint8_t)(((d >> 24) & 0x7Fu) | (cur ? 0x80u : 0u));
            }
            hfv_rx.pauses++;
        }
        /* LOCK can merge two PPM pulses two ticks apart: a short low after a last-position pulse is one
         * extra zero symbol. */
        if (!prev && cur && sof && last_fall) {
            uint32_t low = now - last_fall;
            uint32_t ft = HFV_TICK_FROM_CYC(last_fall - sof_ref);
            unsigned fpos = HFV_PULSE_POS(ft, c256);
            if (low > HFV_LOCK_MERGED_MIN && low < HFV_LOCK_MERGED_MAX &&
                fpos == (c256 ? 511u : 7u) && sym_count < HFV_LOCK_MAX_SYM(c256)) {
                sym_count++;
            }
        }
        if (prev && !cur) {   /* LOCK falling edge = pause start */
            uint32_t gap = now - last_fall;
            if (!sof) {
                if (last_fall &&
                    ((gap > HFV_LOCK_SOF_MIN_1OF4 && gap < HFV_LOCK_SOF_MAX_1OF4) ||
                     (gap > HFV_LOCK_SOF_MIN_1OF256 && gap < HFV_LOCK_SOF_MAX_1OF256))) {
                    c256 = gap > HFV_LOCK_SOF_SPLIT;
                    sof_ref = last_fall;
                    sym_count = 0;
                    sof = true;
                    memset(frame, 0, HF15_RX_MAX);
                }
            } else {
                uint32_t el = now - sof_ref;
                uint32_t pt = HFV_TICK_FROM_CYC(el);
                uint32_t expc = HFV_CYC_FROM_TICK(pt);
                uint32_t terr = el > expc ? el - expc : expc - el;
                unsigned sidx = HFV_SYM_IDX(pt, c256);
                unsigned pos = HFV_PULSE_POS(pt, c256);
                if (terr <= HFV_LOCK_TICK_TOL && sidx == sym_count + 1u && (pos & 1u) &&
                    sym_count < HFV_LOCK_MAX_SYM(c256)) {
                    if (c256) {
                        frame[sym_count] = (uint8_t)((pos - 1u) / 2u);
                    } else {
                        frame[sym_count / 4u] |= (uint8_t)(((pos - 1u) / 2u) << ((sym_count % 4u) * 2u));
                    }
                    sym_count++;
                } else if (terr <= HFV_LOCK_TICK_TOL && sidx == sym_count + 1u &&
                           pos == HFV_LOCK_EOF_POSITION && sym_count >= HFV_LOCK_MIN_SYM(c256) &&
                           (c256 || !(sym_count % 4u))) {
                    *len_out = c256 ? sym_count : sym_count / 4u;
                    *eof_cyc = now;
                    if (w) { hfv_cap_wr = w; hfv_cap_frozen = true; }
                    return 1;
                } else {
                    sof = false;
                    last_fall = 0;
                    hfv_rx.errors++;
                }
            }
            last_fall = now;
        }
        if (sof && now - last_fall > (uint32_t)(c256 ? HFV_LOCK_GAP_1OF256 : HFV_LOCK_GAP_1OF4)) {
            sof = false;
            last_fall = 0;
        }
        prev = cur;
    }
    return 0;
}

static void hfv_window_restore(uint32_t interrupts) {
    NRF_NFCT->TASKS_DISABLE = 1;
    NRF_NFCT->EVENTS_READY = 0;
    NRF_NFCT->EVENTS_FIELDDETECTED = 0;
    NRF_NFCT->EVENTS_FIELDLOST = 0;
    NRF_NFCT->INTENSET = interrupts;
    nrf_gpio_cfg_output(HF_ANT_SEL);
    nrf_gpio_pin_set(HF_ANT_SEL);
}

static void hfv_window(void) {
    uint32_t deadline = rf_timeslot_deadline() - HFV_WINDOW_CLEANUP_CYCLES;
    uint32_t interrupts = NRF_NFCT->INTENSET;
    hfv_windows++;
    NRF_NFCT->INTENCLR = 0xffffffffu;
    NRF_NFCT->EVENTS_READY = 0;
    NRF_NFCT->EVENTS_FIELDDETECTED = 0;
    NRF_NFCT->EVENTS_FIELDLOST = 0;
    NRF_NFCT->SHORTS = 0;
    /* HF_ANT_SEL: strong-drive output, held HIGH = "emulation mode" (HF antenna routed to NFCT) so NFCT
     * can sense the reader field and LOCKDETECT tracks the pauses (LOW routes the antenna to the RC522,
     * leaving NFCT blind: NFCTAGSTATE stuck in RAMPUP, FIELDPRESENT = 0). The transmit toggles it to
     * load-modulate the reply. */
    nrf_gpio_cfg(HF_ANT_SEL, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL,
                 NRF_GPIO_PIN_H0H1, NRF_GPIO_PIN_NOSENSE);
    nrf_gpio_pin_set(HF_ANT_SEL);
    hfv_nfct_sense_activate();
    uint32_t last_wdt = DWT->CYCCNT;
    uint8_t frame[HF15_RX_MAX];
    while (radio_ok && (int32_t)(deadline - DWT->CYCCNT) > HFV_WINDOW_WORK_GUARD_CYCLES) {
        if (nrf_gpio_pin_read(BUTTON_1) || nrf_gpio_pin_read(BUTTON_2)) {
            break;
        }
        if (DWT->CYCCNT - last_wdt > 100000u * HFV_CPU_CYCLES_PER_US) {   /* WDT 8 s, window up to 2 s */
            bsp_wdt_feed();
            last_wdt = DWT->CYCCNT;
        }
        if (!NRF_NFCT->EVENTS_READY) {
            hfv_nfct_sense_activate();
        }
        if (!(NRF_NFCT->FIELDPRESENT & NFCT_FIELDPRESENT_FIELDPRESENT_Msk)) {
            if (hfv_field_known) {
                uint32_t now = DWT->CYCCNT;
                if (!hfv_absence_pending) {
                    hfv_absence_started = now;
                    hfv_absence_pending = true;
                } else if (now - hfv_absence_started > HFV_FIELD_LOSS_CYCLES) {
                    iso15693_model_field_lost();
                    hfv_field_known = false;
                    hfv_absence_pending = false;
                    hfv_responded = false;
                }
            }
            continue;
        }
        unsigned flen = 0;
        uint32_t eof_cyc = 0;
        /* Receive AND reply run with IRQ disabled so neither the PPM symbol timing nor the 423.75 kHz
         * subcarrier is jittered by a preemption (BLE / USB), which otherwise corrupts the odd frame and
         * makes the reader mis-identify the tag. hfv_lock_receive bails at the timeslot deadline, so the
         * IRQ-off stretch stays bounded inside the granted slot (no SoftDevice overrun). */
        uint32_t irq = __get_PRIMASK();
        __disable_irq();
        int r = hfv_lock_receive(frame, &flen, &eof_cyc, deadline);
        bool replied = false, got_frame = false;
        if (r == 1) {
            got_frame = true;
            /* iso15693_model_reply verifies the CRC itself and applies writes/locks before replying. */
            unsigned rlen = iso15693_model_reply(frame, flen, hfv_reply);
            if (rlen) {
                unsigned total = hfv_prepare(hfv_reply, rlen);
                uint32_t reply_at = eof_cyc + HFV_LOCK_RESPONSE_WAIT_CYCLES;
                uint32_t tx_cost = (reply_at - DWT->CYCCNT) +
                                   (uint32_t)total * HFV_TX_HALF_PERIODS *
                                       (HFV_TX_HALF_PERIOD_BASE_CYCLES + 1u) +
                                   HFV_WINDOW_WORK_GUARD_CYCLES;
                if (total && (int32_t)(deadline - DWT->CYCCNT) >= (int32_t)tx_cost) {
                    hfv_transmit(total, reply_at);
                    replied = true;
                }
            }
        }
        __set_PRIMASK(irq);
        if (!got_frame) {
            continue;
        }
        hfv_rx.frames++;
        /* A valid request means a real reader is present: keep the field state, light the LED, etc. */
        if (hf15_check_crc(frame, flen)) {
            hfv_field_known = true;
            hfv_absence_pending = false;
        }
        if (!replied) {
            hfv_late++;
            continue;
        }
        hfv_replies++;
        /* Back to emulation mode (antenna -> NFCT) and bring NFCT back so LOCKDETECT is live again. */
        nrf_gpio_pin_set(HF_ANT_SEL);
        hfv_nfct_sense_activate();
    }
    hfv_window_restore(interrupts);
}

bool nfc_hf15_radio_poll(bool *field_present, bool active) {
    *field_present = false;
    if (!iso15693_model_loaded()) {
        return false;
    }
    if (!reader_open) {
        open_reader();
        write_register(RFCfgReg, HFV_RF_GAIN);
        reader_open = true;
    }
    bool ready = radio_ok && (read_register(TxControlReg) & RC522_FIELD_ENABLE_MASK) == 0 && radio_ok;
    bool resume = false;
    bool paused = ready && hfv_advertising_pause(&resume);
    if (paused) {
        bsp_wdt_feed();
    }
    hfv_responded = false;
    uint32_t window_us = HFV_SERVICE_WINDOW_US;
    if (paused) {
        if (nrfx_power_usbstatus_get() == NRFX_POWER_USB_STATE_DISCONNECTED) {
            window_us = active ? HFV_ACTIVE_WINDOW_US : HFV_IDLE_WINDOW_US;
        } else if (hfv_usb_hfclk_active()) {
            window_us = active ? HFV_ACTIVE_WINDOW_US : HFV_USB_WINDOW_US;
        }
    }
    // Same window policy as ST25TB: HIGH priority; idle + BLE connected keeps the short window so the
    // link gets gaps, an active reader always gets the long window.
    extern volatile bool g_is_ble_connected;
    if (!active && g_is_ble_connected) {
        window_us = HFV_SERVICE_WINDOW_US;
    }
    bool granted = ready && rf_timeslot_run(window_us, true, paused, hfv_window);
    if (resume) {
        advertising_start(false);
    }
    *field_present = hfv_field_known || hfv_responded;
    bool sampled = granted && radio_ok;
    if (!ready || !radio_ok || (granted && !*field_present)) {
        close_reader();
        reader_open = false;
    }
    return sampled;
}

void nfc_hf15_radio_stop(void) {
    if (!reader_open) {
        return;
    }
    close_reader();
    reader_open = false;
    hfv_field_known = false;
    hfv_absence_pending = false;
}

bool nfc_hf15_radio_responded(void) {
    return hfv_responded;
}

static unsigned put16(uint8_t *o, unsigned n, uint32_t v) {
    o[n++] = (uint8_t)(v >> 8);
    o[n++] = (uint8_t)v;
    return n;
}

unsigned nfc_hf15_radio_diag(uint8_t *out, unsigned cap) {
    if (cap < 28u) return 0;
    unsigned n = 0;
    n = put16(out, n, hfv_rx.pauses);
    n = put16(out, n, hfv_rx.frames);
    n = put16(out, n, hfv_rx.errors);
    n = put16(out, n, hfv_rx.eof_only);
    out[n++] = hfv_rx.last_error;
    out[n++] = hfv_rx.peak_level;
    n = put16(out, n, hfv_rx.last_pause_len);
    n = put16(out, n, hfv_replies);
    n = put16(out, n, hfv_late);
    n = put16(out, n, hfv_windows);
    out[n++] = hfv_field_known ? 1u : 0u;
    out[n++] = radio_ok ? 1u : 0u;
    for (int b = 3; b >= 0; b--) { out[n++] = (uint8_t)(((uint32_t)hfv_rx.zero_i) >> (8 * b)); }
    for (int b = 3; b >= 0; b--) { out[n++] = (uint8_t)(((uint32_t)hfv_rx.zero_q) >> (8 * b)); }
    return n;
}

#else  /* !PROJECT_CHAMELEON_ULTRA: no RC522 / antenna switch on the Lite */

bool nfc_hf15_radio_poll(bool *field_present, bool active) { (void)active; *field_present = false; return false; }
void nfc_hf15_radio_stop(void) {}
bool nfc_hf15_radio_responded(void) { return false; }
unsigned nfc_hf15_radio_diag(uint8_t *out, unsigned cap) { (void)out; (void)cap; return 0; }
unsigned nfc_hf15_radio_capture(uint8_t *out, unsigned offset, unsigned max, unsigned *wr_out) {
    (void)out; (void)offset; (void)max; if (wr_out) { *wr_out = 0; } return 0;
}

#endif /* PROJECT_CHAMELEON_ULTRA */
