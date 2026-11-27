/* SPDX-License-Identifier: GPL-3.0-only */
/* ISO15693 (NFC-V) emulation RF. Hardware handling is taken VERBATIM from nfc_st25tb_radio.c (validated
 * on device for ST25TB): RC522 with the field off as an ADC receiver of the reader's carrier, SPIM3
 * streaming TestADCReg into three DMA banks, TIMER3/TIMER4 + PPI timestamping, NFCT SENSE/ACTIVATE only
 * to read FIELDPRESENT, HF_ANT_SEL GPIO load modulation, same windows / watchdog / BLE coexistence.
 * Only the protocol layer differs: the hf15 codec decodes the reader's "1 out of 4/256" pauses and the
 * reply is load-modulated at fc/32 (423.75 kHz) with ISO15693 Manchester coding (SOF/EOF per the Flipper
 * reference). ST25TB and NFC-V never run together (one HF type per slot), so they share the hardware and
 * the RF workspace exactly like ST25TB alone does. DA VALIDARE su device: RF timing constants below. */
#include "nfc_hf15.h"
#include "iso15693/hf15_model.h"
#include "iso15693/hf15_codec.h"
#include "rc522.h"
#include "rf_timeslot.h"
#include "nrfx_power.h"
#include "usb_main.h"
#include "nrfx_ppi.h"
#include "nrfx_timer.h"
#include "../../reader/rf_workspace.h"
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

#define HFV_BATCH 128u
#define HFV_MIN_BATCH_BYTES 32u
#define HFV_DMA_BANK_COUNT 3u
#define HFV_DMA_BANK_BYTES 8192u
#define HFV_DMA_GUARD_BYTES 32u
#define HFV_DMA_LAG_BYTES 16u
#define HFV_DMA_START_MARGIN_TICKS 32u
#define HFV_TX_SETUP_RESERVE_US 10u

enum {
    HFV_CPU_CYCLES_PER_US = 64,
    HFV_TIMER_TICKS_PER_US = 16,
    HFV_SPI_TIMEOUT_CYCLES = 1000 * HFV_CPU_CYCLES_PER_US,
    HFV_DMA_EVENT_TIMEOUT_CYCLES = 100 * HFV_CPU_CYCLES_PER_US,
    HFV_NFCT_READY_TIMEOUT_CYCLES = 100 * HFV_CPU_CYCLES_PER_US,
    HFV_WINDOW_CLEANUP_CYCLES = 2000 * HFV_CPU_CYCLES_PER_US,
    HFV_WINDOW_WORK_GUARD_CYCLES = 2000 * HFV_CPU_CYCLES_PER_US,
    /* ISO15693 t1 = 4352/fc = 320.9 us (+-2.4 us) from the rising edge ending the reader's EOF pause to
     * the start of our SOF, whose first 3 halves (56.64 us) are unmodulated: the transmitted stream starts
     * at the first subcarrier pulse (k_sof = S S S U S), nominally 377.5 us after that edge. The first
     * pulse goes out ~2 us after reply_at (start margin) and eof_end trails the real edge by the ADC /
     * detection lag (~1-3 us, not measured yet): 372 aims at the nominal. DA VALIDARE with a scope or a
     * reader's timing log, then adjust this one constant. */
    HFV_REPLY_FIRST_PULSE_US = 372,
    HFV_RESPONSE_WAIT_TIMEOUT_CYCLES = 1000 * HFV_CPU_CYCLES_PER_US,
    HFV_FIELD_PROBE_CYCLES = 100 * HFV_CPU_CYCLES_PER_US,
    HFV_FIELD_LOSS_CYCLES = 3000 * HFV_CPU_CYCLES_PER_US,
    HFV_FIELD_DROP_CYCLES = 2000 * HFV_CPU_CYCLES_PER_US,
    HFV_FIELD_PRESENT_PROBE_CYCLES = 200000 * HFV_CPU_CYCLES_PER_US,
    HFV_FIELD_ABSENT_PROBE_CYCLES = 15000 * HFV_CPU_CYCLES_PER_US,
    HFV_ACTIVE_WINDOW_US = 2000000,
    HFV_IDLE_WINDOW_US = 500000,
    HFV_SERVICE_WINDOW_US = 25000,
    HFV_USB_WINDOW_US = 500000,
    HFV_TX_GUARD_US = 2500,
    HFV_HALF_BIT_US_CEIL = 19,          /* 18.879 us rounded up, for the transmit-time budget */
    HFV_RX_SETTLE_SAMPLES = 32,         /* skip the ADC recovery after our own load modulation */
    HFV_POWER_SETTLE_MS = 1,
    HFV_RESET_SETTLE_US = 500,
    HFV_RESET_SETTLE_MS = 10,
    HFV_RECEIVE_SETTLE_US = 100,
};

/* RC522 field-off ADC reception + GPIO modulation: identical to the qualified ST25TB settings. */
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
    HFV_ADC_READ_COMMAND = ((TestADCReg << 1) & RC522_SPI_ADDRESS_MASK) | RC522_SPI_READ,
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

_Static_assert(HFV_DMA_BANK_COUNT * HFV_DMA_BANK_BYTES + 2 * HFV_DMA_GUARD_BYTES <= RF_HF_SNIFF_BYTES,
               "SPIM3 banks exceed RF workspace");

static uint8_t *dma_rx;
static bool radio_ok;
static bool reader_open;
static bool hfv_responded;
extern bool g_is_reader_antenna_on;

static hf15_rx_t hfv_rx;
/* DEBUG bring-up: raw ADC-sample ring of the emulation RX, dumped via HF15_EMU_CAPTURE to feed the host
 * codec test with REAL reader waveforms (the synthetic model passes but the device doesn't — tune here). */
#define HFV_CAP_SIZE 4096u
#define HFV_CAP_REARM 0xFFFFu
static uint8_t hfv_cap[HFV_CAP_SIZE];
static volatile unsigned hfv_cap_wr;
static volatile bool hfv_cap_frozen;       /* stop writing once a real command burst landed, so the ring holds it */
static uint16_t hfv_cap_pause_base;        /* pauses counter at the last arm, to detect a real command burst */
/* offset==HFV_CAP_REARM: re-arm (unfreeze, reset) and return. Else: dump a raw slice at `offset`; the host
 * reorders oldest->newest using *wr_out. Freezes after a BURST of pauses (a real reader command) lands, not on
 * the first noise blip, so the dump holds the strong command burst (see the feed loop). */
unsigned nfc_hf15_radio_capture(uint8_t *out, unsigned offset, unsigned max, unsigned *wr_out) {
    if (wr_out) { *wr_out = hfv_cap_wr; }
    if (offset == HFV_CAP_REARM) {
        hfv_cap_pause_base = hfv_rx.pauses;
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
        if (DWT->CYCCNT - start > HFV_SPI_TIMEOUT_CYCLES) {
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
    nrf_gpio_pin_clear(READER_POWER);
    nrf_gpio_pin_set(HF_ANT_SEL);
}

static uint8_t *hfv_dma_at(unsigned bank) {
    return dma_rx + bank * HFV_DMA_BANK_BYTES;
}

/* Reply bit stream (1 = subcarrier half-bit), prepared BEFORE the timed loop: computing it inside would
 * eat into the 75-cycle budget between two GPIO toggles. Table driven (a few cycles per byte): it runs
 * between the reader's EOF and the reply, so even a 160-byte reply is ready well inside the ~300 us left. */
static unsigned hfv_prepare(const uint8_t *data, unsigned length) {
    return hf15_tag_pack(data, length, hfv_halves, sizeof hfv_halves);
}

__attribute__((section(".data.hf15"), noinline, aligned(4))) static void
hfv_transmit(unsigned total, uint32_t reply_at) {
    /* Active NFCT suppresses this board's GPIO load modulation. */
    NRF_NFCT->TASKS_DISABLE = 1;
    __DSB();
    uint32_t mask = 1u << (HF_ANT_SEL & 31u);
    uint32_t deadline = rf_timeslot_deadline() - HFV_WINDOW_CLEANUP_CYCLES;
    uint32_t wait_start = DWT->CYCCNT;
    for (;;) {
        NRF_TIMER4->TASKS_CAPTURE[1] = 1;
        __DSB();
        if ((int32_t)(NRF_TIMER4->CC[1] - reply_at) >= 0) {
            break;
        }
        if (DWT->CYCCNT - wait_start > HFV_RESPONSE_WAIT_TIMEOUT_CYCLES ||
            (int32_t)(deadline - DWT->CYCCNT) < 0) {
            radio_ok = false;
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

/* Returns true when the field appears after being unknown or absent. */
static bool hfv_probe_field(void) {
    if (!NRF_NFCT->EVENTS_READY) {
        return false;
    }
    nrf_gpio_pin_set(HF_ANT_SEL);
    uint32_t start = DWT->CYCCNT;
    bool present;
    do {
        present = (NRF_NFCT->FIELDPRESENT & 1u) != 0;
    } while (!present && DWT->CYCCNT - start < HFV_FIELD_PROBE_CYCLES);
    nrf_gpio_pin_clear(HF_ANT_SEL);
    if (present) {
        bool arrived = !hfv_field_known || hfv_absence_pending;
        hfv_field_known = true;
        hfv_absence_pending = false;
        return arrived;
    }
    hf15_rx_restart(&hfv_rx);
    uint32_t now = DWT->CYCCNT;
    if (hfv_field_known) {
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
    return false;
}

static const nrfx_timer_t hfv_counter = NRFX_TIMER_INSTANCE(3);
static const nrfx_timer_t hfv_clock = NRFX_TIMER_INSTANCE(4);
static nrf_ppi_channel_t hfv_counter_ppi;
static nrf_ppi_channel_t hfv_start_ppi;
static bool hfv_start_owned;
static bool hfv_streaming;
static unsigned hfv_queued;
static uint32_t hfv_bank_start[HFV_DMA_BANK_COUNT];

static void hfv_timer_handler(nrf_timer_event_t event, void *context) {
    (void)event;
    (void)context;
}

static void hfv_dma_release(void);

static bool hfv_dma_acquire(void) {
    nrfx_timer_config_t config = NRFX_TIMER_DEFAULT_CONFIG;
    config.mode = NRF_TIMER_MODE_COUNTER;
    config.bit_width = NRF_TIMER_BIT_WIDTH_32;
    if (nrfx_timer_init(&hfv_counter, &config, hfv_timer_handler) != NRFX_SUCCESS) {
        return false;
    }
    config.mode = NRF_TIMER_MODE_TIMER;
    config.frequency = NRF_TIMER_FREQ_16MHz;
    if (nrfx_timer_init(&hfv_clock, &config, hfv_timer_handler) != NRFX_SUCCESS) {
        nrfx_timer_uninit(&hfv_counter);
        return false;
    }
    if (nrfx_ppi_channel_alloc(&hfv_counter_ppi) != NRFX_SUCCESS) {
        nrfx_timer_uninit(&hfv_clock);
        nrfx_timer_uninit(&hfv_counter);
        return false;
    }
    if (nrfx_ppi_channel_assign(hfv_counter_ppi, (uint32_t)&NRF_SPIM3->EVENTS_END,
                                (uint32_t)&NRF_TIMER3->TASKS_COUNT) != NRFX_SUCCESS ||
        nrfx_ppi_channel_fork_assign(hfv_counter_ppi, (uint32_t)&NRF_TIMER4->TASKS_CAPTURE[2]) != NRFX_SUCCESS) {
        hfv_dma_release();
        return false;
    }
    if (nrfx_ppi_channel_alloc(&hfv_start_ppi) != NRFX_SUCCESS) {
        hfv_dma_release();
        return false;
    }
    hfv_start_owned = true;
    if (nrfx_ppi_channel_assign(hfv_start_ppi, (uint32_t)&NRF_SPIM3->EVENTS_STARTED,
                                (uint32_t)&NRF_TIMER4->TASKS_CAPTURE[0]) != NRFX_SUCCESS ||
        nrfx_ppi_channel_fork_assign(hfv_start_ppi, 0) != NRFX_SUCCESS) {
        hfv_dma_release();
        return false;
    }
    nrfx_timer_enable(&hfv_clock);
    nrfx_timer_enable(&hfv_counter);
    return true;
}

static void hfv_dma_release(void) {
    if (hfv_start_owned) {
        nrfx_ppi_channel_disable(hfv_start_ppi);
        nrfx_ppi_channel_fork_assign(hfv_start_ppi, 0);
        nrfx_ppi_channel_free(hfv_start_ppi);
        hfv_start_owned = false;
    }
    nrfx_ppi_channel_disable(hfv_counter_ppi);
    nrfx_ppi_channel_fork_assign(hfv_counter_ppi, 0);
    nrfx_ppi_channel_free(hfv_counter_ppi);
    nrfx_timer_uninit(&hfv_clock);
    nrfx_timer_uninit(&hfv_counter);
}

static uint32_t hfv_completed(void) {
    NRF_TIMER3->TASKS_CAPTURE[0] = 1;
    __DSB();
    return NRF_TIMER3->CC[0];
}

static void hfv_spi_disable(void) {
    NRF_SPIM3->ENABLE = 0;
    /* Match the SDK's SPIM3 erratum195 disable sequence. */
    *(volatile uint32_t *)0x4002f004u = 1;
}

static void hfv_spi_stop(void) {
    NRF_SPIM3->SHORTS = 0;
    nrfx_ppi_channel_disable(hfv_start_ppi);
    nrfx_ppi_channel_disable(hfv_counter_ppi);
    if (!hfv_streaming) {
        nrf_gpio_pin_set(HF_SPI_SELECT);
        return;
    }
    NRF_SPIM3->EVENTS_STOPPED = 0;
    NRF_SPIM3->TASKS_STOP = 1;
    uint32_t start = DWT->CYCCNT;
    while (!NRF_SPIM3->EVENTS_STOPPED) {
        if (DWT->CYCCNT - start > HFV_DMA_EVENT_TIMEOUT_CYCLES) {
            radio_ok = false;
            break;
        }
    }
    if (NRF_SPIM3->EVENTS_STOPPED) {
        hfv_streaming = false;
    }
    nrf_gpio_pin_set(HF_SPI_SELECT);
}

static void hfv_stream_start(void) {
    if (!radio_ok) {
        return;
    }
    NRF_TIMER3->TASKS_CLEAR = 1;
    NRF_SPIM3->RXD.PTR = (uint32_t)hfv_dma_at(0);
    NRF_SPIM3->EVENTS_END = 0;
    NRF_SPIM3->EVENTS_STARTED = 0;
    NRF_SPIM3->STALLSTAT = 0;
    NRF_SPIM3->SHORTS = SPIM_SHORTS_END_START_Msk;
    if (nrfx_ppi_channel_enable(hfv_counter_ppi) != NRFX_SUCCESS ||
        nrfx_ppi_channel_enable(hfv_start_ppi) != NRFX_SUCCESS) {
        radio_ok = false;
        return;
    }
    nrf_gpio_pin_clear(HF_SPI_SELECT);
    NRF_SPIM3->TASKS_START = 1;
    hfv_streaming = true;
    uint32_t start = DWT->CYCCNT;
    while (!NRF_SPIM3->EVENTS_STARTED) {
        if (DWT->CYCCNT - start > HFV_DMA_EVENT_TIMEOUT_CYCLES) {
            radio_ok = false;
            return;
        }
    }
    hfv_bank_start[0] = NRF_TIMER4->CC[0];
    hfv_queued = 1;
    NRF_SPIM3->RXD.PTR = (uint32_t)hfv_dma_at(1);
    NRF_SPIM3->EVENTS_STARTED = 0;
}

static void hfv_stream_configure(void) {
    uint8_t *storage = rf_hf_sniff_workspace();
    dma_rx = storage + HFV_DMA_GUARD_BYTES;
    memset(storage, 0xa5, HFV_DMA_GUARD_BYTES);
    memset(dma_rx + HFV_DMA_BANK_COUNT * HFV_DMA_BANK_BYTES, 0xa5, HFV_DMA_GUARD_BYTES);
    NRF_SPI0->ENABLE = 0;
    NRF_SPIM0->ENABLE = 0;
    NRF_SPIM3->PSEL.SCK = HF_SPI_SCK;
    NRF_SPIM3->PSEL.MOSI = HF_SPI_MOSI;
    NRF_SPIM3->PSEL.MISO = HF_SPI_MISO;
    NRF_SPIM3->CONFIG = 0;
    NRF_SPIM3->FREQUENCY = SPIM_FREQUENCY_FREQUENCY_M8;
    NRF_SPIM3->ORC = HFV_ADC_READ_COMMAND;
    /* ORC emits F6 without a TX RAM channel (erratum198). */
    NRF_SPIM3->TXD.PTR = 0;
    NRF_SPIM3->TXD.MAXCNT = 0;
    NRF_SPIM3->TXD.LIST = 0;
    NRF_SPIM3->RXD.MAXCNT = HFV_DMA_BANK_BYTES;
    NRF_SPIM3->RXD.LIST = 0;
    NRF_SPIM3->ENABLE = SPIM_ENABLE_ENABLE_Enabled;
    hfv_streaming = false;
    nrf_gpio_cfg(HF_ANT_SEL, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL,
                 NRF_GPIO_PIN_H0H1, NRF_GPIO_PIN_NOSENSE);
    nrf_gpio_pin_clear(HF_ANT_SEL);
}

static unsigned hfv_available(unsigned handled) {
    uint32_t completed = hfv_completed();
    if (completed < handled || completed - handled >= HFV_DMA_BANK_COUNT || completed > hfv_queued) {
        radio_ok = false;
        return 0;
    }
    if (NRF_SPIM3->STALLSTAT) {
        radio_ok = false;
        return 0;
    }
    if (completed == hfv_queued) {
        uint32_t start = DWT->CYCCNT;
        while (!NRF_SPIM3->EVENTS_STARTED) {
            if (DWT->CYCCNT - start > HFV_DMA_EVENT_TIMEOUT_CYCLES) {
                radio_ok = false;
                return 0;
            }
        }
        uint32_t ended = NRF_TIMER4->CC[2];
        uint32_t phase = ended - hfv_bank_start[(completed - 1u) % HFV_DMA_BANK_COUNT] -
                         HFV_DMA_BANK_BYTES * HFV_TIMER_TICKS_PER_US;
        if (NRF_SPIM3->RXD.AMOUNT != HFV_DMA_BANK_BYTES || phase > HFV_DMA_START_MARGIN_TICKS) {
            radio_ok = false;
            return 0;
        }
        hfv_bank_start[completed % HFV_DMA_BANK_COUNT] = NRF_TIMER4->CC[0];
        NRF_SPIM3->EVENTS_STARTED = 0;
        NRF_SPIM3->RXD.PTR = (uint32_t)hfv_dma_at((completed + 1u) % HFV_DMA_BANK_COUNT);
        hfv_queued = completed + 1u;
        if (hfv_completed() != completed) {
            radio_ok = false;
            return 0;
        }
    }
    if (completed > handled) {
        return HFV_DMA_BANK_BYTES;
    }
    NRF_TIMER4->TASKS_CAPTURE[1] = 1;
    __DSB();
    uint32_t elapsed = NRF_TIMER4->CC[1] - hfv_bank_start[handled % HFV_DMA_BANK_COUNT];
    unsigned bytes = elapsed / HFV_TIMER_TICKS_PER_US;
    if (bytes > HFV_DMA_BANK_BYTES + HFV_DMA_EVENT_TIMEOUT_CYCLES / HFV_CPU_CYCLES_PER_US) {
        radio_ok = false;
        return 0;
    }
    if (bytes > HFV_DMA_BANK_BYTES) {
        bytes = HFV_DMA_BANK_BYTES;
    }
    return bytes > HFV_DMA_LAG_BYTES ? bytes - HFV_DMA_LAG_BYTES : 0;
}

static void hfv_window_restore(uint32_t interrupts) {
    NRF_NFCT->TASKS_DISABLE = 1;
    NRF_NFCT->EVENTS_READY = 0;
    NRF_NFCT->EVENTS_FIELDDETECTED = 0;
    NRF_NFCT->EVENTS_FIELDLOST = 0;
    NRF_NFCT->INTENSET = interrupts;
    hfv_spi_disable();
    hfv_streaming = false;
    NRF_SPIM3->SHORTS = 0;
    NRF_SPI0->ENABLE = SPI_ENABLE_ENABLE_Enabled;
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
    // No HW auto-collision: the NFCT must never auto-answer a Type-A REQA while it is in SENSE only to
    // read FIELDPRESENT (otherwise a reader latches a phantom ISO14443-3A tag), as on the ST25TB path.
    NRF_NFCT->AUTOCOLRESCONFIG |= NFCT_AUTOCOLRESCONFIG_MODE_Msk;
    NRF_NFCT->TASKS_SENSE = 1;
    NRF_NFCT->TASKS_ACTIVATE = 1;
    uint32_t start = DWT->CYCCNT;
    while (!NRF_NFCT->EVENTS_READY && DWT->CYCCNT - start < HFV_NFCT_READY_TIMEOUT_CYCLES) {
    }
    hfv_probe_field();
    write_register(CommandReg, PCD_RECEIVE);
    delay_us(HFV_RECEIVE_SETTLE_US);
    hfv_stream_configure();
    unsigned handled = 0;
    unsigned n = 1;                         /* byte 0 of a stream is clocked during the first command */
    uint32_t last_probe = DWT->CYCCNT;
    uint32_t field_seen = last_probe;
    bool field_readable = true;
    hf15_rx_restart(&hfv_rx);
    hfv_stream_start();
    uint32_t sample_origin = hfv_bank_start[0];
    uint32_t last_wdt = DWT->CYCCNT;
    while (radio_ok && (int32_t)(deadline - DWT->CYCCNT) > HFV_WINDOW_WORK_GUARD_CYCLES) {
        if (nrf_gpio_pin_read(BUTTON_1) || nrf_gpio_pin_read(BUTTON_2)) {
            break;
        }
        if (DWT->CYCCNT - last_wdt > 100000u * HFV_CPU_CYCLES_PER_US) {   /* WDT 8 s, window up to 2 s */
            bsp_wdt_feed();
            last_wdt = DWT->CYCCNT;
        }
        unsigned available = hfv_available(handled);
        if (!radio_ok) {
            break;
        }
        if (n == HFV_DMA_BANK_BYTES) {
            handled++;
            n = 0;
            continue;
        }
        if (available <= n) {
            continue;
        }
        if (available - n < HFV_MIN_BATCH_BYTES && available != HFV_DMA_BANK_BYTES) {
            continue;
        }
        unsigned ready = handled % HFV_DMA_BANK_COUNT;
        uint32_t bank_start = hfv_bank_start[ready];
        unsigned bytes = available - n;
        if (bytes > HFV_BATCH) {
            bytes = HFV_BATCH;
        }
        uint32_t t_us = (bank_start - sample_origin) / HFV_TIMER_TICKS_PER_US + n;
        if (!hfv_cap_frozen) {   /* DEBUG: capture the raw ADC bytes into the ring for host-side codec tuning */
            const uint8_t *csrc = hfv_dma_at(ready) + n;
            unsigned w = hfv_cap_wr;
            for (unsigned k = 0; k < bytes; k++) { hfv_cap[w] = csrc[k]; if (++w >= HFV_CAP_SIZE) { w = 0; } }
            hfv_cap_wr = w;
        }
        unsigned used = 0;
        uint32_t eof_end = 0;
        hf15_rx_event_t ev = hf15_rx_feed(&hfv_rx, hfv_dma_at(ready) + n, bytes, t_us, &used, &eof_end);
        /* DEBUG: freeze the capture ring once a BURST of pauses (a real reader command, not a noise blip)
         * has accumulated since arming, so the 4 ms ring holds the strong command waveform. */
        if (!hfv_cap_frozen && ((uint16_t)(hfv_rx.pauses - hfv_cap_pause_base) >= 40u || ev == HF15_RX_FRAME)) {
            hfv_cap_frozen = true;
        }
        n += used;
        bool restart = false;
        bool replied = false;
        if (ev != HF15_RX_NONE) {
            /* Apply the request FIRST (a Write/Lock must land even if we then miss the reply window). */
            unsigned rlen = ev == HF15_RX_FRAME ? iso15693_model_reply(hfv_rx.data, hfv_rx.len, hfv_reply)
                                                : iso15693_model_slot_marker(hfv_reply);
            /* Field presence only from a real reader: a request with a valid CRC, or a slot marker of a
             * 16-slot round we are in. ADC noise that looks like a lone pause must not wake BLE coexistence,
             * stop sleep, light the LED or postpone field-lost. */
            if (rlen || (ev == HF15_RX_FRAME && hf15_check_crc(hfv_rx.data, hfv_rx.len))) {
                hfv_field_known = true;
                hfv_absence_pending = false;
            }
            unsigned total = rlen ? hfv_prepare(hfv_reply, rlen) : 0u;
            if (total) {
                uint32_t eof_tick = sample_origin + eof_end * HFV_TIMER_TICKS_PER_US + HFV_DMA_START_MARGIN_TICKS;
                uint32_t reply_at = eof_tick + (uint32_t)HFV_REPLY_FIRST_PULSE_US * HFV_TIMER_TICKS_PER_US;
                NRF_TIMER4->TASKS_CAPTURE[1] = 1;
                __DSB();
                bool timely = (int32_t)(reply_at - NRF_TIMER4->CC[1]) >=
                              (int32_t)(HFV_TX_SETUP_RESERVE_US * HFV_TIMER_TICKS_PER_US);
                bool fits = (int32_t)(deadline - DWT->CYCCNT) >=
                            (int32_t)((total * HFV_HALF_BIT_US_CEIL + HFV_REPLY_FIRST_PULSE_US + HFV_TX_GUARD_US) *
                                      HFV_CPU_CYCLES_PER_US);
                bool still_timely = false;
                if (timely && fits && hfv_available(handled)) {
                    /* hfv_available() may have waited for a DMA bank boundary: check the window again. */
                    NRF_TIMER4->TASKS_CAPTURE[1] = 1;
                    __DSB();
                    still_timely = (int32_t)(reply_at - NRF_TIMER4->CC[1]) >=
                                   (int32_t)(HFV_TX_SETUP_RESERVE_US * HFV_TIMER_TICKS_PER_US);
                }
                if (still_timely) {
                    hfv_spi_stop();
                    if (!radio_ok) {
                        break;
                    }
                    uint32_t irq = __get_PRIMASK();
                    __disable_irq();
                    hfv_transmit(total, reply_at);
                    NRF_NFCT->EVENTS_READY = 0;
                    NRF_NFCT->AUTOCOLRESCONFIG |= NFCT_AUTOCOLRESCONFIG_MODE_Msk;   // no phantom Type-A on re-sense
                    NRF_NFCT->TASKS_SENSE = 1;
                    NRF_NFCT->TASKS_ACTIVATE = 1;
                    __set_PRIMASK(irq);
                    hfv_replies++;
                    last_probe = DWT->CYCCNT;
                    restart = true;
                    replied = true;
                } else {
                    hfv_late++;             /* reply window missed: the reader retries the request */
                }
            }
        }
        bool present = hfv_field_known && !hfv_absence_pending;
        if (NRF_NFCT->FIELDPRESENT & 1u) {
            field_seen = DWT->CYCCNT;
        }
        bool dropped = field_readable && present && DWT->CYCCNT - field_seen > HFV_FIELD_DROP_CYCLES;
        uint32_t interval = present ? HFV_FIELD_PRESENT_PROBE_CYCLES : HFV_FIELD_ABSENT_PROBE_CYCLES;
        if (!restart && NRF_NFCT->EVENTS_READY &&
            (dropped || (DWT->CYCCNT - last_probe > interval && (!present || hf15_rx_idle(&hfv_rx))))) {
            restart = true;
        }
        if (!restart) {
            continue;
        }
        hfv_spi_stop();
        if (!radio_ok) {
            break;
        }
        if (!replied) {
            if (hfv_probe_field()) {
                break;
            }
            if (dropped && !hfv_absence_pending) {
                field_readable = false;
            }
            last_probe = DWT->CYCCNT;
        }
        handled = 0;
        n = 1;
        hf15_rx_restart(&hfv_rx);
        if (replied) {
            hf15_rx_settle(&hfv_rx, HFV_RX_SETTLE_SAMPLES);
        }
        hfv_stream_start();
        sample_origin = hfv_bank_start[0];
        field_seen = DWT->CYCCNT;
    }
    hfv_spi_stop();
    for (unsigned i = 0; i < HFV_DMA_GUARD_BYTES; i++) {
        if (dma_rx[-(int)HFV_DMA_GUARD_BYTES + (int)i] != 0xa5 ||
            dma_rx[HFV_DMA_BANK_COUNT * HFV_DMA_BANK_BYTES + i] != 0xa5) {
            radio_ok = false;
        }
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
    if (!hfv_dma_acquire()) {
        nfc_hf15_radio_stop();
        return false;
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
    hfv_dma_release();
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
    if (cap < 20u) return 0;
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
    return n;
}

#else  /* !PROJECT_CHAMELEON_ULTRA: no RC522 / antenna switch on the Lite */

bool nfc_hf15_radio_poll(bool *field_present, bool active) { (void)active; *field_present = false; return false; }
void nfc_hf15_radio_stop(void) {}
bool nfc_hf15_radio_responded(void) { return false; }
unsigned nfc_hf15_radio_diag(uint8_t *out, unsigned cap) { (void)out; (void)cap; return 0; }

#endif /* PROJECT_CHAMELEON_ULTRA */
