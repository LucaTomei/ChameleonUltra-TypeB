#include "nfc_st25tb.h"

#include "nfc_poll.h"
#include "fds_util.h"
#include "st25tb/model.h"
#include "tag_persistence.h"
#include "rfid_main.h"
#include "nfc_14a.h"
#include "nrfx_nfct.h"
#include "app_timer.h"

#include "syssleep.h"
#include "ble_main.h"
#include <string.h>

#define ST25TB_RECORD_SIZE 80u       // 512-bit: image 77 padded to a word boundary
#define ST25TB_RECORD_SIZE_4K 528u   // 4K: image 525 padded to a word boundary

static unsigned st25tb_image_size_for(tag_specific_type_t t) {
    return t == TAG_TYPE_ST25TB04K ? ST25TB_IMAGE_SIZE_4K : ST25TB_IMAGE_SIZE;
}
static unsigned st25tb_record_size_for(tag_specific_type_t t) {
    return t == TAG_TYPE_ST25TB04K ? ST25TB_RECORD_SIZE_4K : ST25TB_RECORD_SIZE;
}
static bool is_st25tb_type(tag_specific_type_t t) {
    return t == TAG_TYPE_ST25TB512_AT || t == TAG_TYPE_ST25TB04K;
}

#if defined(PROJECT_CHAMELEON_ULTRA)
extern volatile bool g_usb_led_marquee_enable;
static bool sensing;
static bool watching;
static volatile bool field_active;
static volatile bool probe_due;
static bool probe_timer_created;
static uint32_t last_window;
APP_TIMER_DEF(probe_timer);

// --- ST25TB emulation ↔ BLE coexistence (slave latency, NO disconnect) ---
// Type-B/ST25TB emulation runs on the software radio (SoftDevice timeslot), which a live BLE link
// preempts every connection interval — truncating the long windows the bit-bang needs, so a reader
// shows "don't move" and a truncated read decodes to the wrong data (e.g. wrong MyKey credit). We do
// NOT disconnect (the user wants the link to stay up): instead, when a reader field arrives we raise
// the connection's SLAVE LATENCY (ble_set_rf_coexist(true)) so the SoftDevice lets the peripheral
// skip connection events and grants the ST25TB radio full-length windows. We keep latency high across
// repeated reads, and only drop back to a responsive latency once the reader has been gone HOLD_MS.
#define ST25TB_EMU_HOLD_MS 4000
static bool emu_paused;                 // high RF-coexist latency is currently requested for emulation
static bool resume_timer_created;
static volatile bool resume_due;        // set by the hold timer → restore the responsive BLE latency
APP_TIMER_DEF(resume_timer);

static void resume_tick(void *context) {
    (void)context;
    resume_due = true;
}

static void probe_tick(void *context) {
    (void)context;
    probe_due = true;
}

static void field_event(nrfx_nfct_evt_t const *event) {
    if (event->evt_id != NRFX_NFCT_EVT_FIELD_DETECTED) {
        return;
    }
    // Kill the NFCT immediately so it can't auto-answer a Type-A REQA (the reader would otherwise
    // latch onto a bogus 14443-A UID 0 instead of our Type-B/ST25TB response). The radio layer
    // takes over field handling from here.
    NRF_NFCT->TASKS_DISABLE = 1;
    __DSB();
    field_active = true;
    sleep_timer_stop();
    g_is_tag_emulating = true;
    g_usb_led_marquee_enable = false;
    TAG_FIELD_LED_ON();
}

static void watch_field(void) {
    // Intentionally does NOT put the NFCT in SENSE mode anymore. nrfx_nfct_enable() would make the
    // NFCT auto-answer a Type-A REQA, so a generic reader (e.g. Flipper "Read NFC") latches a
    // phantom ISO14443-3A tag (UID 0) instead of our Type-B/ST25TB response. Field detection is
    // done by the radio layer, which drives the NFCT SENSE->ACTIVATE (ACTIVATED does not
    // auto-answer) only to read FIELDPRESENT. Keep nfc_14a off while an ST25TB slot is active.
    nrfx_nfct_uninit();
    watching = false;
    (void)field_event;   // still referenced by the (now unused) config path; keep the symbol live
}
#endif

int nfc_st25tb_data_loadcb(tag_specific_type_t type, tag_data_buffer_t *buffer) {
    unsigned rec = st25tb_record_size_for(type);
    unsigned img = st25tb_image_size_for(type);
    if (!is_st25tb_type(type) || buffer->length < rec || !st25tb_load(buffer->buffer, img)) {
        st25tb_unload();
        return 0;
    }
    return rec;
}

int nfc_st25tb_data_savecb(tag_specific_type_t type, tag_data_buffer_t *buffer) {
    unsigned rec = st25tb_record_size_for(type);
    // Guardia st25tb_loaded() anche per NON azzerare il buffer HF condiviso quando non c'e' nulla da
    // salvare (st25tb_save fallirebbe e lascerebbe il buffer azzerato inutilmente).
    if (!is_st25tb_type(type) || buffer->length < rec || !st25tb_loaded()) {
        return 0;
    }
    // Azzera TUTTO il record prima del save: st25tb_save scrive st25tb_image_size() byte del modello
    // CARICATO; se quella dimensione e' minore di quella attesa dal tipo (es. 512 caricata in uno
    // slot st25tb04k), i byte in coda resterebbero garbage del buffer HF condiviso e finirebbero in
    // flash. Azzerando prima, la coda non scritta resta 0.
    memset(buffer->buffer, 0, rec);
    if (!st25tb_save(buffer->buffer)) {
        return 0;
    }
    return rec;
}

bool nfc_st25tb_data_factory(uint8_t slot, tag_specific_type_t type) {
    if (!is_st25tb_type(type)) {
        return false;
    }
    static uint8_t image[ST25TB_RECORD_SIZE_4K];      // static, not stack: 528 B would risk a stack overflow
    // Carta ST25TB VERGINE: blocchi dati, OTP, contatori e blocco di sistema/lock tutti a 1 (0xFF)
    // → completamente SCRIVIBILE, come una carta reale nuova. Con tutto a 0 si emulava una carta
    // completamente BLOCCATA (lock attivo-basso, contatori esauriti, OTP gia' programmata) che
    // rifiutava silenziosamente ogni scrittura.
    memset(image, 0xff, sizeof(image));
    image[5] = (type == TAG_TYPE_ST25TB04K) ? 0x1f : 0x33;   // product code in the UID
    image[6] = 0x02;
    image[7] = 0xd0;
    unsigned img = st25tb_image_size_for(type);
    image[img - 1] = 0;                                      // chip_id (id anticollisione), derivato a runtime
    fds_slot_record_map_t map;
    get_fds_map_by_slot_sense_type_for_dump(slot, TAG_SENSE_HF, &map);
    return fds_write_sync(map.id, map.key, st25tb_record_size_for(type), image);
}

bool nfc_st25tb_set_image(const uint8_t *data, unsigned data_len) {
    uint8_t slot = tag_emulation_get_slot();
    tag_slot_specific_type_t types;
    tag_emulation_get_specific_types_by_slot(slot, &types);
    unsigned img = st25tb_image_size_for(types.tag_hf);
    unsigned rec = st25tb_record_size_for(types.tag_hf);
    // The active slot must be an ST25TB type and the payload length must match that type's image
    // (77 for 512-bit, 525 for 4K). The UID must end in D0 02 (ST25 family + STMicro); the product
    // code (data[5]) is reported from the Chip_ID at anticollision, so every variant is accepted.
    if (!is_st25tb_type(types.tag_hf) || !data || data_len != img || data[6] != 0x02 || data[7] != 0xd0) {
        return false;
    }

    tag_data_buffer_t *buffer = get_buffer_by_tag_type(types.tag_hf);
    if (!buffer || buffer->length < rec) {
        return false;
    }

    static uint8_t record[ST25TB_RECORD_SIZE_4K];     // static, not stack: 528 B would risk a stack overflow
    memset(record, 0, sizeof(record));
    memcpy(record, data, img);
    fds_slot_record_map_t map;
    get_fds_map_by_slot_sense_type_for_dump(slot, TAG_SENSE_HF, &map);
#if defined(PROJECT_CHAMELEON_ULTRA)
    bool was_sensing = sensing;
#else
    bool was_sensing = false;
#endif
    nfc_st25tb_sense_switch(false);
    if (!fds_write_sync(map.id, map.key, rec, record)) {
        nfc_st25tb_sense_switch(was_sensing);
        return false;
    }
    // Reload only HF: LF may contain edits that have not been saved to flash.
    memcpy(buffer->buffer, record, rec);
    bool loaded = tag_emulation_load_by_buffer(types.tag_hf, true);
    if (loaded && get_device_mode() == DEVICE_MODE_TAG) {
        tag_emulation_sense_switch(TAG_SENSE_HF, true);
    }
    return loaded;
}

bool nfc_st25tb_get_image(uint8_t *image) {
    tag_slot_specific_type_t types;
    tag_emulation_get_specific_types_by_slot(tag_emulation_get_slot(), &types);
    return is_st25tb_type(types.tag_hf) && st25tb_save(image);
}

unsigned nfc_st25tb_image_size(void) { return st25tb_image_size(); }

void nfc_st25tb_sense_switch(bool enable) {
#if defined(PROJECT_CHAMELEON_ULTRA)
    bool was_sensing = sensing;
    if (watching && !enable) {
        nrfx_nfct_uninit();
        watching = false;
    }
    if (!enable) {
        nfc_st25tb_radio_stop();
    }
    sensing = enable;
    if (enable && !was_sensing) {
        probe_due = false;
        if (!probe_timer_created) {
            probe_timer_created = app_timer_create(&probe_timer, APP_TIMER_MODE_REPEATED, probe_tick) == NRF_SUCCESS;
        }
        if (probe_timer_created) {
            app_timer_start(probe_timer, APP_TIMER_TICKS(500), NULL);
        }
    } else if (!enable && was_sensing) {
        if (probe_timer_created) {
            app_timer_stop(probe_timer);
        }
        probe_due = false;
        // Leaving ST25TB sensing (e.g. slot change / mode switch) while the high RF-coexist latency is
        // active: restore the responsive latency and cancel the hold timer, so the link isn't left slow
        // on a slot that no longer emulates ST25TB.
        if (resume_timer_created) app_timer_stop(resume_timer);
        resume_due = false;
        if (emu_paused) { ble_set_rf_coexist(false); emu_paused = false; }
    }
    if (enable) {
        watch_field();
    } else if (was_sensing) {
        st25tb_field_lost();
        field_active = false;
        g_is_tag_emulating = false;
        TAG_FIELD_LED_OFF();
    }
    // !enable && !was_sensing: ST25TB was already off. Do NOT run watch_field()/nrfx_nfct_uninit()
    // here: this path also runs when the active slot is Type-A (e.g. restoring sense state after an
    // HF sniff), and tearing down the NFCT would leave Type-A emulation dead while its software
    // state still says "enabled".
#endif
}

bool nfc_st25tb_service(void) {
#if defined(PROJECT_CHAMELEON_ULTRA)
    if (!sensing || get_device_mode() != DEVICE_MODE_TAG) {
        return false;
    }
    // Hold window elapsed with no reader field: restore the responsive BLE latency, and only now allow
    // the device to sleep. Guarded by !field_active so a field that arrived meanwhile wins.
    if (resume_due) {
        resume_due = false;
        if (emu_paused && !field_active) {
            ble_set_rf_coexist(false);
            emu_paused = false;
            sleep_timer_start(SLEEP_DELAY_MS_FIELD_NFC_LOST);
        }
    }
    if (field_active) {
        /* A USB reset must not restart animation while the reader owns the field. */
        g_usb_led_marquee_enable = false;
    }
    if (false) {
        return field_active;
    }
    /* Always run the ST25TB radio (no NFCT-SENSE watcher → no phantom Type-A). */
    /* Inter-window gap. While merely sensing, leave 10 ms after each window to drain queued USB
     * replies. But DURING AN ACTIVE TRANSACTION keep the gap tiny (~2 ms): a long read (e.g. a 4K
     * card's 128 blocks) spans several windows, and a reader (Flipper) sends its next Read_Block
     * during the gap — a 10 ms hole makes it get no answer and give up ("don't move"). A ~2 ms gap
     * is short enough that the reader's retry lands in the next open window, so the read is
     * effectively continuous across windows. */
    uint32_t gap_ticks = field_active ? APP_TIMER_TICKS(2) : APP_TIMER_TICKS(10);
    if (app_timer_cnt_diff_compute(app_timer_cnt_get(), last_window) < gap_ticks) {
        return field_active;
    }
    bool active = field_active;
    probe_due = false;
    if (watching) {
        nrfx_nfct_uninit();
        watching = false;
    }
    bool present;
    bool sampled = nfc_st25tb_radio_poll(&present, active);
    last_window = app_timer_cnt_get();
    if (!sampled) {
        if (!active) {
            nfc_st25tb_radio_stop();
        }
        return active;
    }
    if (present) {
        if (!emu_paused) {
            // First reader field of this emulation session: raise the BLE slave latency ONCE so the
            // SoftDevice grants the ST25TB radio full-length windows (a latency-0 link caps the window
            // to ~1 connection interval → "don't move" / truncated read / wrong credit). The link
            // STAYS CONNECTED. We keep the high latency across repeated reads and only restore the
            // responsive latency after the reader has been gone for ST25TB_EMU_HOLD_MS (the hold timer).
            ble_set_rf_coexist(true);
            emu_paused = true;
        }
        if (resume_timer_created) app_timer_stop(resume_timer);   // field back → cancel any pending resume
        field_active = true;
        sleep_timer_stop();
        g_is_tag_emulating = true;
        g_usb_led_marquee_enable = false;
        TAG_FIELD_LED_ON();
        // Feed the slot-polling layer: a reader field is on an ST25TB slot. If our tag actually
        // answered a command this window, treat it as "selected" so polling freezes on this slot
        // instead of cycling it out mid-transaction (ST25TB has no Type-A anticollision hook).
        nfc_poll_on_field(true);
        if (nfc_st25tb_radio_responded()) nfc_poll_notify_selected();
        return true;
    }
    nfc_st25tb_radio_stop();
    if (active) {
        field_active = false;
        st25tb_field_lost();
        g_is_tag_emulating = false;
        TAG_FIELD_LED_OFF();
        nfc_poll_on_field(false);   // ST25TB field gone → let polling restore the saved slot
        if (emu_paused) {
            // Keep the high RF-coexist latency for a hold window so the NEXT read keeps full windows
            // with no renegotiation. Do NOT arm the sleep timer yet (the resume handler arms it): the
            // device stays awake and emulating during the hold. Latency is restored when the timer fires.
            if (!resume_timer_created) {
                resume_timer_created = app_timer_create(&resume_timer, APP_TIMER_MODE_SINGLE_SHOT, resume_tick) == NRF_SUCCESS;
            }
            bool armed = false;
            if (resume_timer_created) {
                app_timer_stop(resume_timer);
                armed = app_timer_start(resume_timer, APP_TIMER_TICKS(ST25TB_EMU_HOLD_MS), NULL) == NRF_SUCCESS;
            }
            if (!armed) {
                // No hold timer (create or start failed): restore latency NOW so the link can never be
                // left stuck at high latency with no timer to recover it.
                ble_set_rf_coexist(false); emu_paused = false;
                sleep_timer_start(SLEEP_DELAY_MS_FIELD_NFC_LOST);
            }
        } else {
            sleep_timer_start(SLEEP_DELAY_MS_FIELD_NFC_LOST);
        }
    }
#endif
    return false;
}
