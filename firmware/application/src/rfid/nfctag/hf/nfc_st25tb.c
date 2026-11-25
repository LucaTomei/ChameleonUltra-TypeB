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
#include <string.h>

#define ST25TB_RECORD_SIZE 80u

#if defined(PROJECT_CHAMELEON_ULTRA)
extern volatile bool g_usb_led_marquee_enable;
static bool sensing;
static bool watching;
static volatile bool field_active;
static volatile bool probe_due;
static bool probe_timer_created;
static uint32_t last_window;
APP_TIMER_DEF(probe_timer);

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
    if (type != TAG_TYPE_ST25TB512_AT || buffer->length < ST25TB_RECORD_SIZE || buffer->buffer[77] ||
        buffer->buffer[78] || buffer->buffer[79] || !st25tb_load(buffer->buffer)) {
        st25tb_unload();
        return 0;
    }
    return ST25TB_RECORD_SIZE;
}

int nfc_st25tb_data_savecb(tag_specific_type_t type, tag_data_buffer_t *buffer) {
    if (type != TAG_TYPE_ST25TB512_AT || buffer->length < ST25TB_RECORD_SIZE || !st25tb_save(buffer->buffer)) {
        return 0;
    }
    memset(buffer->buffer + ST25TB_IMAGE_SIZE, 0, ST25TB_RECORD_SIZE - ST25TB_IMAGE_SIZE);
    return ST25TB_RECORD_SIZE;
}

bool nfc_st25tb_data_factory(uint8_t slot, tag_specific_type_t type) {
    if (type != TAG_TYPE_ST25TB512_AT) {
        return false;
    }
    uint8_t image[ST25TB_RECORD_SIZE] = {0};
    image[5] = 0x33;
    image[6] = 0x02;
    image[7] = 0xd0;
    fds_slot_record_map_t map;
    get_fds_map_by_slot_sense_type_for_dump(slot, TAG_SENSE_HF, &map);
    return fds_write_sync(map.id, map.key, sizeof(image), image);
}

bool nfc_st25tb_set_image(const uint8_t data[ST25TB_IMAGE_SIZE]) {
    uint8_t slot = tag_emulation_get_slot();
    tag_slot_specific_type_t types;
    tag_emulation_get_specific_types_by_slot(slot, &types);
    // Accept any ST25TB/SRI 512-bit chip (UID wire bytes 7..6 = D0 02); the product code (data[5])
    // is reported from image[76] during anticollision, so the 16-block model serves all 512 variants.
    if (types.tag_hf != TAG_TYPE_ST25TB512_AT || !data || data[6] != 0x02 || data[7] != 0xd0) {
        return false;
    }

    tag_data_buffer_t *buffer = get_buffer_by_tag_type(types.tag_hf);
    if (!buffer || buffer->length < ST25TB_RECORD_SIZE) {
        return false;
    }

    uint8_t record[ST25TB_RECORD_SIZE] = {0};
    memcpy(record, data, ST25TB_IMAGE_SIZE);
    fds_slot_record_map_t map;
    get_fds_map_by_slot_sense_type_for_dump(slot, TAG_SENSE_HF, &map);
#if defined(PROJECT_CHAMELEON_ULTRA)
    bool was_sensing = sensing;
#else
    bool was_sensing = false;
#endif
    nfc_st25tb_sense_switch(false);
    if (!fds_write_sync(map.id, map.key, sizeof(record), record)) {
        nfc_st25tb_sense_switch(was_sensing);
        return false;
    }
    // Reload only HF: LF may contain edits that have not been saved to flash.
    memcpy(buffer->buffer, record, sizeof(record));
    bool loaded = tag_emulation_load_by_buffer(types.tag_hf, true);
    if (loaded && get_device_mode() == DEVICE_MODE_TAG) {
        tag_emulation_sense_switch(TAG_SENSE_HF, true);
    }
    return loaded;
}

bool nfc_st25tb_get_image(uint8_t image[ST25TB_IMAGE_SIZE]) {
    tag_slot_specific_type_t types;
    tag_emulation_get_specific_types_by_slot(tag_emulation_get_slot(), &types);
    return types.tag_hf == TAG_TYPE_ST25TB512_AT && st25tb_save(image);
}

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
    if (field_active) {
        /* A USB reset must not restart animation while the reader owns the field. */
        g_usb_led_marquee_enable = false;
    }
    if (false) {
        return field_active;
    }
    /* Always run the ST25TB radio (no NFCT-SENSE watcher → no phantom Type-A). */
    /* Leave 10 ms after every bounded RF window to drain queued USB replies.
     * Host traffic must not prevent the next listening window. */
    if (app_timer_cnt_diff_compute(app_timer_cnt_get(), last_window) < APP_TIMER_TICKS(10)) {
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
        sleep_timer_start(SLEEP_DELAY_MS_FIELD_NFC_LOST);
    }
#endif
    return false;
}
