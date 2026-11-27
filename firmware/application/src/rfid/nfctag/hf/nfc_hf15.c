/* SPDX-License-Identifier: GPL-3.0-only */
/* ISO15693 (NFC-V) tag emulation: slot glue. Mirrors nfc_st25tb.c (validated on device) — same field
 * handling, inter-window gaps, BLE coexistence through slave latency, hold timer, stuck-window watchdog
 * and slot-polling hooks — with the ISO15693 model instead of the ST25TB one. */
#include "nfc_hf15.h"

#include "nfc_poll.h"
#include "fds_util.h"
#include "iso15693/hf15_model.h"
#include "tag_persistence.h"
#include "rfid_main.h"
#include "nfc_14a.h"
#include "nrfx_nfct.h"
#include "app_timer.h"
#include "syssleep.h"
#include "ble_main.h"
#include <string.h>

static unsigned record_size(unsigned image_size) { return (image_size + 3u) & ~3u; }

/* One scratch record shared by factory / set_image (main-loop context only, never re-entered). */
static uint8_t s_record[ISO15693_RECORD_MAX];

#if defined(PROJECT_CHAMELEON_ULTRA)
extern volatile bool g_usb_led_marquee_enable;
static bool sensing;
static volatile bool field_active;
static volatile bool probe_due;
static bool probe_timer_created;
static uint32_t last_window;
APP_TIMER_DEF(hf15_probe_timer);

#define HF15_EMU_HOLD_MS 4000
static bool emu_paused;                 // high RF-coexist latency is currently requested for emulation
static bool resume_timer_created;
static volatile bool resume_due;
static uint32_t active_miss_since;
APP_TIMER_DEF(hf15_resume_timer);

static void resume_tick(void *context) {
    (void)context;
    resume_due = true;
}

static void probe_tick(void *context) {
    (void)context;
    probe_due = true;
}
#endif

int nfc_hf15_data_loadcb(tag_specific_type_t type, tag_data_buffer_t *buffer) {
    unsigned size = iso15693_model_image_size_for(buffer->buffer, buffer->length);
    if (!IS_ISO15693_TYPE(type) || !size || record_size(size) > buffer->length ||
            !iso15693_model_load(buffer->buffer, size)) {
        iso15693_model_unload();
        return 0;
    }
    return (int)record_size(size);
}

int nfc_hf15_data_savecb(tag_specific_type_t type, tag_data_buffer_t *buffer) {
    // Only when an image is loaded: never zero the shared HF buffer for nothing.
    if (!IS_ISO15693_TYPE(type) || !iso15693_model_loaded()) {
        return 0;
    }
    unsigned rec = record_size(iso15693_model_image_size());
    if (rec > buffer->length) {
        return 0;
    }
    memset(buffer->buffer, 0, rec);         // the padding tail must not carry stale bytes into flash
    if (!iso15693_model_save(buffer->buffer)) {
        return 0;
    }
    return (int)rec;
}

bool nfc_hf15_data_factory(uint8_t slot, tag_specific_type_t type) {
    if (!IS_ISO15693_TYPE(type)) {
        return false;
    }
    memset(s_record, 0, sizeof(s_record));
    unsigned size = iso15693_model_factory_image(s_record, sizeof(s_record));
    if (!size) {
        return false;
    }
    fds_slot_record_map_t map;
    get_fds_map_by_slot_sense_type_for_dump(slot, TAG_SENSE_HF, &map);
    return fds_write_sync(map.id, map.key, record_size(size), s_record);
}

bool nfc_hf15_set_image(const uint8_t *data, unsigned data_len) {
    uint8_t slot = tag_emulation_get_slot();
    tag_slot_specific_type_t types;
    tag_emulation_get_specific_types_by_slot(slot, &types);
    // The active slot must be an NFC-V slot and the payload exactly one valid v1 image.
    if (!IS_ISO15693_TYPE(types.tag_hf) || !data || iso15693_model_image_size_for(data, data_len) != data_len) {
        return false;
    }
    unsigned rec = record_size(data_len);
    tag_data_buffer_t *buffer = get_buffer_by_tag_type(types.tag_hf);
    if (!buffer || buffer->length < rec || rec > sizeof(s_record)) {
        return false;
    }
    memset(s_record, 0, sizeof(s_record));
    memcpy(s_record, data, data_len);
    fds_slot_record_map_t map;
    get_fds_map_by_slot_sense_type_for_dump(slot, TAG_SENSE_HF, &map);
#if defined(PROJECT_CHAMELEON_ULTRA)
    bool was_sensing = sensing;
#else
    bool was_sensing = false;
#endif
    nfc_hf15_sense_switch(false);
    if (!fds_write_sync(map.id, map.key, rec, s_record)) {
        nfc_hf15_sense_switch(was_sensing);
        return false;
    }
    // Reload only HF: LF may hold edits not yet saved to flash.
    memcpy(buffer->buffer, s_record, rec);
    bool loaded = tag_emulation_load_by_buffer(types.tag_hf, true);
    if (loaded && get_device_mode() == DEVICE_MODE_TAG) {
        tag_emulation_sense_switch(TAG_SENSE_HF, true);
    }
    return loaded;
}

const uint8_t *nfc_hf15_image_view(unsigned *size) {
    tag_slot_specific_type_t types;
    tag_emulation_get_specific_types_by_slot(tag_emulation_get_slot(), &types);
    const uint8_t *img = IS_ISO15693_TYPE(types.tag_hf) ? iso15693_model_image() : NULL;
    *size = img ? iso15693_model_image_size() : 0u;
    return img;
}

void nfc_hf15_sense_switch(bool enable) {
#if defined(PROJECT_CHAMELEON_ULTRA)
    bool was_sensing = sensing;
    // NFC-V was not active and stays off: touch NOTHING. This path also runs for Type-A / ST25TB slots
    // (sense routing turns the other HF engines off first), and tearing down the NFCT here would leave
    // Type-A emulation dead while its software state still says "enabled".
    if (!enable && !was_sensing) {
        return;
    }
    if (!enable) {
        nfc_hf15_radio_stop();
    }
    sensing = enable;
    if (enable && !was_sensing) {
        probe_due = false;
        if (!probe_timer_created) {
            probe_timer_created = app_timer_create(&hf15_probe_timer, APP_TIMER_MODE_REPEATED, probe_tick) == NRF_SUCCESS;
        }
        if (probe_timer_created) {
            app_timer_start(hf15_probe_timer, APP_TIMER_TICKS(500), NULL);
        }
    } else if (!enable && was_sensing) {
        if (probe_timer_created) {
            app_timer_stop(hf15_probe_timer);
        }
        probe_due = false;
        // Leaving NFC-V with the high RF-coexist latency active: restore the responsive latency.
        if (resume_timer_created) app_timer_stop(hf15_resume_timer);
        resume_due = false;
        if (emu_paused) { ble_set_rf_coexist(false); emu_paused = false; }
        iso15693_model_field_lost();
        field_active = false;
        g_is_tag_emulating = false;
        TAG_FIELD_LED_OFF();
    }
    if (enable) {
        // As ST25TB's watch_field() on every enable: the radio drives the NFCT itself (SENSE->ACTIVATE
        // only to read FIELDPRESENT), so release the nrfx driver — it must never auto-answer a Type-A
        // REQA (a reader would latch a phantom ISO14443-A tag) while an NFC-V slot is active.
        nrfx_nfct_uninit();
    }
#else
    (void)enable;
#endif
}

bool nfc_hf15_service(void) {
#if defined(PROJECT_CHAMELEON_ULTRA)
    if (!sensing || get_device_mode() != DEVICE_MODE_TAG) {
        return false;
    }
    // Hold window elapsed with no reader field: restore the responsive BLE latency, then allow sleep.
    if (resume_due) {
        resume_due = false;
        if (emu_paused && !field_active) {
            ble_set_rf_coexist(false);
            emu_paused = false;
            sleep_timer_start(SLEEP_DELAY_MS_FIELD_NFC_LOST);
        }
    }
    if (field_active) {
        g_usb_led_marquee_enable = false;   // a USB reset must not restart animation mid-read
    }
    // Inter-window gap: 10 ms while merely sensing (drain USB), ~2 ms during a transaction so the
    // reader's next request lands in the next window (same reasoning as ST25TB).
    uint32_t gap_ticks = field_active ? APP_TIMER_TICKS(2) : APP_TIMER_TICKS(10);
    if (app_timer_cnt_diff_compute(app_timer_cnt_get(), last_window) < gap_ticks) {
        return field_active;
    }
    bool active = field_active;
    probe_due = false;
    bool present;
    bool sampled = nfc_hf15_radio_poll(&present, active);
    last_window = app_timer_cnt_get();
    if (!sampled) {
        if (!active) {
            nfc_hf15_radio_stop();
            active_miss_since = 0;
            return active;
        }
        // Active but nothing sampled: normally a transient (timeslot briefly denied). If it persists,
        // declare the field lost and restore the BLE coexistence (same watchdog as ST25TB).
        uint32_t now = app_timer_cnt_get();
        if (active_miss_since == 0) {
            active_miss_since = now;
        } else if (app_timer_cnt_diff_compute(now, active_miss_since) >= APP_TIMER_TICKS(HF15_EMU_HOLD_MS)) {
            active_miss_since = 0;
            nfc_hf15_radio_stop();
            field_active = false;
            iso15693_model_field_lost();
            g_is_tag_emulating = false;
            TAG_FIELD_LED_OFF();
            nfc_poll_on_field(false);
            if (emu_paused) { ble_set_rf_coexist(false); emu_paused = false; }
            sleep_timer_start(SLEEP_DELAY_MS_FIELD_NFC_LOST);
            return false;
        }
        return active;
    }
    active_miss_since = 0;
    if (present) {
        if (!emu_paused) {
            ble_set_rf_coexist(true);       // raise slave latency once: full-length RF windows, link kept
            emu_paused = true;
        }
        if (resume_timer_created) app_timer_stop(hf15_resume_timer);
        field_active = true;
        sleep_timer_stop();
        g_is_tag_emulating = true;
        g_usb_led_marquee_enable = false;
        TAG_FIELD_LED_ON();
        nfc_poll_on_field(true);
        if (nfc_hf15_radio_responded()) nfc_poll_notify_selected();
        return true;
    }
    nfc_hf15_radio_stop();
    if (active) {
        field_active = false;
        iso15693_model_field_lost();
        g_is_tag_emulating = false;
        TAG_FIELD_LED_OFF();
        nfc_poll_on_field(false);
        if (emu_paused) {
            if (!resume_timer_created) {
                resume_timer_created = app_timer_create(&hf15_resume_timer, APP_TIMER_MODE_SINGLE_SHOT, resume_tick) == NRF_SUCCESS;
            }
            bool armed = false;
            if (resume_timer_created) {
                app_timer_stop(hf15_resume_timer);
                armed = app_timer_start(hf15_resume_timer, APP_TIMER_TICKS(HF15_EMU_HOLD_MS), NULL) == NRF_SUCCESS;
            }
            if (!armed) {
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
