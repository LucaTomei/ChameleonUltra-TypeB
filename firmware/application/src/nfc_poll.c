#include "nfc_poll.h"
#include "settings.h"
#include "tag_emulation.h"

#define NRF_LOG_MODULE_NAME nfc_poll
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

// State for field-triggered slot polling.
static bool     m_field_present = false;
static uint8_t  m_saved_slot    = 0;      // slot to restore when the field leaves
static uint8_t  m_cursor        = 0;      // slot currently shown while polling
static uint32_t m_last_switch   = 0;      // timestamp of the last slot advance

// Is this slot usable for polling (HF or LF sense enabled)?
static bool slot_usable(uint8_t slot) {
    return is_slot_enabled(slot, TAG_SENSE_HF) || is_slot_enabled(slot, TAG_SENSE_LF);
}

// Next enabled slot after `from` (wraps). Returns `from` if none other is enabled.
static uint8_t next_enabled_slot(uint8_t from) {
    for (uint8_t step = 1; step <= TAG_MAX_SLOT_NUM; step++) {
        uint8_t s = (from + step) % TAG_MAX_SLOT_NUM;
        if (slot_usable(s)) return s;
    }
    return from;
}

void nfc_poll_init(void) {
    m_field_present = false;
    m_saved_slot = tag_emulation_get_slot();
    m_cursor = m_saved_slot;
    m_last_switch = 0;
}

void nfc_poll_on_field(bool present) {
    if (!settings_get_polling_enable()) return;
    if (present == m_field_present) return;
    m_field_present = present;
    if (present) {
        // Field just arrived: remember the active slot and start cycling from it.
        m_saved_slot = tag_emulation_get_slot();
        m_cursor = m_saved_slot;
        m_last_switch = 0;   // allow the first advance immediately on the next tick
    } else {
        // Field left: restore the original slot.
        if (tag_emulation_get_slot() != m_saved_slot) {
            tag_emulation_change_slot(m_saved_slot, true);
        }
        NRF_LOG_INFO("poll: field gone, restored slot %d", m_saved_slot);
    }
}

void nfc_poll_tick(uint32_t now_ms) {
    if (!settings_get_polling_enable() || !m_field_present) return;
    uint16_t interval = settings_get_polling_interval();
    if (m_last_switch != 0 && (now_ms - m_last_switch) < interval) return;
    uint8_t next = next_enabled_slot(m_cursor);
    if (next != m_cursor) {
        m_cursor = next;
        tag_emulation_change_slot(m_cursor, true);
        NRF_LOG_INFO("poll: advance to slot %d", m_cursor);
    }
    m_last_switch = now_ms;
}
