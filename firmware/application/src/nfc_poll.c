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
static bool     m_selected      = false;  // the reader selected the current slot → stop cycling
static bool     m_restore_pending = false; // field left → restore the saved slot from the main loop
static uint16_t m_advance_budget = 0;     // advances left before we give up cycling (see tick)

// Is this slot usable for polling (HF or LF sense enabled)?
static bool slot_usable(uint8_t slot) {
    return is_slot_enabled(slot, TAG_SENSE_HF) || is_slot_enabled(slot, TAG_SENSE_LF);
}

static uint8_t usable_slot_count(void) {
    uint8_t n = 0;
    for (uint8_t s = 0; s < TAG_MAX_SLOT_NUM; s++) if (slot_usable(s)) n++;
    return n;
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
        // Field just arrived. Remember the active slot to restore later — but ONLY if we are not
        // mid-restore from a just-ended session. On a field bounce (FIELD_LOST then FIELD_DETECTED
        // before nfc_poll_tick runs the deferred restore), the current slot is a ROTATED one, not
        // the user's original: overwriting m_saved_slot with it would lose the real slot forever.
        // Keep the original saved slot and cancel the pending restore instead.
        if (!m_restore_pending) {
            m_saved_slot = tag_emulation_get_slot();
        }
        m_restore_pending = false;   // we're active again; the saved slot still points at the original
        m_cursor = tag_emulation_get_slot();
        m_last_switch = 0;   // allow the first advance immediately on the next tick
        m_selected = false;  // new field session: cycle until the reader selects a slot
        // Budget: at most ~2 full sweeps of the usable slots before giving up (see tick). Bounds
        // the "rotate forever" case when a field-loss notification is missed (e.g. the field was LF
        // but we rotated onto an HF slot, so LPCOMP was off and never reported the drop).
        uint8_t usable = usable_slot_count();
        m_advance_budget = (usable > 1) ? (uint16_t)usable * 2 : 1;
    } else {
        // Field left. NB: this runs in the NFCT FIELD_LOST interrupt — do NOT change slots here,
        // because tag_emulation_change_slot() may fds_write_sync() and block waiting for FDS events
        // that cannot run from the ISR (hang → watchdog reset). Defer the restore to the main loop.
        m_selected = false;
        if (tag_emulation_get_slot() != m_saved_slot) m_restore_pending = true;
    }
}

void nfc_poll_notify_selected(void) {
    // The reader completed anticollision on the current slot: freeze here so the transaction
    // can finish, instead of cycling the slot out from under it. Reset on field loss.
    if (settings_get_polling_enable() && m_field_present) m_selected = true;
}

void nfc_poll_tick(uint32_t now_ms) {
    // Deferred slot restore from the FIELD_LOST ISR (safe here in the main loop; may fds_write_sync).
    if (m_restore_pending) {
        m_restore_pending = false;
        if (tag_emulation_get_slot() != m_saved_slot) tag_emulation_change_slot(m_saved_slot, true);
    }
    if (!settings_get_polling_enable()) {
        // Disabled: forget any field state so a later re-enable starts clean (no phantom cycling
        // if a field had been present when it was turned off).
        m_field_present = false;
        m_selected = false;
        return;
    }
    if (!m_field_present || m_selected) return;
    uint16_t interval = settings_get_polling_interval();
    if (m_last_switch != 0 && (now_ms - m_last_switch) < interval) return;
    // Give up after the budget runs out without a selection: a field-loss notification was likely
    // missed (sense of the lost field not armed on the current slot). Restore the ORIGINAL slot and
    // park there — this re-arms that slot's own sense, so a later real field change is seen again.
    if (m_advance_budget == 0) {
        if (tag_emulation_get_slot() != m_saved_slot) tag_emulation_change_slot(m_saved_slot, true);
        m_cursor = m_saved_slot;
        // FORGET the field session (not just m_selected): parking re-armed the saved slot's own
        // sense, which now owns field-loss detection. Leaving m_field_present=true would make a
        // later nfc_poll_on_field(true) a no-op (present==m_field_present → early return), so the
        // next field would be ignored and polling would never restart. Clearing it lets a fresh
        // field event start a new cycle cleanly.
        m_field_present = false;
        m_selected = false;
        m_last_switch = now_ms;
        NRF_LOG_INFO("poll: budget exhausted, parked on saved slot %d", m_saved_slot);
        return;
    }
    m_advance_budget--;
    uint8_t next = next_enabled_slot(m_cursor);
    if (next != m_cursor) {
        m_cursor = next;
        tag_emulation_change_slot(m_cursor, true);
        NRF_LOG_INFO("poll: advance to slot %d", m_cursor);
    }
    m_last_switch = now_ms;
}
