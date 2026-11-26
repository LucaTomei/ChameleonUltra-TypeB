/* SPDX-License-Identifier: GPL-3.0-only */
/* ST25TB / SRI model, 512-bit (16 blocks) and 4K (128 blocks).
 * RF writes affect RAM only (not persisted to flash), but reproduce the real chip's write rules so a
 * reader that updates the tag sees what a genuine chip would do. The two families DIFFER (datasheets
 * ST25TB512-AT / SRI512 and ST25TB04K / SRIX4K), so the rules are per-type:
 *
 *   ST25TB512-AT / SRI512 (16 blocks):
 *     - blocks 0..15 : ordinary rewritable EEPROM, each INDIVIDUALLY lockable via OTP_Lock_Reg
 *                      (block 255) bits b16..b31, ACTIVE LOW: b(16+addr)=0 locks block `addr`
 *                      (b16->0, b17->1, ... b31->15). NOT OTP — an unlocked block takes any value.
 *     - blocks 5,6   : also 32-bit count-down counters (a write must DECREASE) on top of being lockable.
 *
 *   ST25TB04K / SRIX4K (128 blocks):
 *     - blocks 0..4   : OTP — a Write_block can only program bits 1->0, never 0->1;
 *     - blocks 5,6    : 32-bit count-down counters (a write must DECREASE);
 *     - blocks 7..15  : lockable via OTP_Lock_Reg bits b24..b31, ACTIVE LOW; b24 locks blocks 7 AND 8,
 *                       b25->9, b26->10, ... b31->15;
 *     - blocks 16..127: plain writable EEPROM (not lockable).
 *
 * Common: block 255 is the system block / OTP_Lock_Reg; its lock bits are OTP (1->0 only). The real
 * card loads NEW protections only after a valid SELECT, so the active lock mask is LATCHED at load and
 * at each SELECT (`active_locks`), not re-read live mid-session. The inventory Chip_ID (image tail
 * byte) is the anticollision slot id, not the card type: a reader derives 512 vs 4K from the UID
 * product code (image[5]), so Chip_ID 0 is fine.
 *
 * KNOWN LIMIT (DA VALIDARE): the SRIX4K OTP auto-erase/reload mode (re-enabling 0->1 on blocks 0..4
 * after a specific counter sequence) is NOT modelled — OTP blocks stay 1->0 only. Reproducing it
 * wrong would be worse than leaving it out; a reader that relies on it is not supported. */
#include "model.h"
#include "iso15693/frame.h"
#include <string.h>

typedef enum { READY, INVENTORY, SELECTED, DESELECTED, DEACTIVATED } state_t;
static uint8_t image[ST25TB_IMAGE_MAX];
static unsigned num_blocks;      /* 16 (512-bit) or 128 (4K) */
static unsigned img_size;        /* 77 or 525 */
static state_t state;
static bool loaded;
/* OTP_Lock_Reg (system block) latched at load and at each SELECT: the real card applies new
 * protections only after a valid SELECT, so writes consult THIS, not the live system block. */
static uint32_t active_locks;

static uint32_t read32(const uint8_t *p) {
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 |
           (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}
/* Offset of the system block: right after the N data blocks. Chip_ID follows the system block. */
static unsigned sys_off(void) { return 8u + num_blocks * 4u; }
static unsigned chip_off(void) { return sys_off() + 4u; }

bool st25tb_loaded(void) { return loaded; }
unsigned st25tb_image_size(void) { return loaded ? img_size : 0u; }
void st25tb_unload(void) { loaded = false; state = READY; }
void st25tb_field_lost(void) { state = READY; }

bool st25tb_save(uint8_t *data) {
    if (!loaded) return false;
    memcpy(data, image, img_size);
    return true;
}

bool st25tb_load(const uint8_t *data, unsigned image_bytes) {
    /* Accept any ST25TB / SRI chip: the UID always ends (wire order, bytes 7..6) with D0 02 (ST25
     * family + STMicroelectronics IC manufacturer). The block count comes from the image size:
     * 77 bytes = 512-bit (16 blocks), 525 bytes = 4K (128 blocks). The product code (data[5]) is
     * reported from the Chip_ID during anticollision; the generic N-block model serves all variants. */
    if (!data || data[7] != 0xd0 || data[6] != 0x02) return false;
    unsigned nb;
    if (image_bytes == ST25TB_IMAGE_SIZE) nb = 16;
    else if (image_bytes == ST25TB_IMAGE_SIZE_4K) nb = 128;
    else return false;
    num_blocks = nb;
    img_size = image_bytes;
    memcpy(image, data, image_bytes);
    state = READY;
    active_locks = read32(image + sys_off());          /* latch the OTP_Lock_Reg at load */
    loaded = true;
    return true;
}

/* Is `address` write-protected by the lock mask latched at the last SELECT/load? ACTIVE LOW: a 0
 * bit means locked. The bit<->block map differs between the two families (see the header). */
static bool block_locked(unsigned address) {
    if (num_blocks == 16) {
        /* 512-AT / SRI512: every block 0..15 lockable, bit b(16+addr). */
        if (address > 15) return false;
        return !(active_locks & (1u << (16u + address)));
    }
    /* 4K / SRIX4K: only blocks 7..15 lockable; b24 locks 7 AND 8, b25..b31 -> 9..15. */
    if (address < 7 || address > 15) return false;
    unsigned bit = (address <= 8) ? 24u : (24u + (address - 8u));
    return !(active_locks & (1u << bit));
}

static void write_block(unsigned address, const uint8_t *data) {
    uint32_t value = read32(data);
    if (address == 255) {
        /* System block / OTP_Lock_Reg: factory-reserved bits 15..8 stay unchanged; lock bits are OTP
         * (only clear, 1->0). The NEW protections take effect at the next SELECT (active_locks is
         * re-latched there), like the real card — not for the writes that follow in this session. */
        value = read32(image + sys_off()) & (value | 0x0000ff00u);
        for (unsigned b = 0; b < 4; b++) image[sys_off() + b] = value >> (8 * b);
        return;
    }
    if (address >= num_blocks) return;                 /* out of range for this chip */
    uint8_t *block = image + 8 + 4 * address;
    if (block_locked(address)) return;                 /* write-protected (lock latched at SELECT) */
    /* 4K ONLY: blocks 0..4 are OTP (bits may only go 1->0). On the 512-AT the same blocks are ordinary
     * rewritable EEPROM (handled by the plain memcpy below), so this branch is 4K-specific. */
    if (num_blocks == 128 && address <= 4) {
        uint32_t merged = read32(block) & value;
        for (unsigned b = 0; b < 4; b++) block[b] = merged >> (8 * b);
        return;
    }
    /* Blocks 5,6 are 32-bit count-down counters: a write is accepted only if it DECREASES. (On the
     * 512-AT they are also lockable — already refused above if locked.) */
    if (address == 5 || address == 6) {
        if (value < read32(block)) memcpy(block, data, 4);
        return;
    }
    memcpy(block, data, 4);                             /* unlocked EEPROM block */
}

unsigned st25tb_reply(const uint8_t *frame, unsigned length,
                      uint8_t reply[ST25TB_RESPONSE_MAX]) {
    /* CRC-B and ISO15693 share the same CRC parameters and wire order. */
    if (!loaded || length < 3 || length > ST25TB_REQUEST_MAX ||
            !iso15693_check_crc(frame, length)) return 0;
    unsigned n = 0, payload = length - 2;
    uint8_t command = frame[0], chip = image[chip_off()];
    if (payload == 2 && command == 6) {
        if (frame[1] == 0 && (state == READY || state == INVENTORY || state == SELECTED)) {
            state = INVENTORY;
            reply[n++] = chip;
        } else if (frame[1] == 4 && state == INVENTORY && !(chip & 15)) {
            reply[n++] = chip;
        }
    } else if (payload == 1 && (command & 15) == 6 && state == INVENTORY) {
        if ((command >> 4) == (chip & 15)) reply[n++] = chip;
    } else if (payload == 2 && command == 0x0e) {
        if (frame[1] == chip && (state == INVENTORY || state == SELECTED || state == DESELECTED)) {
            state = SELECTED;
            active_locks = read32(image + sys_off());   /* real card loads protections at SELECT */
            reply[n++] = chip;
        } else if (state == SELECTED && frame[1] != chip) {
            state = DESELECTED;
        }
    } else if (state == SELECTED) {
        if (payload == 1 && command == 0x0b) {
            memcpy(reply, image, 8);
            n = 8;
        } else if (payload == 2 && command == 8) {
            unsigned address = frame[1];
            if (address < num_blocks || address == 255) {
                memcpy(reply, image + 8 + (address == 255 ? num_blocks : address) * 4, 4);
                n = 4;
            }
        } else if (payload == 1 && command == 0x0c) {
            state = INVENTORY;
        } else if (payload == 1 && command == 0x0f) {
            state = DEACTIVATED;
        } else if (payload == 6 && command == 9) {
            write_block(frame[1], frame + 2);
        }
    }
    if (!n) return 0;
    iso15693_append_crc(reply, n);
    return n + 2;
}
