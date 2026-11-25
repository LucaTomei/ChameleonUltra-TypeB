/* SPDX-License-Identifier: GPL-3.0-only */
/* ST25TB / SRI model, 512-bit (16 blocks) and 4K (128 blocks).
 * RF writes affect RAM only (not persisted to flash). All data blocks are writable so a reader that
 * updates a block sees its new value on read-back, like the real card; the low blocks keep the
 * 512-bit lock/OTP + count-down-counter semantics, while a 4K's high blocks (16..127) are plain
 * writable EEPROM (the 4K's own per-block lock bits are not modelled — a fidelity limit, not a read
 * or clone problem). The inventory Chip_ID (image tail byte) is the anticollision slot id, not the
 * card type: a reader derives 512 vs 4K from the UID product code (image[5]), so Chip_ID 0 is fine. */
#include "model.h"
#include "iso15693/frame.h"
#include <string.h>

typedef enum { READY, INVENTORY, SELECTED, DESELECTED, DEACTIVATED } state_t;
static uint8_t image[ST25TB_IMAGE_MAX];
static unsigned num_blocks;      /* 16 (512-bit) or 128 (4K) */
static unsigned img_size;        /* 77 or 525 */
static state_t state;
static uint32_t active_locks;
static bool loaded;

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
    active_locks = read32(image + sys_off()) >> 16;
    loaded = true;
    return true;
}

static void write_block(unsigned address, const uint8_t *data) {
    uint32_t value = read32(data);
    if (address == 255) {
        /* System block: factory-reserved bits 15..8 stay unchanged; OTP and locks only clear. */
        value = read32(image + sys_off()) & (value | 0x0000ff00u);
        for (unsigned b = 0; b < 4; b++) image[sys_off() + b] = value >> (8 * b);
        return;
    }
    if (address >= num_blocks) return;                 /* out of range for this chip */
    uint8_t *block = image + 8 + 4 * address;
    /* Count-down counters (blocks 5 and 6) accept a write only if it decreases the value. */
    if (address == 5 || address == 6) {
        if (value < read32(block)) memcpy(block, data, 4);
        return;
    }
    /* A locked low block is write-protected (ST25TB OTP/lock area). Every other data block —
     * including all of a 4K's high blocks (16..127) — is writable, so a reader that updates a
     * block (e.g. a counter/balance) sees its value on read-back, like the real card. */
    if (address < 16 && (active_locks & (1u << address))) return;
    memcpy(block, data, 4);
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
            active_locks = read32(image + sys_off()) >> 16;
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
