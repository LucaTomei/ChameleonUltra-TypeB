/* SPDX-License-Identifier: GPL-3.0-only */
/* ISO/IEC 15693 (NFC-V) tag emulation model. See hf15_model.h for the image layout. */
#include "hf15_model.h"
#include "frame.h"
#include <string.h>

/* Request flag bits (ISO15693-3 §7.3.1); bits 4..6 change meaning when the Inventory flag is set. */
#define REQ_INVENTORY   0x04u
#define REQ_SELECT      0x10u   /* non-inventory */
#define REQ_ADDRESS     0x20u   /* non-inventory */
#define REQ_OPTION      0x40u
#define REQ_INV_AFI     0x10u   /* inventory: AFI byte present */
#define REQ_INV_1SLOT   0x20u   /* inventory: 1 slot (clear = 16 slots) */

#define ERR_NOT_SUPPORTED   0x01u
#define ERR_FORMAT          0x02u
#define ERR_UNKNOWN         0x0Fu
#define ERR_BLOCK_NA        0x10u
#define ERR_ALREADY_LOCKED  0x11u
#define ERR_LOCKED          0x12u

#define NXP_MFG             0x04u

enum { ST_READY, ST_QUIET, ST_SELECTED };

static uint8_t  image[ISO15693_IMAGE_MAX];
static unsigned img_size, block_size, block_count;
static bool     loaded;
static int      state;
static int      pending_slot = -1;      /* 16-slot inventory: EOF-only markers still to wait for */

static unsigned off_blocks(void) { return ISO15693_HEADER_SIZE; }
static unsigned off_security(void) { return ISO15693_HEADER_SIZE + block_count * block_size; }
static unsigned off_nxp(void) { return off_security() + block_count; }
static bool has_nxp(void) { return (image[15] & ISO15693_FLAG_NXP) != 0; }

unsigned iso15693_model_image_size_for(const uint8_t *h, unsigned available) {
    if (!h || available < ISO15693_HEADER_SIZE || h[0] != ISO15693_IMAGE_VERSION || h[1] != 0xE0) return 0;
    unsigned bs = h[12], bc = (unsigned)h[13] | ((unsigned)h[14] << 8);
    if (bs < 1u || bs > ISO15693_MAX_BLOCK_SZ || bc < 1u || bc > ISO15693_MAX_BLOCKS) return 0;
    if (h[15] & ~ISO15693_FLAG_NXP) return 0;                 /* unknown flag bits: refuse */
    unsigned size = ISO15693_HEADER_SIZE + bc * bs + bc + ((h[15] & ISO15693_FLAG_NXP) ? ISO15693_NXP_EXT_SIZE : 0u);
    return size <= available && size <= ISO15693_IMAGE_MAX ? size : 0u;
}

unsigned iso15693_model_build_image(uint8_t *out, unsigned cap, const uint8_t uid[8], uint8_t dsfid, uint8_t afi,
                                    uint8_t ic_ref, unsigned bs, unsigned bc, const uint8_t *blocks,
                                    const uint8_t *security) {
    if (!out || !uid || uid[0] != 0xE0 || bs < 1u || bs > ISO15693_MAX_BLOCK_SZ || bc < 1u || bc > ISO15693_MAX_BLOCKS)
        return 0;
    unsigned size = ISO15693_HEADER_SIZE + bc * bs + bc;
    if (size > cap) return 0;
    memset(out, 0, size);
    out[0] = ISO15693_IMAGE_VERSION;
    memcpy(out + 1, uid, 8);
    out[9] = dsfid;
    out[10] = afi;
    out[11] = ic_ref;
    out[12] = (uint8_t)bs;
    out[13] = (uint8_t)(bc & 0xFFu);
    out[14] = (uint8_t)(bc >> 8);
    out[15] = 0;
    if (blocks) memcpy(out + ISO15693_HEADER_SIZE, blocks, bc * bs);
    if (security) memcpy(out + ISO15693_HEADER_SIZE + bc * bs, security, bc);
    return size;
}

unsigned iso15693_model_factory_image(uint8_t *out, unsigned cap) {
    static const uint8_t uid[8] = {0xE0, 0x07, 0x00, 0x00, 0xC0, 0xFF, 0xEE, 0x01};
    return iso15693_model_build_image(out, cap, uid, 0x00, 0x00, 0x00, 4, 28, NULL, NULL);
}

bool iso15693_model_loaded(void) { return loaded; }
unsigned iso15693_model_image_size(void) { return loaded ? img_size : 0u; }
void iso15693_model_unload(void) { loaded = false; state = ST_READY; pending_slot = -1; }

void iso15693_model_field_lost(void) {
    state = ST_READY;                       /* power loss: the VICC restarts in Ready */
    pending_slot = -1;
}

bool iso15693_model_save(uint8_t *out) {
    if (!loaded) return false;
    memcpy(out, image, img_size);
    return true;
}

bool iso15693_model_load(const uint8_t *data, unsigned image_bytes) {
    unsigned size = iso15693_model_image_size_for(data, image_bytes);
    if (!size) return false;
    memcpy(image, data, size);
    img_size = size;
    block_size = image[12];
    block_count = (unsigned)image[13] | ((unsigned)image[14] << 8);
    state = ST_READY;
    pending_slot = -1;
    loaded = true;
    return true;
}

/* UID helpers: image holds it MSB-first (image[1] = 0xE0), the air sends the least significant byte first. */
static uint8_t uid_wire(unsigned i) { return image[1 + 7 - i]; }       /* i = 0: first byte on the air */
static unsigned uid_bit(unsigned b) { return (uid_wire(b >> 3) >> (b & 7u)) & 1u; }

static unsigned put_uid(uint8_t *r, unsigned n) {
    for (unsigned i = 0; i < 8u; i++) r[n++] = uid_wire(i);
    return n;
}

static unsigned error_reply(uint8_t *r, uint8_t code) {
    r[0] = 0x01;
    r[1] = code;
    iso15693_append_crc(r, 2);
    return 4;
}

static unsigned inventory_reply(uint8_t *r) {
    unsigned n = 0;
    r[n++] = 0x00;
    r[n++] = image[9];                      /* DSFID */
    n = put_uid(r, n);
    iso15693_append_crc(r, (uint16_t)n);
    return n + 2;
}

/* Inventory parameters: [AFI if flagged] mask_len (bits) mask[ceil(mask_len/8)].
 * Returns -1 to stay silent, 0 to answer now, or the slot number (1..15) to answer later. */
static int inventory_match(uint8_t flags, const uint8_t *p, unsigned prem) {
    unsigned i = 0;
    if (flags & REQ_INV_AFI) {
        if (prem < 1u) return -1;
        uint8_t afi = p[i++];
        if (afi != 0u && afi != image[10]) return -1;   /* AFI 0 = all families */
    }
    if (prem < i + 1u) return -1;
    unsigned mask_len = p[i++];
    bool one_slot = (flags & REQ_INV_1SLOT) != 0;
    if (mask_len > (one_slot ? 64u : 60u)) return -1;
    unsigned mask_bytes = (mask_len + 7u) >> 3;
    if (prem < i + mask_bytes) return -1;
    for (unsigned b = 0; b < mask_len; b++)
        if (uid_bit(b) != ((p[i + (b >> 3)] >> (b & 7u)) & 1u)) return -1;
    if (one_slot) return 0;
    unsigned slot = 0;
    for (unsigned b = 0; b < 4u; b++) slot |= uid_bit(mask_len + b) << b;
    return (int)slot;
}

unsigned iso15693_model_slot_marker(uint8_t reply[ISO15693_RESPONSE_MAX]) {
    if (!loaded || pending_slot <= 0) return 0;
    if (--pending_slot > 0) return 0;
    pending_slot = -1;
    return inventory_reply(reply);
}

unsigned iso15693_model_reply(const uint8_t *f, unsigned length, uint8_t reply[ISO15693_RESPONSE_MAX]) {
    if (!loaded || length < 4u || length > ISO15693_REQUEST_MAX || !iso15693_check_crc(f, (uint16_t)length))
        return 0;
    pending_slot = -1;                      /* any new request ends a pending 16-slot round */
    unsigned plen = length - 2u;
    uint8_t flags = f[0], cmd = f[1];
    const uint8_t *p = f + 2;
    unsigned prem = plen - 2u;
    bool inventory = (flags & REQ_INVENTORY) != 0;
    bool option = (flags & REQ_OPTION) != 0;

    if (inventory) {
        if (cmd != 0x01u || state == ST_QUIET) return 0;
        int slot = inventory_match(flags, p, prem);
        if (slot < 0) return 0;
        if (slot > 0) {
            pending_slot = slot;
            return 0;
        }
        return inventory_reply(reply);
    }

    /* Custom/proprietary commands carry the IC manufacturer code BEFORE the optional UID. */
    if (cmd >= 0xA0u) {
        if (prem < 1u || p[0] != NXP_MFG || image[2] != NXP_MFG || !has_nxp()) return 0;
        p++;
        prem--;
    }
    bool addressed = false;
    if (flags & REQ_ADDRESS) {
        if (prem < 8u) return 0;
        for (unsigned i = 0; i < 8u; i++)
            if (p[i] != uid_wire(i)) {
                if (cmd == 0x25u && state == ST_SELECTED) state = ST_READY;   /* Select for another tag */
                return 0;
            }
        addressed = true;
        p += 8;
        prem -= 8;
    }
    if (flags & REQ_SELECT) {
        if (state != ST_SELECTED) return 0;
    } else if (!addressed && state == ST_QUIET) {
        return 0;                           /* a quiet VICC answers addressed requests only */
    }

    unsigned n = 0;
    switch (cmd) {
    case 0x02:                              /* Stay Quiet: addressed, never answered */
        if (addressed) state = ST_QUIET;
        return 0;
    case 0x25:                              /* Select */
        if (!addressed) return 0;
        state = ST_SELECTED;
        reply[n++] = 0x00;
        break;
    case 0x26:                              /* Reset To Ready */
        state = ST_READY;
        reply[n++] = 0x00;
        break;
    case 0x20:                              /* Read Single Block */
    case 0x23: {                            /* Read Multiple Blocks */
        if (prem < (cmd == 0x23u ? 2u : 1u)) return error_reply(reply, ERR_FORMAT);
        unsigned first = p[0], count = cmd == 0x23u ? (unsigned)p[1] + 1u : 1u;
        if (first + count > block_count) return error_reply(reply, ERR_BLOCK_NA);
        if (1u + count * (block_size + (option ? 1u : 0u)) + 2u > ISO15693_RESPONSE_MAX)
            return error_reply(reply, ERR_UNKNOWN);
        reply[n++] = 0x00;
        for (unsigned b = first; b < first + count; b++) {
            if (option) reply[n++] = image[off_security() + b];
            memcpy(reply + n, image + off_blocks() + b * block_size, block_size);
            n += block_size;
        }
        break;
    }
    case 0x21: {                            /* Write Single Block */
        if (prem < 1u + block_size) return error_reply(reply, ERR_FORMAT);
        unsigned b = p[0];
        if (b >= block_count) return error_reply(reply, ERR_BLOCK_NA);
        if (image[off_security() + b] & 0x01u) return error_reply(reply, ERR_LOCKED);
        memcpy(image + off_blocks() + b * block_size, p + 1, block_size);
        reply[n++] = 0x00;
        break;
    }
    case 0x22: {                            /* Lock Block */
        if (prem < 1u) return error_reply(reply, ERR_FORMAT);
        unsigned b = p[0];
        if (b >= block_count) return error_reply(reply, ERR_BLOCK_NA);
        if (image[off_security() + b] & 0x01u) return error_reply(reply, ERR_ALREADY_LOCKED);
        image[off_security() + b] |= 0x01u;
        reply[n++] = 0x00;
        break;
    }
    case 0x2B:                              /* Get System Info */
        reply[n++] = 0x00;
        reply[n++] = 0x0F;                  /* DSFID | AFI | memory size | IC reference */
        n = put_uid(reply, n);
        reply[n++] = image[9];
        reply[n++] = image[10];
        reply[n++] = (uint8_t)(block_count - 1u);
        reply[n++] = (uint8_t)((block_size - 1u) & 0x1Fu);
        reply[n++] = image[11];
        break;
    case 0x2C: {                            /* Get Multiple Block Security Status */
        if (prem < 2u) return error_reply(reply, ERR_FORMAT);
        unsigned first = p[0], count = (unsigned)p[1] + 1u;
        if (first + count > block_count) return error_reply(reply, ERR_BLOCK_NA);
        if (1u + count + 2u > ISO15693_RESPONSE_MAX) return error_reply(reply, ERR_UNKNOWN);
        reply[n++] = 0x00;
        for (unsigned b = first; b < first + count; b++) reply[n++] = image[off_security() + b];
        break;
    }
    case 0xAB:                              /* NXP Get NXP System Info */
        reply[n++] = 0x00;
        memcpy(reply + n, image + off_nxp() + ISO15693_NXP_SIG_SIZE, ISO15693_NXP_INFO_SIZE);
        n += ISO15693_NXP_INFO_SIZE;
        break;
    case 0xBD:                              /* NXP Read Signature */
        reply[n++] = 0x00;
        memcpy(reply + n, image + off_nxp(), ISO15693_NXP_SIG_SIZE);
        n += ISO15693_NXP_SIG_SIZE;
        break;
    default:
        return error_reply(reply, ERR_NOT_SUPPORTED);
    }
    iso15693_append_crc(reply, (uint16_t)n);
    return n + 2u;
}
