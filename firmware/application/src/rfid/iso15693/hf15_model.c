/* SPDX-License-Identifier: GPL-3.0-only */
/* ISO/IEC 15693 (NFC-V) tag emulation model. See model.h for the image layout. Pure protocol logic:
 * no RF here (the hf15 radio feeds decoded request frames in and transmits the returned reply). */
#include "hf15_model.h"
#include "frame.h"
#include <string.h>

/* Request flag bits (ISO15693-3 §7.3.1). Bits 5/6 are reinterpreted when the Inventory flag is set. */
#define REQ_INVENTORY   0x04
#define REQ_SELECT      0x10   /* non-inventory */
#define REQ_ADDRESS     0x20   /* non-inventory: a UID follows the command, LSB-first */
#define REQ_OPTION      0x40

enum { ST_READY, ST_QUIET, ST_SELECTED };

static uint8_t  image[ISO15693_IMAGE_MAX];
static unsigned img_size;
static unsigned block_size, block_count;
static bool     loaded;
static int      state;

bool iso15693_model_loaded(void) { return loaded; }
unsigned iso15693_model_image_size(void) { return loaded ? img_size : 0u; }
void iso15693_model_unload(void) { loaded = false; state = ST_READY; }
void iso15693_model_field_lost(void) { if (state == ST_SELECTED) state = ST_READY; }

bool iso15693_model_save(uint8_t *out) {
    if (!loaded) return false;
    memcpy(out, image, img_size);
    return true;
}

bool iso15693_model_load(const uint8_t *data, unsigned image_bytes) {
    if (!data || image_bytes < ISO15693_HEADER_SIZE) return false;
    unsigned bs = data[10], bc = data[11];
    if (bs < 1 || bs > ISO15693_MAX_BLOCK_SZ || bc < 1 || bc > ISO15693_MAX_BLOCKS) return false;
    if (image_bytes != ISO15693_HEADER_SIZE + bc * bs) return false;
    if (data[0] != 0xE0) return false;                     /* ISO15693 UID MSB is 0xE0 */
    block_size = bs; block_count = bc; img_size = image_bytes;
    memcpy(image, data, image_bytes);
    state = ST_READY; loaded = true;
    return true;
}

/* Append the 8-byte UID to the reply LSB-first (wire order); image holds it MSB-first (uid[0]=0xE0). */
static unsigned put_uid(uint8_t *reply, unsigned n) {
    for (unsigned i = 0; i < ISO15693_UID_SIZE; i++) reply[n++] = image[ISO15693_UID_SIZE - 1 - i];
    return n;
}

unsigned iso15693_model_reply(const uint8_t *f, unsigned length,
                              uint8_t reply[ISO15693_RESPONSE_MAX]) {
    if (!loaded || length < 4 || length > ISO15693_REQUEST_MAX || !iso15693_check_crc(f, length))
        return 0;
    unsigned plen = length - 2;                /* flags + command + params (CRC stripped) */
    if (plen < 2) return 0;
    uint8_t flags = f[0], cmd = f[1];
    const uint8_t *p = f + 2;                  /* parameters after the command byte */
    unsigned prem = plen - 2;
    bool inventory = (flags & REQ_INVENTORY) != 0;
    bool option    = (flags & REQ_OPTION) != 0;

    /* Addressed (non-inventory): the next 8 bytes are our UID, LSB-first. Answer only if it matches. */
    bool addressed = false;
    if (!inventory && (flags & REQ_ADDRESS)) {
        if (prem < ISO15693_UID_SIZE) return 0;
        for (unsigned i = 0; i < ISO15693_UID_SIZE; i++)
            if (p[i] != image[ISO15693_UID_SIZE - 1 - i]) return 0;
        addressed = true; p += ISO15693_UID_SIZE; prem -= ISO15693_UID_SIZE;
    }
    /* Select flag (non-inventory): request is only for the tag already in Selected state. */
    if (!inventory && (flags & REQ_SELECT) && state != ST_SELECTED && !addressed) return 0;

    unsigned n = 0;
    switch (cmd) {
    case 0x01:                                 /* Inventory */
        if (!inventory || state == ST_QUIET) return 0;   /* quiet tags skip inventory */
        reply[n++] = 0x00;                     /* response flags: no error */
        reply[n++] = image[8];                 /* DSFID */
        n = put_uid(reply, n);
        break;
    case 0x02:                                 /* Stay Quiet (addressed, no response) */
        if (addressed) state = ST_QUIET;
        return 0;
    case 0x25:                                 /* Select */
        if (!addressed) return 0;
        state = ST_SELECTED; reply[n++] = 0x00;
        break;
    case 0x26:                                 /* Reset To Ready */
        state = ST_READY; reply[n++] = 0x00;
        break;
    case 0x20: {                               /* Read Single Block */
        if (prem < 1) return 0;
        unsigned blk = p[0];
        if (blk >= block_count) { reply[n++] = 0x01; reply[n++] = 0x03; break; }  /* err: block N/A */
        reply[n++] = 0x00;
        if (option) reply[n++] = 0x00;         /* block security status (not locked) */
        memcpy(reply + n, image + ISO15693_HEADER_SIZE + blk * block_size, block_size);
        n += block_size;
        break;
    }
    case 0x21: {                               /* Write Single Block */
        if (prem < 1u + block_size) return 0;
        unsigned blk = p[0];
        if (blk >= block_count) { reply[n++] = 0x01; reply[n++] = 0x03; break; }
        memcpy(image + ISO15693_HEADER_SIZE + blk * block_size, p + 1, block_size);
        reply[n++] = 0x00;
        break;
    }
    case 0x2B:                                 /* Get System Info */
        reply[n++] = 0x00;
        reply[n++] = 0x0F;                     /* info flags: DSFID | AFI | mem size | IC ref */
        n = put_uid(reply, n);
        reply[n++] = image[8];                 /* DSFID */
        reply[n++] = image[9];                 /* AFI */
        reply[n++] = (uint8_t)(block_count - 1);
        reply[n++] = (uint8_t)(block_size - 1);
        reply[n++] = 0x01;                     /* IC reference */
        break;
    default:
        return 0;                              /* unsupported command → stay silent */
    }
    if (!n) return 0;
    iso15693_append_crc(reply, n);
    return n + 2;
}
