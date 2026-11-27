/* SPDX-License-Identifier: GPL-3.0-only */
/* ISO/IEC 15693 (NFC-V) tag EMULATION model — protocol/state layer, independent of the RF bit-bang
 * (nfc_hf15_radio.c feeds decoded request frames in and transmits the returned reply). RF writes
 * change RAM only; the slot record is persisted by the normal tag-emulation save path.
 *
 * Supported: Inventory (0x01: AFI, mask, 1 or 16 slots), Stay Quiet (0x02), Read Single Block (0x20),
 * Write Single Block (0x21, honours locks), Lock Block (0x22), Read Multiple Blocks (0x23),
 * Select (0x25), Reset To Ready (0x26), Get System Info (0x2B), Get Multiple Block Security (0x2C);
 * NXP custom (manufacturer 0x04, only if the image carries the NXP extension): Get NXP System Info
 * (0xAB), Read Signature (0xBD). Ready / Quiet / Selected states follow ISO15693-3 §9.
 * Known limits (DA VALIDARE su device): the Option flag on writes/locks (TI "wait for EOF" mode) is
 * answered immediately; SLIX passwords/privacy/EAS commands are not modelled.
 *
 * Image layout v1 (what HF15_EMU_SET loads / the slot record stores; UID in DISPLAY order, E0 first):
 *   [0]      format version (1)
 *   [1..8]   UID, uid[1] = 0xE0 ... uid[8] = least significant byte (sent first on the air)
 *   [9]      DSFID      [10] AFI      [11] IC reference
 *   [12]     block size in bytes (1..8)
 *   [13..14] block count, little endian (1..256)
 *   [15]     flags: bit0 = NXP extension present
 *   [16..]   block data (count * size), then block security status (count bytes, 0 = unlocked),
 *            then, if NXP: signature (32) + NXP system info payload (7)
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>

#define ISO15693_IMAGE_VERSION   1u
#define ISO15693_UID_SIZE        8u
#define ISO15693_MAX_BLOCKS      256u
#define ISO15693_MAX_BLOCK_SZ    8u
#define ISO15693_HEADER_SIZE     16u
#define ISO15693_NXP_SIG_SIZE    32u
#define ISO15693_NXP_INFO_SIZE   7u
#define ISO15693_NXP_EXT_SIZE    (ISO15693_NXP_SIG_SIZE + ISO15693_NXP_INFO_SIZE)
#define ISO15693_MAX_DATA        1024u   /* user memory cap (SLIX 112, SLIX2 320, ICODE DNA 252, Tag-it Plus 256) */
#define ISO15693_IMAGE_MAX       (ISO15693_HEADER_SIZE + ISO15693_MAX_DATA + ISO15693_MAX_BLOCKS + ISO15693_NXP_EXT_SIZE)
#define ISO15693_RECORD_MAX      ((ISO15693_IMAGE_MAX + 3u) & ~3u)   /* flash record: image padded to a word */
#define ISO15693_REQUEST_MAX     64u
#define ISO15693_RESPONSE_MAX    160u
#define ISO15693_FLAG_NXP        0x01u

/* Expected image size for a header, or 0 if the header is not a valid v1 header. */
unsigned iso15693_model_image_size_for(const uint8_t *header, unsigned available);
/* Build a v1 image (no NXP extension). `security` may be NULL (all unlocked). Returns its size or 0. */
unsigned iso15693_model_build_image(uint8_t *out, unsigned cap, const uint8_t uid[8], uint8_t dsfid, uint8_t afi,
                                    uint8_t ic_ref, unsigned block_size, unsigned block_count,
                                    const uint8_t *blocks, const uint8_t *security);
/* Default image for a fresh slot: generic (non-NXP, so readers use the plain ISO15693 path) tag,
 * UID E0 07 ..., 28 blocks x 4 bytes of zeros. Returns its size. */
unsigned iso15693_model_factory_image(uint8_t *out, unsigned cap);

bool iso15693_model_load(const uint8_t *data, unsigned image_bytes);
void iso15693_model_unload(void);
bool iso15693_model_loaded(void);
unsigned iso15693_model_image_size(void);
bool iso15693_model_save(uint8_t *out);         /* copies the current image out; false if not loaded */
const uint8_t *iso15693_model_image(void);      /* the live image (NULL if not loaded), for read-back */
void iso15693_model_field_lost(void);           /* reader field gone: state back to Ready */

/* Handle one decoded request (CRC included and verified here). Returns the reply length (CRC
 * included) written to `reply`, or 0 to stay silent. */
unsigned iso15693_model_reply(const uint8_t *frame, unsigned length, uint8_t reply[ISO15693_RESPONSE_MAX]);
/* An EOF-only frame arrived (16-slot inventory slot marker). Returns a reply length or 0. */
unsigned iso15693_model_slot_marker(uint8_t reply[ISO15693_RESPONSE_MAX]);
