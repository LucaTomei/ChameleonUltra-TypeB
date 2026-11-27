/* SPDX-License-Identifier: GPL-3.0-only */
/* ISO/IEC 15693 (NFC-V) tag EMULATION model — the protocol/state layer, independent of the RF
 * bit-bang (which lives in the hf15 radio, Increment B). Given a decoded reader request frame it
 * produces the tag response bytes (CRC appended), exactly as a real ISO15693 tag would. RF writes
 * affect RAM only (not flash), like the ST25TB model.
 *
 * Supported now (Milestone 1): Inventory (0x01), Stay Quiet (0x02), Get System Info (0x2B),
 * Read Single Block (0x20), Write Single Block (0x21), Select (0x25), Reset To Ready (0x26).
 * Addressed/Select/Inventory request flags are honoured. This is the generic vicinity-card model
 * (ICODE/SLIX-like): fixed block size, N blocks; NXP SLIX custom commands come in a later step.
 *
 * Image layout (what HF15_EMU_SET loads; UID in DISPLAY order, E0 first):
 *   [0..7]  UID (8 bytes, uid[0]=0xE0 ... uid[7]=last serial byte)  — transmitted LSB-first on the air
 *   [8]     DSFID
 *   [9]     AFI
 *   [10]    block size in bytes (1..4)
 *   [11]    block count (1..ISO15693_MAX_BLOCKS)
 *   [12..]  block data (block_count * block_size bytes)
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>

#define ISO15693_UID_SIZE      8u
#define ISO15693_MAX_BLOCKS    256u
#define ISO15693_MAX_BLOCK_SZ  4u
#define ISO15693_HEADER_SIZE   12u   /* UID(8)+DSFID+AFI+blockSize+blockCount */
#define ISO15693_IMAGE_MAX     (ISO15693_HEADER_SIZE + ISO15693_MAX_BLOCKS * ISO15693_MAX_BLOCK_SZ)
#define ISO15693_REQUEST_MAX   64u
#define ISO15693_RESPONSE_MAX  (3u + ISO15693_MAX_BLOCK_SZ + 2u + 32u)

/* Load / unload the emulated image. `image_bytes` must be ISO15693_HEADER_SIZE + count*size. */
bool iso15693_model_load(const uint8_t *data, unsigned image_bytes);
void iso15693_model_unload(void);
bool iso15693_model_loaded(void);
unsigned iso15693_model_image_size(void);
bool iso15693_model_save(uint8_t *out);         /* copies the current image out; false if not loaded */
void iso15693_model_field_lost(void);           /* reader field gone → back to Ready */

/* Handle one decoded reader request (without its CRC validated by the caller? — this validates it).
 * `frame`/`length` include the 2 trailing CRC bytes. Returns the response length (data + 2 CRC) in
 * `reply`, or 0 to stay silent (bad CRC, addressed-to-another, or a command with no response). */
unsigned iso15693_model_reply(const uint8_t *frame, unsigned length,
                              uint8_t reply[ISO15693_RESPONSE_MAX]);
