#pragma once
#include <stdbool.h>
#include <stdint.h>

#define ST25TB_IMAGE_SIZE 77u          /* 512-bit: UID(8) + 16 blocks*4 + system(4) + chip(1) */
#define ST25TB_IMAGE_SIZE_4K 525u      /* 4K:      UID(8) + 128 blocks*4 + system(4) + chip(1) */
#define ST25TB_IMAGE_MAX ST25TB_IMAGE_SIZE_4K
#define ST25TB_REQUEST_MAX 8u
#define ST25TB_RESPONSE_MAX 10u

/* Wire UID, N 4-byte blocks (16 for 512-bit, 128 for 4K), system block, inventory Chip_ID.
 * `image_bytes` is 77 (512-bit) or 525 (4K); the model derives the block count from it. */
bool st25tb_load(const uint8_t *image, unsigned image_bytes);
bool st25tb_save(uint8_t *image);          /* writes st25tb_image_size() bytes */
unsigned st25tb_image_size(void);          /* size of the loaded image (77 or 525); 0 if none */
bool st25tb_loaded(void);
void st25tb_unload(void);
void st25tb_field_lost(void);
unsigned st25tb_reply(const uint8_t *frame, unsigned length,
                      uint8_t reply[ST25TB_RESPONSE_MAX]);
