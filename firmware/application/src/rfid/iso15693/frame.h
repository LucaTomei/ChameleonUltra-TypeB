#pragma once
#include <stdint.h>
#include <stdbool.h>

#define ISO15693_CRC_SIZE 2u

/* ISO15693 / ST25TB (SRIx) CRC-16: reflected poly 0x8408, init 0xFFFF, final one's complement. */
uint16_t iso15693_crc16(const uint8_t *data, uint16_t len);
void iso15693_append_crc(uint8_t *data, uint16_t len);      /* writes 2 CRC bytes (LSB,MSB) at data[len] */
bool iso15693_check_crc(const uint8_t *data, uint16_t len); /* len includes the trailing 2 CRC bytes */
