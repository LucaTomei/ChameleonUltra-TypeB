#include "iso15693/frame.h"

uint16_t iso15693_crc16(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 1u) ? (uint16_t)((crc >> 1) ^ 0x8408) : (uint16_t)(crc >> 1);
    }
    return (uint16_t)~crc;
}

void iso15693_append_crc(uint8_t *data, uint16_t len) {
    uint16_t crc = iso15693_crc16(data, len);
    data[len]     = (uint8_t)(crc & 0xFF);
    data[len + 1] = (uint8_t)((crc >> 8) & 0xFF);
}

bool iso15693_check_crc(const uint8_t *data, uint16_t len) {
    if (len < 2) return false;
    uint16_t crc = iso15693_crc16(data, len - 2);
    return data[len - 2] == (uint8_t)(crc & 0xFF) && data[len - 1] == (uint8_t)((crc >> 8) & 0xFF);
}
