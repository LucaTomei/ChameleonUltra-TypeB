#ifndef NFC_ST25TB_H
#define NFC_ST25TB_H

#include "tag_emulation.h"

int nfc_st25tb_data_loadcb(tag_specific_type_t type, tag_data_buffer_t *buffer);
int nfc_st25tb_data_savecb(tag_specific_type_t type, tag_data_buffer_t *buffer);
bool nfc_st25tb_data_factory(uint8_t slot, tag_specific_type_t type);
void nfc_st25tb_sense_switch(bool enable);
bool nfc_st25tb_service(void);
bool nfc_st25tb_set_image(const uint8_t *image, unsigned image_len);   // len 77 (512-bit) or 525 (4K)
bool nfc_st25tb_get_image(uint8_t *image);                             // writes the active slot's image
unsigned nfc_st25tb_image_size(void);                                  // size written by get_image (77/525, 0 if none)
bool nfc_st25tb_radio_poll(bool *field_present, bool active);
void nfc_st25tb_radio_stop(void);
bool nfc_st25tb_radio_responded(void);

#endif
