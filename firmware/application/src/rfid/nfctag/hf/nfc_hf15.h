#ifndef NFC_HF15_H
#define NFC_HF15_H

/* ISO15693 (NFC-V) tag emulation: slot glue (nfc_hf15.c) + software radio (nfc_hf15_radio.c).
 * Ultra only (needs the RC522 / HF antenna switch); on the Lite every entry point is a safe no-op.
 * DA VALIDARE su device: the protocol/codec layer is host-tested, the RF timing is not yet. */

#include "tag_emulation.h"

int nfc_hf15_data_loadcb(tag_specific_type_t type, tag_data_buffer_t *buffer);
int nfc_hf15_data_savecb(tag_specific_type_t type, tag_data_buffer_t *buffer);
bool nfc_hf15_data_factory(uint8_t slot, tag_specific_type_t type);
void nfc_hf15_sense_switch(bool enable);
bool nfc_hf15_service(void);
bool nfc_hf15_set_image(const uint8_t *image, unsigned image_len);   // v1 image (see hf15_model.h)
const uint8_t *nfc_hf15_image_view(unsigned *size);                  // active NFC-V slot's live image, or NULL

bool nfc_hf15_radio_poll(bool *field_present, bool active);
void nfc_hf15_radio_stop(void);
bool nfc_hf15_radio_responded(void);
/* Emulation diagnostics (decoder + radio counters, big endian), for tuning on hardware. */
unsigned nfc_hf15_radio_diag(uint8_t *out, unsigned cap);

#endif
