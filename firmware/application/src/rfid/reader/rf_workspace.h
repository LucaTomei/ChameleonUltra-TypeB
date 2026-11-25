#pragma once
#include <stdint.h>

/* Minimal RF scratch buffer for the ST25TB emulation DMA banks. The ST25TB radio needs
 * STB_DMA_BANK_COUNT(3) * STB_DMA_BANK_BYTES(8192) + guards <= RF_HF_SNIFF_BYTES. 25600 fits
 * that and keeps RAM use modest. (SecLabz's full rf_workspace also backs NFC-V/LF; we only
 * vendor the HF sniff buffer the ST25TB path needs.) */
#define RF_HF_SNIFF_BYTES 25600u

void *rf_hf_sniff_workspace(void);
