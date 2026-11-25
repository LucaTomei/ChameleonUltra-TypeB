/* Minimal HF sniff scratch buffer for the ST25TB emulation (see rf_workspace.h). */
#include "rf_workspace.h"

static uint32_t m_hf_sniff[RF_HF_SNIFF_BYTES / sizeof(uint32_t)];

void *rf_hf_sniff_workspace(void) { return m_hf_sniff; }
