#ifndef ST25TB_EMU_H
#define ST25TB_EMU_H

#include <stdint.h>
#include <stdbool.h>

// ST25TB / SRIx (ISO14443-B) TAG emulation — SPERIMENTALE.
//
// NB: la LETTURA/scrittura ST25TB resta nel percorso esistente (HF14B_BITBANG lato reader),
// NON toccato. Questo modulo aggiunge solo l'EMULAZIONE (il Chameleon fa da TAG e risponde a
// un lettore). L'nRF52840 non ha un controller NFC-B nativo: la risposta è una sottoportante
// BPSK a ~847,5 kHz in load-modulation, generata in software (riferimento: tecnica HF_ANT_SEL
// di SecLabz/ChameleonUltra-Tonie per NFC-V). DA VALIDARE SU DEVICE.
//
// Due livelli:
//  - PROTOCOLLO (st25tb_emu_load / st25tb_emu_handle): completo e deterministico, testabile.
//  - RF (st25tb_emu_run): scaffold sperimentale (demod ASK del reader + risposta BPSK) da
//    mettere a punto con oscilloscopio su hardware reale.

#define ST25TB_EMU_MAX_BLOCKS 128
#define ST25TB_EMU_BLOCK_SIZE 4
#define ST25TB_EMU_UID_SIZE   8

// Carica il dump da emulare (uid 8B, system 4B, blocks nblocks×4B). chip_id derivato dall'UID.
void st25tb_emu_load(const uint8_t *uid, const uint8_t *system, const uint8_t *blocks, uint8_t nblocks);

// È stato caricato un dump valido?
bool st25tb_emu_ready(void);

// Macchina a stati del protocollo ST25TB/SRIx (DETERMINISTICA, testabile).
// Dato un comando del lettore (senza CRC, già validato), costruisce la risposta (con CRC-B).
// Ritorna true se va inviata una risposta; false se il comando non prevede risposta.
bool st25tb_emu_handle(const uint8_t *cmd, uint16_t clen, uint8_t *resp, uint16_t *rlen);

// Reset della sessione (stato deselezionato).
void st25tb_emu_reset(void);

// Entry della modalità emulazione (SPERIMENTALE, RF da validare). Resta attiva finché c'è campo.
void st25tb_emu_run(void);

#endif // ST25TB_EMU_H
