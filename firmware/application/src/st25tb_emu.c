#include "st25tb_emu.h"
#include <string.h>

#define NRF_LOG_MODULE_NAME st25tb_emu
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

// ---- Stored emulated dump ----
typedef enum { ST_DESELECTED = 0, ST_READY, ST_SELECTED } st25tb_state_t;

static st25tb_state_t m_state = ST_DESELECTED;
static bool    m_loaded = false;
static uint8_t m_uid[ST25TB_EMU_UID_SIZE];
static uint8_t m_system[ST25TB_EMU_BLOCK_SIZE];
static uint8_t m_blocks[ST25TB_EMU_MAX_BLOCKS][ST25TB_EMU_BLOCK_SIZE];
static uint8_t m_nblocks = 0;
static uint8_t m_chip_id = 0;

// ISO14443-B CRC (CRC-16/X-25: poly 0x8408 reflected, init 0xFFFF, final XOR 0xFFFF),
// trasmesso low byte first. Identico al CRC usato dal reader (St25tb.crcB) e a ISO13239.
static void crc_b(const uint8_t *data, uint16_t len, uint8_t *out2) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) crc = (crc & 1) ? ((crc >> 1) ^ 0x8408) : (crc >> 1);
    }
    crc = ~crc;
    out2[0] = (uint8_t)(crc & 0xFF);
    out2[1] = (uint8_t)((crc >> 8) & 0xFF);
}

static const uint8_t *block_ptr(uint8_t addr) {
    if (addr == 0xFF) return m_system;
    if (addr < m_nblocks) return m_blocks[addr];
    static const uint8_t zero[ST25TB_EMU_BLOCK_SIZE] = {0};
    return zero;
}

// OTP (0..4), contatori (5..6) e system (0xFF) sono non riscrivibili su tag reale.
static bool block_protected(uint8_t addr) {
    return addr <= 6 || addr == 0xFF;
}

void st25tb_emu_load(const uint8_t *uid, const uint8_t *system, const uint8_t *blocks, uint8_t nblocks) {
    memcpy(m_uid, uid, ST25TB_EMU_UID_SIZE);
    memcpy(m_system, system, ST25TB_EMU_BLOCK_SIZE);
    if (nblocks > ST25TB_EMU_MAX_BLOCKS) nblocks = ST25TB_EMU_MAX_BLOCKS;
    m_nblocks = nblocks;
    memcpy(m_blocks, blocks, (size_t)nblocks * ST25TB_EMU_BLOCK_SIZE);
    // chip_id deterministico derivato dall'UID (deve solo essere coerente tra INITIATE e SELECT).
    m_chip_id = (uint8_t)(m_uid[0] ^ m_uid[7]);
    m_state = ST_DESELECTED;
    m_loaded = true;
    NRF_LOG_INFO("st25tb_emu: dump caricato (%d blocchi, chip_id %02X)", nblocks, m_chip_id);
}

bool st25tb_emu_ready(void) { return m_loaded; }

void st25tb_emu_reset(void) { m_state = ST_DESELECTED; }

bool st25tb_emu_handle(const uint8_t *cmd, uint16_t clen, uint8_t *resp, uint16_t *rlen) {
    *rlen = 0;
    if (!m_loaded || clen == 0) return false;

    switch (cmd[0]) {
        case 0x06: // INITIATE (06 00) -> chip_id
            if (clen >= 2 && cmd[1] == 0x00) {
                m_state = ST_READY;
                resp[0] = m_chip_id;
                crc_b(resp, 1, &resp[1]);
                *rlen = 3;
                return true;
            }
            return false;

        case 0x0E: // SELECT(chip_id) -> chip_id
            if (clen >= 2 && cmd[1] == m_chip_id) {
                m_state = ST_SELECTED;
                resp[0] = m_chip_id;
                crc_b(resp, 1, &resp[1]);
                *rlen = 3;
                return true;
            }
            return false;

        case 0x0B: // GET_UID -> 8 byte UID
            memcpy(resp, m_uid, ST25TB_EMU_UID_SIZE);
            crc_b(resp, ST25TB_EMU_UID_SIZE, &resp[ST25TB_EMU_UID_SIZE]);
            *rlen = ST25TB_EMU_UID_SIZE + 2;
            return true;

        case 0x08: // READ_BLOCK(addr) -> 4 byte
            if (clen >= 2) {
                memcpy(resp, block_ptr(cmd[1]), ST25TB_EMU_BLOCK_SIZE);
                crc_b(resp, ST25TB_EMU_BLOCK_SIZE, &resp[ST25TB_EMU_BLOCK_SIZE]);
                *rlen = ST25TB_EMU_BLOCK_SIZE + 2;
                return true;
            }
            return false;

        case 0x09: // WRITE_BLOCK(addr, 4 byte) -> nessuna risposta
            if (clen >= 6) {
                uint8_t addr = cmd[1];
                if (!block_protected(addr) && addr < m_nblocks) {
                    memcpy(m_blocks[addr], &cmd[2], ST25TB_EMU_BLOCK_SIZE);
                }
            }
            return false;

        case 0x0C: // RESET_TO_INVENTORY
            m_state = ST_READY;
            return false;

        case 0x0F: // COMPLETION (deseleziona)
            m_state = ST_DESELECTED;
            return false;

        default:
            return false;
    }
}

// --- Livello RF: SPERIMENTALE, DA VALIDARE SU DEVICE ---------------------------------------
//
// Qui va il loop di emulazione NFC-B: finché c'è campo del lettore,
//   1) demodulare il comando ASK (~10%) del lettore,
//   2) validare il CRC-B,
//   3) chiamare st25tb_emu_handle() per costruire la risposta,
//   4) rispondere con una sottoportante BPSK (~847,5 kHz) in load-modulation via l'antenna HF.
// L'nRF52840 non ha un controller NFC-B: va bit-bangato in software (riferimento: la tecnica
// HF_ANT_SEL del fork Tonie per NFC-V, da adattare a 847,5 kHz BPSK e alle temporizzazioni
// ISO14443-B). Serve bring-up con oscilloscopio. Finché non è validato, l'entry sotto è uno
// scaffold: non attivare questa modalità in produzione.
void st25tb_emu_run(void) {
    NRF_LOG_WARNING("st25tb_emu_run: RF layer sperimentale NON implementato — da validare su device");
    // TODO (device): field-detect -> demod ASK -> st25tb_emu_handle() -> risposta BPSK.
}
