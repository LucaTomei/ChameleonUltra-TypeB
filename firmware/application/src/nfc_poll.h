#ifndef NFC_POLL_H
#define NFC_POLL_H

#include <stdint.h>
#include <stdbool.h>

// Field-triggered slot polling (16-slot firmware, Mazha0309-style).
//
// Quando un lettore porta il campo, il device cicla gli slot ABILITATI ogni
// `settings_get_polling_interval()` ms, saltando gli slot vuoti, e ripristina lo slot
// attivo originale quando il campo sparisce. Senza campo resta fermo.
//
// INTEGRAZIONE (DA VALIDARE SU DEVICE):
//  1) nfc_poll_init() all'avvio (dopo il load delle impostazioni e degli slot).
//  2) nfc_poll_on_field(bool present): chiamare dall'evento/sense del campo lettore
//     (dove il firmware rileva l'arrivo/uscita del campo HF/LF).
//  3) nfc_poll_tick(uint32_t now_ms): chiamare periodicamente dal main loop/timer;
//     fa avanzare lo slot quando è trascorso l'intervallo e il campo è presente.
// Tutte le funzioni sono no-op se settings_get_polling_enable() è false.
void nfc_poll_init(void);
void nfc_poll_on_field(bool present);
void nfc_poll_tick(uint32_t now_ms);

#endif // NFC_POLL_H
