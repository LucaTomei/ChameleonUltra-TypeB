# Crediti

Questo firmware è un fork di
[RfidResearchGroup/ChameleonUltra](https://github.com/RfidResearchGroup/ChameleonUltra).

Spunti presi da altri progetti e riadattati qui:

- **Polling a 16 slot su evento campo** (`application/src/nfc_poll.c`, cablato in
  `app_main.c` e `rfid/nfctag/hf/nfc_14a.c`): approccio ispirato al lavoro di
  **Mazha0309** sul ciclo degli slot abilitati quando un lettore porta il campo.
  Implementazione riscritta per questo fork. DA VALIDARE su hardware.

- **Emulazione ST25TB / SRIx (NFC-B)** (`rfid/st25tb/`, `rfid/nfctag/hf/nfc_st25tb*.c`,
  `utils/rf_timeslot.*`): il sottosistema RF è **vendorizzato da
  [SecLabz/ChameleonUnleashed](https://github.com/SecLabz/ChameleonUnleashed)** (GPL-3.0),
  che implementa l'emulazione Type B sull'nRF52840: NFCT disabilitato, risposta **BPSK a
  847,5 kHz** bit-bangata su `HF_ANT_SEL` (timing a ciclo di CPU via DWT/TIMER4), ricezione
  del campo lettore campionando l'**ADC del RC522** via SPIM3+PPI+TIMER3 in DMA, dentro un
  **radio timeslot** della SoftDevice (`rf_timeslot`). File vendorizzati: `st25tb/model.*`,
  `st25tb/receiver.*`, `st25tb/repair_data.h`, `nfctag/hf/nfc_st25tb_radio.c`,
  `nfctag/hf/nfc_st25tb.*`, `utils/rf_timeslot.*`. Adattati alla nostra fork: shim minimi
  `rfid/reader/rf_workspace.*` (buffer DMA) e `rfid/iso15693/frame.*` (CRC-16). Il livello RF
  resta sperimentale e va tarato su hardware. DA VALIDARE col Flipper.

- **(storico)** La tecnica BPSK/`HF_ANT_SEL` era stata studiata dal fork Tonie; l'implementazione
  completa qui usata è quella di SecLabz (sopra).
