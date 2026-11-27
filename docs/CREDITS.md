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

- **NFC-V / ISO15693 (lettura + emulazione)** (`rfid/iso15693/hf15_codec.*`, `hf15_model.*`,
  `rfid/nfctag/hf/nfc_hf15*.c`, `rfid/reader/hf/hf15_reader.*`, test host `firmware/tests/hf15/`):
  codice scritto per questa fork; il livello hardware dell'emulazione è quello ST25TB qui sopra
  (riusato identico), cambiano protocollo e sottoportante. Riferimenti usati per i parametri
  dell'interfaccia aerea (studiati, non copiati):
  - [Flipper Zero firmware](https://github.com/flipperdevices/flipperzero-firmware) (GPL-3.0):
    `lib/digital_signal/presets/nfc/iso15693_signal.c` (risposta tag: SOF S S S U S, Manchester,
    EOF S U S S S a fc/32), `lib/signal_reader/parsers/iso15693/iso15693_parser.c` (decodifica
    comandi 1-su-4 / 1-su-256, SOF 0x21/0x81, EOF 0x04), `targets/f7/furi_hal/furi_hal_nfc_iso15693.c`
    (poller in OOK 100 %, encode/decode, pattern SOF 0x17 / EOF 0x1D),
    `lib/nfc/protocols/iso15693_3/*` (sequenza di lettura: Inventory 1 slot, Get System Info,
    Read Single Block, Get Multiple Block Security; FDT) e `lib/nfc/protocols/slix/*` (flusso SLIX).
    I test host verificano che le nostre risposte siano accettate da una replica del decoder Flipper
    e che i nostri comandi coincidano con il suo encoder.
  - [RfidResearchGroup/proxmark3](https://github.com/RfidResearchGroup/proxmark3) (GPL-3.0):
    `armsrc/iso15693.c` — ritardo di risposta del tag (311,5 µs dal fronte di fine EOF + lead SOF).
  - ISO/IEC 15693-3: flag di richiesta, comandi, codici di errore, stati Ready/Quiet/Selected.
  - Tecnica di emulazione NFC-V software su `HF_ANT_SEL`: idea dal fork
    [SecLabz/ChameleonUltra-Tonie](https://github.com/SecLabz/ChameleonUltra-Tonie).
  Stato: codec e modello testati su host; timing RF DA VALIDARE su device (Flipper).
