# PCF7930/7931 e T55xx raw — stato reale

Onestà prima di tutto: cosa è verificato e cosa no, al 2026-11-24.

## Comandi aggiunti
- `3022 DATA_CMD_PCF7931_RAW_SCAN` — cattura fronti LF grezzi (demod su host).
- `3023 DATA_CMD_T55XX_RAW_SCAN` — idem per T5577 default-read.
- `3024 DATA_CMD_PCF7931_WRITE` — scrittura blocco 128-bit (downlink gap-based).

## Verificato su hardware (Chameleon Ultra, USB)
- Il firmware builda e si flasha (vedi `BUILD_FLASH_MACOS.md`).
- `3022` risponde `LF_TAG_OK` e restituisce intervalli: il comando esiste e cattura.
- Lato app il demod `Pcf7931.decode` (porting proxmark) ha unit test verdi su dati sintetici.

## Design attuale (redesign SAADC — 2026-11-24)
La read NON usa più la cattura a fronti GPIOTE (inadeguata, produceva micro-raffiche).
Ora `3022`/`3023` usano `raw_read_to_buffer_decim` (SAADC, inviluppo analogico, settle 2ms,
`bsp_wdt_feed`). Il frame TX max è 4096 byte e un blocco PCF (~65 ms) a 125 ksps sarebbe
~8192 campioni: si **decima 1 su 4** così 4096 campioni coprono ~131 ms ≈ 2 blocchi
(garantisce ≥1 blocco completo indipendentemente dalla fase), con ~8 campioni per simbolo
32·T0. La demod gira sul host: `Pcf7931.decodeEnvelope` (soglia Schmitt → fronti →
stima del periodo base → riscalatura a 16/32/64 → regole PCF). Validata da unit test su
inviluppi sintetici continui a più risoluzioni (incluso decim-4). T55xx: decim 2.

## NON verificato / NON funzionante
- **La lettura di una carta PCF reale non è ancora confermata.** L'unica carta provata dà
  un inviluppo con span ~5 LSB dopo il settle (modulazione sotto-soglia): nessun fronte
  affidabile, nessun blocco. Non modula in modo usabile sull'antenna del CU (probabile
  non-PCF, o accoppiamento/alimentazione insufficienti). Serve una PCF7930/7931
  **confermata e funzionante** per validare davvero la catena.
- La **scrittura** (`3024`): temporizzazioni gap-based in `lf_pcf7931_data.c`
  (`PCF_INIT_GAP_US`, `PCF_BIT_US`, `PCF_GAP_US`) sono solo la struttura corretta, **mai
  calibrate su device**.

## Come testare via USB (senza app)
1. `python3 resource/tools/dfu_flash_macos.py firmware/objects/ultra-dfu-app.zip`
2. Mandare `CHANGE_DEVICE_MODE=1001` con `\x01` (reader), poi `3022`.
   (Client in `software/script`, vedi `BUILD_FLASH_MACOS.md`.)
