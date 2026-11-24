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

## NON verificato / NON funzionante
- **La lettura di una carta PCF reale non è ancora riuscita.** Con la carta di test il
  capture produce solo micro-raffiche di 3–5 fronti separate da gap saturati (0xff), mai
  128 bit → zero blocchi. L'inviluppo ADC (`3009`) non mostra modulazione periodica.
  Non è chiaro se la carta non fosse una PCF che streama o se il metodo di capture sia
  inadeguato: serve una PCF7930/7931 **confermata** per distinguere.
- La **scrittura** (`3024`): temporizzazioni gap-based in `lf_pcf7931_data.c`
  (`PCF_INIT_GAP_US`, `PCF_BIT_US`, `PCF_GAP_US`) sono solo la struttura corretta, **mai
  calibrate su device**.

## Difetto di design noto (da sistemare)
`pcf7931_read_raw` cattura i **fronti GPIOTE** (come em410x) e parte senza settle
dell'antenna. Per una modulazione debole come PCF è meglio il percorso **SAADC**
(inviluppo analogico) già presente in `lf_reader_generic.c::raw_read_to_buffer`
(settle 2ms + `bsp_wdt_feed`), con demod host sull'inviluppo invece che sugli intervalli.
Prossimo passo quando si avrà una carta buona: portare `3022`/`3023` su SAADC e riscrivere
il demod host di conseguenza.

## Come testare via USB (senza app)
1. `python3 resource/tools/dfu_flash_macos.py firmware/objects/ultra-dfu-app.zip`
2. Mandare `CHANGE_DEVICE_MODE=1001` con `\x01` (reader), poi `3022`.
   (Client in `software/script`, vedi `BUILD_FLASH_MACOS.md`.)
