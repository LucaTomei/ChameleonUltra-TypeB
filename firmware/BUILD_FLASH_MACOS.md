# Build & flash del firmware su macOS (Apple Silicon)

Procedura verificata su macOS 27 (arm64), Xcode 27, con Chameleon Ultra via USB-C.

## 1. Toolchain ARM

Il firmware richiede l'Arm GNU Toolchain **12.2.rel1** (stessa versione del `Dockerfile`).
La formula brew `arm-none-eabi-gcc` **non va**: è un compilatore senza newlib → errori
`stdint.h: No such file or directory`. Il cask `gcc-arm-embedded` richiede `sudo`.

Soluzione senza sudo — tarball ufficiale Arm, nativo arm64:

```bash
curl -fL -o /tmp/arm12.tar.xz \
  "https://developer.arm.com/-/media/Files/downloads/gnu/12.2.rel1/binrel/arm-gnu-toolchain-12.2.rel1-darwin-arm64-arm-none-eabi.tar.xz"
mkdir -p "$HOME/opt" && tar xJf /tmp/arm12.tar.xz -C "$HOME/opt"
```

Il `Makefile.posix` dell'SDK ha i percorsi hard-coded a `/usr/bin/`. Si sovrascrivono via
env (sono `?=`), senza toccare file tracciati.

## 2. Build

`build.sh` ha lo shebang `#!/bin/env bash` (errato su macOS): lanciarlo con `bash`.

```bash
cd firmware
export GNU_INSTALL_ROOT="$HOME/opt/arm-gnu-toolchain-12.2.rel1-darwin-arm64-arm-none-eabi/bin/"
export GNU_VERSION=12.2.1          # la dir lib è 12.2.1, non 12.2.rel1
export GNU_PREFIX=arm-none-eabi
export CURRENT_DEVICE_TYPE=ultra   # hw_version=0
bash build.sh
```

`mergehex` (nRF Command Line Tools) **non è installato** e lo step fallisce a fine script:
irrilevante per il DFU. I pacchetti servono già pronti:
`objects/ultra-dfu-app.zip` (solo applicazione) e `objects/ultra-dfu-full.zip`.
`mergehex`/`fullimage.hex` servono solo per flash via SWD/J-Link.

## 3. Flash via DFU (USB)

### Il problema di nrfutil su macOS
- `nrfutil device program --firmware objects/ultra-dfu-app.zip --traits nordicDfu`
  (metodo ufficiale) **esce subito con RC 0 senza flashare** su questo macOS: il backend
  non programma un device già nel bootloader e non può triggerare il DFU dall'app mode
  (il CU in app mode ha VID non-Nordic 0x6868).
- `nrfutil nrf5sdk-tools dfu usb-serial` fallisce con
  `TypeError: cannot convert 'NoneType' object to bytearray` in `__get_mtu`.

### Causa reale
Il bootloader (CDC `0x1915:0x521f`) risponde al Secure DFU seriale **solo con DTR
asserito**. nrfutil apre la porta con DTR basso → il GetSerialMTU non riceve risposta.
Verificato: con `dtr=True`, `GetSerialMTU` risponde `60 07 01 03 08` (MTU=2051).

### Soluzione: `resource/tools/dfu_flash_macos.py`
Implementa il Nordic Secure DFU seriale (SLIP) aprendo la porta con `dtr=True`, con
verifica CRC32 a ogni oggetto (se il protocollo fosse errato darebbe CRC mismatch, non
un flash corrotto; e il bootloader rifiuta un'app non valida restando in DFU).

```bash
# mette da solo il device in DFU (o usare prima resource/tools/enter_dfu.py)
python3 resource/tools/dfu_flash_macos.py firmware/objects/ultra-dfu-app.zip
```

A fine flash il CRC dell'app stampato deve combaciare con l'"Application CRC" del
log di `build.sh` (settings generate). Il device riavvia in app mode (`0x6868:0x8686`).

Nota: la stringa versione mostrata (`fw_vNNN`) non cambia col nostro build; verificare il
flash dalla presenza dei nuovi comandi, non dalla versione.

## 4. Pulizia
Dopo i build: `rm -rf firmware/objects firmware/application/_build firmware/bootloader/_build`.
Il toolchain in `$HOME/opt` si può tenere per i rebuild o rimuovere per liberare spazio.
