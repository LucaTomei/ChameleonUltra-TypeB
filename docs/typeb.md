# ISO14443-B on the Chameleon Ultra

Experimental support for ISO14443-B tags on the Chameleon Ultra reader, currently able to fully
read **ST25TB / SRIX** tags (tested with a ST25TB04K: UID + 128 blocks + system block).

## Why it is not trivial

The Ultra uses a **Ci522** reader chip (MFRC522-compatible). Its register map clearly derives from
an A/B capable die (it has `TypeBReg`, `TxBitPhaseReg`, `GsNOffReg` with PN512-like defaults),
but the framing bits `[1:0]` of `TxModeReg` / `RxModeReg` are hard-wired to `00` (ISO14443-A):
they never store a written value, and no combination of up to two configuration, reserved or test
registers unlocks them. The chip package also does not expose `MFIN` / `MFOUT` / `AUX`.

So Type B has to be done in software, in both directions.

## How it works

### PCD -> PICC (transmit)

- The frame (SOF, 10-bit characters, EOF, NRZ-L) is turned into a waveform with **one value per
  SPI byte**.
- The waveform is sent to the reader as a single **SPIM EasyDMA burst**: RC522-family chips write
  every byte after the address byte to the same register, so the driver conductance register
  `CWGsPReg` is rewritten every ~1 us (measured 1002.5 ns/byte at 8 MHz). This gives ASK
  modulation with a stable timing, independent of CPU interrupts.
- SPI0 and SPIM0 share the same peripheral slot on the nRF52840: the firmware switches `ENABLE`
  to SPIM for the burst and back to the legacy SPI used by the rest of the driver (clearing any
  stale `READY` event / `RXD` byte, otherwise every later register read is shifted by one).
- Default for ST25TB: `CWGsPReg` `0x3F` (logic 1) / `0x09` (logic 0), 1 etu = 9.44 us.

The same mechanism can generate ISO14443-A frames (modified Miller, 100% ASK through
`TxControlReg`): a software REQA gets a correct ATQA from a MIFARE Classic, which is how the TX
path was validated.

### PICC -> PCD (receive)

- The RC522 receiver only decodes ISO14443-A Manchester/OOK, not Type B BPSK.
- `TestADCReg` (0x3B) exposes the receiver's 4-bit signed I/Q ADC. While the receiver runs, the
  firmware reads it continuously in a second DMA burst (~1 MS/s).
- The 847.5 kHz subcarrier sampled at ~1 MS/s aliases to ~150 kHz. The host
  ([chameleon_14b_soft.py](../software/script/chameleon_14b_soft.py)) estimates that frequency
  (squaring removes the BPSK), mixes down, low-pass filters, takes the phase relative to TR1
  (logic 1), finds the SOF and slices bits at 1 etu.
- A frame is accepted only if its **CRC_B** is valid; otherwise the command is retried.

## Usage

```
hw connect
hf 14b st25tb                    # Chip_ID + UID
hf 14b st25tb -b 128 -f d.bin    # ST25TB04K: UID + blocks 0..127 + system block, save blocks
```

Use `-b 16` for ST25TB512 and `-b 64` for ST25TB02K. Expect about one second per block.

Low-level experimental commands:

| Command | ID | Purpose |
|---|---|---|
| `hf 14b test` | 2050 | Write/read back RC522 registers and send raw frames with the native transceiver |
| (client API) `hf14b_bitbang` | 2051 | Software-generated frame or arbitrary waveform, then receive and/or sample a register |
| (client API) `hf14b_unlock_scan` | 2052 | On-device sweep of register pairs looking for writable bits |

Writable registers are limited to `0x11..0x3F`, excluding `TestPinEnReg` / `TestPinValueReg`
(0x33/0x34): they enable output drivers on pins shared with the SPI bus.

Note: a soft reset does **not** restore `0x39`, `0x3A`, `0x3C..0x3F`; power-on defaults are
`3F 04 FF 00 03 00`.

## Build and flash

```
cd firmware
docker compose up --pull=always build-ultra     # -> firmware/objects/ultra-dfu-app.zip
```

Flash **only the application** via DFU (never the full package with the bootloader):

```
# from software/: put the device in DFU mode
uv run script/chameleon_cli_main.py -c "hw connect" -c "hw dfu"
nrfutil device program --firmware firmware/objects/ultra-dfu-app.zip --traits nordicDfu
```

Keep an official `ultra-dfu-app.zip` from the upstream releases around to roll back.

## Limitations

- Read only (no `WRITE_BLOCK` yet).
- Modulation depth and receiver gain are tuned for a tag lying on the antenna; other distances
  or tags may need different `CWGsPReg` values.
- Slow: every exchange goes through USB and host-side demodulation.
