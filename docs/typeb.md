# ISO14443-B on the Chameleon Ultra

By Luca Tomei. This is how I made the Chameleon Ultra read and write **ST25TB / SRIX** tags
(ISO14443-B) even though its reader chip does not support Type B. Tested with a ST25TB04K:
UID, 128 blocks, system block, block writes, all cross-checked with a Flipper Zero.

## How I got there

1. **The baseline.** `hf 14a scan` on a ST25TB04K: nothing, as expected, since the tag only speaks
   Type B. The schematic shows the Ultra's reader chip is a **Ci522**, an MFRC522-compatible part.
2. **Looking for native Type B.** I added a command to write and read back any reader register.
   The framing bits `[1:0]` of `TxModeReg` / `RxModeReg` never stick: they read back `00` (Type A).
   Mapping every writable bit showed the die clearly derives from an A/B capable chip
   (`TypeBReg`, `TxBitPhaseReg`, `GsNOffReg` with PN512-like reset values), so I looked for a hidden
   unlock: single registers, then an on-device sweep of **every pair** of configuration, reserved and
   test registers (27.5 million combinations). Nothing unlocks them. In self-test mode
   (`AutoTestReg` = `x9`) the bits do store `11`, but the RF front-end is then inactive. The chip
   package does not expose `MFIN`, `MFOUT` or `AUX` either. Conclusion: native Type B is fused off.
3. **Transmitting Type B myself.** RC522-family chips write every byte that follows the address byte
   to the same register. Switching the nRF52840 SPI to its EasyDMA mode for a single burst, I can
   rewrite `CWGsPReg` (the carrier driver conductance) every ~1 us (measured 1002.5 ns per byte),
   which is enough to draw SOF, characters and EOF with 10% ASK at 9.44 us per bit.
4. **Validating the transmitter on a known case.** I used the same technique to generate an
   ISO14443-A REQA in software (modified Miller, 100% ASK pauses through `TxControlReg`): a MIFARE
   Classic answered with its ATQA 10/10 times. Along the way I found and fixed a subtle bug: after
   the DMA burst the legacy SPI kept a stale byte, shifting every later register read by one.
5. **Hearing the tag.** The RC522 receiver cannot decode BPSK, but `TestADCReg` exposes its I/Q ADC.
   Sampling it in a second DMA burst while the receiver runs, I built an answer detector, validated on
   the MIFARE (102 active samples when it answers, 5 when silent). On the ST25TB, a valid INITIATE gave
   ~640 active samples and a corrupted-CRC copy ~85: **the tag was answering my software frames.**
6. **Decoding it.** The 847.5 kHz subcarrier sampled at ~1 MS/s aliases to ~150 kHz. I estimate that
   frequency on the squared signal (squaring removes the BPSK), mix it down, filter it, take the phase
   against TR1, find the SOF and slice bits at 1 etu. The first INITIATE answers came back with valid
   CRC_B, then SELECT, GET_UID and all the blocks.
7. **Making it solid.** Block writing with read-back verification and protection of the irreversible
   blocks, diff and restore, a dump vault in the Ultra's flash, Flipper Zero `.nfc` files byte for
   byte identical to the Flipper's own, and automatic modulation search. I verified reads, writes and
   files against a Flipper Zero, and reading with the tag offset, lifted and rotated (15/15).

## Why native Type B is impossible on this chip

The Ultra uses a **Ci522** reader chip (MFRC522-compatible). Its register map clearly derives from
an A/B capable die (it has `TypeBReg`, `TxBitPhaseReg`, `GsNOffReg` with PN512-like defaults),
but the framing bits `[1:0]` of `TxModeReg` / `RxModeReg` are hard-wired to `00` (ISO14443-A):
they never store a written value, and no combination of up to two configuration, reserved or test
registers unlocks them. The chip package also does not expose `MFIN` / `MFOUT` / `AUX`.

So I do Type B in software, in both directions.

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

Writing a block (the tag does not answer `WRITE_BLOCK`, so the block is read back to verify):

```
hf 14b st25tb --wrbl 42 -d 11223344
```

Blocks 0..4 (OTP, bits can only go 1 -> 0), 5..6 (count-down counters) and 255 (OTP lock bits)
are irreversible and require `--force`.

Dumps, comparison and restore:

```
hf 14b st25tb -b 128 -f d.bin         # read all blocks to a file
hf 14b st25tb --diff d.bin            # list blocks that differ from the dump
hf 14b st25tb --restore d.bin         # write back only the differing blocks, verified
```

Device vault: up to 8 named dumps stored in the Chameleon flash (FDS file `0x1300`), kept across
reboots and firmware updates:

```
hf 14b st25tb --save-slot 1 --name card   # read the tag into vault slot 1
hf 14b st25tb --diff-slot 1               # compare the tag with vault slot 1
hf 14b st25tb --restore-slot 1            # restore the tag from vault slot 1
hf 14b vault list | show -s 1 | delete -s 1
hf 14b vault export -s 1 -f d.bin | import -s 2 -f d.bin --uid <hex> --name x
```

The vault stores data only: a ST25TB cannot be emulated by the Ultra (see Limitations).

Flipper Zero: any dump path ending in `.nfc` is read and written in the Flipper Zero format
(`Device type: ST25TB`), byte for byte identical to the files the Flipper saves. This works with
`-f`, `--diff`, `--restore` and `hf 14b vault import/export`:

```
hf 14b st25tb -b 128 -f card.nfc          # read the tag into a Flipper file
hf 14b st25tb --restore card.nfc          # restore the tag from a Flipper file
hf 14b vault import -s 2 -f card.nfc      # store a Flipper read in the device vault
hf 14b vault export -s 1 -f card.nfc      # take a vault dump to the Flipper
```

Low-level experimental commands:

| Command | ID | Purpose |
|---|---|---|
| `hf 14b test` | 2050 | Write/read back RC522 registers and send raw frames with the native transceiver |
| (client API) `hf14b_bitbang` | 2051 | Software-generated frame or arbitrary waveform, then receive and/or sample a register |
| (client API) `hf14b_unlock_scan` | 2052 | On-device sweep of register pairs looking for writable bits |
| `hf 14b vault ...` | 2060..2063 | Save / load / list / delete dump blobs in the device flash |

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

## Releases

Every firmware change is published as a GitHub release tagged `v2.3.0-typeb.N`, with the DFU
packages (`ultra-dfu-app.zip`, ...) and the CLI clients for Linux, macOS and Windows attached.

## Limitations

- Reader only: emulating a Type B tag is not possible on this hardware. In card mode the Ultra uses
  the nRF52840 NFCT peripheral, which implements ISO14443-A framing in hardware and gives no access
  to the raw field, so it can neither decode the reader's 10% ASK nor produce a BPSK answer.
- Modulation is searched automatically: if the tag does not answer INITIATE with the default
  `CWGsPReg` `0x3F`/`0x09`, the client tries the other settings that produced valid answers
  (`CWGsPReg`, `GsNReg`, one TX driver off) and keeps the working one for the session. Tested
  15/15 with the tag offset by 1 cm, lifted a few millimetres and rotated by 90 degrees.
- Slow: every exchange goes through USB and host-side demodulation.

## NFC-V (ISO15693) — sperimentale, DA VALIDARE su device

Software radio on the same HF hardware as the ST25TB emulation (RC522 field-off ADC sampling +
GPIO load modulation), commands `2300`-`2303`, slot tag type `2100`. Ultra only.

- Reader → tag: only **100% OOK** requests are decoded (Flipper Zero, phones, Proxmark). 10% ASK
  readers are not seen: on the 4-bit RC522 ADC a 10% dip is below the noise.
- Tag → reader: high data rate, one subcarrier (fc/32). A request asking for the low data rate or
  two subcarriers gets the high-rate reply anyway (most readers use the high rate).
- Replies up to 160 bytes (Read Multiple Blocks of up to ~38 four-byte blocks per request); a bigger
  request gets error `0x0F`.
- The emulation decoder learns the ADC zero inside the reader's pauses, so a small DC offset of the
  ADC does not hide them; `HF15_EMU_DIAG` reports pauses/frames/replies/late to tune on hardware.
- The Chameleon reader reads replies up to ~27 bytes per request (single-block reads are used).
- Not modelled: SLIX passwords / privacy mode, AFI family matching, the Write/Lock option flag.
