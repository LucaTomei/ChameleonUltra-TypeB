# ChameleonUltra-TypeB

**ISO14443-B / ST25TB reading and writing on the Chameleon Ultra**, by Luca Tomei.

The Chameleon Ultra was never meant to talk to ISO14443-B tags: its HF reader chip, a Ci522
(MFRC522-compatible), only does Type A framing. I found a way around it and made the Ultra read and
write **ST25TB / SRIX** tags anyway, entirely in software:

- I generate the Type B frames myself, by rewriting a carrier driver register about once per
  microsecond in a single SPI DMA burst, which gives ASK modulation with stable timing;
- I sample the reader's internal ADC while the tag answers and demodulate its BPSK subcarrier on the
  host, accepting only frames with a valid CRC_B.

On top of that I added block writing with verification, dump diff and restore, a dump vault in the
Ultra's flash and Flipper Zero `.nfc` compatibility. Every result was cross-checked with a Flipper Zero.

```
hf 14b st25tb -b 128 -f card.nfc
```

How I got there, how it works, usage, building and flashing: [docs/typeb.md](docs/typeb.md) and the
[wiki](https://github.com/LucaTomei/ChameleonUltra-TypeB/wiki/iso14443b).

This project is based on the [RfidResearchGroup/ChameleonUltra](https://github.com/RfidResearchGroup/ChameleonUltra)
firmware and client (GPL-3.0) and is not affiliated with RfidResearchGroup. The rest of this README is
the upstream one.

---

![logo](docs/images/ultra-logo.png)

![ultra picture](docs/images/ultra-overview.png)

# ChameleonUltra Authorized Distributors

Lyon, France: [Lab401](https://lab401.com/)

Santa Ana, United States: [Hackerwarehouse](https://hackerwarehouse.com/)

Hastings, UK: [KSEC](https://labs.ksec.co.uk/product/proxgrind-chameleon-ultra/)

Montreal, Canada: [TechSecurityTools](https://techsecuritytools.com/product/chameleon-ultra/)

Shenzhen, China: [Sneaktechnology](https://sneaktechnology.com)

Guangdong, China: [MTools Tec](https://shop.mtoolstec.com/)

Lazada One, Singapore: [Aliexpress by RRG](https://proxgrind.aliexpress.com/store/1101312023)

# What is it and how to use ?

Read the [available documentation](https://github.com/RfidResearchGroup/ChameleonUltra/wiki).

# Compatible applications

* [ChameleonUltraGUI](https://github.com/GameTec-live/ChameleonUltraGUI)
* [ChameLab](https://github.com/m0n5ter/ChameLab)
* [Chameleon Ultra (Sailfish OS only)](https://sailfishos-chum.github.io/apps/harbour-chameleon-ultra)
* [Chameleon GO (iOS only)](https://apps.apple.com/us/app/id6802015306)
* [Device console for ChameleonUltra (Web)](https://sec1.dk/phreakbyte.html)
* [MTools BLE](https://github.com/RfidResearchGroup/ChameleonUltra/wiki/mtoolsble)
* [Mifare Chameleon Tool (iOS only, Beta)](https://apps.apple.com/it/app/mifare-chameleon-tool/id6761231484)

# Videos

*Beware some of the instructions might have changed since recording, check the current documentation when in doubt!*

* [Downloading and compiling the official CLI](https://www.youtube.com/watch?v=VGpAeitNXH0)
* [Downloading ChameleonUltraGUI](https://www.youtube.com/watch?v=rHH7iqbX3nY)
* [ChameleonUltraGUI features overview](https://www.youtube.com/watch?v=YqE8wyVSse4)
* [Using ChameleonUltraGUI and the Chameleon Ultra](https://www.youtube.com/watch?v=9jtKNJ5-kVY)
* [MTools BLE - How to clone a card with ChameleonUltra](https://youtu.be/IvH-xtdW1Wk?si=4exqgAAeJ-kxU3aN)

# Official channels

Where do you find the community?
* [RFID Hacking community discord server](https://t.ly/d4_C)
  * Software/chameleon-dev for firmware and clients development discussions
  * Devices/chameleon-ultra for usage discussions
* [GameTec_live discord server](https://discord.gg/DJ2A4wxncK)

Other notable projects:
* [Fantasi](https://fantasi.cloud)
  * [Discord](https://fantasi.cloud/discord)
* [chameleon-ultra.js](https://github.com/taichunmin/chameleon-ultra.js/) 

###### Searching for the docs repo? Find it [here](https://github.com/RfidResearchGroup/ChameleonUltraDocs)
