# Crediti

Questo firmware è un fork di
[RfidResearchGroup/ChameleonUltra](https://github.com/RfidResearchGroup/ChameleonUltra).

Spunti presi da altri progetti e riadattati qui:

- **Polling a 16 slot su evento campo** (`application/src/nfc_poll.c`, cablato in
  `app_main.c` e `rfid/nfctag/hf/nfc_14a.c`): approccio ispirato al lavoro di
  **Mazha0309** sul ciclo degli slot abilitati quando un lettore porta il campo.
  Implementazione riscritta per questo fork. DA VALIDARE su hardware.

- **Emulazione ST25TB** (`application/src/rfid/nfctag/hf/`): la tecnica di risposta
  BPSK a ~847,5 kHz tramite `HF_ANT_SEL` deriva dallo studio del **fork Tonie** di
  ChameleonUltra. Il livello RF (`st25tb_emu_run`) è ancora sperimentale e va portato
  su con oscilloscopio. DA VALIDARE su hardware.
