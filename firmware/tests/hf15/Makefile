# Host tests for the ISO15693 (NFC-V) codec + model: plain gcc, no hardware, no nRF SDK.
#   make -C firmware/tests/hf15        build and run
# Not part of the firmware build (build.sh only runs make in bootloader/ and application/).
SRC := ../../application/src/rfid/iso15693
CFLAGS := -O2 -Wall -Wextra -Werror -Wno-misleading-indentation -I$(SRC) -I$(SRC)/..

run: hf15_codec_test
	./hf15_codec_test

hf15_codec_test: hf15_codec_test.c $(SRC)/hf15_codec.c $(SRC)/hf15_model.c $(SRC)/frame.c $(SRC)/hf15_codec.h $(SRC)/hf15_model.h
	$(CC) $(CFLAGS) -o $@ hf15_codec_test.c $(SRC)/hf15_codec.c $(SRC)/hf15_model.c $(SRC)/frame.c -lm

clean:
	rm -f hf15_codec_test

.PHONY: run clean
