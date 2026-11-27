#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "hf15_codec.h"
int main(int argc, char **argv) {
    FILE *f = fopen(argv[1], "rb");
    if (!f) { printf("no file\n"); return 1; }
    fseek(f, 0, SEEK_END); long n = ftell(f); fseek(f, 0, SEEK_SET);
    unsigned char *buf = malloc(n); if (fread(buf, 1, n, f) != (size_t)n) {} fclose(f);
    hf15_rx_t rx; hf15_rx_reset(&rx);
    unsigned i = 0; unsigned frames = 0;
    while (i < (unsigned)n) {
        unsigned consumed = 0; uint32_t eof = 0;
        unsigned batch = (unsigned)n - i; if (batch > 128) batch = 128;
        hf15_rx_event_t ev = hf15_rx_feed(&rx, buf + i, batch, i, &consumed, &eof);
        if (ev == HF15_RX_FRAME) { frames++; printf("FRAME @%u len=%u: ", i, rx.len); for (unsigned k=0;k<rx.len;k++) printf("%02X ", rx.data[k]); printf("\n"); }
        i += consumed ? consumed : batch;
    }
    printf("RESULT: pauses=%u frames=%u errors=%u last_error=%u peak_level=%u last_pause_len=%u zero_i=%d zero_q=%d\n",
           rx.pauses, rx.frames, rx.errors, rx.last_error, rx.peak_level, rx.last_pause_len, rx.zero_i, rx.zero_q);
    return 0;
}
