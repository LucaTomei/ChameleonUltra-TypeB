#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "hf15_codec.h"
int main(int argc, char **argv) {
    FILE *f = fopen(argv[1], "rb"); fseek(f,0,SEEK_END); long n=ftell(f); fseek(f,0,SEEK_SET);
    unsigned char *buf=malloc(n); if(fread(buf,1,n,f)!=(size_t)n){} fclose(f);
    hf15_rx_t rx; hf15_rx_reset(&rx);
    unsigned prevp=0, prevpos=0, printed=0;
    for (unsigned i=0;i<(unsigned)n;i++){
        unsigned consumed=0; uint32_t eof=0;
        hf15_rx_feed(&rx, buf+i, 1, i, &consumed, &eof);
        if (rx.pauses>prevp){
            unsigned pos=(unsigned)rx.last_pause_us; unsigned len=(unsigned)rx.last_pause_len;
            if (printed<40) printf("pause#%u @%u len=%u gap=%u state=%u\n", rx.pauses, pos, len, pos-prevpos, rx.state);
            prevpos=pos; prevp=rx.pauses; printed++;
        }
    }
    printf("TOTAL pauses=%u frames=%u errors=%u last_error=%u\n", rx.pauses, rx.frames, rx.errors, rx.last_error);
    return 0;
}
