#include "../src/bitstream.h"
#include <stdio.h>

int main(void) {
    u8 buf[16] = {0};
    bitstream_t bs;
    bs_init(&bs, buf, sizeof buf);
    bs_put_bits(&bs, 1, 1);
    printf("after put_bits(1,1): cur=0x%08x n_in_cur=%d byte_pos=%d\n",
           bs.cur, bs.n_in_cur, bs.byte_pos);
    int n = bs_byte_count(&bs);
    printf("byte_count=%d buf[0]=0x%02x\n", n, buf[0]);

    /* second test: bs_put_ue(0) */
    bitstream_t bs2;
    u8 buf2[16] = {0};
    bs_init(&bs2, buf2, sizeof buf2);
    bs_put_ue(&bs2, 0);
    printf("after put_ue(0): cur=0x%08x n_in_cur=%d byte_pos=%d\n",
           bs2.cur, bs2.n_in_cur, bs2.byte_pos);
    int n2 = bs_byte_count(&bs2);
    printf("byte_count=%d buf[0]=0x%02x\n", n2, buf2[0]);
    return 0;
}
