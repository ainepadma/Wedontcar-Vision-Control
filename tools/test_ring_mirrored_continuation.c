#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../demo/user/src/track_vision.c"

#define H 154U
#define W 304U
static uint8_t pixels[H*W];
static track_vision_result_t road;
static ring_corner_guide_t guide[2];

int main(void)
{
    memset(pixels,255,sizeof(pixels));
    memset(&road,0,sizeof(road));
    memset(guide,0,sizeof(guide));
    for(unsigned y=0;y<H;++y) {
        road.row_valid[y]=1;
        road.center[y]=100;
    }
    for(unsigned side=0;side<2;++side) {
        ring_corner_guide_t *g=&guide[side];
        g->valid=1;g->clip_upper=1;g->active_top=80;
        for(unsigned i=8;i<16;++i) {
            g->center[i]=side ? 203 : 100;
            g->center_valid|=(uint16_t)(1U<<i);
            if(i>8) g->center_link_valid|=(uint16_t)(1U<<i);
        }
        for(unsigned y=0;y<H;++y) road.center[y]=side ? 203 : 100;
        track_continue_ring_road(pixels,H,W,&road,g,side);
        assert(g->active_top==0 && (g->center_valid&1U));
    }
    assert(guide[0].center_valid==guide[1].center_valid);
    assert(guide[0].center_link_valid==guide[1].center_link_valid);
    for(unsigned i=0;i<16;++i)
        assert(guide[0].center[i]+guide[1].center[i]==W-1U);
    assert(abs(guide[0].error+guide[1].error)<=1);
    assert(abs(guide[0].heading+guide[1].heading)<=1);
    puts("PASS mirrored S5/S6 continuation");
    return 0;
}
