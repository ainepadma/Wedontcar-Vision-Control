#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "img_process.h"
#include "track_vision.h"
#define CHECK(c) do {if(!(c)){fprintf(stderr,"FAIL %d: %s\n",__LINE__,#c);exit(1);}}while(0)
static uint8_t pixels[240U*376U], saved[sizeof(pixels)], scratch[1880];
static uint32_t histogram[256];
static track_vision_result_t result;
static void lane(uint8_t dark,uint8_t light,unsigned left,unsigned width)
{
    memset(pixels,dark,sizeof(pixels));
    for(unsigned y=0;y<240U;++y) memset(pixels+y*376U+left,light,width);
}
static void process(image_track_preprocess_state_t *state,uint32_t now)
{
    uint8_t threshold;
    CHECK(image_track_preprocess_stable(pixels,240,376,72,163,scratch,sizeof(scratch),histogram,&threshold,state,now));
    track_vision_process(pixels,240,376,&result);
}
static void temporal(void)
{
    image_track_preprocess_state_t state={0};
    lane(10,31,126,124);process(&state,1000);CHECK(result.valid && state.ready);
    uint16_t previous=state.gain_q8;
    lane(10,32,126,124);process(&state,1053);
    CHECK(result.valid && state.gain_q8<previous && state.gain_q8>192U*256U/32U);
    previous=state.gain_q8;
    lane(10,33,126,124);process(&state,1123); /* Even >60ms still smooths. */
    CHECK(result.valid && state.gain_q8<previous && state.gain_q8>192U*256U/33U);
    lane(10,34,126,124);process(&state,1374); /* >250ms resets. */
    CHECK(state.gain_q8==192U*256U/34U);
    lane(10,34,126,124);process(&state,1374);CHECK(result.valid); /* dt=0 */
    state.last_ms=0xfffffff0U;
    lane(10,34,126,124);process(&state,37U);CHECK(result.valid); /* wrap: 53ms */
    lane(0,1,126,124);process(&state,400);CHECK(result.valid);
    lane(20,60,126,124);process(&state,453);CHECK(result.valid); /* sudden brighter */
    lane(0,1,126,124);process(&state,506);CHECK(result.valid); /* sudden darker */
    lane(20,60,80,110);process(&state,800);CHECK(result.valid);
    lane(20,60,190,110);process(&state,853);
    CHECK(result.valid && pixels[120U*376U+100U]==0U && pixels[120U*376U+270U]==255U);
    memset(pixels,0,sizeof(pixels));process(&state,906);CHECK(!state.ready && !result.valid);
    puts("PASS actual 53/70ms smoothing, dt=0, timeout/wrap, bright/dark steps, current moving image, zero frame");
}
static void holes(void)
{
    memset(pixels,255,sizeof(pixels));
    for(unsigned y=100;y<107;++y) memset(pixels+y*376U+120,0,7);
    CHECK(image_fill_small_black_holes(pixels,240,376)==49U);
    for(unsigned y=100;y<104;++y) memset(pixels+y*376U+120,0,30);
    CHECK(image_fill_small_black_holes(pixels,240,376)==0U);
    for(unsigned y=0;y<110;++y) pixels[y*376U+y]=0;
    pixels[110U*376U+110U]=0;
    CHECK(image_fill_small_black_holes(pixels,240,376)==0U); /* diagonal escape to edge */
    memset(pixels,0,sizeof(pixels));
    CHECK(image_fill_small_black_holes(pixels,240,376)==0U);
    clock_t start=clock();
    for(unsigned y=0;y<240;++y) for(unsigned x=0;x<376;++x) pixels[y*376U+x]=((x+y)&1U)?255:0;
    memcpy(saved,pixels,sizeof(pixels));
    CHECK(image_fill_small_black_holes(pixels,240,376)==0U);
    CHECK(memcmp(saved,pixels,sizeof(pixels))==0);
    printf("PASS 7x7 holes, 30x4 island, diagonal exit, full black/checkerboard (host %.2fms)\n",1000.*(clock()-start)/CLOCKS_PER_SEC);
}
static void illumination_and_islands(void)
{
    image_track_preprocess_state_t state={0};
    lane(20,60,126,124);
    for(unsigned y=0;y<240;++y) for(unsigned x=0;x<376;++x)
        pixels[y*376U+x]=(uint8_t)((unsigned)pixels[y*376U+x]*(60U+x*40U/376U)/100U);
    memcpy(saved,pixels,sizeof(pixels));process(&state,1000);CHECK(result.valid);
    uint8_t expected[sizeof(pixels)];memcpy(expected,pixels,sizeof(pixels));
    memset(&state,0,sizeof(state));
    for(unsigned y=0;y<240;++y) for(unsigned x=0;x<376;++x) pixels[y*376U+x]=saved[y*376U+375U-x];
    process(&state,1000);CHECK(result.valid);
    for(unsigned y=72;y<163;++y) for(unsigned x=0;x<376;++x) CHECK(pixels[y*376U+x]==expected[y*376U+375U-x]);
    /* A minority black class must survive even with two white grey levels. */
    for(unsigned y=0;y<240;++y) for(unsigned x=0;x<376;++x) pixels[y*376U+x]=(uint8_t)(58U+(x&1U)*2U);
    for(unsigned y=110;y<114;++y) memset(pixels+y*376U+160,20,50);
    memset(&state,0,sizeof(state));process(&state,1000);
    CHECK(pixels[100U*376U+180U]==255U && pixels[111U*376U+180U]==0U);
    /* A visible island with headroom above the geometric minimum remains. */
    lane(20,60,70,235);
    for(unsigned y=110;y<114;++y) memset(pixels+y*376U+170,20,34);
    memset(&state,0,sizeof(state));process(&state,1000);
    unsigned supported=0;
    for(unsigned y=110;y<114;++y) {unsigned n=0;for(unsigned x=160;x<215;++x) if(!pixels[y*376U+x]) ++n;if(n>=30) ++supported;}
    CHECK(supported==4U);
    puts("PASS smooth left/right illumination and mirror, noisy-white minority black class, 34x4 real island");
}
int main(void)
{
    temporal();holes();illumination_and_islands();
    puts("All stability/illumination tests passed; no hardware accessed.");return 0;
}
