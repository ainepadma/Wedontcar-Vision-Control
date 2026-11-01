#include "track_vision.h"
#include "ring_control.h"
#include "img_process.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define H 154U
#define W 304U
#define CHECK(c) do {if(!(c)){fprintf(stderr,"FAIL %d: %s\n",__LINE__,#c);exit(1);}}while(0)
static struct {unsigned char before[32],image[H*W],after[32];} guarded;
#define image guarded.image
static unsigned char saved[H*W],scratch[5U*W];
static uint32_t histogram[256];
static track_vision_result_t result;
static track_ring_vision_state_t model;
static ring_corner_frame_t corner;
static uint16_t left[H],right[H];static uint8_t flags[H];
static ring_corner_context_t context;
static void road(int shift,int curve) {
    memset(image,0,sizeof(image));
    for(unsigned y=0;y<H;++y) {
        int c=(int)W/2+shift+curve*(int)(H-1-y)/(int)H;
        for(int x=c-60;x<c+60;++x) if(x>=0 && x<(int)W) image[y*W+(unsigned)x]=255;
    }
}
static void process(void) {
    memcpy(saved,image,sizeof(image));
    track_vision_process(image,H,W,&result);
    result.frame_number++;
    track_ring_vision_process(&model,image,H,W,&result);
    CHECK(memcmp(saved,image,sizeof(image))==0);
    track_vision_filter_display(image,H,W);
    CHECK(memcmp(saved,image,sizeof(image))==0);
    for(unsigned i=0;i<32;++i) CHECK(guarded.before[i]==0xa5 && guarded.after[i]==0xa5);
}
static void acute(unsigned tip,int mirror) {
    memset(image,0,sizeof(image));
    unsigned edge=W*29U/100U,end=W*70U/100U;
    for(unsigned y=0;y<H;++y) for(unsigned x=0;x<W;++x) {
        unsigned sx=mirror?W-1-x:x;
        int white=(x>=edge && x<=end) || (x<edge && (float)y<(float)tip+.5f*(float)(edge-x));
        image[y*W+sx]=white?255:0;
    }
    for(unsigned y=0;y<H;++y) {
        unsigned a=W/2,b=a;
        while(a && image[y*W+a-1]) --a;
        while(b+1<W && image[y*W+b+1]) ++b;
        left[y]=(uint16_t)a;right[y]=(uint16_t)b;
        flags[y]=(uint8_t)((a>1?1:4)|(b<W-2?2:8));
    }
    memset(&context,0,sizeof(context));context.left=left;context.right=right;context.flags=flags;
    context.model_ready=1;context.width_near=(float)(end-edge);context.center_near=(float)(W-1)/2;
    ring_corner_process(image,H,W,&context,&corner);
    CHECK(corner.count[mirror]!=0 && corner.count[mirror]<3);
    CHECK(corner.point[mirror][0].kind==RING_CORNER_A);
    CHECK(corner.detector_near_row>H*68U/100U);
}
int main(void) {
    CHECK(TRACK_VISION_ROI_START_PERCENT==0 && TRACK_VISION_ROI_END_PERCENT==100);
    CHECK(RING_SEARCH_START_PERCENT==0 && RING_SEARCH_END_PERCENT==100);
    CHECK(RING_GUIDE_FAR_PERCENT==0 && RING_GUIDE_NEAR_PERCENT==100);
    memset(&guarded,0xa5,sizeof(guarded));
    road(0,0);process();CHECK(result.valid && result.roi_rows==H);
    CHECK(result.row_valid[0] && result.row_valid[H-1]);CHECK(abs(result.error)<10);
    CHECK(result.near_geometry_valid && abs(result.near_error)<10 && result.near_heading==0);
    CHECK(result.bottom_valid && abs(result.bottom_error)<10);
    road(-20,0);process();CHECK(result.valid && result.error<0);
    CHECK(result.near_error<0 && result.near_heading==0);
    CHECK(result.bottom_valid && result.bottom_error<0);
    road(20,0);process();CHECK(result.valid && result.error>0);
    road(0,-25);process();CHECK(result.valid && result.heading<0);
    CHECK(result.near_geometry_valid && result.near_heading<0);
    road(0,25);process();CHECK(result.valid && result.heading>0);
    CHECK(result.near_geometry_valid && result.near_heading>0);
    /* Sharp bends remain continuous when one visible boundary reaches the crop. */
    for (int sign=-1;sign<=1;sign+=2) {
        road(0,sign*125);process();
        CHECK(result.valid && result.row_valid[0] && result.row_valid[H-1]);
        CHECK(sign<0 ? result.left[0]==0 : result.right[0]==W-1);
        for(unsigned y=1;y<H;++y) CHECK(result.row_valid[y]);
    }
    /* Even a one-row separator must not be bridged to the road beyond. */
    road(0,0);memset(image+80U*W,0,W);process();
    CHECK(result.row_valid[81] && !result.row_valid[80] && !result.row_valid[79]);
    CHECK(!result.valid);
    /* A short black bar only across the centre also blocks the connection. */
    road(0,0);memset(image+80U*W+140U,0,24U);process();
    CHECK(result.row_valid[81] && !result.row_valid[79]);
    /* A tolerated two-pixel run gap cannot put a valid centre on black. */
    road(0,0);image[80U*W+151U]=0;image[80U*W+152U]=0;process();
    CHECK(!result.row_valid[80] && !result.row_valid[79]);
    /* Top and bottom beyond the old 30..68% must actually affect observations. */
    road(0,0);for(unsigned y=0;y<30;++y) memset(image+y*W,0,W);
    process();CHECK(!result.row_valid[0] && result.valid_rows<H);
    road(0,0);for(unsigned y=110;y<H;++y) memset(image+y*W,0,W);
    process();CHECK(!result.row_valid[H-1] && !result.bottom_valid);
    for(int side=0;side<2;++side) {acute(40,side);acute(110,side);}
    /* Bilateral junction corners still produce the 270 connect geometry. */
    road(0,0);
    for(unsigned y=45;y<85;++y) memset(image+y*W,255,W);
    for(unsigned y=0;y<H;++y) {
        unsigned a=W/2,b=a;
        while(a && image[y*W+a-1]) --a;
        while(b+1<W && image[y*W+b+1]) ++b;
        left[y]=(uint16_t)a;right[y]=(uint16_t)b;
        flags[y]=(uint8_t)((a>1?1:4)|(b<W-2?2:8));
    }
    context.width_near=120;context.center_near=(float)(W-1)/2;
    ring_corner_process(image,H,W,&context,&corner);
    CHECK(corner.count[0]==2 && corner.count[1]==2);
    CHECK(corner.guide[RING_GUIDE_270_CONNECT].valid);
    /* New full-frame preprocessing accepts roi_first=0, roi_end=H. */
    road(0,0);for(unsigned i=0;i<sizeof(image);++i) image[i]=image[i]?60:20;
    uint8_t threshold;image_track_preprocess_state_t state={0};
    CHECK(image_track_preprocess_stable(image,H,W,0,H,scratch,sizeof(scratch),histogram,&threshold,&state,100));
    CHECK(threshold!=255);process();CHECK(result.valid && result.row_valid[0] && result.row_valid[H-1]);
    memset(image,255,sizeof(image));
    CHECK(image_track_preprocess_stable(image,H,W,0,H,scratch,sizeof(scratch),histogram,&threshold,&state,400));
    CHECK(threshold==255);CHECK(image[0]==255 && image[sizeof(image)-1]==255);
    puts("PASS production 304x154 full window: all-row vision, offsets/curves, top/bottom influence, mirrored A tips outside old ROI, immutable display, full preprocess, weak-frame preview and guards");
    return 0;
}
