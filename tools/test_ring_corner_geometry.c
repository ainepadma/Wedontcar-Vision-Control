#include "ring_corner.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x) do {if(!(x)){fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x);exit(1);}}while(0)
#define H 240U
#define W 376U
static uint8_t image[480U*752U],original[480U*752U];
static uint16_t left[480],right[480];static uint8_t flags[480];
static ring_corner_frame_t result;
static ring_corner_context_t ctx;

static void road(unsigned rows,unsigned cols)
{
    memset(image,0,sizeof(image));memset(flags,0,sizeof(flags));
    for(unsigned y=0U;y<rows;++y) for(unsigned x=cols*29U/100U;x<=cols*70U/100U;++x)
        image[y*cols+x]=255U;
    memset(&ctx,0,sizeof(ctx));ctx.left=left;ctx.right=right;ctx.flags=flags;
    ctx.model_ready=1U;ctx.width_near=(float)(cols*70U/100U-cols*29U/100U);
    ctx.center_near=(float)(cols*70U/100U+cols*29U/100U)*.5f;
}
static void raw(unsigned rows,unsigned cols)
{
    for(unsigned y=0U;y<rows;++y) {
        unsigned x=cols/2U;
        if(!image[y*cols+x]) {flags[y]=0U;continue;}
        unsigned a=x,b=x;
        while(a>0U && image[y*cols+a-1U]) --a;
        while(b+1U<cols && image[y*cols+b+1U]) ++b;
        left[y]=(uint16_t)a;right[y]=(uint16_t)b;
        flags[y]=(uint8_t)((a>1U ? 1U : 4U)|(b<cols-2U ? 2U : 8U));
    }
}
static void process(unsigned rows,unsigned cols)
{
    raw(rows,cols);memcpy(original,image,rows*cols);
    ring_corner_process(image,(uint16_t)rows,(uint16_t)cols,&ctx,&result);
    CHECK(memcmp(image,original,rows*cols)==0);
}
static void crossing(unsigned rows,unsigned cols,unsigned start,unsigned end)
{
    road(rows,cols);
    for(unsigned y=start;y<end;++y) memset(image+y*cols,255,cols);
}
static void acute_slope(unsigned rows,unsigned cols,unsigned ytip,uint8_t type,float slope)
{
    road(rows,cols);int edge=(int)(cols*29U/100U);
    for(unsigned y=0U;y<rows;++y) for(int x=0;x<edge;++x) {
        float diagonal=type==RING_CORNER_A ? (float)ytip+slope*(float)(edge-x) : (float)ytip-slope*(float)(edge-x);
        if((type==RING_CORNER_A && (float)y<diagonal) || (type==RING_CORNER_B && (float)y>diagonal)) image[y*cols+(unsigned)x]=255U;
    }
}
static void acute(unsigned rows,unsigned cols,unsigned ytip,uint8_t type)
{
    acute_slope(rows,cols,ytip,type,.5f);
}
static void mirror(unsigned rows,unsigned cols)
{
    for(unsigned y=0U;y<rows;++y) for(unsigned x=0U;x<cols/2U;++x) {
        uint8_t p=image[y*cols+x];image[y*cols+x]=image[y*cols+cols-1U-x];image[y*cols+cols-1U-x]=p;
    }
    ctx.center_near=(float)(cols-1U)-ctx.center_near;ctx.center_slope=-ctx.center_slope;
}
static void perspective(unsigned rows,unsigned cols,float shear)
{
    memcpy(original,image,rows*cols);
    float near=(float)(rows*68U/100U-1U),center=ctx.center_near;
    for(unsigned y=0U;y<rows;++y) for(unsigned x=0U;x<cols;++x) {
        float dy=(float)y-near,scale=1.f+.003f*dy;
        int source=(int)lroundf(((float)x-center-shear*dy)/scale+center);
        image[y*cols+x]=source>=0 && source<(int)cols ? original[y*cols+(unsigned)source] : 0U;
    }
    ctx.width_slope=ctx.width_near*.003f;ctx.center_slope=shear;
}
static void check_guides(unsigned rows,unsigned cols)
{
    for(unsigned id=1U;id<RING_CORNER_GUIDE_COUNT;++id) {
        const ring_corner_guide_t *g=&result.guide[id];if(!g->valid) continue;
        CHECK(g->confidence>=55U && g->center_valid!=0U);
        CHECK(g->boundary[0].valid || g->boundary[1].valid);
        for(unsigned side=0U;side<2U;++side) if(g->boundary[side].valid) {
            const ring_corner_line_t *line=&g->boundary[side];
            CHECK(isfinite(line->x0) && isfinite(line->x1));
            CHECK(line->x0>=0.f && line->x0<(float)cols && line->x1>=0.f && line->x1<(float)cols);
            CHECK(line->y0>=(float)(rows*30U/100U) && line->y0<(float)(rows*68U/100U));
            CHECK(line->y1>=(float)(rows*30U/100U)-.01f && line->y1<(float)(rows*68U/100U));
        }
        for(unsigned k=0U;k<RING_CORNER_CENTER_SAMPLES;++k) if(g->center_valid&(uint16_t)(1U<<k)) {
            unsigned y=rows*40U/100U+k*(rows*68U/100U-1U-rows*40U/100U)/(RING_CORNER_CENTER_SAMPLES-1U);
            CHECK(g->center[k]<cols && image[y*cols+g->center[k]]!=0U);
            if(k>0U && (g->center_link_valid&(uint16_t)(1U<<k)))
                CHECK(g->center_valid&(uint16_t)(1U<<(k-1U)));
        }
    }
}
static void check_no_guides(void)
{
    for(unsigned k=1U;k<RING_CORNER_GUIDE_COUNT;++k) CHECK(!result.guide[k].valid);
}
int main(void)
{
    road(H,W);process(H,W);CHECK(result.count[0]==0U && result.count[1]==0U && result.road_clear);
    crossing(H,W,0U,125U);process(H,W);
    CHECK(result.count[0]==1U && result.count[1]==1U);
    CHECK(result.guide[RING_GUIDE_270_EXTEND].valid && !result.guide[RING_GUIDE_270_CONNECT].valid);
    check_guides(H,W);
    crossing(H,W,98U,132U);process(H,W);
    CHECK(result.count[0]==2U && result.count[1]==2U);
    CHECK(result.guide[RING_GUIDE_270_CONNECT].valid && !result.guide[RING_GUIDE_270_EXTEND].valid);
    check_guides(H,W);
    for(unsigned tilted=0U;tilted<2U;++tilted) {
        crossing(H,W,98U,132U);perspective(H,W,tilted ? .3f : -.3f);process(H,W);
        CHECK(result.count[0]==2U && result.count[1]==2U);
        CHECK(result.guide[RING_GUIDE_270_CONNECT].valid);check_guides(H,W);
    }
    /* A black noise row is never silently joined by the diagnostic midline. */
    crossing(H,W,98U,132U);memset(image+117U*W,0,W);process(H,W);
    CHECK(result.guide[RING_GUIDE_270_CONNECT].valid);
    CHECK((result.guide[RING_GUIDE_270_CONNECT].center_valid&((1U<<4)|(1U<<5)))==((1U<<4)|(1U<<5)));
    CHECK(!(result.guide[RING_GUIDE_270_CONNECT].center_link_valid&(1U<<5)));
    acute(H,W,125U,RING_CORNER_A);process(H,W);
    CHECK(result.count[0]==1U && result.count[1]==0U && result.point[0][0].kind==RING_CORNER_A);
    CHECK(result.guide[RING_GUIDE_A_LEFT].valid);
    int a=result.guide[RING_GUIDE_A_LEFT].error;
    mirror(H,W);process(H,W);
    CHECK(result.count[0]==0U && result.count[1]==1U && result.point[1][0].kind==RING_CORNER_A);
    CHECK(result.guide[RING_GUIDE_A_RIGHT].valid && abs(a+result.guide[RING_GUIDE_A_RIGHT].error)<=3);
    /* With approach distance and 120-degree-lens perspective, the same A
     * evolves from an obvious oblique tip into an almost right-angle tip.
     * Counts still identify 360; direction quantization must not flicker A/NONE. */
    const float approach_slopes[]={.50f,.25f,.08f,0.f};
    for(unsigned approach=0U;approach<sizeof(approach_slopes)/sizeof(approach_slopes[0]);++approach) {
        acute_slope(H,W,125U,RING_CORNER_A,approach_slopes[approach]);process(H,W);
        CHECK(result.count[0]==1U && result.count[1]==0U && result.point[0][0].kind==RING_CORNER_A);
        CHECK(result.guide[RING_GUIDE_A_LEFT].valid);
        mirror(H,W);process(H,W);
        CHECK(result.count[0]==0U && result.count[1]==1U && result.point[1][0].kind==RING_CORNER_A);
        CHECK(result.guide[RING_GUIDE_A_RIGHT].valid);
    }
    acute(H,W,105U,RING_CORNER_B);process(H,W);
    CHECK(result.count[0]==1U && result.count[1]==0U && result.point[0][0].kind==RING_CORNER_B);
    CHECK(result.guide[RING_GUIDE_B_BRANCH_LEFT].valid && result.guide[RING_GUIDE_B_MAIN_LEFT].valid);
    CHECK(result.guide[RING_GUIDE_B_BRANCH_LEFT].error<result.guide[RING_GUIDE_B_MAIN_LEFT].error);
    CHECK(result.guide[RING_GUIDE_B_BRANCH_LEFT].boundary[1].valid);
    CHECK(result.guide[RING_GUIDE_B_MAIN_LEFT].boundary[0].valid);
    CHECK(result.guide[RING_GUIDE_B_BRANCH_LEFT].error<0 && result.guide[RING_GUIDE_B_BRANCH_LEFT].heading<0);
    int b_error=result.guide[RING_GUIDE_B_BRANCH_LEFT].error,b_heading=result.guide[RING_GUIDE_B_BRANCH_LEFT].heading;
    check_guides(H,W);mirror(H,W);process(H,W);check_guides(H,W);
    CHECK(result.guide[RING_GUIDE_B_BRANCH_RIGHT].valid && result.guide[RING_GUIDE_B_MAIN_RIGHT].valid);
    CHECK(abs(b_error+result.guide[RING_GUIDE_B_BRANCH_RIGHT].error)<=3);
    CHECK(abs(b_heading+result.guide[RING_GUIDE_B_BRANCH_RIGHT].heading)<=3);
    for(unsigned tip=105U;tip<=135U;tip+=3U) {
        acute(H,W,tip,RING_CORNER_B);process(H,W);
        if(!result.guide[RING_GUIDE_B_BRANCH_LEFT].valid) fprintf(stderr,"B2 near tip %u rejected\n",tip);
        CHECK(result.guide[RING_GUIDE_B_BRANCH_LEFT].valid);
        CHECK(result.guide[RING_GUIDE_B_BRANCH_LEFT].error<0 && result.guide[RING_GUIDE_B_BRANCH_LEFT].heading<0);
        CHECK(result.guide[RING_GUIDE_B_MAIN_LEFT].valid);
        int turn_error=result.guide[RING_GUIDE_B_BRANCH_LEFT].error;
        int turn_heading=result.guide[RING_GUIDE_B_BRANCH_LEFT].heading;
        check_guides(H,W);mirror(H,W);process(H,W);check_guides(H,W);
        CHECK(result.guide[RING_GUIDE_B_BRANCH_RIGHT].valid);
        CHECK(abs(turn_error+result.guide[RING_GUIDE_B_BRANCH_RIGHT].error)<=5);
        CHECK(abs(turn_heading+result.guide[RING_GUIDE_B_BRANCH_RIGHT].heading)<=5);
    }
    /* Too-late B2 has less than 55% visible path. Do not quietly substitute
     * the valid B4 straight policy to make this turning observation valid. */
    acute(H,W,138U,RING_CORNER_B);process(H,W);
    CHECK(result.count[0]==1U && result.point[0][0].kind==RING_CORNER_B);
    CHECK(!result.guide[RING_GUIDE_B_BRANCH_LEFT].valid && result.guide[RING_GUIDE_B_MAIN_LEFT].valid);

    /* The same counts work at three native resolutions, without resizing. */
    const unsigned heights[]={120U,240U,480U},widths[]={188U,376U,752U};
    for(unsigned n=0U;n<3U;++n) {
        unsigned h=heights[n],w=widths[n];
        crossing(h,w,h*41U/100U,h*56U/100U);process(h,w);
        CHECK(result.count[0]==2U && result.count[1]==2U);
        CHECK(result.guide[RING_GUIDE_270_CONNECT].valid);check_guides(h,w);
    }
    /* Sparse impulses must not create extra corners or erase the true pair. */
    crossing(H,W,98U,132U);
    for(unsigned y=77U;y<160U;y+=13U) for(unsigned x=12U;x<W-8U;x+=43U)
        image[y*W+x]=(uint8_t)(255U-image[y*W+x]);
    process(H,W);CHECK(result.count[0]==2U && result.count[1]==2U);
    CHECK(result.guide[RING_GUIDE_270_CONNECT].valid);check_guides(H,W);

    /* A sharp tip on a separate white island must not borrow the ego road's
     * opposite edge across a thin black separator that sparse samples miss. */
    acute(H,W,125U,RING_CORNER_A);
    for(unsigned y=0U;y<H;++y) memset(image+y*W+150U,0,10U);
    ctx.center_near=211.5f;ctx.width_near=103.f;
    process(H,W);CHECK(result.count[0]==0U && result.count[1]==0U);check_no_guides();
    acute(H,W,125U,RING_CORNER_A);raw(H,W);
    memset(flags+72U,0,60U); /* caller rejects this far disconnected component */
    ring_corner_process(image,H,W,&ctx,&result);
    CHECK(result.count[0]==0U && result.count[1]==0U);check_no_guides();

    /* Smooth ordinary bends have no junction tip, even when their curvature
     * and apparent width change substantially over the private ROI. */
    for(unsigned direction=0U;direction<2U;++direction) for(unsigned strength=1U;strength<=5U;++strength) {
        road(H,W);memset(image,0,H*W);
        for(unsigned y=0U;y<H;++y) {
            float delta=(float)y-162.f;
            float offset=(direction ? 1.f : -1.f)*.002f*(float)strength*delta*delta;
            for(unsigned x=0U;x<W;++x)
                if((float)x>=109.f+offset && (float)x<=263.f+offset) image[y*W+x]=255U;
        }
        process(H,W);CHECK(result.count[0]==0U && result.count[1]==0U);check_no_guides();
    }

    /* A cropped vertex has no two-arm support; an all-white/black frame or
     * isolated little black square is not a complete road-junction event. */
    crossing(H,W,0U,75U);process(H,W);CHECK(result.count[0]==0U && result.count[1]==0U);
    road(H,W);memset(image,0,H*W);process(H,W);check_no_guides();
    memset(image,255,H*W);process(H,W);check_no_guides();
    road(H,W);for(unsigned y=112U;y<115U;++y) for(unsigned x=140U;x<143U;++x) image[y*W+x]=0U;
    process(H,W);CHECK(result.count[0]==0U && result.count[1]==0U);check_no_guides();

    /* Do not resurrect the discarded closed circular-block detector. */
    for(int radius=8;radius<=28;radius+=2) {
        road(H,W);
        for(unsigned y=80U;y<150U;++y) for(unsigned x=110U;x<180U;++x)
            if(((int)x-145)*((int)x-145)+((int)y-115)*((int)y-115)<radius*radius) image[y*W+x]=0U;
        process(H,W);
        if(result.count[0] || result.count[1]) fprintf(stderr,"circular false tips radius %d: %u/%u\n",radius,result.count[0],result.count[1]);
        CHECK(result.count[0]==0U && result.count[1]==0U);check_no_guides();
    }

    /* Dense interference exceeds the fixed work budget: fail ambiguous, not a
     * partial left-only observation that could misclassify a 270 as a 360. */
    road(H,W);for(unsigned y=75U;y<160U;++y) for(unsigned x=12U;x<W-12U;++x)
        image[y*W+x]=(uint8_t)(((x+y)&1U) ? 255U : 0U);
    process(H,W);CHECK(result.count[0]==3U && result.count[1]==3U);check_no_guides();

    ring_corner_process(NULL,H,W,&ctx,&result);check_no_guides();
    ring_corner_process(image,481U,W,&ctx,&result);check_no_guides();
    ring_corner_process(image,H,W,NULL,&result);check_no_guides();
    puts("PASS ring corner geometry: count sequence, mirror A/B, distinct B policies, native sizes, noise, crop rejection, bounded ambiguity, finite white-supported paths and immutable input");return 0;
}
