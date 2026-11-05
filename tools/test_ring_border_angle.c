#include "ring_corner.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint8_t image[480U*752U],saved[480U*752U];
static ring_corner_frame_t frame;

static void detect(unsigned h,unsigned w)
{
    memcpy(saved,image,h*w);
    ring_corner_process(image,(uint16_t)h,(uint16_t)w,0,&frame);
    assert(!memcmp(saved,image,h*w));
}
static void lane(unsigned h,unsigned w,float slope,float curve)
{
    memset(image,0,h*w);
    for(unsigned y=0;y<h;++y) {
        float dy=(float)y-h*.725f;
        int left=(int)(w*.18f),right=(int)(w*.65f+slope*dy+curve*dy*dy+.5f);
        if(right>(int)w-1) right=(int)w-1;
        if(right>=left) memset(image+y*w+(unsigned)left,255,(unsigned)(right-left+1));
    }
}
static void mirror(unsigned h,unsigned w)
{
    for(unsigned y=0;y<h;++y) for(unsigned x=0;x<w/2U;++x) {
        uint8_t temp=image[y*w+x];image[y*w+x]=image[y*w+w-1U-x];image[y*w+w-1U-x]=temp;
    }
}
static void straight_tests(void)
{
    const unsigned sizes[][2]={{154,304},{240,376},{480,752}};
    for(unsigned i=0;i<3;++i) {
        unsigned h=sizes[i][0],w=sizes[i][1];
        lane(h,w,.577350269f,0.f);detect(h,w);
        assert(frame.border_angle_valid==3U);
        assert(abs(frame.border_angle10[1]-600)<=5);
        assert(frame.border_angle10[0]==900);
        int right=frame.border_angle10[1];
        mirror(h,w);detect(h,w);
        assert(frame.border_angle_valid==3U);
        assert(abs(frame.border_angle10[0]-1200)<=5);
        assert(abs(frame.border_angle10[0]+right-1800)<=1);
        assert(frame.border_angle10[1]==900);
        lane(h,w,0.f,0.f);detect(h,w);
        assert(frame.border_angle_valid==3U && frame.border_angle10[0]==900 && frame.border_angle10[1]==900);
    }
    puts("PASS actual binary borders: 60/120 mirror, 90 vertical, three sensor sizes, immutable image");
}
static void rejection_tests(void)
{
    lane(154,304,0.f,.03f);detect(154,304);
    assert(frame.border_angle_valid==1U && !frame.border_angle10[1]);
    /* Missing rows split the fit; separated short sections cannot be joined. */
    lane(154,304,0.f,0.f);
    for(unsigned y=84;y<=138;y+=5) memset(image+y*304,255,304);
    detect(154,304);assert(!frame.border_angle_valid);
    /* A jump divides an otherwise straight right border into two short runs. */
    lane(154,304,0.f,0.f);
    for(unsigned y=112;y<=138;++y) memset(image+y*304+198,255,20);
    detect(154,304);assert(frame.border_angle_valid==1U);
    memset(image,255,154*304);detect(154,304);
    assert(!frame.border_angle_valid && !frame.border_angle10[0] && !frame.border_angle10[1]);
    memset(image,0,154*304);detect(154,304);assert(!frame.border_angle_valid);
    puts("PASS curved residual, short disconnected segments, large jump, image edges and no borders rejected");
}
static void support_tests(void)
{
    /* Native h=154 requires span ceil(30.8)=31, i.e. at least 32 rows. */
    memset(image,255,154*304);
    for(unsigned y=84;y<=114;++y) {
        memset(image+y*304,0,304);memset(image+y*304+55,255,144);
    }
    detect(154,304);assert(!frame.border_angle_valid);
    memset(image+115*304,0,304);memset(image+115*304+55,255,144);
    detect(154,304);assert(frame.border_angle_valid==3U);
    /* The eight-point floor independently applies at the minimum height. */
    lane(20,32,0.f,0.f);detect(20,32);assert(frame.border_angle_valid==3U);
    memset(image+11*32,255,32);detect(20,32);assert(!frame.border_angle_valid);
    puts("PASS minimum span and point-count boundaries; invalid observations clear previous angles");
}
int main(void)
{
    straight_tests();rejection_tests();support_tests();
    return 0;
}
