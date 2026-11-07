#include "ring_control.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* Do not invoke the Windows CRT abort handler or show a desktop error dialog. */
#undef assert
#define assert(test) do { if(!(test)) { \
    fprintf(stderr,"FAIL %s:%d: %s\n",__FILE__,__LINE__,#test);exit(EXIT_FAILURE); \
} } while(0)
static uint8_t image[154*304],saved[154*304];
static ring_corner_frame_t frame;
static ring_corner_tracker_t tracker;
static void load(const char *dir,const char *name,int mirror)
{
    char path[1024];snprintf(path,sizeof(path),"%s/%s.bin",dir,name);
    FILE *f=fopen(path,"rb");assert(f);assert(fread(image,1,sizeof(image),f)==sizeof(image));fclose(f);
    if(mirror) for(unsigned y=0;y<154;++y) for(unsigned x=0;x<152;++x) {
        uint8_t p=image[y*304+x];image[y*304+x]=image[y*304+303-x];image[y*304+303-x]=p;
    }
}
static ring_corner_guide_t *guide(unsigned inner)
{return &frame.guide[ring_pdf_guide(5,inner)];}
static float at(const ring_corner_line_t *l,float y)
{return l->x0+(l->x1-l->x0)*(y-l->y0)/(l->y1-l->y0);}
static float slope(const ring_corner_line_t *l)
{return (l->x1-l->x0)/(l->y1-l->y0);}
static void parallel(unsigned inner)
{
    ring_corner_guide_t *g=guide(inner);const ring_corner_line_t *l=&g->boundary[1U-inner];
    assert(g->valid && g->third_repair && l->valid && !g->boundary[inner].valid);
    assert(l->x1==g->corner_x && l->y1==g->corner_y);
    unsigned count=0;
    for(unsigned i=0;i<16;++i) if(g->center_valid&(1U<<i)) {
        unsigned y=i*153U/15U;float offset=(inner ? 1.f : -1.f)*304.f*.30f;
        assert(y>=g->active_top && fabsf(g->center[i]-(at(l,(float)y)+offset))<1.01f);
        assert(image[y*304+g->center[i]]);++count;
    }
    assert(count>=5);
}
int main(int argc,char **argv)
{
    setbuf(stdout,0);assert(argc==2);const char *names[]={"timing_p4","timing_p1","timing_p2"};
    for(unsigned inner=0;inner<2;++inner) for(unsigned fixture=0;fixture<3;++fixture) {
        printf("CHECK %s mirror=%u\n",names[fixture],inner);
        load(argv[1],names[fixture],(int)inner);memcpy(saved,image,sizeof(image));
        ring_third_left_lean=0.f;ring_corner_process(image,154,304,0,&frame);
        ring_corner_line_t initial=guide(inner)->boundary[1U-inner];assert(initial.valid);
        ring_third_left_lean=.05f;ring_corner_process(image,154,304,0,&frame);parallel(inner);
        const ring_corner_line_t *l=&guide(inner)->boundary[1U-inner];
        assert(l->x1==initial.x1 && l->y1==initial.y1);
        assert(fabsf((slope(l)-slope(&initial))-(inner ? -1.f : 1.f)*.05f*304.f/154.f)<.0001f);
        memset(&tracker,0,sizeof(tracker));ring_corner_stabilize(image,0,&tracker,&frame);parallel(inner);
        float x=guide(inner)->corner_x,y=guide(inner)->corner_y;
        /* False competing detector: unchanged CURRENT pixels must keep the
         * same real mouth tip rather than translate by sixty pixels. */
        guide(inner)->corner_x+=inner ? -60.f : 60.f;
        guide(inner)->boundary[1U-inner].x0+=inner ? -60.f : 60.f;
        guide(inner)->boundary[1U-inner].x1+=inner ? -60.f : 60.f;
        ring_corner_stabilize(image,0,&tracker,&frame);puts("CHECK false-anchor recovery");parallel(inner);
        assert(fabsf(guide(inner)->corner_x-x)<8.f && fabsf(guide(inner)->corner_y-y)<8.f);
        /* Short raw-detector miss must recover from pixels, with no stored
         * old-frame coordinates blindly promoted into a current guide. */
        memset(guide(inner),0,sizeof(*guide(inner)));
        ring_corner_stabilize(image,0,&tracker,&frame);puts("CHECK missing-anchor recovery");parallel(inner);
        assert(fabsf(guide(inner)->corner_x-x)<8.f && fabsf(guide(inner)->corner_y-y)<8.f);
        for(unsigned n=0;n<100;++n) {
            ring_corner_process(image,154,304,0,&frame);
            ring_corner_stabilize(image,0,&tracker,&frame);parallel(inner);
            assert(fabsf(guide(inner)->corner_x-x)<8.f && fabsf(guide(inner)->corner_y-y)<8.f);
        }
        assert(!memcmp(saved,image,sizeof(image)));
        for(unsigned n=0;n<120;++n) {
            int dx=(int)(n%5U)-2,dy=(int)(n%3U)-1;
            memset(image,0,sizeof(image));
            for(int yy=0;yy<154;++yy) for(int xx=0;xx<304;++xx)
                if(xx-dx>=0 && xx-dx<304 && yy-dy>=0 && yy-dy<154)
                    image[yy*304+xx]=saved[(yy-dy)*304+xx-dx];
            ring_corner_process(image,154,304,0,&frame);
            ring_corner_stabilize(image,0,&tracker,&frame);parallel(inner);
            assert(fabsf(guide(inner)->corner_x-x-dx)<=3.f &&
                   fabsf(guide(inner)->corner_y-y-dy)<=3.f);
        }
        memcpy(image,saved,sizeof(image));memset(&tracker,0,sizeof(tracker));
        ring_corner_process(image,154,304,0,&frame);
        ring_corner_stabilize(image,0,&tracker,&frame);
        ring_corner_guide_t reference=*guide(inner);
        const ring_corner_line_t *cut=&reference.boundary[1U-inner];
        for(unsigned yy=(unsigned)ceilf(y)+12U;yy<154;++yy) for(unsigned xx=0;xx<304;++xx)
            if(inner ? xx+3.f<at(cut,(float)yy) : xx>at(cut,(float)yy)+3.f) image[yy*304+xx]=0;
        ring_corner_process(image,154,304,0,&frame);
        ring_corner_stabilize(image,0,&tracker,&frame);parallel(inner);
        assert(fabsf(guide(inner)->corner_x-reference.corner_x)<.01f &&
               fabsf(guide(inner)->corner_y-reference.corner_y)<.01f);
        assert(!memcmp(guide(inner)->center,reference.center,sizeof(reference.center)));
        assert(guide(inner)->error==reference.error && guide(inner)->heading==reference.heading);
    }
    ring_third_left_lean=.05f;
    puts("PASS third-only: 600 stable + 720 translated frames, mirrored pivot, tunable lean, parallel center, false-anchor rejection, pixel recovery, discarded-side immunity, immutable image");
    return 0;
}
