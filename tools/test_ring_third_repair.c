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
static void check_center(unsigned inner)
{
    ring_corner_guide_t *g=guide(inner);const ring_corner_line_t *l=&g->boundary[1U-inner];
    assert(g->valid && g->third_repair && l->valid && !g->boundary[inner].valid);
    assert(l->x1==g->corner_x && l->y1==g->corner_y);
    unsigned near_start=154U-(154U-g->active_top)/3U,count=0;
    for(unsigned i=0;i<16;++i) if(g->center_valid&(1U<<i)) {
        unsigned y=i*153U/15U,x=g->center[i];
        float offset=(inner ? 1.f : -1.f)*304.f*.30f;
        float baseline=at(l,(float)y)+offset;
        assert(y>=g->active_top && image[y*304+x]);
        if(inner || y>=near_start) assert(fabsf(x-baseline)<1.01f);
        else {
            assert(x<=baseline && baseline-x<=304.f*ring_third_left_center_shift+1.f);
            for(unsigned xx=x;xx<=(unsigned)baseline;++xx) assert(image[y*304+xx]);
        }
        if(i && !inner && (g->center_link_valid&(1U<<i))) {
            unsigned y0=(i-1U)*153U/15U,x0=g->center[i-1U];
            for(unsigned row=y0+1U;row<y;++row) {
                unsigned col=(x0*(y-row)+x*(row-y0))/(y-y0);
                assert(image[row*304+col]);
            }
        }
        ++count;
    }
    assert(count>=5);
}
static void prefer_near_tip(unsigned inner)
{
    memset(image,0,sizeof(image));
    /* The sharper upper notch cannot reach the roof. A weaker, nearer
     * corner on the same outside edge can form a complete repair. */
    for(int y=19;y<154;++y) {
        int upper=48-6*abs(y-36),lower=25-3*abs(y-94);
        int notch=upper>lower ? upper : lower;
        int right=285-(notch>0 ? notch : 0),left=y>=34 ? 20 : 210;
        for(int x=left;x<right;++x) image[y*304+(inner ? 303-x : x)]=255;
    }
    ring_third_left_lean=.30f;
    ring_third_left_center_shift=0.f;
    ring_corner_process(image,154,304,0,&frame);
    ring_corner_guide_t parallel=*guide(inner);
    ring_third_left_center_shift=.45f;
    ring_corner_process(image,154,304,0,&frame);
    assert(parallel.valid && guide(inner)->valid);
    assert(!memcmp(parallel.boundary,guide(inner)->boundary,sizeof(parallel.boundary)));
    assert(parallel.center_valid==guide(inner)->center_valid);
    assert(inner ? parallel.heading==guide(inner)->heading : guide(inner)->heading<parallel.heading);
    if(inner) assert(!memcmp(parallel.center,guide(inner)->center,sizeof(parallel.center)));
    check_center(inner);
    assert(guide(inner)->corner_y>=93.f && guide(inner)->corner_y<=96.f);
    assert(fabsf(guide(inner)->corner_x-(inner ? 41.f : 262.f))<=2.f);
    if(!inner) {
        const ring_corner_guide_t *g=guide(inner);
        const ring_corner_line_t *line=&g->boundary[1];
        unsigned bent=0;
        for(unsigned i=0;i<16;++i) if(g->center_valid&(1U<<i))
            if(at(line,(float)(i*153U/15U))-304.f*.30f-g->center[i]>4.f) ++bent;
        assert(bent>=2);
    }
}
int main(int argc,char **argv)
{
    setbuf(stdout,0);assert(argc==2);const char *names[]={"timing_p4","timing_p1","timing_p2"};
    prefer_near_tip(0);prefer_near_tip(1);
    puts("PASS the nearer usable corner wins on both sides");
    for(unsigned inner=0;inner<2;++inner) for(unsigned fixture=0;fixture<3;++fixture) {
        printf("CHECK %s mirror=%u\n",names[fixture],inner);
        load(argv[1],names[fixture],(int)inner);memcpy(saved,image,sizeof(image));
        ring_third_left_lean=0.f;ring_corner_process(image,154,304,0,&frame);
        ring_corner_line_t initial=guide(inner)->boundary[1U-inner];assert(initial.valid);
        ring_third_left_lean=.05f;ring_corner_process(image,154,304,0,&frame);check_center(inner);
        const ring_corner_line_t *l=&guide(inner)->boundary[1U-inner];
        assert(l->x1==initial.x1 && l->y1==initial.y1);
        assert(fabsf((slope(l)-slope(&initial))-(inner ? -1.f : 1.f)*.05f*304.f/154.f)<.0001f);
        memset(&tracker,0,sizeof(tracker));ring_corner_stabilize(image,0,&tracker,&frame);check_center(inner);
        float x=guide(inner)->corner_x,y=guide(inner)->corner_y;
        /* False competing detector: unchanged CURRENT pixels must keep the
         * same real mouth tip rather than translate by sixty pixels. */
        guide(inner)->corner_x+=inner ? -60.f : 60.f;
        guide(inner)->boundary[1U-inner].x0+=inner ? -60.f : 60.f;
        guide(inner)->boundary[1U-inner].x1+=inner ? -60.f : 60.f;
        ring_corner_stabilize(image,0,&tracker,&frame);puts("CHECK false-anchor recovery");check_center(inner);
        assert(fabsf(guide(inner)->corner_x-x)<8.f && fabsf(guide(inner)->corner_y-y)<8.f);
        /* Short raw-detector miss must recover from pixels, with no stored
         * old-frame coordinates blindly promoted into a current guide. */
        memset(guide(inner),0,sizeof(*guide(inner)));
        ring_corner_stabilize(image,0,&tracker,&frame);puts("CHECK missing-anchor recovery");check_center(inner);
        assert(fabsf(guide(inner)->corner_x-x)<8.f && fabsf(guide(inner)->corner_y-y)<8.f);
        for(unsigned n=0;n<100;++n) {
            ring_corner_process(image,154,304,0,&frame);
            ring_corner_stabilize(image,0,&tracker,&frame);check_center(inner);
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
            ring_corner_stabilize(image,0,&tracker,&frame);check_center(inner);
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
        ring_corner_stabilize(image,0,&tracker,&frame);check_center(inner);
        assert(fabsf(guide(inner)->corner_x-reference.corner_x)<.01f &&
               fabsf(guide(inner)->corner_y-reference.corner_y)<.01f);
        assert(!memcmp(guide(inner)->center,reference.center,sizeof(reference.center)));
        assert(guide(inner)->error==reference.error && guide(inner)->heading==reference.heading);
    }
    ring_third_left_lean=.05f;
    puts("PASS third-only: 600 stable + 720 translated frames, mirrored pivot, tunable lean, supported inward center, false-anchor rejection, pixel recovery, discarded-side immunity, immutable image");
    return 0;
}
