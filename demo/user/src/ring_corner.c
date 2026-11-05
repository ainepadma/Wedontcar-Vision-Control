#include "ring_corner.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* Port of E:/八邻域_v2.0: mirrored 8-neighbour black/white transitions,
 * row borders, and slope/intercept gap filling. Coordinates and sums are
 * widened for the native 304x154 sensor; source input is never painted. */
#define LIMIT 2464U
static uint16_t edge[2][LIMIT][2],length[2];
static uint16_t b[2][480];
static uint8_t flags[480];
static const int8_t step[2][8][2]={
 {{0,1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,0},{1,1}},
 {{0,1},{1,1},{1,0},{1,-1},{0,-1},{-1,-1},{-1,0},{-1,1}}};
static float clamp(float x,float lo,float hi){return fmaxf(lo,fminf(x,hi));}
static int white(const uint8_t *im,unsigned h,unsigned w,int x,int y)
{
    if(x<=0 || y<=0 || x>=(int)w-1 || y>=(int)h-1) return 0;
    return im[(unsigned)y*w+(unsigned)x]!=0;
}
unsigned ring_pdf_guide(unsigned stage,unsigned side)
{return RING_GUIDE_PDF_BASE+(stage-1U)*2U+side;}
static void exit_white_band(const uint8_t *im,unsigned h,unsigned w,ring_corner_frame_t *f)
{
    unsigned run=0,required=(h*RING_EXIT_WHITE_BAND_PERCENT+99U)/100U;
    for(unsigned y=0;y<h;++y) {
        unsigned x=0;
        while(x<w && im[y*w+x]) ++x;
        run=x==w ? run+1U : 0U;
        if(run>=required) {
            f->exit_white_band=1;
            f->exit_white_top=(uint16_t)(y+1U-run);
            f->exit_white_bottom=(uint16_t)y;
        }
    }
}
static void trace(const uint8_t *im,unsigned h,unsigned w,unsigned side,int x,int y,ring_corner_frame_t *f)
{
    unsigned n=0;int px=-1,py=-1;
    while(n<LIMIT && y>1) {
        edge[side][n][0]=(uint16_t)x;edge[side][n][1]=(uint16_t)y;
        if(f->border_index[side][y]==UINT16_MAX) f->border_index[side][y]=(uint16_t)n;
        int nx=x,ny=y,best=(int)h+1;
        for(unsigned k=0;k<8;++k) {
            int ax=x+step[side][k][0],ay=y+step[side][k][1];
            int bx=x+step[side][(k+1U)&7U][0],by=y+step[side][(k+1U)&7U][1];
            if(ax<0 || ax>=(int)w || ay<1 || ay>=(int)h-1 || (ax==px && ay==py)) continue;
            if(!white(im,h,w,ax,ay) && white(im,h,w,bx,by) && ay<best) {nx=ax;ny=ay;best=ay;}
        }
        ++n;
        if((nx==x && ny==y) || (n>8 && nx==edge[side][n-4][0] && ny==edge[side][n-4][1])) break;
        px=x;py=y;x=nx;y=ny;
    }
    length[side]=(uint16_t)n;
}
static ring_corner_point_t point(float x,float y,unsigned kind)
{
    ring_corner_point_t p;memset(&p,0,sizeof(p));p.x=x;p.y=y;p.kind=(uint8_t)kind;p.quality=90;return p;
}
static int valid(unsigned side,unsigned y){return (flags[y]&(1U<<side))!=0;}
static void border_angles(unsigned h,unsigned w,ring_corner_frame_t *f)
{
    /* Fit only the longest uninterrupted real border in 55%-90% of the
     * native image. Missing/edge rows and large jumps split the support.
     * A high coverage score elsewhere must not make a curved fit valid. */
    unsigned first=h*55U/100U,last=h*90U/100U;
    unsigned min_span=(h*20U+99U)/100U,jump=w/40U;
    if(jump<2U) jump=2U;
    for(unsigned side=0;side<2U;++side) {
        unsigned start=first,count=0,best_start=0,best_count=0;
        for(unsigned y=first;y<=last;++y) {
            int supported=valid(side,y) && b[side][y]>0U && b[side][y]+1U<w;
            if(!supported) {count=0;continue;}
            if(!count || abs((int)b[side][y]-(int)b[side][y-1U])>(int)jump) {
                start=y;count=1U;
            } else ++count;
            if(count>=best_count) {best_start=start;best_count=count;}
        }
        if(best_count<8U || best_count-1U<min_span) continue;
        float n=(float)best_count,sy=0.f,sx=0.f,syy=0.f,sxy=0.f;
        /* Local y keeps the least-squares sums small at full sensor size. */
        for(unsigned i=0;i<best_count;++i) {
            float y=(float)i,x=(float)b[side][best_start+i];
            sy+=y;sx+=x;syy+=y*y;sxy+=y*x;
        }
        float den=n*syy-sy*sy;
        if(den<=0.f) continue;
        float k=(n*sxy-sy*sx)/den,c=(sx-k*sy)/n;
        float max_error=0.f,sum_error2=0.f;
        float tolerance=fminf(3.f,fmaxf(1.5f,w*.01f));
        for(unsigned i=0;i<best_count;++i) {
            float error=fabsf(b[side][best_start+i]-(k*i+c));
            max_error=fmaxf(max_error,error);sum_error2+=error*error;
        }
        if(max_error>tolerance || sum_error2>n*tolerance*tolerance*.25f) continue;
        float angle=atan2f(1.f,k)*572.95779513f;
        if(!isfinite(angle) || angle<0.f || angle>1800.f) continue;
        f->border_angle10[side]=(int16_t)(angle+.5f);
        f->border_angle_valid|=(uint8_t)(1U<<side);
    }
}
static void features(unsigned h,unsigned w,unsigned side,ring_corner_frame_t *f)
{
    unsigned q=0,good=0,total=0,near_good=0,jump=w/12U+2U,rad=h/20U+2U;
    float sign=side ? -1.f : 1.f;
    for(unsigned y=2;y+2<h;++y) {
        if(valid(side,y)) ++q;
        if(y>h*2U/3U && valid(side,y)) ++near_good;
        if(y>h/5U && y+1<h && valid(side,y) && valid(side,y+1)) {
            ++total;int d=(int)b[side][y]-(int)b[side][y+1];
            /* Both monotonicity and bounded neighbour difference (PDF 1.2). */
            if(abs(d)<=(int)(w/40U+2U) && sign*(float)d>=-2.f) ++good;
        }
        if(y<h/5U || y>h*9U/10U) continue;
        if(y+1<h && abs((int)b[side][y]-(int)b[side][y+1])>(int)jump) {
            f->gap[side]=1;
            if(!f->breakpoint[side].quality || y>f->breakpoint[side].y)
                f->breakpoint[side]=point(b[side][y+1],y+1,RING_CORNER_B);
            /* Lower A: row discontinuity maps into a bounded contour search. */
            unsigned idx=f->border_index[side][y+1];
            if(idx!=UINT16_MAX) {
                unsigned first=idx>24 ? idx-24 : 0,last=idx+25;
                if(last>length[side]) last=length[side];
                for(unsigned i=first+3;i+3<last;++i) {
                    int x=edge[side][i][0],yy=edge[side][i][1];
                    int x0=edge[side][i-3][0],y0=edge[side][i-3][1];
                    int x1=edge[side][i+3][0],y1=edge[side][i+3][1];
                    if(yy>=y0 && yy>=y1 && abs(x1-x0)>=3 && (y0<yy || y1<yy) &&
                       x>2 && x<(int)w-3 && yy>(int)h/3 &&
                       (!f->a[side].quality || yy>f->a[side].y))
                        f->a[side]=point(x,yy,RING_CORNER_A);
                }
            }
            /* A at the end of a near boundary can be exactly horizontal. */
            if(valid(side,y+1) && !valid(side,y) && y>h/3U)
                f->a[side]=point(b[side][y+1],y+1,RING_CORNER_A);
        }
        if(y>rad && y+rad<h && valid(side,y-rad) && valid(side,y) && valid(side,y+rad)) {
            float x=sign*b[side][y],up=sign*b[side][y-rad],down=sign*b[side][y+rad];
            if(x>up+1.f && x>down+1.f && (!f->p[side].quality || x>sign*f->p[side].x))
                f->p[side]=point(b[side][y],y,RING_CORNER_NONE);
        }
    }
    unsigned missing=0;
    for(unsigned y=h/5U;y<h*9U/10U;++y) {
        if(!valid(side,y)) {
            if(!missing && valid(side,y-1) && (!f->breakpoint[side].quality || y>f->breakpoint[side].y)) {
                f->gap[side]=1;f->breakpoint[side]=point(b[side][y-1],y-1,RING_CORNER_B);
            }
            ++missing;
        }
        else {
            if(missing>=h/14U && (!f->breakpoint[side].quality || y>f->breakpoint[side].y)) {f->gap[side]=1;f->breakpoint[side]=point(b[side][y],y,RING_CORNER_B);}
            missing=0;
        }
    }
    /* A is the inward lip of the LOWER boundary component, below a blank
     * interval. Search the row-to-contour mapped neighbourhood; the P belongs
     * to the separate upper island, not to this lip. */
    ring_corner_point_t contour_a=f->a[side];
    memset(&f->a[side],0,sizeof(f->a[side]));
    for(unsigned y=h/3U;y+4<h;++y) {
        if(!valid(side,y) || valid(side,y-1)) continue;
        unsigned bottom=y;
        while(bottom+1<h && valid(side,bottom+1)) ++bottom;
        if(bottom-y<h/15U || bottom<h*9U/10U) continue;
        unsigned best=y;
        for(unsigned yy=y;yy<=bottom;++yy) if(sign*b[side][yy]>sign*b[side][best]) best=yy;
        if(best+3<h && sign*((float)b[side][best]-b[side][h-2])>w*.015f)
            f->a[side]=point(b[side][best],best,RING_CORNER_A);
    }
    if(!f->a[side].quality && contour_a.quality) f->a[side]=contour_a;
    if(f->a[side].quality) {
        memset(&f->p[side],0,sizeof(f->p[side]));
        unsigned end=(unsigned)f->a[side].y;
        for(unsigned y=rad+1;y+rad<end;++y) {
            if(!valid(side,y-rad) || !valid(side,y) || !valid(side,y+rad)) continue;
            float x=sign*b[side][y];
            if(x>sign*b[side][y-rad]+1 && x>sign*b[side][y+rad]+1 &&
               (!f->p[side].quality || x>sign*f->p[side].x))
                f->p[side]=point(b[side][y],y,RING_CORNER_NONE);
        }
    }
    /* V: downward tip with contour arms on opposite x sides, not the rounded P. */
    unsigned r=h/30U+3U;
    for(unsigned i=r;i+r<length[side];++i) {
        float x=edge[side][i][0],y=edge[side][i][1];
        float dx0=(float)edge[side][i-r][0]-x,dy0=(float)edge[side][i-r][1]-y;
        float dx1=(float)edge[side][i+r][0]-x,dy1=(float)edge[side][i+r][1]-y;
        if(y<h*.20f || y>h*.88f || x<w*.08f || x>w*.92f) continue;
        if(dy0<=0 && dy1<=0 && dy0+dy1<-(float)r*.45f && dx0*dx1<0 && fabsf(dx0)+fabsf(dx1)>r*.6f)
            if(!f->v[side].quality || y>f->v[side].y) f->v[side]=point(x,y,RING_CORNER_B);
    }
    f->quality[side]=(uint8_t)(q*100U/(h-4U));
    f->continuous[side]=(uint8_t)(q*100U>=(h-4U)*70U && total && good*100U>=total*83U && q*100U>=(h-4U)*70U);
    f->bottom_open[side]=(uint8_t)(near_good*100U<(h/3U)*45U);
    f->count[side]=(uint8_t)(f->a[side].quality!=0);
    if(f->count[side]) f->point[side][0]=f->a[side];
}
/* Row-indexed coarse search, then local inward tip at each discontinuity.
 * Median support rejects LCD annotations/single-pixel staircases. Two arms
 * must have different slopes; a smooth bend cannot create a mouth pair. */
static float profile(unsigned side,int y,unsigned w)
{
    float values[5];
    for(int i=0;i<5;++i) values[i]=side ? (float)(w-1U-b[side][y+i-2]) : b[side][y+i-2];
    for(unsigned i=1;i<5;++i) for(unsigned j=i;j && values[j]<values[j-1];--j) {
        float t=values[j];values[j]=values[j-1];values[j-1]=t;
    }
    return values[2];
}
static void mouth(unsigned side,unsigned h,unsigned w,ring_corner_frame_t *f)
{
    int radius=(int)h/24+2,best=-1;float score=0;
    for(int y=(int)h/3;y<(int)(h*4U/5U);++y) {
        float before=profile(side,y-radius,w),here=profile(side,y,w),after=profile(side,y+radius,w);
        float turn=2*here-before-after;
        if(here-before>w*.04f && turn>w*.055f && turn>score) {score=turn;best=y;}
    }
    if(best<0) return;
    int lower=best;
    for(int y=best-radius/2;y<=best+radius/2;++y)
        if(profile(side,y,w)>profile(side,lower,w)) lower=y;
    int upper=-1;score=0;
    for(int y=radius+3;y<lower-radius*2;++y) {
        float here=profile(side,y,w),drop=here-profile(side,y+radius,w);
        if(here>w*.12f && drop>w*.05f && drop>score) {score=drop;upper=y;}
    }
    if(upper<0 || lower-upper<(int)h/10) return;
    float lx=profile(side,lower,w),ux=profile(side,upper,w);
    if(side) {lx=w-1-lx;ux=w-1-ux;}
    f->mouth_lower[side]=point(lx,lower,RING_CORNER_A);
    f->mouth_upper[side]=point(ux,upper,RING_CORNER_B);
}
static float line_x(const ring_corner_line_t *l,float y)
{return l->x0+(l->x1-l->x0)*(y-l->y0)/(l->y1-l->y0);}
static void segment(ring_corner_guide_t *g,unsigned side,float x0,float y0,float x1,float y1)
{
    if(fabsf(y1-y0)<2.f) return;
    g->boundary[side]=(ring_corner_line_t){x0,y0,x1,y1,1};
}
/* Source calculate_s_i: fit the intact ten-row section above the break,
 * then extrapolate only the missing section. uint32/float prevent overflow. */
static int extend(ring_corner_guide_t *g,unsigned side,unsigned h,unsigned w,const ring_corner_frame_t *f)
{
    if(!f->breakpoint[side].quality) return 0;
    int end=(int)f->breakpoint[side].y-(int)h/30-1,start=end-(int)h/12;
    if(start<2) start=2;
    unsigned near=0;
    for(unsigned y=h*2U/3U;y+1<h;++y) if(valid(side,y)) ++near;
    int approaching=near>h/4U;
    if(approaching) {start=(int)(h*4U/5U);end=(int)h-3;}
    float n=0,sy=0,sx=0,syy=0,sxy=0;
    for(int y=start;y<end;++y) if(valid(side,(unsigned)y)) {float x=b[side][y];++n;sy+=y;sx+=x;syy+=y*y;sxy+=y*x;}
    float den=n*syy-sy*sy;if(n<4 || den<1) return 0;
    float k=(n*sxy-sy*sx)/den,c=(sx-k*sy)/n;
    if(approaching) segment(g,side,clamp(k*2+c,0,w-1),2,clamp(k*start+c,0,w-1),start);
    else segment(g,side,clamp(k*end+c,0,w-1),end,clamp(k*(h-1)+c,0,w-1),h-1);
    return 1;
}
static void finish(const uint8_t *im,unsigned h,unsigned w,ring_corner_guide_t *g)
{
    float sum=0,weight=0,ns=0,fs=0;unsigned n=0,nc=0,fc=0;uint8_t linked=1;
    unsigned top=0,end=h;
    if(g->clip_upper) {
        top=h;
        for(unsigned side=0;side<2;++side) if(g->boundary[side].valid) {
            unsigned y=(unsigned)ceilf(fminf(g->boundary[side].y0,g->boundary[side].y1));
            if(y<top) top=y;
        }
    }
    if(top>=end || end-top<6U) return;
    g->active_top=(uint16_t)top;g->active_end=(uint16_t)end;
    unsigned span=end-top,zone=span/3U;
    unsigned near_start=g->clip_upper ? end-zone : h*2U/3U;
    unsigned far_end=g->clip_upper ? top+zone : h/3U;
    for(unsigned y=0;y<h;++y) {
        float l=b[0][y],r=b[1][y];int lv=valid(0,y),rv=valid(1,y);
        for(unsigned side=0;side<2;++side) {
            const ring_corner_line_t *s=&g->boundary[side];if(!s->valid) continue;
            float low=fminf(s->y0,s->y1),high=fmaxf(s->y0,s->y1);
            if(y+1>=low && y<=high+1) {if(side) {r=line_x(s,y);rv=1;}else {l=line_x(s,y);lv=1;}}
        }
        int supported=lv || rv;
        if(!lv && l<=w*.04f) {l=0;lv=1;}
        if(!rv && r>=w*.96f) {r=w-1;rv=1;}
        int ok=y>=top && supported && lv && rv && r-l>w*.05f && l>=0 && r<w;
        unsigned x=(unsigned)clamp((l+r)*.5f,0,w-1);
        if(ok && !im[y*w+x]) ok=0;
        unsigned index=0;
        while(index+1<RING_CORNER_CENTER_SAMPLES && (index+1)*(h-1)/(RING_CORNER_CENTER_SAMPLES-1)<=y) ++index;
        if(!ok) linked=0;
        if(y==index*(h-1)/(RING_CORNER_CENTER_SAMPLES-1)) {
            if(ok) {g->center[index]=(uint16_t)x;g->center_valid|=(uint16_t)(1U<<index);
                if(linked && index) g->center_link_valid|=(uint16_t)(1U<<index);}
            linked=(uint8_t)ok;
        }
        if(!ok) continue;
        float wt=y>=near_start ? 3.f : (y>=far_end ? 2.f : 1.f);
        sum+=x*wt;weight+=wt;++n;
        if(y>=near_start) {ns+=x;++nc;}if(y<far_end) {fs+=x;++fc;}
    }
    g->confidence=(uint8_t)(n*100U/span);
    if(n*100U<span*50U || nc<span/8U || fc<3U || !weight) return;
    g->error=(int16_t)clamp((sum/weight*2/(w-1)-1)*1000,-1000,1000);
    g->heading=(int16_t)clamp((fs/fc-ns/nc)*2000/(w-1),-1000,1000);g->valid=1;
}
static void exit_diagonal(const uint8_t *im,unsigned h,unsigned w,unsigned inner,
                          const ring_corner_frame_t *f,ring_corner_guide_t *g)
{
    if(!f->exit_white_band) return;
    unsigned side=1U-inner;
    const ring_corner_point_t *a=&f->a[side];
    float angle=RING_EXIT_LINE_ANGLE_DEG;
    if(a->kind!=RING_CORNER_A || a->quality<RING_CORNER_MIN_QUALITY || !isfinite(a->x) || !isfinite(a->y) ||
       a->x<0.f || a->x>w-1.f || a->y<0.f || a->y>h-1.f ||
       !isfinite(angle) || angle<=0.f || angle>=90.f) return;
    float slope=tanf(angle*0.01745329251994329577f);
    float x=a->x,y=a->y;
    /* Anchor is the actual opposite-side A, never the white-band row or an
     * image corner. Clip distance along the ray so the requested angle and
     * starting point both remain exact when the ray meets a side edge. */
    float span=fminf(y,(side ? x : w-1.f-x)/slope);
    if(span<2.f) return;
    segment(g,side,x,y,clamp(x+(side ? -span : span)*slope,0.f,w-1.f),y-span);
    g->clip_upper=1;g->corner_side=(uint8_t)side;g->corner_kind=RING_CORNER_A;
    g->corner_x=x;g->corner_y=y;
    finish(im,h,w,g);
}
/* Entry P needs a supported black component, not a tiny speck or the entire
 * background. Edge contact is independently configurable for partial islands. */
#define ISLAND_QUEUE 8192U
static uint32_t island_queue[ISLAND_QUEUE];
static uint8_t island_seen[(480U*752U+7U)/8U];
static int island_point(const uint8_t *im,unsigned h,unsigned w,unsigned side,const ring_corner_point_t *p)
{
    if(!p->quality) return 0;
    int x=(int)p->x,y=(int)p->y,dir=side ? 1 : -1;
    if(y<1 || y>=(int)h-1) return 0;
    x+=dir;
    for(unsigned n=0;n<5U && x>0 && x<(int)w-1 && im[(unsigned)y*w+(unsigned)x];++n) x+=dir;
    if(x<=0 || x>=(int)w-1 || im[(unsigned)y*w+(unsigned)x]) return 0;
    memset(island_seen,0,sizeof(island_seen));
    unsigned head=0,tail=1;uint32_t seed=(uint32_t)y*w+(unsigned)x;
    island_queue[0]=seed;island_seen[seed/8U]|=(uint8_t)(1U<<(seed%8U));
    unsigned min_y=(unsigned)y,max_y=(unsigned)y;
    while(head<tail) {
        uint32_t pos=island_queue[head++];unsigned cy=pos/w,cx=pos%w;
#if RING_ENTRY_REQUIRE_CLOSED_ISLAND
        if(cx==0 || cy==0 || cx+1==w || cy+1==h) return 0;
#endif
        if(cy<min_y) min_y=cy;
        if(cy>max_y) max_y=cy;
        for(int dy=-1;dy<=1;++dy) for(int dx=-1;dx<=1;++dx) {
            int nx=(int)cx+dx,ny=(int)cy+dy;
            if(nx<0 || ny<0 || nx>=(int)w || ny>=(int)h) continue;
            uint32_t next=(uint32_t)ny*w+(unsigned)nx;
            uint8_t bit=(uint8_t)(1U<<(next%8U));
            if(!im[next] && !(island_seen[next/8U]&bit)) {
                if(tail>=ISLAND_QUEUE) return 0;
                island_seen[next/8U]|=bit;island_queue[tail++]=next;
            }
        }
    }
    /* Preserve minimum area/height and bounded maximum area even when an
     * otherwise supported partial island touches the image edge. */
    return tail>=w*h/400U && max_y-min_y>=h/20U;
}
void ring_corner_process(const uint8_t *im,uint16_t h,uint16_t w,const ring_corner_context_t *ctx,ring_corner_frame_t *f)
{
    (void)ctx;if(!f) return;memset(f,0,sizeof(*f));
    if(!im || h<20 || h>480 || w<32 || w>752) return;
    f->rows=h;f->cols=w;f->detector_near_row=h-2;
    exit_white_band(im,h,w,f);
    memset(f->border_index,255,sizeof(f->border_index));memset(flags,0,sizeof(flags));
    float anchor=(w-1)*.5f;int seed[2]={0,w-1},seed_y=h-2;
    for(int y=h-2;y>=0;--y) {
        float best=1e9f;int bl=0,br=w-1,found=0;
        for(int x=0;x<w;) {
            while(x<w && !im[y*w+x]) ++x;
            int l=x;
            while(x<w && im[y*w+x]) ++x;
            int r=x-1;
            if(r-l<(int)w/25) continue;
            float dist=anchor<l ? l-anchor : (anchor>r ? anchor-r : 0);
            float score=dist*4+fabsf((l+r)*.5f-anchor)*.1f;
            if(score<best) {best=score;bl=l;br=r;found=1;}
        }
        b[0][y]=(uint16_t)bl;b[1][y]=(uint16_t)br;
        if(found) {
            if(bl>(int)(w/40U+1U)) flags[y]|=1;
            if(br<(int)(w-w/40U-2U)) flags[y]|=2;
            anchor=(bl+br)*.5f;
            if(y==h-2 || (seed_y==h-2 && !flags[h-2])) {seed[0]=bl>1 ? bl-1 : 0;seed[1]=br<w-2 ? br+1 : w-1;seed_y=y;}
        }
    }
    b[0][h-1]=b[0][h-2];b[1][h-1]=b[1][h-2];flags[h-1]=flags[h-2];
    border_angles(h,w,f);
    for(unsigned side=0;side<2;++side) {
        trace(im,h,w,side,seed[side],seed_y,f);features(h,w,side,f);
    }
    for(unsigned side=0;side<2;++side) {
        if(f->p[side].quality && (side ? f->p[side].x<w*.5f : f->p[side].x>w*.5f))
            memset(&f->p[side],0,sizeof(f->p[side]));
    }
    /* A contour may traverse the whole roof after the two tracers meet.
     * Assign its V tip by native lateral location, not the tracer of origin. */
    if(f->v[0].quality && f->v[0].x>w*.55f) {f->v[1]=f->v[0];memset(&f->v[0],0,sizeof(f->v[0]));}
    if(f->v[1].quality && f->v[1].x<w*.45f) {f->v[0]=f->v[1];memset(&f->v[1],0,sizeof(f->v[1]));}
    f->paired=(uint8_t)(!f->bottom_open[0] && !f->bottom_open[1]);
    f->road_clear=(uint8_t)(f->paired && !f->gap[0] && !f->gap[1] && f->quality[0]>65 && f->quality[1]>65);
    for(unsigned side=0;side<2;++side) {
        float ap_gap=f->a[side].y-f->p[side].y;
        f->entry[side]=(uint8_t)(f->gap[side] && f->a[side].quality && f->p[side].quality &&
            ap_gap>0.f && ap_gap*100.f>=h*RING_ENTRY_AP_MIN_GAP_PERCENT &&
            ap_gap*100.f<=h*RING_ENTRY_AP_MAX_GAP_PERCENT &&
            island_point(im,h,w,side,&f->p[side]) &&
            f->continuous[1-side] && !f->gap[1-side]);
    }
    for(unsigned side=0;side<2;++side) mouth(side,h,w,f);
    f->paired_mouth=(uint8_t)(f->mouth_lower[0].quality && f->mouth_lower[1].quality &&
        fabsf(f->mouth_lower[0].y-f->mouth_lower[1].y)<h*.20f &&
        fabsf(f->mouth_upper[0].y-f->mouth_upper[1].y)<h*.20f &&
        f->mouth_lower[0].x<f->mouth_lower[1].x && f->mouth_upper[0].x<f->mouth_upper[1].x);
    if(f->paired_mouth) {
        f->entry[0]=0;f->entry[1]=0;
        for(unsigned side=0;side<2;++side) {
            f->count[side]=2;f->point[side][0]=f->mouth_lower[side];f->point[side][1]=f->mouth_upper[side];
        }
    }
    f->cross_entry=(uint8_t)(f->paired_mouth || (f->bottom_open[0] && f->bottom_open[1] && f->gap[0] && f->gap[1] && !f->entry[0] && !f->entry[1]));
    ring_corner_guide_t *cross=&f->guide[RING_GUIDE_270_EXTEND];
    if(f->cross_entry && extend(cross,0,h,w,f) && extend(cross,1,h,w,f)) finish(im,h,w,cross);
    ring_corner_guide_t *connected=&f->guide[RING_GUIDE_270_CONNECT];
    if(f->paired_mouth) {
        for(unsigned side=0;side<2;++side) {
            ring_corner_point_t u=f->mouth_upper[side],l=f->mouth_lower[side];
            segment(connected,side,u.x,u.y,l.x,l.y);
        }
        finish(im,h,w,connected);
    }
    if(connected->valid) *cross=*connected;
    for(unsigned side=0;side<2;++side) for(unsigned stage=1;stage<=7;++stage) {
        unsigned other=1-side;float outer=side ? w-1.f : 0.f;
        ring_corner_guide_t *g=&f->guide[ring_pdf_guide(stage,side)];
        ring_corner_point_t a=f->a[side],p=f->p[side],v=f->v[side],ov=f->v[other],bp=f->breakpoint[other];
        g->corner_side=(uint8_t)side;g->corner_x=a.x;g->corner_y=a.y;
        if(stage==1 && a.quality && p.quality) segment(g,side,p.x,p.y,a.x,a.y);
        if(stage==2 && p.quality) segment(g,side,p.x,p.y,outer,h-1);
        if(stage==3 && v.quality) segment(g,other,v.x,v.y,b[other][h-2],h-1);
        if(stage==4 && ov.quality) segment(g,other,ov.x,ov.y,side ? 0.f : w-1.f,h-1);
        if(stage==5 && bp.quality) {
            /* User's exit sketch: close the local mouth from its lower outer
             * corner to the REAL upper cross-boundary, not an image corner.
             * Probe one quarter of the near lane width inward, then search
             * upward for a supported black roof (three consecutive pixels).
             * The mirrored construction serves a clockwise/right ring. */
            float lane=clamp((float)b[1][h-2]-b[0][h-2],w*.20f,w*.80f);
            int x=(int)clamp(bp.x+(side ? 1.f : -1.f)*lane*.25f,2,w-3);
            unsigned black=0;int roof=-1;
            for(int y=(int)bp.y-3;y>1;--y) {
                if(!im[(unsigned)y*w+(unsigned)x]) ++black;else black=0;
                if(black>=3) {roof=y+3;break;}
            }
            if(roof>1 && roof<bp.y-3) segment(g,other,x,roof,bp.x,bp.y);
        }
        if(stage==6 || stage==7) {exit_diagonal(im,h,w,side,f,g);continue;}
        if(stage==3 || stage==4) g->clip_upper=1;
        if(stage==6 || stage==5 || g->boundary[0].valid || g->boundary[1].valid) finish(im,h,w,g);
    }
    /* Preview starts at the recognized entry, never guesses a motion stage. */
    for(unsigned side=0;side<2;++side) f->guide[RING_GUIDE_A_LEFT+side]=f->guide[ring_pdf_guide(1,side)];
    memcpy(f->border,b,sizeof(b));memcpy(f->border_flags,flags,sizeof(flags));
}
static int black_patch(const uint8_t *im,unsigned cols,int x,int y)
{
    unsigned n=0;
    for(int dy=-1;dy<=1;++dy) for(int dx=-1;dx<=1;++dx) if(!im[(unsigned)(y+dy)*cols+(unsigned)(x+dx)]) ++n;
    return n>=5;
}
static int reacquire(const uint8_t *im,unsigned h,unsigned w,unsigned side,int upper,
                     const ring_corner_point_t *old,ring_corner_point_t *found)
{
    float best=1e9f;int sign=side ? -1 : 1,radius=4;
    for(int dy=-5;dy<=5;++dy) for(int dx=-7;dx<=7;++dx) {
        int x=(int)old->x+dx,y=(int)old->y+dy;
        if(x<6 || y<6 || x>=(int)w-6 || y>=(int)h-6) continue;
        int inward=x+sign*radius,outward=x-sign*radius;
        int by=y+(upper ? -radius : radius),wy=y+(upper ? radius : -radius);
        if(!black_patch(im,w,outward,by) || black_patch(im,w,inward,by) ||
           black_patch(im,w,outward,wy) || black_patch(im,w,inward,wy)) continue;
        float score=(float)(dx*dx+dy*dy);
        if(score<best) {best=score;*found=point(x,y,upper ? RING_CORNER_B : RING_CORNER_A);}
    }
    return best<1e9f;
}
void ring_corner_stabilize(const uint8_t *im,const ring_corner_context_t *ctx,ring_corner_tracker_t *s,ring_corner_frame_t *f)
{
    (void)ctx;if(!s || !f || !im || !f->rows || !f->cols) return;
    if(s->rows!=f->rows || s->cols!=f->cols) {memset(s,0,sizeof(*s));s->rows=f->rows;s->cols=f->cols;}
    if(!f->paired_mouth && s->valid[0] && s->valid[1]) {
        ring_corner_point_t lower[2],upper[2];int ok=1;
        for(unsigned side=0;side<2;++side) {
            if(!reacquire(im,f->rows,f->cols,side,0,&s->point[side],&lower[side]) ||
               !reacquire(im,f->rows,f->cols,side,1,&s->upper[side],&upper[side])) ok=0;
        }
        if(ok) {
            ring_corner_guide_t *g=&f->guide[RING_GUIDE_270_CONNECT];memset(g,0,sizeof(*g));
            for(unsigned side=0;side<2;++side) segment(g,side,upper[side].x,upper[side].y,lower[side].x,lower[side].y);
            finish(im,f->rows,f->cols,g);
            if(g->valid) {
                f->paired_mouth=1;f->cross_entry=1;f->entry[0]=0;f->entry[1]=0;
                for(unsigned side=0;side<2;++side) {f->mouth_lower[side]=lower[side];f->mouth_upper[side]=upper[side];}
                f->guide[RING_GUIDE_270_EXTEND]=*g;
            }
        }
    }
    /* Reacquire a temporarily missed V from CURRENT contour samples. Smaller
     * support is allowed only inside the previous vertex neighbourhood; a
     * rounded ordinary bend cannot establish a new V through this tracker. */
    for(unsigned side=0;side<2;++side) {
        if(!f->v[side].quality && s->v_valid[side]) {
            float best=1e9f;ring_corner_point_t vertex;memset(&vertex,0,sizeof(vertex));
            for(unsigned i=3;i+3<length[side];++i) {
                int x=edge[side][i][0],y=edge[side][i][1];
                float dx=x-s->v[side].x,dy=y-s->v[side].y;
                if(fabsf(dx)>f->cols*.08f || fabsf(dy)>f->rows*.12f || x<5 || x>=f->cols-5 || y<5 || y>=f->rows-5) continue;
                int x0=edge[side][i-3][0],y0=edge[side][i-3][1],x1=edge[side][i+3][0],y1=edge[side][i+3][1];
                if(y0>y || y1>y || y0+y1>2*y-2 || (x0-x)*(x1-x)>=0) continue;
                if(!black_patch(im,f->cols,x,y-3) || black_patch(im,f->cols,x,y+3)) continue;
                float distance=dx*dx+dy*dy;
                if(distance<best) {best=distance;vertex=point(x,y,RING_CORNER_B);}
            }
            if(vertex.quality) {
                f->v[side]=vertex;f->v_reacquired|=(uint8_t)(1U<<side);
                unsigned other=1U-side;
                ring_corner_guide_t *g=&f->guide[ring_pdf_guide(3,side)];memset(g,0,sizeof(*g));
                g->clip_upper=1;segment(g,other,vertex.x,vertex.y,b[other][f->rows-2],f->rows-1);finish(im,f->rows,f->cols,g);
                g=&f->guide[ring_pdf_guide(4,other)];memset(g,0,sizeof(*g));
                g->clip_upper=1;segment(g,side,vertex.x,vertex.y,side ? f->cols-1.f : 0.f,f->rows-1);finish(im,f->rows,f->cols,g);
                /* S7 is white-band based; a recovered V cannot replace its
                 * opposite-side diagonal with the obsolete inner-side line. */
            }
        }
        if(f->v[side].quality) {s->v[side]=f->v[side];s->v_valid[side]=1;s->v_misses[side]=0;}
        else if(s->v_misses[side]<3) ++s->v_misses[side];else s->v_valid[side]=0;
    }
    for(unsigned side=0;side<2;++side) {
        if(f->paired_mouth && f->guide[RING_GUIDE_270_CONNECT].valid) {
            s->point[side]=f->mouth_lower[side];s->upper[side]=f->mouth_upper[side];s->valid[side]=1;s->misses[side]=0;
        } else if(s->misses[side]<3) ++s->misses[side];else s->valid[side]=0;
    }
}
