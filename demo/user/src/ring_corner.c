#include "ring_corner.h"
#include "vision_geometry.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

/* All limits are geometric support limits, not 90-degree element classifiers.
 * A smooth edge occupies half of a sampling circle; a black convex vertex
 * occupies a smaller, contiguous sector at several radii. Sampling the original
 * image recovers both arms even when a row-run selector discarded a branch. */
#define RC_MAX_ROWS 480U
#define RC_MAX_COLS 752U
#define RC_TOP_PERCENT VISION_SEARCH_START_PERCENT
#define RC_END_PERCENT VISION_SEARCH_END_PERCENT
#define RC_GUIDE_PERCENT VISION_TRACK_START_PERCENT
#define RC_NEAR_PERCENT VISION_NEAR_START_PERCENT
#define RC_DIRECTIONS 32U
#define RC_POOL 24U
#define RC_VERTEX_BUDGET 4096U
/* Direction samples are quantized to 11.25 degrees. Treat an arm within about
 * 5.7 degrees of transverse as belonging to the expected forward/backward
 * half-plane; otherwise one physical tip flickers between A/B and NONE. */
#define RC_DIRECTION_Y_TOLERANCE 0.10f

typedef struct { float x,y; } rc_vec_t;
typedef struct { ring_corner_point_t p; uint8_t side; } rc_candidate_t;
static const rc_vec_t rc_direction[RC_DIRECTIONS]={
    {1.f,0.f},{.9807853f,.1950903f},{.9238795f,.3826834f},{.8314696f,.5555702f},
    {.7071068f,.7071068f},{.5555702f,.8314696f},{.3826834f,.9238795f},{.1950903f,.9807853f},
    {0.f,1.f},{-.1950903f,.9807853f},{-.3826834f,.9238795f},{-.5555702f,.8314696f},
    {-.7071068f,.7071068f},{-.8314696f,.5555702f},{-.9238795f,.3826834f},{-.9807853f,.1950903f},
    {-1.f,0.f},{-.9807853f,-.1950903f},{-.9238795f,-.3826834f},{-.8314696f,-.5555702f},
    {-.7071068f,-.7071068f},{-.5555702f,-.8314696f},{-.3826834f,-.9238795f},{-.1950903f,-.9807853f},
    {0.f,-1.f},{.1950903f,-.9807853f},{.3826834f,-.9238795f},{.5555702f,-.8314696f},
    {.7071068f,-.7071068f},{.8314696f,-.5555702f},{.9238795f,-.3826834f},{.9807853f,-.1950903f}
};

static float rc_clamp(float v,float lo,float hi) {return fmaxf(lo,fminf(hi,v));}
static rc_vec_t rc_sub(rc_vec_t a,rc_vec_t b) {rc_vec_t r={a.x-b.x,a.y-b.y};return r;}
static float rc_cross(rc_vec_t a,rc_vec_t b) {return a.x*b.y-a.y*b.x;}
static float rc_length(rc_vec_t a) {return sqrtf(a.x*a.x+a.y*a.y);}
static rc_vec_t rc_unit(rc_vec_t a)
{
    float n=rc_length(a);if(n>.001f) {a.x/=n;a.y/=n;}return a;
}
static uint8_t rc_white(const uint8_t *image,unsigned cols,int x,int y)
{
    return (uint8_t)(image[(unsigned)y*cols+(unsigned)x]!=0U);
}
static uint8_t rc_black_vote(const uint8_t *image,unsigned cols,int x,int y)
{
    unsigned white=0U;
    for(int dy=-1;dy<=1;++dy) for(int dx=-1;dx<=1;++dx)
        white+=rc_white(image,cols,x+dx,y+dy);
    return (uint8_t)(white<5U);
}

/* Return the two boundary rays of one contiguous black sector. */
static uint8_t rc_sector(const uint8_t *image,unsigned cols,float x,float y,float radius,
                         rc_vec_t ray[2],unsigned *sector_size)
{
    uint8_t black[RC_DIRECTIONS];unsigned count=0U,starts=0U,start=0U;
    for(unsigned k=0U;k<RC_DIRECTIONS;++k) {
        int px=(int)lroundf(x+radius*rc_direction[k].x);
        int py=(int)lroundf(y+radius*rc_direction[k].y);
        black[k]=rc_black_vote(image,cols,px,py);count+=black[k];
    }
    if(count<3U || count>14U) return 0U;
    for(unsigned k=0U;k<RC_DIRECTIONS;++k)
        if(black[k] && !black[(k+RC_DIRECTIONS-1U)%RC_DIRECTIONS]) {start=k;++starts;}
    if(starts!=1U) return 0U;
    unsigned last=(start+count-1U)%RC_DIRECTIONS;
    rc_vec_t a=rc_direction[start],b=rc_direction[(start+RC_DIRECTIONS-1U)%RC_DIRECTIONS];
    rc_vec_t c=rc_direction[last],d=rc_direction[(last+1U)%RC_DIRECTIONS];
    a.x+=b.x;a.y+=b.y;c.x+=d.x;c.y+=d.y;
    ray[0]=rc_unit(a);ray[1]=rc_unit(c);*sector_size=count;
    return 1U;
}

static float rc_width(const ring_corner_context_t *ctx,unsigned cols,unsigned end,float y)
{
    float value=ctx->width_near+ctx->width_slope*(y-(float)(end-1U));
    return rc_clamp(value,(float)cols*.10f,(float)cols*.75f);
}
static float rc_center(const ring_corner_context_t *ctx,unsigned cols,unsigned end,float y)
{
    float value=ctx->model_ready ? ctx->center_near+ctx->center_slope*(y-(float)(end-1U)) :
        (float)(cols-1U)*.5f;
    return rc_clamp(value,1.f,(float)cols-2.f);
}

static uint8_t rc_vertex(const uint8_t *image,unsigned cols,float x,float y,float radius,
                         ring_corner_point_t *point,uint8_t tracked)
{
    rc_vec_t rays[3][2],arm[2],anchor[2];unsigned sizes[3];float residual=0.f;
    for(unsigned k=0U;k<3U;++k)
        if(!rc_sector(image,cols,x,y,radius*(float)(k+1U),rays[k],&sizes[k])) return 0U;
    unsigned maximum=sizes[0],minimum=sizes[0];
    for(unsigned k=1U;k<3U;++k) {if(sizes[k]>maximum) maximum=sizes[k];if(sizes[k]<minimum) minimum=sizes[k];}
    /* Smooth circular arcs change sector aperture with radius; a junction tip
     * retains two locally straight arms. Do not call every round island a tip. */
    if(maximum-minimum>(tracked ? 3U : 2U)) return 0U;
    for(unsigned side=0U;side<2U;++side) {
        rc_vec_t first={x+radius*rays[0][side].x,y+radius*rays[0][side].y};
        rc_vec_t middle={x+2.f*radius*rays[1][side].x,y+2.f*radius*rays[1][side].y};
        rc_vec_t last={x+3.f*radius*rays[2][side].x,y+3.f*radius*rays[2][side].y};
        arm[side]=rc_unit(rc_sub(last,first));anchor[side]=first;
        float error=fabsf(rc_cross(rc_sub(middle,first),arm[side]));
        if(error>radius*(tracked ? .35f : .25f)+.65f) return 0U;
        residual+=error;
    }
    float den=rc_cross(arm[0],arm[1]);
    if(fabsf(den)<.25f) return 0U;
    float distance=rc_cross(rc_sub(anchor[1],anchor[0]),arm[1])/den;
    rc_vec_t vertex={anchor[0].x+distance*arm[0].x,anchor[0].y+distance*arm[0].y};
    rc_vec_t seed={x,y};float shift=rc_length(rc_sub(vertex,seed));
    if(shift>radius*1.15f) return 0U;
    /* Two tangents on a curved island intersect OUTSIDE its actual contour.
     * A real sharp vertex must have a black support pixel at that intersection;
     * do not accept a circle merely because its local arc looks convex. */
    uint8_t vertex_black=0U;
    int vx=(int)lroundf(vertex.x),vy=(int)lroundf(vertex.y);
    for(int iy=vy-2;iy<=vy+2;++iy) for(int ix=vx-2;ix<=vx+2;++ix) {
        float ex=(float)ix-vertex.x,ey=(float)iy-vertex.y;
        if(ex*ex+ey*ey<=1.69f && !rc_white(image,cols,ix,iy)) vertex_black=1U;
    }
    if(!vertex_black) return 0U;
    unsigned main=fabsf(arm[0].y)>=fabsf(arm[1].y) ? 0U : 1U,branch=1U-main;
    if(fabsf(arm[main].y)<.65f) return 0U;
    point->x=vertex.x;point->y=vertex.y;
    point->main_dx=arm[main].x;point->main_dy=arm[main].y;
    point->branch_dx=arm[branch].x;point->branch_dy=arm[branch].y;
    /* Counts classify 270. A/B only classify directed forward/backward arms
     * for 360. During approach the second arm evolves from oblique to nearly
     * transverse, so a small quantization tolerance keeps one physical corner
     * in the same A/B class without using 90 degrees to classify ring type. */
    point->kind=RING_CORNER_NONE;
    if(arm[main].y>.45f && arm[branch].y>-RC_DIRECTION_Y_TOLERANCE)
        point->kind=RING_CORNER_A;
    if(arm[main].y<-.45f && arm[branch].y<RC_DIRECTION_Y_TOLERANCE)
        point->kind=RING_CORNER_B;
    point->quality=(uint8_t)rc_clamp(98.f-residual*5.f-shift*1.5f,0.f,100.f);
    return (uint8_t)(point->quality>=RING_CORNER_MIN_QUALITY);
}

/* This is association to the present ego road, not a count of every dark
 * connected component. The caller's flags come from near-seeded route_build.
 * Verify the whole short connection, not sparse samples that can jump across
 * a black separator into an unrelated white patch. Two-pixel seams match the
 * ordinary row-run tolerance; they are not new spatial route seeds. */
static uint8_t rc_associated(const uint8_t *image,unsigned cols,unsigned end,
                            const ring_corner_context_t *ctx,const ring_corner_point_t *p,
                            float radius,uint8_t *side)
{
    float center=rc_center(ctx,cols,end,p->y);
    int row=(int)lroundf(p->y);
    if(ctx->flags==NULL || ctx->flags[row]==0U) return 0U;
    if(ctx->flags!=NULL && ctx->left!=NULL && ctx->right!=NULL && (ctx->flags[row]&3U)==3U)
        center=((float)ctx->left[row]+(float)ctx->right[row])*.5f;
    float delta=center-p->x;
    if(fabsf(delta)<radius*1.7f || fabsf(delta)>(float)cols*.48f) return 0U;
    *side=(uint8_t)(delta<0.f);
    unsigned good=0U,total=0U,black_run=0U;
    int step=delta>0.f ? 1 : -1;
    int stop=(int)lroundf(center),start=(int)lroundf(p->x)+2*step;
    for(int px=start;step>0 ? px<=stop : px>=stop;px+=step) {
        if(px<1 || px>=(int)cols-1) return 0U;
        if(rc_white(image,cols,px,row)) {++good;black_run=0U;}
        else if(++black_run>2U) return 0U;
        ++total;
    }
    return (uint8_t)(total>0U && good*100U>=total*90U);
}

static void rc_add(rc_candidate_t pool[RC_POOL],unsigned *size,const ring_corner_point_t *point,
                   uint8_t side,float radius,uint8_t *overflow)
{
    for(unsigned k=0U;k<*size;++k) {
        float dx=pool[k].p.x-point->x,dy=pool[k].p.y-point->y;
        if(pool[k].side==side && dx*dx+dy*dy<radius*radius*6.25f) {
            if(point->quality>pool[k].p.quality) pool[k].p=*point;
            return;
        }
    }
    if(*size>=RC_POOL) {*overflow=1U;return;}
    pool[*size].p=*point;pool[*size].side=side;++*size;
}

static uint8_t rc_line_x(const ring_corner_line_t *line,float y,float *x)
{
    if(!line->valid || fabsf(line->y1-line->y0)<.5f ||
       y<fminf(line->y0,line->y1)-.5f || y>fmaxf(line->y0,line->y1)+.5f) return 0U;
    *x=line->x0+(y-line->y0)*(line->x1-line->x0)/(line->y1-line->y0);
    return (uint8_t)isfinite(*x);
}
static ring_corner_line_t rc_ray(float x,float y,float dx,float dy,unsigned top,unsigned end,unsigned cols)
{
    ring_corner_line_t line={0};
    if(fabsf(dy)<.12f || !isfinite(dx) || !isfinite(dy)) return line;
    float limit=dy<0.f ? ((float)top-y)/dy : ((float)(end-1U)-y)/dy;
    if(dx>0.f) limit=fminf(limit,((float)cols-2.f-x)/dx);
    else if(dx<0.f) limit=fminf(limit,(1.f-x)/dx);
    if(limit<3.f || y<(float)top || y>(float)(end-1U)) return line;
    line.x0=x;line.y0=y;line.x1=x+limit*dx;line.y1=y+limit*dy;line.valid=1U;
    return line;
}

static uint8_t rc_corridor_white(const uint8_t *image,unsigned cols,unsigned row,float left,float right)
{
    if(right-left<(float)cols/16.f) return 0U;
    int start=(int)ceilf(fmaxf(0.f,left+2.f)),end=(int)floorf(fminf((float)(cols-1U),right-2.f));
    unsigned white=0U,visible=0U,black_run=0U;
    for(int x=start;x<=end;++x) {
        if(rc_white(image,cols,x,(int)row)) {++white;black_run=0U;}
        else if(++black_run>2U) return 0U;
        ++visible;
    }
    int center=(int)lroundf((left+right)*.5f);
    return (uint8_t)(center>=1 && center<(int)cols-1 && visible>=7U &&
        (float)visible>=(right-left)*.55f &&
        white*100U>=visible*90U && rc_white(image,cols,center,(int)row));
}

/* Beyond a B2 patch, continue along the observed branch arm. The ordinary row
 * selector can still prefer the main street, so pairing its edge with our new
 * branch would silently bring steering back to the wrong road. The line only
 * predicts WHERE to search: returned edges must be genuine binary transitions
 * belonging to the same white run, never more invented line pixels. */
static uint8_t rc_real_arm(const uint8_t *image,unsigned cols,unsigned row,
                           const ring_corner_line_t *line,unsigned side,float width,
                           float *left,float *right,uint8_t *have_left,uint8_t *have_right)
{
    if(!line->valid || fabsf(line->y1-line->y0)<.5f) return 0U;
    float expected=line->x0+((float)row-line->y0)*(line->x1-line->x0)/(line->y1-line->y0);
    if(expected<1.f || expected>(float)(cols-2U)) return 0U;
    float best=width*.20f+3.f;uint8_t found=0U;
    for(unsigned cursor=0U;cursor<cols;) {
        while(cursor<cols && !image[row*cols+cursor]) ++cursor;
        if(cursor>=cols) break;
        unsigned a=cursor,b=cursor;
        while(cursor<cols && image[row*cols+cursor]) {b=cursor;++cursor;}
        uint8_t reliable=(uint8_t)(side==0U ? a>1U : b<cols-2U);
        if(!reliable || b-a<3U) continue;
        float edge=side==0U ? (float)a : (float)b,error=fabsf(edge-expected);
        if(error<best) {
            best=error;*left=(float)a;*right=(float)b;
            *have_left=(uint8_t)(a>1U);*have_right=(uint8_t)(b<cols-2U);found=1U;
        }
    }
    return found;
}

/* Build steering from the finite repaired boundaries and actual other edge.
 * Width fallback is allowed only with a learned model and white support. */
static float rc_finish(const uint8_t *image,unsigned rows,unsigned cols,
                       const ring_corner_context_t *ctx,ring_corner_guide_t *guide)
{
    unsigned start=rows*RC_GUIDE_PERCENT/100U,end=rows*RC_END_PERCENT/100U;
    unsigned span=end-start,zone=span/3U,valid=0U,near_count=0U,far_count=0U,gap=0U,repaired=0U;
    unsigned branch_side=1U-guide->corner_side;
    uint8_t branch=(uint8_t)(guide->corner_kind==RING_CORNER_B && guide->corner_side<2U &&
        guide->boundary[branch_side].valid && !guide->boundary[guide->corner_side].valid);
    uint8_t cropped_branch=0U;
    float sum=0.f,weights=0.f,near_sum=0.f,far_sum=0.f,previous=0.f,cost=0.f;
    float centers[RC_MAX_ROWS];uint8_t accepted[RC_MAX_ROWS];
    memset(accepted,0,sizeof(accepted));
    guide->valid=0U;guide->confidence=0U;guide->error=0;guide->heading=0;
    guide->center_valid=0U;guide->center_link_valid=0U;memset(guide->center,0,sizeof(guide->center));
    for(unsigned next=end;next>start;--next) {
        unsigned row=next-1U;float left=0.f,right=0.f;
        uint8_t have_left=rc_line_x(&guide->boundary[0],(float)row,&left);
        uint8_t have_right=rc_line_x(&guide->boundary[1],(float)row,&right);
        uint8_t uses_repair=(uint8_t)(have_left || have_right);
        float width=rc_width(ctx,cols,end,(float)row);
        uint8_t real_branch=0U;
        if(branch && !uses_repair) {
            if(ctx->flags==NULL || ctx->flags[row]==0U ||
               !rc_real_arm(image,cols,row,&guide->boundary[branch_side],branch_side,width,
                            &left,&right,&have_left,&have_right)) {++gap;continue;}
            real_branch=1U;
        }
        if(!real_branch && !have_left && ctx->flags!=NULL && ctx->left!=NULL && (ctx->flags[row]&1U)) {
            left=(float)ctx->left[row];have_left=1U;
        }
        if(!real_branch && !have_right && ctx->flags!=NULL && ctx->right!=NULL && (ctx->flags[row]&2U)) {
            right=(float)ctx->right[row];have_right=1U;
        }
        if(ctx->model_ready && have_left!=have_right) {
            if(have_left) {right=left+width;have_right=1U;}
            else {left=right-width;have_left=1U;}
        }
        float center=(left+right)*.5f;
        if(real_branch && valid>0U && have_left && have_right &&
           ((left<0.f && center<(float)cols*.05f) ||
            (right>(float)(cols-1U) && center>(float)cols*.95f))) cropped_branch=1U;
        if(!have_left || !have_right || right-left>(float)cols*.80f ||
           (ctx->model_ready && (right-left<width*.45f || right-left>width*1.8f)) ||
           !rc_corridor_white(image,cols,row,left,right)) {++gap;continue;}
        if(valid>0U && fabsf(center-previous)>width*.25f+3.f) {++gap;continue;}
        if(valid>0U && gap>3U) break;
        gap=0U;previous=center;centers[row]=center;accepted[row]=1U;
        float weight=row>=end-zone ? 3.f : (row<start+zone ? 1.f : 2.f);
        sum+=weight*center;weights+=weight;++valid;
        if(uses_repair) ++repaired;
        if(row>=end-zone) {near_sum+=center;++near_count;}
        if(row<start+zone) {far_sum+=center;++far_count;}
        cost+=fabsf(center-rc_center(ctx,cols,end,(float)row));
    }
    /* A correctly selected left/right branch may leave the camera before the
     * ordinary fixed far band. Use only the already observed continuous path
     * for heading, and only after an observed arm leads out of the image. A
     * black gap or a missing corner is not permission to shorten lookahead. */
    if(branch && cropped_branch && valid*100U>=span*55U && far_count*100U<zone*45U) {
        unsigned first=start,last=end-1U;
        while(first<end && !accepted[first]) ++first;
        while(last>first && !accepted[last]) --last;
        if(first<last) {
            unsigned local_zone=(last-first+1U)/3U;
            near_sum=0.f;far_sum=0.f;near_count=0U;far_count=0U;
            for(unsigned row=first;row<=last;++row) if(accepted[row]) {
                if(row>=last+1U-local_zone) {near_sum+=centers[row];++near_count;}
                if(row<first+local_zone) {far_sum+=centers[row];++far_count;}
            }
            zone=local_zone;
        }
    }
    if(valid*100U<span*55U || near_count*100U<zone*45U || far_count*100U<zone*45U ||
       weights<1.f || repaired<3U || repaired*12U<span) return 1.e9f;
    guide->error=(int16_t)lroundf(rc_clamp(sum/weights*2000.f/(float)(cols-1U)-1000.f,-1000.f,1000.f));
    guide->heading=(int16_t)lroundf(rc_clamp((far_sum/(float)far_count-near_sum/(float)near_count)*
                                           2000.f/(float)(cols-1U),-1000.f,1000.f));
    guide->confidence=(uint8_t)(valid*100U/span);guide->valid=1U;
    for(unsigned k=0U;k<RING_CORNER_CENTER_SAMPLES;++k) {
        unsigned row=start+k*(end-1U-start)/(RING_CORNER_CENTER_SAMPLES-1U);
        if(accepted[row]) {guide->center[k]=(uint16_t)lroundf(centers[row]);guide->center_valid|=(uint16_t)(1U<<k);}
        if(k>0U) {
            unsigned previous_row=start+(k-1U)*(end-1U-start)/(RING_CORNER_CENTER_SAMPLES-1U);
            uint8_t connected=1U;
            for(unsigned sample=previous_row;sample<=row;++sample) if(!accepted[sample]) connected=0U;
            if(connected) guide->center_link_valid|=(uint16_t)(1U<<k);
        }
    }
    return cost/(float)valid+(100.f-(float)guide->confidence);
}

static void rc_270(const uint8_t *image,unsigned rows,unsigned cols,const ring_corner_context_t *ctx,
                    ring_corner_frame_t *out)
{
    unsigned top=rows*RC_TOP_PERCENT/100U,end=rows*RC_END_PERCENT/100U;
    uint8_t count=out->count[0];
    if(count!=out->count[1] || (count!=1U && count!=2U)) return;
    unsigned index=count==1U ? RING_GUIDE_270_EXTEND : RING_GUIDE_270_CONNECT;
    ring_corner_guide_t *guide=&out->guide[index];
    for(unsigned side=0U;side<2U;++side) {
        const ring_corner_point_t *near=&out->point[side][0];
        if(count==1U) {
            float sign=near->main_dy>0.f ? -1.f : 1.f;
            guide->boundary[side]=rc_ray(near->x,near->y,sign*near->main_dx,sign*near->main_dy,top,end,cols);
        } else {
            const ring_corner_point_t *far=&out->point[side][1];
            if(near->y-far->y<(float)rows*.025f) return;
            guide->boundary[side]=(ring_corner_line_t){near->x,near->y,far->x,far->y,1U};
        }
    }
    if(!guide->boundary[0].valid || !guide->boundary[1].valid) return;
    guide->corner_side=0U;guide->corner_kind=out->point[0][0].kind;
    guide->corner_x=out->point[0][0].x;guide->corner_y=fmaxf(out->point[0][0].y,out->point[1][0].y);
    (void)rc_finish(image,rows,cols,ctx,guide);
}

static void rc_360(const uint8_t *image,unsigned rows,unsigned cols,const ring_corner_context_t *ctx,
                    ring_corner_frame_t *out)
{
    unsigned top=rows*RC_TOP_PERCENT/100U,end=rows*RC_END_PERCENT/100U;
    for(unsigned side=0U;side<2U;++side) {
        if(out->count[side]>=RING_CORNER_MAX_PER_SIDE) continue;
        for(unsigned k=0U;k<out->count[side];++k) {
            const ring_corner_point_t *p=&out->point[side][k];
            if(p->kind==RING_CORNER_NONE) continue;
            unsigned policies=p->kind==RING_CORNER_A ? 1U : 2U;
            for(unsigned policy=0U;policy<policies;++policy) {
                float dx=p->main_dx,dy=p->main_dy;
                unsigned index=RING_GUIDE_A_LEFT+side;
                if(p->kind==RING_CORNER_B) {
                    index=(policy==0U ? RING_GUIDE_B_BRANCH_LEFT : RING_GUIDE_B_MAIN_LEFT)+side;
                    if(policy==0U) {dx=p->branch_dx;dy=p->branch_dy;}
                }
                ring_corner_line_t line=rc_ray(p->x,p->y,-dx,-dy,top,end,cols);
                if(!line.valid) continue;
                ring_corner_guide_t best={0};
                /* A left-hand corner can produce the RIGHT boundary when its
                 * branch arm is extended through the junction. Both adjacent
                 * corridors may be white: nearest-center scoring alone would
                 * choose the wrong branch. Preserve the selected arm's road
                 * side, then reject rather than switch to the opposite route. */
                unsigned boundary_side=(p->kind==RING_CORNER_B && policy==0U) ? 1U-side : side;
                best.boundary[boundary_side]=line;best.corner_side=(uint8_t)side;best.corner_kind=p->kind;
                best.corner_x=p->x;best.corner_y=p->y;
                (void)rc_finish(image,rows,cols,ctx,&best);
                /* Keep the nearest valid occurrence of each class; state-machine
                 * event tracking prevents a persistent tip being consumed twice. */
                if(best.valid && (!out->guide[index].valid || p->y>out->guide[index].corner_y)) out->guide[index]=best;
            }
        }
    }
}

void ring_corner_process(const uint8_t *binary,uint16_t rows,uint16_t cols,
                         const ring_corner_context_t *context,ring_corner_frame_t *out)
{
    if(out==NULL) return;
    memset(out,0,sizeof(*out));out->rows=rows;out->cols=cols;
    if(binary==NULL || context==NULL || rows<60U || rows>RC_MAX_ROWS || cols<64U || cols>RC_MAX_COLS) return;
    if(context->model_ready && (!isfinite(context->width_near) || !isfinite(context->width_slope) ||
       !isfinite(context->center_near) || !isfinite(context->center_slope) || context->width_near<1.f)) return;
    unsigned top=(unsigned)rows*RC_TOP_PERCENT/100U,end=(unsigned)rows*RC_END_PERCENT/100U;
    unsigned radius=rows/48U;if(radius<3U) radius=3U;
    unsigned margin=radius*3U+2U;
    if(end-top<=margin*2U || (unsigned)cols<=margin*2U) return;
    out->detector_near_row=(uint16_t)(end-margin-1U);
    rc_candidate_t pool[RC_POOL];unsigned size=0U,evaluations=0U;uint8_t overflow=0U;
    for(unsigned row=top+margin;row+margin<end;++row) {
        for(unsigned col=margin;col+margin<cols;++col) {
            if(binary[row*(unsigned)cols+col] ||
               (!binary[row*(unsigned)cols+col-1U] && !binary[row*(unsigned)cols+col+1U] &&
                !binary[(row-1U)*(unsigned)cols+col] && !binary[(row+1U)*(unsigned)cols+col])) continue;
            if(++evaluations>RC_VERTEX_BUDGET) {
                out->count[0]=RING_CORNER_MAX_PER_SIDE;out->count[1]=RING_CORNER_MAX_PER_SIDE;return;
            }
            ring_corner_point_t p;uint8_t side;
            if(!rc_vertex(binary,cols,(float)col,(float)row,(float)radius,&p,0U)) continue;
            if(p.y<(float)(top+margin) || p.y>(float)(end-margin-1U) || p.x<(float)margin || p.x>(float)(cols-margin-1U)) continue;
            if(!rc_associated(binary,cols,end,context,&p,(float)radius,&side)) continue;
            rc_add(pool,&size,&p,side,(float)radius,&overflow);
        }
    }
    if(overflow) {out->count[0]=RING_CORNER_MAX_PER_SIDE;out->count[1]=RING_CORNER_MAX_PER_SIDE;return;}
    for(unsigned k=0U;k<size;++k) {
        unsigned side=pool[k].side,count=out->count[side];
        if(count<RING_CORNER_MAX_PER_SIDE) {
            out->point[side][count]=pool[k].p;++out->count[side];
        }
    }
    for(unsigned side=0U;side<2U;++side) for(unsigned a=0U;a<out->count[side];++a)
        for(unsigned b=a+1U;b<out->count[side];++b) if(out->point[side][b].y>out->point[side][a].y) {
            ring_corner_point_t p=out->point[side][a];out->point[side][a]=out->point[side][b];out->point[side][b]=p;
        }
    if(context->flags!=NULL && out->count[0]==0U && out->count[1]==0U) {
        unsigned paired=0U,near_paired=0U,near=(unsigned)rows*RC_NEAR_PERCENT/100U;
        for(unsigned row=top;row<end;++row) if((context->flags[row]&3U)==3U) {
            ++paired;if(row>=near) ++near_paired;
        }
        out->road_clear=(uint8_t)(paired*100U>=(end-top)*65U && near_paired*100U>=(end-near)*65U);
    }
    rc_270(binary,rows,cols,context,out);rc_360(binary,rows,cols,context,out);
}

void ring_corner_stabilize(const uint8_t *binary,const ring_corner_context_t *ctx,
                          ring_corner_tracker_t *tracker,ring_corner_frame_t *out)
{
    if(tracker==NULL || out==NULL) return;
    unsigned rows=out->rows,cols=out->cols;
    out->continued_mask=0U;
    if(binary==NULL || ctx==NULL || rows<60U || rows>RC_MAX_ROWS || cols<64U || cols>RC_MAX_COLS ||
       !ctx->model_ready || !isfinite(ctx->width_near) || !isfinite(ctx->width_slope) ||
       !isfinite(ctx->center_near) || !isfinite(ctx->center_slope) || ctx->width_near<1.f) {
        memset(tracker,0,sizeof(*tracker));return;
    }
    if(tracker->rows!=rows || tracker->cols!=cols) {
        memset(tracker,0,sizeof(*tracker));tracker->rows=(uint16_t)rows;tracker->cols=(uint16_t)cols;
    }
    /* Never turn bilateral/ambiguous quantity evidence into a single A. */
    if((out->count[0] && out->count[1]) || out->count[0]>=RING_CORNER_MAX_PER_SIDE ||
       out->count[1]>=RING_CORNER_MAX_PER_SIDE) {
        memset(tracker->valid,0,sizeof(tracker->valid));return;
    }
    unsigned radius=rows/48U;if(radius<3U) radius=3U;
    unsigned margin=radius*3U+2U,top=rows*RC_TOP_PERCENT/100U,end=rows*RC_END_PERCENT/100U;
    for(unsigned side=0U;side<2U;++side) {
        unsigned id=RING_GUIDE_A_LEFT+side;
        if(out->count[1U-side]) {tracker->valid[side]=0U;continue;}
        if(out->guide[id].valid) {
            tracker->valid[side]=0U;
            for(unsigned i=0U;i<out->count[side];++i) {
                const ring_corner_point_t *p=&out->point[side][i];
                if(p->kind==RING_CORNER_A && fabsf(p->x-out->guide[id].corner_x)<1.f &&
                   fabsf(p->y-out->guide[id].corner_y)<1.f) {
                    tracker->point[side]=*p;tracker->valid[side]=1U;tracker->misses[side]=0U;break;
                }
            }
            continue;
        }
        if(!tracker->valid[side]) continue;
        if(++tracker->misses[side]>RING_CORNER_REACQUIRE_FRAMES || out->count[side]>1U) {
            tracker->valid[side]=0U;continue;
        }
        const ring_corner_point_t old=tracker->point[side];
        if(out->count[side] && (out->point[side][0].kind==RING_CORNER_B ||
           fabsf(out->point[side][0].x-old.x)>8.f || fabsf(out->point[side][0].y-old.y)>8.f)) {
            tracker->valid[side]=0U;continue;
        }
        ring_corner_guide_t best={0};ring_corner_point_t best_point={0};float best_score=-1.e9f;
        /* Small bounded search: the old location only seeds current geometry.
         * Aperture/residual hysteresis is local; global discovery is unchanged. */
        for(int dy=-4;dy<=4;dy+=2) for(int dx=-4;dx<=4;dx+=2) {
            float x=old.x+(float)dx,y=old.y+(float)dy;
            if(x<(float)margin || x>(float)(cols-margin-1U) ||
               y<(float)(top+margin) || y>(float)(end-margin-1U)) continue;
            ring_corner_point_t p;uint8_t found_side;
            if(!rc_vertex(binary,cols,x,y,(float)radius,&p,1U) || p.kind!=RING_CORNER_A ||
               fabsf(p.x-old.x)>8.f || fabsf(p.y-old.y)>8.f ||
               p.main_dx*old.main_dx+p.main_dy*old.main_dy<.85f ||
               p.branch_dx*old.branch_dx+p.branch_dy*old.branch_dy<.70f ||
               !rc_associated(binary,cols,end,ctx,&p,(float)radius,&found_side) || found_side!=side) continue;
            ring_corner_guide_t g={0};
            g.boundary[side]=rc_ray(p.x,p.y,-p.main_dx,-p.main_dy,top,end,cols);
            g.corner_side=(uint8_t)side;g.corner_kind=p.kind;g.corner_x=p.x;g.corner_y=p.y;
            (void)rc_finish(binary,rows,cols,ctx,&g);
            if(!g.valid || g.confidence<70U) continue;
            float score=(float)p.quality-fabsf(p.x-old.x)-fabsf(p.y-old.y);
            if(score>best_score) {best=g;best_point=p;best_score=score;}
        }
        if(best.valid) {
            out->guide[id]=best;tracker->point[side]=best_point;
            out->continued_mask|=(uint8_t)(1U<<side);out->road_clear=0U;
        }
    }
}
