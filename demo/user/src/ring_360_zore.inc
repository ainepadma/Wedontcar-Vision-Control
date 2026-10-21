/* SPDX-License-Identifier: GPL-3.0-or-later
 * ZORE ring rules adapted to the CURRENT native raw boundaries and binary.
 * No image buffer, resize, resampling, extra binarization, or pixel filtering.
 * Reference row/count/offset parameters are converted at their point of use.
 * Never writes binary, raw_left/right/flags, ordinary center or 270 features.
 */
typedef struct {
    int16_t left,right,center,first[2],last[2];
    char found[2];
} z360_native_row_t;
static z360_native_row_t z360_rows[TRACK_VISION_MAX_ROWS],z360_saved[TRACK_VISION_MAX_ROWS];
static const uint8_t *z360_binary;
static unsigned z360_h,z360_w;
static int z360_top,z360_end,z360_off,z360_lost[2],z360_white;
static float z360_sy,z360_width_near,z360_width_slope;
static uint8_t z360_width_ready,z360_active_hint;
static int8_t z360_active_direction;
static unsigned z360_previous_h,z360_previous_w;
static const int z360_reference_half[60]={2,2,2,2,3,3,3,3,4,4,4,4,4,5,5,5,5,5,5,6,
    7,7,7,8,8,8,8,8,8,8,9,9,9,9,10,10,11,11,12,13,14,15,16,17,18,19,20,20,21,21,
    21,22,22,23,23,23,24,24,25,25};
void track_ring_360_set_active(uint8_t stage,int8_t direction)
{
    if(!RING_FEATURE_ENABLED) {z360_active_hint=0U;z360_active_direction=0;return;}
    z360_active_hint=stage;z360_active_direction=direction;
}
static int z360_limit(int x,int lo,int hi) {return x<lo ? lo : (x>hi ? hi : x);}
static int z360_y(float reference_row)
{
    return z360_top+(int)lroundf(reference_row*z360_sy);
}
static int z360_step(float reference_rows)
{
    return reference_rows<=0.f ? 0 : (int)fmaxf(1.f,roundf(reference_rows*z360_sy));
}
static float z360_width(int y)
{
    float w=z360_width_near+z360_width_slope*((float)y-(float)(z360_end-1));
    return fmaxf((float)z360_w/16.f,fminf(w,(float)z360_w*0.65f));
}
static float z360_sx(int y)
{
    float r=((float)y-(float)z360_top)/z360_sy;
    int a=z360_limit((int)floorf(r),0,59),b=a<59 ? a+1 : a;
    float t=fmaxf(0.f,fminf(r-(float)a,1.f));
    float half=(float)z360_reference_half[a]+t*(float)(z360_reference_half[b]-z360_reference_half[a]);
    return z360_width(y)/(2.f*half);
}
static int z360_dx(float reference_pixels,int y)
{
    return (int)lroundf(reference_pixels*z360_sx(y));
}
static float z360_center_x(void)
{
    return (float)(z360_w-1U)*RING_360_STEER_CENTER_PERCENT/100.f;
}
static int z360_x(float reference_column,int y)
{
    return (int)lroundf(z360_center_x()+(reference_column-37.f)*z360_sx(y));
}
static int z360_edge(unsigned side,int y)
{
    if(y<z360_top || y>=z360_end) return side==0U ? 0 : (int)z360_w-1;
    return side==0U ? z360_rows[y].left : z360_rows[y].right;
}
static char z360_found(unsigned side,int y)
{
    return y>=z360_top && y<z360_end ? z360_rows[y].found[side] : 'F';
}
static int z360_px(int y,int x)
{
    if(y<z360_top || y>=z360_end || x<0 || x>=(int)z360_w) return 0;
    return z360_binary[(unsigned)y*z360_w+(unsigned)x]!=0U && route_member((unsigned)y,(unsigned)x);
}
static float z360_variance(unsigned side,int start,int end)
{
    if(start==end) return 9999.f;
    if(start>end) return 0.f; /* Original Straight_Judge empty loop. */
    start=z360_limit(start,z360_top,z360_end-1);end=z360_limit(end,z360_top,z360_end-1);
    if(end<=start) return 9999.f;
    float a=(float)z360_edge(side,start),slope=((float)z360_edge(side,end)-a)/(float)(end-start),sum=0.f;
    for(int y=start;y<end;++y) {
        float error=floorf((a+slope*(float)(y-start)-(float)z360_edge(side,y))*16.f);
        sum+=error*error;
    }
    return sum/(float)(end-start);
}
static float z360_variance_limit(float reference_limit,int start,int end)
{
    float scale=z360_sx((start+end)/2);
    return reference_limit*scale*scale;
}
static void z360_boundary(void)
{
    static const int ay[4]={-1,0,1,0},ax[4]={0,1,0,-1};
    static const int ly[4]={-1,-1,1,1},lx[4]={-1,1,1,-1};
    static const int ry[4]={-1,1,1,-1},rx[4]={1,1,-1,-1};
    int bottom=z360_limit(z360_y(58),z360_top,z360_end-1);
    int x[2]={z360_edge(0U,bottom),z360_edge(1U,bottom)},y[2]={bottom,bottom},d[2]={0,0};
    int py[2]={bottom,bottom},px[2]={x[0],x[1]},row=bottom;
    int limit=(int)ceilf(400.f*fmaxf((float)z360_w/80.f,z360_sy));
    for(int steps=0;steps<limit;++steps) {
        if(row>=py[0] && row>=py[1]) {if(row<z360_y(5)) break;--row;}
        for(unsigned side=0U;side<2U;++side) if(py[side]>row || row==z360_y(5)) {
            int dir=d[side];py[side]=y[side]+ay[dir];px[side]=x[side]+ax[dir];
            if(!z360_px(py[side],px[side])) {d[side]=(dir+(side==0U ? 1 : 3))%4;continue;}
            int ny=y[side]+(side==0U ? ly[dir] : ry[dir]),nx=x[side]+(side==0U ? lx[dir] : rx[dir]);
            py[side]=ny;px[side]=nx;
            int diagonal=z360_px(ny,nx);
            if(diagonal) {y[side]=ny;x[side]=nx;d[side]=(dir+(side==0U ? 3 : 1))%4;}
            else {y[side]+=ay[dir];x[side]+=ax[dir];}
            if(y[side]<z360_top || y[side]>=z360_end || x[side]<0 || x[side]>=(int)z360_w) return;
            z360_native_row_t *r=&z360_rows[y[side]];
            if(r->first[side]==(side==0U ? 0 : (int)z360_w-1)) {r->first[side]=x[side];if(!diagonal) r->last[side]=x[side];}
            if(diagonal) r->last[side]=x[side];
        }
        if(abs(px[1]-px[0])<z360_dx(3.f,row)) break;
    }
}
static int z360_admit(unsigned side,int *p1,int *p2)
{
    unsigned outer=1U-side;
    int narrow=0;
    float safe=(float)RING_360_SAFE_WIDTH*z360_width_near/50.f;
    for(int y=z360_y(side==0U ? 57.f : 59.f);y>=z360_y(54.f);--y)
        if(z360_found(0U,y)=='T' && z360_found(1U,y)=='T' && (float)(z360_edge(1U,y)-z360_edge(0U,y))<safe) ++narrow;
    if((float)narrow>=(side==0U ? 3.f : 2.f)*z360_sy) return 0;
    float max_outer=side==0U ? RING_360_LEFT_OUTER_LOSS_MAX : RING_360_RIGHT_OUTER_LOSS_MAX;
    float min_loss=side==0U ? RING_360_ZORE_LEFT_LOSS : RING_360_ZORE_RIGHT_LOSS;
    float max_variance=side==0U ? RING_360_ZORE_RIGHT_VARIANCE : RING_360_ZORE_LEFT_VARIANCE;
    int a=z360_y(25),b=z360_y(45);
    if((float)z360_lost[outer]>max_outer*z360_sy || (float)z360_lost[side]<min_loss*z360_sy ||
       z360_off>z360_y(RING_360_OFFLINE_MAX) || (float)z360_white>RING_360_BOTH_LOSS_MAX*z360_sy ||
       z360_variance(outer,a,b)>z360_variance_limit(max_variance,a,b)) return 0;
    *p1=*p2=0;int step=z360_step(1);
    for(int y=z360_y(58);y>z360_y(RING_360_CORNER_TOP);--y) {
        if(y-step<z360_top || y>=z360_end) continue;
        int delta=side==0U ? z360_rows[y].first[side]-z360_rows[y-step].first[side] : z360_rows[y-step].first[side]-z360_rows[y].first[side];
        if(delta>z360_dx(RING_360_CORNER_JUMP,y)) {*p1=y;break;}
    }
    for(int y=z360_y(58);y>z360_y(RING_360_CORNER_TOP);--y) {
        if(y+step>=z360_end) continue;
        int delta=side==0U ? z360_rows[y+step].last[side]-z360_rows[y].last[side] : z360_rows[y].last[side]-z360_rows[y+step].last[side];
        if(delta>z360_dx(RING_360_CORNER_JUMP,y)) {*p2=y;break;}
    }
    int help=0;
    for(int y=*p1;y>(side==0U ? z360_off : z360_y(10));--y) {
        if(y+z360_step(6)>=z360_end) continue;
        int aa=z360_edge(side,y+z360_step(6)),bb=z360_edge(side,y+z360_step(5)),c=z360_edge(side,y+z360_step(3));
        int dd=z360_edge(side,y+z360_step(2)),e=z360_edge(side,y+step);
        if(side==0U ? (aa<c && bb<c && c>dd && c>e) : (aa>c && bb>c && c<dd && c<e)) {help=1;break;}
    }
    int gap=z360_step(side==0U ? RING_360_LEFT_CORNER_GAP : RING_360_RIGHT_CORNER_GAP);
    if(*p2>*p1+gap && !help && (float)z360_lost[side]>(side==0U ? RING_360_LEFT_HELP_LOSS : RING_360_RIGHT_HELP_LOSS)*z360_sy) help=1;
    return *p2>*p1+gap && help;
}
static void z360_route_filter(void)
{
    int step=z360_step(1);
    for(int y=z360_y(58);y>=z360_off+z360_step(5);--y) {
        if(z360_found(0U,y)=='W' && z360_found(1U,y)=='W' && y<=z360_y(45) &&
           z360_found(0U,y-step)=='W' && z360_found(1U,y-step)=='W') {
            for(int p=y-1;p>=z360_off+z360_step(5);--p) if(z360_found(0U,p)=='T' && z360_found(1U,p)=='T') {
                int a=z360_limit(y+z360_step(2),z360_top,z360_end-1),b=z360_limit(p-step,z360_top,z360_end-1);
                if(a!=b) {
                    int ca=z360_rows[a].center,cb=z360_rows[b].center;
                    for(;y>=p;--y) z360_rows[y].center=ca+(cb-ca)*(y-a)/(b-a);
                }
                break;
            }
        }
        if(y>=z360_top+step && y<z360_end) z360_rows[y].center=(z360_rows[y-step].center+2*z360_rows[y].center)/3;
    }
}
static void z360_arc_repair(unsigned side)
{
    int anchor_y=0,anchor_x=0,one=z360_step(1),two=z360_step(2);
    for(int y=z360_y(55);y>z360_off;--y) {
        for(int x=z360_edge(0U,y)+1;x<z360_edge(1U,y)-1;++x) if(z360_px(y,x) && !z360_px(y,x+1)) {anchor_y=y;anchor_x=x;break;}
        if(anchor_y) break;
    }
    if(!anchor_y) for(int y=z360_off+(side==0U ? 0 : z360_step(5));y<z360_y(30);++y) {
        if(z360_found(side,y)=='T' && z360_found(side,y+one)=='T' && z360_found(side,y+two)=='W' &&
           abs(z360_edge(side,y)-z360_edge(side,y+two))>z360_dx(side==0U ? 7.f : 5.f,y)) {
            anchor_y=y;anchor_x=z360_edge(side,y);z360_off=y;break;
        }
    }
    if(!anchor_y) return;
    int bottom=z360_y(59),endpoint=z360_x(side==0U ? 79.f : 0.f,bottom);
    if(bottom==anchor_y) return;
    int slope=(endpoint-anchor_x)*256/(bottom-anchor_y);
    int end=side==0U ? z360_end : z360_y(58);
    for(int y=anchor_y;y<end;++y) {
        int edge=anchor_x+(int)floorf((float)slope*(float)(y-anchor_y)/256.f);
        if(side==0U) z360_rows[y].right=edge;else z360_rows[y].left=edge;
        int center=(z360_rows[y].left+z360_rows[y].right)/2;
        if(side==0U) center=center<z360_x(4.f,y) ? z360_x(4.f,y) : center;
        else center=center>z360_x(79.f,y) ? z360_x(79.f,y) : center;
        z360_rows[y].center=center;
    }
    if(side==0U) z360_rows[anchor_y].right=anchor_x;else z360_rows[anchor_y].left=anchor_x;
    for(int y=anchor_y-1;y>z360_y(10);--y) {
        int previous=z360_edge(1U-side,y+1);
        int start=previous+z360_dx(side==0U ? -18.f : 8.f,y),finish=previous+z360_dx(side==0U ? 2.f : -4.f,y);
        int increment=side==0U ? 1 : -1;
        for(int x=start;side==0U ? x<finish : x>finish;x+=increment) {
            if(z360_px(y,x) && !z360_px(y,x+increment)) {
                if(side==0U) z360_rows[y].right=x;else z360_rows[y].left=x;
                int c=(z360_rows[y].left+z360_rows[y].right)/2;
                int lo=z360_x(side==0U ? 4.f : 5.f,y),hi=z360_x(79.f,y);
                z360_rows[y].center=side==0U ? (c<lo ? lo : c) : z360_limit(c,lo,hi);break;
            }
        }
        if(z360_edge(1U,y)-z360_edge(0U,y)>z360_dx(8.f,y) &&
           (side==0U ? z360_edge(1U,y)<z360_edge(1U,y+two) : z360_edge(0U,y)>z360_edge(0U,y+two))) continue;
        z360_off=y+two;break;
    }
}
static unsigned z360_handle(unsigned side,unsigned state,uint8_t *completed)
{
    int num=0,one=z360_step(1),two=z360_step(2),three=z360_step(3);
    for(int y=z360_y(55);y>z360_y(30);--y) {
        if(z360_found(side,y)=='W') ++num;
        if(z360_found(side,y+three)=='W' && z360_found(side,y+two)=='W' && z360_found(side,y+one)=='W' && z360_found(side,y)=='T') break;
    }
    if(state==1U && (float)num>RING_360_ENTRY_LOSS_ROWS*z360_sy) state=2U;
    if(state==2U && (float)num<(side==0U ? RING_360_LEFT_RING5TO6 : RING_360_RIGHT_RING5TO6)*z360_sy) state=5U;
    if(state==5U && (float)z360_lost[1U-side]>(side==0U ? RING_360_LEFT_5TO6_OUTER_LOSS : RING_360_RIGHT_5TO6_OUTER_LOSS)*z360_sy) state=6U;
    if(state==6U && (float)z360_lost[1U-side]<(side==0U ? RING_360_LEFT_6TO7_OUTER_LOSS : RING_360_RIGHT_6TO7_OUTER_LOSS)*z360_sy) state=7U;
    if(state==7U) {
        int point=0;
        for(int y=z360_y(side==0U ? 50.f : 45.f);y>z360_off+three;--y) {
            int c=z360_edge(1U-side,y),match=1;
            for(int d=1;d<=(side==0U ? 2 : 3);++d) {
                int shift=z360_step((float)d),a=z360_edge(1U-side,y+shift),b=z360_edge(1U-side,y-shift);
                if(side==0U ? (c>a || c>b) : (c<a || c<b)) {match=0;break;}
            }
            if(match) {point=y;break;}
        }
        if(point>z360_y(RING_360_EXIT_CORNER_ROW)) state=8U;
    }
    if(state==8U) {
        int a=z360_off+z360_step(10),b=z360_y(45);
        float limit=side==0U ? RING_360_LEFT_EXIT_VARIANCE : RING_360_RIGHT_EXIT_VARIANCE;
        if(z360_variance(1U-side,a,b)<z360_variance_limit(limit,a,b) &&
           (float)z360_lost[1U-side]<(side==0U ? RING_360_LEFT_EXIT_LOSS_MAX : RING_360_RIGHT_EXIT_LOSS_MAX)*z360_sy &&
           z360_off<z360_y(side==0U ? RING_360_LEFT_EXIT_OFF_MAX : RING_360_RIGHT_EXIT_OFF_MAX)) {state=9U;*completed=1U;}
    }
    if(state==9U) {
        num=0;for(int y=z360_y(40);y>z360_y(10);--y) if(z360_found(side,y)=='W') ++num;
        if((float)num<(side==0U ? RING_360_LEFT_FINISH_LOSS : RING_360_RIGHT_FINISH_LOSS)*z360_sy) state=0U;
    }
    if(state>=1U && state<=4U) {
        for(int y=z360_y(side==0U ? 57.f : 59.f);y>z360_off;--y) {
            float half=z360_width(y)*0.5f*RING_360_HALF_WIDTH_SCALE;
            z360_rows[y].center=(int)lroundf(side==0U ? (float)z360_edge(1U,y)-half+(float)RING_360_LEFT_ENTRY_TRIM*z360_sx(y) : (float)z360_edge(0U,y)+half);
        }
    }
    if(state==5U || state==6U) z360_arc_repair(side);
    if(state==8U) {
        int repair_y=z360_y(0),repair_x=z360_x(20.f,repair_y);
        float column=side==0U ? RING_360_LEFT_REPAIR_COLUMN : RING_360_RIGHT_REPAIR_COLUMN;
        for(int y=z360_y(40);y>z360_y(side==0U ? 5.f : 8.f);--y) {
            int x=z360_x(column,y),above=z360_x(column,y-one);
            if(z360_px(y,x) && !z360_px(y-one,above)) {repair_x=above;repair_y=y-one;z360_off=y+one;break;}
        }
        int bottom=z360_y(58),den=bottom-repair_y,anchor=z360_edge(1U-side,bottom);
        if(den!=0) for(int y=z360_y(57);y>repair_y-three && y>=z360_top;--y) {
            int edge=(anchor-repair_x)*(y-bottom)/den+anchor;
            if(side==0U) z360_rows[y].right=edge;else z360_rows[y].left=edge;
            z360_rows[y].center=(z360_rows[y].left+z360_rows[y].right)/2;
        }
    }
    if(state==9U || (side==0U && state==10U)) for(int y=z360_y(59);y>z360_off;--y) {
        float half=z360_width(y)*0.5f*RING_360_HALF_WIDTH_SCALE;
        z360_rows[y].center=(int)lroundf(side==0U ? (float)z360_edge(1U,y)-half : (float)z360_edge(0U,y)+half);
    }
    return state;
}
static int z360_deviation(unsigned state,float *scale)
{
    static const int far_weight[5]={40,35,30,10,0},near_weight[5]={25,20,0,0,0};
    float reference=22.f;
    if(state==1U || state==2U) reference=RING_360_LOOKAHEAD_ENTRY;
    else if(state==3U || state==4U) reference=24.f;
    else if(state==5U) reference=RING_360_LOOKAHEAD_IN;
    else if(state==6U) reference=RING_360_LOOKAHEAD_STAGE6;
    else if(state==7U || state==8U) reference=RING_360_LOOKAHEAD_OUT;
    else if(state==9U) reference=RING_360_LOOKAHEAD_FINISH;
    int tow=z360_y(reference);
    if(tow<z360_off) tow=z360_off+1;
    if(tow>=z360_y(49)) tow=z360_y(49);
    if(tow<=z360_off || tow>=z360_end) return -1;
    *scale=z360_sx(tow);
    float sum=(float)z360_rows[tow].center*96.f*z360_sy,total=96.f*z360_sy;
    int radius=z360_step(5);
    for(int distance=1;distance<=radius;++distance) {
        int index=(int)((float)(distance-1)/z360_sy);if(index>4) index=4;
        int a=tow-distance,b=tow+distance;
        if(a>=z360_off && b<z360_end) {
            sum+=(float)(far_weight[index]*z360_rows[a].center+near_weight[index]*z360_rows[b].center);
            total+=(float)(far_weight[index]+near_weight[index]);
        }
    }
    return (int)floorf(sum/total);
}
static int ring_360_zore(const uint8_t *binary,unsigned rows,unsigned cols,
                         const track_ring_vision_state_t *model,track_vision_result_t *out)
{
    track_ring_observation_t *v=&out->ring;
    z360_binary=binary;z360_h=rows;z360_w=cols;
    z360_top=(int)(rows*RING_360_VIEW_START_PERCENT/100U);z360_end=(int)(rows*RING_360_VIEW_END_PERCENT/100U);
    /* Reuse only rows provided by the existing raw-boundary extractor. */
    int raw_top=(int)(rows*RING_SEARCH_START_PERCENT/100U),raw_end=(int)(rows*RING_SEARCH_END_PERCENT/100U);
    if(z360_top<raw_top) z360_top=raw_top;
    if(z360_end>raw_end) z360_end=raw_end;
    if(z360_end-z360_top<12) return 0;
    z360_sy=(float)(z360_end-z360_top-1)/59.f;
    if(rows!=z360_previous_h || cols!=z360_previous_w) {z360_width_ready=0U;z360_previous_h=rows;z360_previous_w=cols;}
    if(!z360_active_hint && model->ready && v->near_paired && !v->junction && out->valid) {
        int model_near=(int)(rows*RING_SEARCH_END_PERCENT/100U)-1;
        z360_width_near=model->width_near+model->width_slope*(float)(z360_end-1-model_near);
        z360_width_slope=model->width_slope;z360_width_ready=1U;
    }
    v->zore360_native_rows=(uint16_t)rows;v->zore360_native_cols=(uint16_t)cols;
    v->zore360_top=(uint16_t)z360_top;v->zore360_end=(uint16_t)z360_end;
    v->zore360_width_ready=z360_width_ready;
    if(!z360_width_ready) return 0; /* Need the current car's normal straight first. */
    v->zore360_width_near=z360_width_near;v->zore360_scale_y=z360_sy;v->zore360_center_x=z360_center_x();
    memset(z360_rows,0,sizeof(z360_rows));
    z360_lost[0]=z360_lost[1]=z360_white=0;z360_off=z360_y(RING_360_IGNORE_TOP);
    for(int y=z360_top;y<z360_end;++y) {
        unsigned flags=out->raw_flags[y];
        z360_rows[y].left=out->raw_left[y];z360_rows[y].right=out->raw_right[y];
        z360_rows[y].center=(z360_rows[y].left+z360_rows[y].right)/2;
        z360_rows[y].found[0]=(flags&1U) ? 'T' : ((flags&4U) ? 'W' : 'F');
        z360_rows[y].found[1]=(flags&2U) ? 'T' : ((flags&8U) ? 'W' : 'F');
        z360_rows[y].first[1]=z360_rows[y].last[1]=(int)cols-1;
    }
    /* Adapt the existing native raw edges to ZORE T/W semantics. The input's
     * "opening" flag can coexist with a continuous reliable outside border;
     * it is not automatically a lost edge in the original ring algorithm.
     * Read continuity/pixels only; never re-write the caller's raw arrays. */
    for(int y=z360_y(54)+1;y<z360_end;++y) {
        if(out->raw_flags[y]!=0U) z360_rows[y].found[0]=z360_rows[y].found[1]='T';
    }
    for(int y=z360_y(54);y>z360_off;--y) {
        if(out->raw_flags[y]==0U) {z360_off=y+1;break;}
        int window=(int)fmaxf(1.f,ceilf(RING_360_BORDER_STEP*z360_sx(y)/z360_sy));
        for(unsigned side=0U;side<2U;++side) {
            int actual=side==0U ? out->raw_left[y] : out->raw_right[y];
            int previous=z360_edge(side,y+1),outward=side==0U ? -1 : 1;
            char found='T';int edge=actual;
            int real_edge=actual>0 && actual<(int)cols-1 && z360_px(y,actual) && !z360_px(y,actual+outward);
            if(!(real_edge && abs(actual-previous)<=window)) {
                if(z360_px(y,previous)) {found='W';edge=previous;}
                else if(!real_edge) {found='H';edge=previous;}
            }
            if(side==0U) z360_rows[y].left=edge;else z360_rows[y].right=edge;
            z360_rows[y].found[side]=found;
            if(found=='W') ++z360_lost[side];
        }
        z360_rows[y].center=(z360_rows[y].left+z360_rows[y].right)/2;
        if(z360_found(0U,y)=='W' && z360_found(1U,y)=='W') ++z360_white;
    }
    z360_boundary();int p1[2]={0,0},p2[2]={0,0};
    int left=z360_admit(0U,&p1[0],&p2[0]),right=left ? 0 : z360_admit(1U,&p1[1],&p2[1]);
    int detected=left ? 1 : (right ? -1 : 0);
    for(unsigned side=0U;side<2U;++side) {
        v->zore360_loss[side]=(uint16_t)z360_lost[side];v->zore360_point1[side]=(uint16_t)p1[side];v->zore360_point2[side]=(uint16_t)p2[side];
        v->zore360_variance[side]=(uint32_t)fminf(z360_variance(side,z360_y(25),z360_y(45)),4294967040.f);
    }
    v->zore360_both_lost=(uint16_t)z360_white;
    memcpy(z360_saved,z360_rows,sizeof(z360_rows));int raw_off=z360_off;
    unsigned capture_stage=z360_active_hint>=1U && z360_active_hint<=9U ? z360_active_hint : 1U;
    int capture_direction=z360_active_hint>=1U && z360_active_hint<=9U ? z360_active_direction : detected;
    for(unsigned side=0U;side<2U;++side) for(unsigned stage=1U;stage<=9U;++stage) {
        memcpy(z360_rows,z360_saved,sizeof(z360_rows));z360_off=raw_off;
        z360_route_filter();
        unsigned next=z360_handle(side,stage,&v->zore360_done[side][stage]);float scale=1.f;
        int det=z360_deviation(next,&scale);
        v->zore360_next[side][stage]=(uint8_t)next;v->zore360_det[side][stage]=(int16_t)(det<0 ? 0 : det);
        v->zore360_det_valid[side][stage]=(uint8_t)(det>=0);v->zore360_off[side][stage]=(int16_t)z360_off;
        v->zore360_error_scale[side][stage]=scale;
        if(det>=0) v->zore360_path[side][stage]=(track_path_observation_t){track_clamp_error((int64_t)lroundf(((float)det-z360_center_x())*2000.f/(float)(cols-1U))),0,1U,100U};
        if(stage==capture_stage && (side==0U ? 1 : -1)==capture_direction) {
            v->zore360_center_stage=(uint8_t)stage;v->zore360_center_direction=(int8_t)capture_direction;
            for(int y=z360_top;y<z360_end;++y) v->zore360_center[y]=(int16_t)z360_rows[y].center;
        }
    }
    v->zore360_ready=1U;v->zore360_match=(uint8_t)(detected!=0);return detected;
}
