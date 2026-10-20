/* 360-only adapter for ZORE app_camer.c:
 * Element_Judgment_Left_Rings / Element_Judgment_Right_Rings.
 * Private 80x60 view and wall tracing; never writes the shared binary image,
 * ordinary rows, the 270 detector, or the motion state transitions.
 * ZORE's 80x60 predicate constants retain their original coordinate units.
 */
typedef struct {
    uint8_t pixel[60][80];
    int border[2][60], first[2][60], last[2][60];
    char found[2][60];
    int lost[2], white, off, variance[2], point1[2], point2[2], help[2];
    int narrow[2];
} zore360_frame_t;
static zore360_frame_t zore360_frame;

static int z360_limit(int x,int lo,int hi) {return x<lo ? lo : (x>hi ? hi : x);}
static int z360_pixel(const zore360_frame_t *f,int y,int x)
{
    return y>=0 && y<60 && x>=0 && x<80 ? f->pixel[y][x] : 0;
}
static int z360_variance(const zore360_frame_t *f,unsigned side)
{
    /* Straight_Judge(side,25,45), including the original fixed-point shifts. */
    int k=(f->border[side][25]-f->border[side][45])*256/-20,sum=0;
    for(int i=0;i<20;++i) {
        int scaled=f->border[side][25]*256+k*i-f->border[side][25+i]*256;
        int error=scaled>=0 ? scaled/16 : -((-scaled+15)/16);
        sum+=error*error;
    }
    return sum/20;
}
static void z360_border(zore360_frame_t *f)
{
    /* DrawLinesFirst: stable bottom anchors, then the reference +/-2 search. */
    int centre=40;
    if(!f->pixel[59][centre]) {
        for(int distance=1;distance<39;++distance) {
            int left=40-distance,right=40+distance;
            if(f->pixel[59][left]) {centre=left;break;}
            if(f->pixel[59][right]) {centre=right;break;}
        }
        int l=centre,r=centre;
        while(l>0 && f->pixel[59][l-1]) --l;
        while(r<79 && f->pixel[59][r+1]) ++r;
        centre=(l+r)/2;
    }
    for(int y=59;y>=55;--y) {
        int l=0,r=79;
        for(int x=2;x+1<centre;++x)
            if(!f->pixel[y][x-2] && !f->pixel[y][x-1] && f->pixel[y][x] && f->pixel[y][x+1]) {l=x;break;}
        for(int x=77;x-1>centre;--x)
            if(!f->pixel[y][x+2] && !f->pixel[y][x+1] && f->pixel[y][x] && f->pixel[y][x-1]) {r=x;break;}
        f->border[0][y]=l;f->border[1][y]=r;
        f->found[0][y]=f->found[1][y]='T';centre=z360_limit((l+r)/2,3,76);
    }
    f->off=10;
    for(int y=54;y>f->off;--y) {
        for(unsigned side=0U;side<2U;++side) {
            int previous=f->border[side][y+1];
            int lo=z360_limit(previous-2,1,78),hi=z360_limit(previous+2,1,78);
            int x=side==0U ? hi : lo,step=side==0U ? -1 : 1;
            char flag='W';int edge=previous;
            for(;x>=lo && x<=hi;x+=step) {
                if(f->pixel[y][x] && !f->pixel[y][x+step]) {edge=x;flag='T';break;}
            }
            if(flag=='W' && !f->pixel[y][(lo+hi)/2]) {
                flag='H';
                /* H: search inward for the real black/white edge. */
                int bound=f->border[1U-side][y+1];
                for(x=previous;x>=1 && x<=78 && (side==0U ? x<bound : x>bound);x-=step)
                    if(f->pixel[y][x] && !f->pixel[y][x+step]) {edge=x;flag='T';break;}
            }
            f->border[side][y]=edge;f->found[side][y]=flag;
        }
        if(f->border[1][y]-f->border[0][y]<=7 || f->border[1][y]<=1 || f->border[0][y]>=70) {f->off=y+1;break;}
        for(unsigned side=0U;side<2U;++side) if(f->found[side][y]=='W') ++f->lost[side];
        if(f->found[0][y]=='W' && f->found[1][y]=='W') ++f->white;
    }
    f->variance[0]=z360_variance(f,0U);f->variance[1]=z360_variance(f,1U);
}
static void z360_walk(zore360_frame_t *f,unsigned side)
{
    /* Same left/right-hand neighbour tables as Search_Left_and_Right_Lines.
     * The guarded private frame replaces the reference's unchecked indexes. */
    static const int dy[4]={-1,0,1,0},dx[4]={0,1,0,-1};
    static const int diagonal_y[4]={-1,-1,1,1};
    static const int left_diagonal_x[4]={-1,1,1,-1};
    static const int right_diagonal_y[4]={-1,1,1,-1};
    static const int right_diagonal_x[4]={1,1,-1,-1};
    int x=f->border[side][58],y=58,d=0;
    if(!f->pixel[y][x]) return;
    for(unsigned n=0U;n<800U;++n) {
        int a=x+dx[d],b=y+dy[d];
        if(!z360_pixel(f,b,a)) {d=(d+(side==0U ? 1 : 3))%4;continue;}
        int cx=x+(side==0U ? left_diagonal_x[d] : right_diagonal_x[d]);
        int cy=y+(side==0U ? diagonal_y[d] : right_diagonal_y[d]);
        if(!z360_pixel(f,cy,cx)) {x=a;y=b;}
        else {x=cx;y=cy;d=(d+(side==0U ? 3 : 1))%4;}
        if(y<1 || y>58 || x<1 || x>78) break;
        int sentinel=side==0U ? 0 : 79;
        if(f->first[side][y]==sentinel) f->first[side][y]=x;
        f->last[side][y]=x;
        if(y<=5) break;
    }
}
static int z360_decide(zore360_frame_t *f,unsigned side)
{
    /* The admission predicates are the reference's left/right ring functions,
     * not Cross_Test2 (which is the separate 270 cross detector). */
    unsigned outer=1U-side;
    int max_outer=RING_360_OUTER_LOSS_MAX;
    int min_loss=side==0U ? RING_360_ZORE_LEFT_LOSS : RING_360_ZORE_RIGHT_LOSS;
    int max_variance=side==0U ? RING_360_ZORE_RIGHT_VARIANCE : RING_360_ZORE_LEFT_VARIANCE;
    if(f->narrow[side]>=(side==0U ? 3 : 2) || f->lost[outer]>max_outer ||
       f->lost[side]<min_loss || f->off>RING_360_OFFLINE_MAX || f->white>RING_360_BOTH_LOSS_MAX || f->variance[outer]>max_variance) return 0;
    int p1=0,p2=0,help=0;
    int initial_jump=RING_360_CORNER_JUMP<4 ? 4 : RING_360_CORNER_JUMP;
    for(int y=58;y>RING_360_CORNER_TOP;--y) {
        int delta=side==0U ? f->first[side][y]-f->first[side][y-1] : f->first[side][y-1]-f->first[side][y];
        if(delta>initial_jump) {p1=y;break;}
    }
    for(int y=58;y>RING_360_CORNER_TOP;--y) {
        int delta=side==0U ? f->last[side][y+1]-f->last[side][y] : f->last[side][y]-f->last[side][y+1];
        if(delta>initial_jump) {p2=y;break;}
    }
    if(p1==0 || p2<=p1+RING_360_CORNER_GAP) {
        p1=0;p2=0;
        for(int y=58;y>RING_360_CORNER_TOP;--y) {
            int delta=side==0U ? f->first[side][y]-f->first[side][y-1] : f->first[side][y-1]-f->first[side][y];
            if(delta>RING_360_CORNER_JUMP) {p1=y;break;}
        }
        for(int y=58;y>RING_360_CORNER_TOP;--y) {
            int delta=side==0U ? f->last[side][y+1]-f->last[side][y] : f->last[side][y]-f->last[side][y+1];
            if(delta>RING_360_CORNER_JUMP) {p2=y;break;}
        }
    }
    /* A lower jump threshold can reveal a small near-edge ripple before the
     * actual corner. Find the ordered pair instead of rejecting that frame. */
    if(p1==0 || p2==0) return 0;
    if(p2<=p1+RING_360_CORNER_GAP) {
        p1=0;
        for(int y=p2-RING_360_CORNER_GAP-1;y>RING_360_CORNER_TOP;--y) {
            int delta=side==0U ? f->first[side][y]-f->first[side][y-1] : f->first[side][y-1]-f->first[side][y];
            if(delta>RING_360_CORNER_JUMP) {p1=y;break;}
        }
        if(p1==0) return 0;
    }
    for(int y=p1;y>(side==0U ? f->off : 10);--y) {
        if(y+6>=60) continue;
        int a=f->border[side][y+6],b=f->border[side][y+5],c=f->border[side][y+3];
        int d=f->border[side][y+2],e=f->border[side][y+1];
        if(side==0U ? (a<c && b<c && c>d && c>e) : (a>c && b>c && c<d && c<e)) {help=1;break;}
    }
    int gap=RING_360_CORNER_GAP;
    if(p2>p1+gap && !help && f->lost[side]>min_loss+1) help=1;
    f->point1[side]=p1;f->point2[side]=p2;f->help[side]=help;
    return p2>p1+gap && help;
}
static int ring_360_zore(const uint8_t *binary,unsigned rows,unsigned cols,
                         const track_ring_vision_state_t *model,track_ring_observation_t *v)
{
    zore360_frame_t *f=&zore360_frame;memset(f,0,sizeof(*f));
    unsigned top=rows*RING_SEARCH_START_PERCENT/100U,end=rows*RING_SEARCH_END_PERCENT/100U;
    for(unsigned y=0U;y<60U;++y) {
        unsigned source_y=top+(end-top-1U)*y/59U;
        for(unsigned x=0U;x<80U;++x) {
            unsigned source_x=(cols-1U)*x/79U;
            f->pixel[y][x]=(uint8_t)(binary[source_y*cols+source_x]!=0U && route_member(source_y,source_x));
        }
        f->first[1][y]=f->last[1][y]=79;
    }
    z360_border(f);
    /* ZORE's 40px obstacle-width veto assumes its road occupies >half an 80px
     * image. Scale only that width veto to this camera's learned road width. */
    float expected=ring_width(model,end-1U,end-1U)*79.f/(float)(cols-1U);
    int safe_width=z360_limit((int)(expected*RING_360_ZORE_NARROW_RATIO),8,40);
    for(int y=59;y>=54;--y) if(f->found[0][y]=='T' && f->found[1][y]=='T' &&
       f->border[1][y]-f->border[0][y]<safe_width) {
        ++f->narrow[1];if(y<=57) ++f->narrow[0];
    }
    /* Search_Border_OTSU changes only this private copy's border pixels. */
    memset(f->pixel[0],0,80U);memset(f->pixel[59],0,80U);
    for(unsigned y=0U;y<60U;++y) {f->pixel[y][0]=0U;f->pixel[y][79]=0U;}
    z360_walk(f,0U);z360_walk(f,1U);
    int left=z360_decide(f,0U),right=z360_decide(f,1U);
    v->zore360_loss[0]=(uint8_t)f->lost[0];v->zore360_loss[1]=(uint8_t)f->lost[1];
    v->zore360_both_lost=(uint8_t)f->white;
    for(unsigned side=0U;side<2U;++side) {
        v->zore360_point1[side]=(uint8_t)f->point1[side];v->zore360_point2[side]=(uint8_t)f->point2[side];
        v->zore360_help[side]=(uint8_t)f->help[side];
        v->zore360_variance[side]=(uint16_t)(f->variance[side]>65535 ? 65535 : f->variance[side]);
    }
    if(left==right) return 0; /* No evidence, or ambiguous two-sided geometry. */
    unsigned outer=left ? 1U : 0U;
    /* Reliable outside border -> projected centre using the existing row-width
     * model. This is a 360 observation only; ordinary/270 paths are untouched. */
    float sy=0.f,sx=0.f,syy=0.f,sxy=0.f,n=0.f;
    for(unsigned y=25U;y<=54U;++y) {
        if(f->found[outer][y]!='T') continue;
        float source_y=(float)top+(float)(end-top-1U)*(float)y/59.f;
        float edge=(float)f->border[outer][y]*(float)(cols-1U)/79.f;
        float centre=edge+(outer==0U ? 0.5f : -0.5f)*ring_width(model,(unsigned)source_y,end-1U);
        sy+=source_y;sx+=centre;syy+=source_y*source_y;sxy+=source_y*centre;n+=1.f;
    }
    float den=n*syy-sy*sy;
    if(n<5.f || den<1.f) return 0;
    float k=(n*sxy-sy*sx)/den,b=(sx-k*sy)/n;
    v->edge_near[outer]=(b+k*(float)rows*RING_GUIDE_NEAR_PERCENT/100.f)*2000.f/(float)(cols-1U)-1000.f;
    v->edge_far[outer]=(b+k*(float)rows*RING_GUIDE_FAR_PERCENT/100.f)*2000.f/(float)(cols-1U)-1000.f;
    v->edge_line_valid[outer]=1U;v->zore360_match=1U;
    return left ? 1 : -1;
}
