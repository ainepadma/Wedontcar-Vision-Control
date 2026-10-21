/* White-component association only: anchor in the nearest visible road, then
 * propagate upward. A disconnected distant road never becomes a fresh seed.
 * The shared binary image and all steering/speed algorithms stay untouched. */
#define ROUTE_STRIDE ((TRACK_VISION_MAX_COLS+7U)/8U)
static uint8_t route_mask[TRACK_VISION_MAX_ROWS][ROUTE_STRIDE];
static uint8_t route_previous[TRACK_VISION_MAX_COLS];
static uint8_t route_current[TRACK_VISION_MAX_COLS];
static uint8_t route_seed_ambiguous;
static uint8_t route_member(unsigned row,unsigned x)
{
    return (uint8_t)((route_mask[row][x/8U]>>(x%8U))&1U);
}
static uint8_t route_run(const uint8_t *pixels,unsigned cols,unsigned *cursor,unsigned *l,unsigned *r)
{
    unsigned x=*cursor;
    while(x<cols && !pixels[x]) ++x;
    if(x==cols) {*cursor=x;return 0U;}
    *l=x;*r=x;unsigned gap=0U;
    for(;x<cols;++x) {
        if(pixels[x]) {*r=x;gap=0U;}
        else if(++gap>2U) {++x;break;}
    }
    *cursor=x;return 1U;
}
static void route_build(const uint8_t *binary,unsigned rows,unsigned cols)
{
    memset(route_mask,0,sizeof(route_mask));memset(route_previous,0,cols);
    route_seed_ambiguous=0U;
    unsigned top=rows*30U/100U,end=rows*TRACK_VISION_ROI_END_PERCENT/100U;
    unsigned seed_top=rows*TRACK_ROUTE_SEED_START_PERCENT/100U;
    unsigned minimum=cols/16U;if(minimum<6U) minimum=6U;
    uint8_t seeded=0U;unsigned missing=0U;
    for(unsigned next=end;next>top;--next) {
        unsigned y=next-1U,cursor=0U,l=0U,r=0U;const uint8_t *pixels=binary+y*cols;
        memset(route_current,0,cols);
        if(!seeded) {
            if(y<seed_top) break;
            unsigned best_l=0U,best_r=0U,best=UINT32_MAX,second=UINT32_MAX;
            while(route_run(pixels,cols,&cursor,&l,&r)) {
                unsigned white=0U;for(unsigned x=l;x<=r;++x) if(pixels[x]) ++white;
                if(r-l+1U<minimum || white*4U<(r-l+1U)*3U) continue;
                unsigned centre=(l+r)/2U,dist=centre>cols/2U ? centre-cols/2U : cols/2U-centre;
                if(dist<best) {second=best;best=dist;best_l=l;best_r=r;}
                else if(dist<second) second=dist;
            }
            if(best==UINT32_MAX) continue;
            /* With two similarly near roads there is no reliable ego seed. */
            if(second!=UINT32_MAX && second-best<=cols/20U) {route_seed_ambiguous=1U;break;}
            for(unsigned x=best_l;x<=best_r;++x) route_current[x]=1U;
            seeded=1U;
        } else {
            while(route_run(pixels,cols,&cursor,&l,&r)) {
                uint8_t connected=0U;
                unsigned lo=l>0U ? l-1U : l,hi=r+1U<cols ? r+1U : r;
                for(unsigned x=lo;x<=hi;++x) if(route_previous[x]) {connected=1U;break;}
                if(!connected) continue;
                for(unsigned x=l;x<=r;++x) route_current[x]=1U;
            }
        }
        uint8_t any=0U;
        for(unsigned x=0U;x<cols;++x) if(route_current[x]) {
            route_mask[y][x/8U]|=(uint8_t)(1U<<(x%8U));any=1U;
        }
        if(!any) {if(++missing>TRACK_ROUTE_MAX_GAP_ROWS) break;continue;}
        missing=0U;memcpy(route_previous,route_current,cols);
    }
    /* A ring can connect a lower branch via its upper white cap. Complete the
     * same component in both directions; do not confuse that branch with an
     * unrelated road. No new seed or jump across a black band is introduced. */
    if(!seeded || route_seed_ambiguous) return;
    for(unsigned pass=0U;pass<2U;++pass) {
        uint8_t changed=0U;
        for(unsigned direction=0U;direction<2U;++direction) {
            for(unsigned offset=0U;offset<end-top;++offset) {
                unsigned y=direction==0U ? top+offset : end-1U-offset;
                unsigned cursor=0U,l=0U,r=0U;
                while(route_run(binary+y*cols,cols,&cursor,&l,&r)) {
                    unsigned lo=l>0U ? l-1U : l,hi=r+1U<cols ? r+1U : r;
                    uint8_t connected=0U;
                    for(unsigned x=lo;x<=hi;++x) {
                        if(route_member(y,x) || (y>top && route_member(y-1U,x)) ||
                           (y+1U<end && route_member(y+1U,x))) {connected=1U;break;}
                    }
                    if(!connected) continue;
                    for(unsigned x=l;x<=r;++x) if(!route_member(y,x)) {
                        route_mask[y][x/8U]|=(uint8_t)(1U<<(x%8U));changed=1U;
                    }
                }
            }
        }
        if(!changed) break;
    }
}
