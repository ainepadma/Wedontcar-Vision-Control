#include "track_stripe.h"
#include <string.h>

typedef struct { unsigned left, right; } stripe_run_t;
typedef struct { unsigned count, left, right, width; } stripe_row_t;

static unsigned distance(unsigned a, unsigned b) { return a>b ? a-b : b-a; }

/* Interior black runs only: both sides must be white. Large exterior black
 * regions cannot become stripes. Search seven first, then partial five/six. */
static stripe_row_t stripe_row(const uint8_t *p, unsigned cols)
{
    stripe_run_t runs[32];
    stripe_row_t best={0U,0U,0U,0U};
    unsigned n=0U,x=0U;
    while(x<cols) {
        unsigned start;
        if(p[x]) { ++x; continue; }
        start=x;
        while(x<cols && !p[x]) ++x;
        if(start && x<cols) {
            if(n==32U) return best; /* noisy frame, not a marker */
            runs[n].left=start; runs[n++].right=x-1U;
        }
    }
    for(unsigned count=7U;count>=5U;--count) {
        for(unsigned i=0U;i+count<=n;++i) {
            unsigned minw=cols,maxw=0U,ming=cols,maxg=0U,sum=0U;
            unsigned end=i+count-1U;
            unsigned span=runs[end].right-runs[i].left+1U;
            for(unsigned j=i;j<=end;++j) {
                unsigned w=runs[j].right-runs[j].left+1U;
                if(w<minw) minw=w;
                if(w>maxw) maxw=w;
                sum+=w;
                if(j<end) {
                    unsigned gap=runs[j+1U].left-runs[j].right-1U;
                    if(gap<ming) ming=gap;
                    if(gap>maxg) maxg=gap;
                }
            }
            if(span<cols/10U || span>cols*85U/100U || maxw>cols/12U ||
               maxw>minw*3U || maxg>ming*6U || maxg>maxw*3U || maxw>ming*8U)
                continue;
            /* Reject a subset of a regular eight-or-more-stripe pattern. */
            if(i && runs[i-1U].right-runs[i-1U].left+1U>=minw &&
               runs[i-1U].right-runs[i-1U].left+1U<=maxw*2U &&
               runs[i].left-runs[i-1U].right-1U<=maxg*2U) continue;
            if(end+1U<n && runs[end+1U].right-runs[end+1U].left+1U>=minw &&
               runs[end+1U].right-runs[end+1U].left+1U<=maxw*2U &&
               runs[end+1U].left-runs[end].right-1U<=maxg*2U) continue;
            best.count=count; best.left=runs[i].left; best.right=runs[end].right;
            best.width=sum/count;
            return best;
        }
    }
    return best;
}

void track_stripe_detect(const uint8_t *binary, uint16_t rows, uint16_t cols,
                         track_stripe_observation_t *out)
{
    unsigned top=0U,bottom=0U,full=0U,matched=0U,prev_center=0U,prev_span=0U;
    unsigned best_height=0U;
    if(!out) return;
    memset(out,0,sizeof(*out));
    if(!binary || rows<20U || rows>480U || cols<40U || cols>752U) return;
    out->reliable=1U;
    for(unsigned y=rows/5U;y<rows;++y) {
        stripe_row_t row=stripe_row(binary+(unsigned long)y*cols,cols);
        unsigned center=(row.left+row.right)/2U;
        unsigned span=row.right-row.left+1U;
        if(!row.count) continue;
        if(!matched || y-bottom>2U || distance(center,prev_center)>cols/20U ||
           distance(span,prev_span)>cols/10U) {
            top=y; full=0U; matched=0U;
        }
        bottom=y; ++matched;
        if(row.count==7U) ++full;
        prev_center=center; prev_span=span;
        unsigned height=bottom-top+1U;
        if(matched>=6U && height>=row.width*2U && matched*100U>=height*80U &&
           height>=best_height) {
            best_height=height;
            out->visible=1U;
            out->complete=(uint8_t)(full>=3U);
            out->top=(uint16_t)top; out->bottom=(uint16_t)bottom;
            out->center=(uint16_t)center; out->span=(uint16_t)span;
            out->near=(uint8_t)((bottom+1U)*100U>=rows*TRACK_STRIPE_BOTTOM_PERCENT &&
                                height*100U>=rows*TRACK_STRIPE_HEIGHT_PERCENT);
        }
    }
}

void track_stripe_init(track_stripe_state_t *s)
{
    if(s) memset(s,0,sizeof(*s));
}

void track_stripe_restart(track_stripe_state_t *s, uint8_t marker_near)
{
    uint8_t bypass;
    if(!s) return;
    bypass=(uint8_t)(s->stopped || s->armed || s->bypass || marker_near);
    track_stripe_init(s);
    s->bypass=bypass;
}

uint8_t track_stripe_observe(track_stripe_state_t *s,
    const track_stripe_observation_t *o, uint32_t ms, uint32_t now, uint8_t running)
{
    uint32_t elapsed;
    if(!s || !o) return 0U;
    if(s->stopped) return 1U;
    elapsed=ms-s->last_ms;
    if(s->has_time && (!elapsed || elapsed>UINT32_C(0x7fffffff))) return 0U;
    if(s->has_time && elapsed>TRACK_STRIPE_MAX_AGE_MS)
        s->arm_votes=s->near_votes=s->clear_votes=0U;
    s->has_time=1U; s->last_ms=ms;
    if(!o->reliable || now-ms>TRACK_STRIPE_MAX_AGE_MS) {
        s->arm_votes=s->near_votes=s->clear_votes=0U;
        return 0U;
    }
    if(!running) { s->arm_votes=s->near_votes=s->clear_votes=0U; return 0U; }
    if(s->bypass) {
        if(o->visible) s->clear_votes=0U;
        else if(++s->clear_votes>=TRACK_STRIPE_CLEAR_FRAMES) {
            s->bypass=0U; s->clear_votes=0U;
        }
        return 0U;
    }
    if(!o->visible) {
        s->arm_votes=s->near_votes=0U;
        if(++s->clear_votes>=TRACK_STRIPE_CLEAR_FRAMES) {
            s->armed=0U; s->clear_votes=0U;
        }
        return 0U;
    }
    s->clear_votes=0U;
    /* A different group cannot inherit the previous group's close votes. */
    if((s->arm_votes || s->armed) &&
       distance(o->center,s->center)>(unsigned)(o->span+s->span)/4U) {
        s->armed=s->arm_votes=s->near_votes=0U;
    }
    s->center=o->center; s->span=o->span;
    if(!s->armed) {
        s->arm_votes=o->complete ? (uint8_t)(s->arm_votes+1U) : 0U;
        if(s->arm_votes>=TRACK_STRIPE_ARM_FRAMES) s->armed=1U;
        s->near_votes=0U;
        return 0U; /* separate recognition and approach stages */
    }
    s->near_votes=o->near ? (uint8_t)(s->near_votes+1U) : 0U;
    if(s->near_votes>=TRACK_STRIPE_NEAR_FRAMES) s->stopped=1U;
    return s->stopped;
}
