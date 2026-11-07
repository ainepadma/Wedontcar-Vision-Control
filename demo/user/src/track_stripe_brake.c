#include "track_stripe_brake.h"
#include <string.h>

typedef struct { unsigned left, right; } stripe_run_t;
typedef struct { unsigned count, left, right, width; } stripe_row_t;
static unsigned stripe_distance(unsigned a, unsigned b) { return a>b ? a-b : b-a; }

/* Interior black runs only: reject exterior black, broad bands and regular 8+.
 * Seven arms the latch; partial five/six can approach an already armed marker. */
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
            if(n==32U) return best;
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

void stripe_brake_detect(const uint8_t *binary, uint16_t rows, uint16_t cols,
                         stripe_brake_observation_t *out)
{
    unsigned top=0U,bottom=0U,full=0U,matched=0U,prev_center=0U,prev_span=0U;
    unsigned best_score=0U;
    if(!out) return;
    memset(out,0,sizeof(*out));
    if(!binary || rows<20U || rows>480U || cols<40U || cols>752U) return;
    out->reliable=1U;
    for(unsigned y=rows/5U;y<rows;++y) {
        stripe_row_t row=stripe_row(binary+(unsigned long)y*cols,cols);
        unsigned center,span,height,clipped,normal,score;
        if(!row.count) continue;
        center=(row.left+row.right)/2U;
        span=row.right-row.left+1U;
        if(!matched || y-bottom>2U || stripe_distance(center,prev_center)>cols/20U ||
           stripe_distance(span,prev_span)>cols/10U) {
            top=y; full=0U; matched=0U;
        }
        bottom=y; ++matched;
        if(row.count==7U) ++full;
        prev_center=center; prev_span=span;
        height=bottom-top+1U;
        clipped=(unsigned)(bottom+2U>=rows);
        normal=(unsigned)(height>=row.width*2U);
        if(matched<6U || matched*100U<height*80U || (!normal && !clipped)) continue;
        /* Prefer the near-bottom candidate over unrelated distant texture. */
        score=height+(clipped ? (unsigned)rows : 0U);
        if(score<best_score) continue;
        best_score=score;
        out->visible=1U;
        out->complete=(uint8_t)(full>=3U && normal);
        out->top=(uint16_t)top; out->bottom=(uint16_t)bottom;
        out->center=(uint16_t)center; out->span=(uint16_t)span;
        out->height=(uint16_t)height;
        out->clipped=(uint8_t)clipped;
        out->large=(uint8_t)(height*100U>=rows*STRIPE_BRAKE_HEIGHT_PERCENT);
        out->near=(uint8_t)((bottom+1U)*100U>=rows*STRIPE_BRAKE_BOTTOM_PERCENT && out->large);
    }
}

void stripe_brake_init(stripe_brake_state_t *s)
{
    if(s) memset(s,0,sizeof(*s));
}

uint8_t stripe_brake_observe(stripe_brake_state_t *s,
    const stripe_brake_observation_t *o, uint32_t ms, uint32_t now)
{
    uint32_t elapsed;
    if(!s || !o) return 0U;
    if(s->stopped) return 1U;
    elapsed=ms-s->last_ms;
    if(s->has_time && (!elapsed || elapsed>UINT32_C(0x7fffffff))) return 0U;
    if(s->has_time && elapsed>STRIPE_BRAKE_MAX_AGE_MS && !s->armed)
        s->arm_votes=0U;
    s->has_time=1U; s->last_ms=ms;
    if(!o->reliable || now-ms>STRIPE_BRAKE_MAX_AGE_MS || !o->visible) {
        if(!s->armed) s->arm_votes=0U;
        return 0U;
    }
    if(!s->armed) {
        /* Only initial recognition needs consecutive full-seven frames. */
        if(s->arm_votes && stripe_distance(o->center,s->center)>
                             (unsigned)(o->span+s->span)/4U)
            s->arm_votes=0U;
        s->arm_votes=o->complete ? (uint8_t)(s->arm_votes+1U) : 0U;
        s->center=o->center; s->span=o->span;
        if(s->arm_votes<STRIPE_BRAKE_ARM_FRAMES) return 0U;
        s->armed=1U;
    }
    if(o->large) s->size_seen=1U;
    /* One P4 frame stops, including the frame completing initial arming.
     * Once size was seen, bottom cropping cannot revoke close recognition. */
    if(o->near || (o->clipped && s->size_seen)) s->stopped=1U;
    return s->stopped;
}

int16_t stripe_brake_hold(stripe_brake_state_t *s, uint8_t wheel, int16_t delta,
                         uint8_t capture_reference)
{
    int32_t position,command;
    if(!s || !s->stopped || wheel>1U) return 0;
    position=capture_reference ? 0 : s->position_error[wheel]+(int32_t)delta;
    /* Bound accumulated displacement and arithmetic, with no integrator. */
    if(position>4096) position=4096;
    if(position<-4096) position=-4096;
    s->position_error[wheel]=position;
    command=-STRIPE_BRAKE_POSITION_KP*position-STRIPE_BRAKE_SPEED_KD*(int32_t)delta;
    if(command>STRIPE_BRAKE_PWM_LIMIT) command=STRIPE_BRAKE_PWM_LIMIT;
    if(command<-STRIPE_BRAKE_PWM_LIMIT) command=-STRIPE_BRAKE_PWM_LIMIT;
    return (int16_t)command;
}
