#include "ring_control.h"
#include <math.h>
#include <string.h>

/* All image access and line fitting are on the main loop. This module consumes
 * immutable frame observations and selects one precomputed path in the ISR. */
static track_path_observation_t invalid_path(void)
{
    const track_path_observation_t p={0,0,0U,0U};return p;
}
static uint8_t saturating_increment(uint8_t value)
{
    return value<UINT8_MAX ? (uint8_t)(value+1U) : value;
}
static uint8_t pair_is(const ring_corner_frame_t *f,uint8_t count)
{
    return (uint8_t)(f->count[0]==count && f->count[1]==count);
}
static uint8_t ambiguous(const ring_corner_frame_t *f)
{
    return (uint8_t)(f->count[0]>=RING_CORNER_MAX_PER_SIDE || f->count[1]>=RING_CORNER_MAX_PER_SIDE);
}
static uint8_t line_valid(const ring_corner_line_t *line)
{
    return (uint8_t)(line->valid && isfinite(line->x0) && isfinite(line->x1) &&
                     isfinite(line->y0) && isfinite(line->y1));
}
static uint8_t guide_valid(const ring_corner_frame_t *f,uint8_t id)
{
    if(id==RING_GUIDE_NONE || id>=RING_CORNER_GUIDE_COUNT || ambiguous(f)) return 0U;
    const ring_corner_guide_t *g=&f->guide[id];
    if(!g->valid || g->confidence<RING_CORNER_MIN_QUALITY) return 0U;
    uint8_t left=line_valid(&g->boundary[0]),right=line_valid(&g->boundary[1]);
    if(id==RING_GUIDE_270_EXTEND || id==RING_GUIDE_270_CONNECT) return (uint8_t)(left && right);
    return (uint8_t)((left || right) && g->corner_side<2U &&
                    isfinite(g->corner_x) && isfinite(g->corner_y));
}
void ring_control_init(ring_control_state_t *s)
{
    if(s!=NULL) {memset(s,0,sizeof(*s));s->protection_enabled=1U;}
}
static void reset_event(ring_control_state_t *s,uint32_t now)
{
    uint8_t protection=s->protection_enabled,completed=s->completed_mask;
    uint8_t count270=s->completed_270,count360=s->completed_360;
    uint8_t had_frame=s->have_frame,reset=(uint8_t)(s->action_locked || s->line_active);
    uint32_t id=s->event_id,frame=s->corner_frame_number,last_frame=s->last_frame_ms;
    ring_control_init(s);
    s->protection_enabled=protection;s->completed_mask=completed;
    s->completed_270=count270;s->completed_360=count360;s->event_id=id;
    s->have_frame=had_frame;s->corner_frame_number=frame;s->last_frame_ms=last_frame;
    s->last_tick_ms=now;s->have_tick=1U;s->reset_tracking_history=reset;s->control_refresh=1U;
}
static void disabled_reset(ring_control_state_t *s)
{
    uint8_t protection=s->protection_enabled;
    ring_control_init(s);s->protection_enabled=protection;
}
static void set_phase(ring_control_state_t *s,uint8_t phase,uint32_t now)
{
    s->phase=phase;s->phase_started_ms=now;s->clear_frames=0U;
    s->corner_vote_frames=0U;s->corner_vote_guide=RING_GUIDE_NONE;
    s->control_refresh=1U;s->reset_tracking_history=1U;
}
static void fail(ring_control_state_t *s)
{
    s->selected_path=invalid_path();s->line_active=0U;s->corner_guide_held=0U;
    s->control_refresh=1U;
    if(!s->protection_enabled) return;
    if(!s->fault) s->fault_resume_phase=s->phase;
    s->fault=1U;s->phase=RING_STATE_FAULT;
}
void ring_control_set_protection(ring_control_state_t *s,uint8_t enabled)
{
    if(s==NULL) return;
    s->protection_enabled=(uint8_t)(enabled!=0U);
    if(!enabled && s->fault) {
        s->fault=0U;s->phase=s->fault_resume_phase;
        s->corner_have_guide=0U;s->corner_guide_held=0U;
        s->control_refresh=1U;s->reset_tracking_history=1U;
    }
}
uint8_t ring_control_guidance_active(const ring_control_state_t *s)
{
    return (uint8_t)(RING_FEATURE_ENABLED && s!=NULL && s->action_locked && !s->fault);
}
/* Retained only for callers of the older diagnostics. New overlays use the
 * selected boundary segments and never this former center-line constructor. */
float ring_control_line_x(const ring_control_state_t *s,float row)
{
    (void)row;return s!=NULL ? (float)s->selected_path.error : 0.f;
}
void ring_control_360_motion(ring_control_state_t *s,float center,float left,float right,
                             float base_speed,float *angle,float *speed)
{
    if(!RING_FEATURE_ENABLED && s!=NULL) disabled_reset(s);
    (void)center;(void)left;(void)right;(void)base_speed;(void)angle;(void)speed;
}
void ring_control_tick(ring_control_state_t *s,uint8_t running,uint8_t fresh,
                       float rate,float distance,uint32_t now)
{
    if(s==NULL) return;
    if(!RING_FEATURE_ENABLED) {disabled_reset(s);return;}
    uint32_t dt=s->have_tick ? now-s->last_tick_ms : 0U;
    if(dt>100U) dt=0U;
    s->last_tick_ms=now;s->have_tick=1U;
    if(!running) {
        /* An automatic protection stop keeps its diagnostic until C13 starts
         * a new run (init) or C14 explicitly clears it. Manual stops cancel. */
        if(s->action_locked && !s->fault) reset_event(s,now);
        return;
    }
    if(isfinite(distance) && distance>0.f && distance<32768.f) s->travel_counts+=distance;
    if(fresh && isfinite(rate)) {
        s->have_yaw=1U;s->last_yaw_ms=now;s->yaw_rate_dps=rate;
        if(s->action_locked) s->yaw_progress_deg+=rate*(float)dt/1000.f;
    } else s->yaw_rate_dps=0.f;
    s->imu_wait=(uint8_t)(!s->have_yaw || now-s->last_yaw_ms>RING_360_YAW_STALE_MS);
    if(s->fault) return;
    if(s->phase==RING_STATE_COOLDOWN) return; /* Fresh clear images release cooldown. */
    if(!s->action_locked) return;
    if(s->have_frame && now-s->last_frame_ms>RING_VISION_STALE_MS) {fail(s);return;}
    if(now-s->event_started_ms>RING_ACTIVE_TIMEOUT_MS) {fail(s);return;}
    if(s->line_active && (now-s->line_started_ms>RING_LINE_MAX_MS ||
       s->travel_counts-s->line_started_counts>RING_LINE_MAX_COUNTS)) {fail(s);return;}
    if(s->corner_have_guide && now-s->corner_last_guide_ms>RING_CORNER_HOLD_MS)
        s->control_refresh=1U;
}
static uint8_t progress_since_event(const ring_control_state_t *s)
{
    return (uint8_t)(s->travel_counts-s->corner_event_travel>=RING_CORNER_MIN_TRAVEL ||
                    (!s->imu_wait && fabsf(s->yaw_progress_deg-s->corner_event_yaw)>=RING_YAW_TURN_ASSIST_DEG));
}
static uint8_t same_position(float x,float y,float ox,float oy,const ring_corner_frame_t *f)
{
    return (uint8_t)(fabsf(x-ox)<=fmaxf(3.f,(float)f->cols*0.08f) &&
                    fabsf(y-oy)<=fmaxf(3.f,(float)f->rows*0.08f));
}
static void clear_vote(ring_control_state_t *s)
{
    s->corner_vote_frames=0U;s->corner_vote_guide=RING_GUIDE_NONE;s->corner_vote_misses=0U;
}
static uint8_t vote(ring_control_state_t *s,const ring_corner_frame_t *f,uint8_t guide)
{
    if(!guide_valid(f,guide)) {clear_vote(s);return 0U;}
    const ring_corner_guide_t *g=&f->guide[guide];
    uint8_t spatial=(uint8_t)(guide>=RING_GUIDE_A_LEFT);
    if(s->corner_vote_guide!=guide || s->last_frame_ms-s->corner_vote_ms>RING_VISION_STALE_MS ||
       (spatial && !same_position(g->corner_x,g->corner_y,
       s->corner_vote_x,s->corner_vote_y,f))) {
        s->corner_vote_frames=0U;s->corner_vote_guide=guide;
    }
    s->corner_vote_x=g->corner_x;s->corner_vote_y=g->corner_y;
    s->corner_vote_ms=s->last_frame_ms;s->corner_vote_misses=0U;
    s->corner_vote_frames=saturating_increment(s->corner_vote_frames);
    return (uint8_t)(s->corner_vote_frames>=RING_CORNER_CONFIRM_FRAMES);
}
/* A 270 mouth can lose one side of its corner pair for one processed frame.
 * Preserve the existing vote without adding evidence; a second miss, stale
 * frame, or conflicting pair still cancels it. */
static void miss_270_vote(ring_control_state_t *s,const ring_corner_frame_t *f,
                          uint8_t guide,uint8_t expected)
{
    if(s->corner_vote_guide==guide && s->corner_vote_frames && !s->corner_vote_misses &&
       s->last_frame_ms-s->corner_vote_ms<=RING_VISION_STALE_MS &&
       !ambiguous(f) && f->count[0]<=expected && f->count[1]<=expected &&
       (f->count[0]==expected || f->count[1]==expected || pair_is(f,0U))) {
        s->corner_vote_misses=1U;
        return;
    }
    clear_vote(s);
}
static void select_guide(ring_control_state_t *s,const ring_corner_frame_t *f,uint8_t id,uint32_t now)
{
    const ring_corner_guide_t *g=&f->guide[id];
    s->corner_guide_id=id;s->corner_selected_guide=*g;
    s->corner_guide_frame_number=f->frame_number;s->corner_last_guide_ms=now;
    s->corner_have_guide=1U;s->corner_guide_held=0U;
    s->selected_path=(track_path_observation_t){g->error,g->heading,g->valid,g->confidence};
}
static void start_action(ring_control_state_t *s,uint8_t kind,const ring_corner_frame_t *f,
                         uint8_t id,uint32_t now)
{
    s->kind=kind;s->action_locked=1U;s->entry_seen=1U;s->entry_passed=0U;s->exit_seen=0U;
    s->corner_stage=1U;s->event_started_ms=now;++s->event_id;
    s->corner_event_travel=s->travel_counts;s->corner_event_yaw=s->yaw_progress_deg;
    s->line_started_ms=now;s->line_started_counts=s->travel_counts;
    set_phase(s,RING_STATE_ENTERING,now);select_guide(s,f,id,now);
}
static void consume_corner(ring_control_state_t *s,const ring_corner_frame_t *f,uint8_t id,uint32_t now)
{
    const ring_corner_guide_t *g=&f->guide[id];
    s->corner_have_consumed=1U;s->corner_consumed_side=g->corner_side;s->corner_consumed_kind=g->corner_kind;
    s->corner_x=g->corner_x;s->corner_y=g->corner_y;
    s->corner_missing=0U;s->corner_passed=0U;
    s->corner_event_travel=s->travel_counts;s->corner_event_yaw=s->yaw_progress_deg;
    s->line_started_ms=now;s->line_started_counts=s->travel_counts;
    if(s->corner_stage>=1U && s->corner_stage<=4U) {
        ring_corner_consumed_t *old=&s->corner_consumed[s->corner_stage-1U];
        old->x=g->corner_x;old->y=g->corner_y;old->travel=s->travel_counts;old->yaw=s->yaw_progress_deg;
        old->side=g->corner_side;old->kind=g->corner_kind;old->missing=0U;old->passed=0U;
    }
    select_guide(s,f,id,now);clear_vote(s);s->reset_tracking_history=1U;
}
static uint8_t near_road(const track_ring_observation_t *v)
{
    return (uint8_t)(v->path[TRACK_PATH_NORMAL].valid && (v->near_road || v->near_paired || v->corners.road_clear));
}
static uint8_t clear_road(const track_ring_observation_t *v)
{
    return (uint8_t)(pair_is(&v->corners,0U) && v->corners.road_clear && near_road(v));
}
static float passage_row(const ring_corner_frame_t *f)
{
    unsigned radius=f->rows/48U;if(radius<3U) radius=3U;
    float detector_bottom=f->detector_near_row ? (float)f->detector_near_row :
        (float)f->rows*(float)RING_SEARCH_END_PERCENT/100.f-(float)(radius*3U+3U);
    return detector_bottom-fmaxf(2.f,(float)f->rows*.02f);
}
static void track_consumed(ring_control_state_t *s,const track_ring_observation_t *v)
{
    const ring_corner_frame_t *f=&v->corners;
    if(!s->corner_have_consumed) return;
    for(unsigned event=0U;event<s->corner_stage && event<4U;++event) {
        ring_corner_consumed_t *old=&s->corner_consumed[event];
        if(old->passed) continue;
        const ring_corner_point_t *best=NULL;float best_distance=1.0e20f;
        unsigned side=old->side;
        for(unsigned i=0;i<f->count[side] && i<RING_CORNER_MAX_PER_SIDE;++i) {
            const ring_corner_point_t *p=&f->point[side][i];
            /* A/B flips at the same position remain the same physical target. */
            if(p->quality<RING_CORNER_MIN_QUALITY || !same_position(p->x,p->y,old->x,old->y,f)) continue;
            float d=fabsf(p->x-old->x)+fabsf(p->y-old->y);
            if(d<best_distance) {best=p;best_distance=d;}
        }
        if(best!=NULL) {old->x=best->x;old->y=best->y;old->missing=0U;}
        else {
            old->missing=saturating_increment(old->missing);
            uint8_t progress=(uint8_t)(s->travel_counts-old->travel>=RING_CORNER_MIN_TRAVEL ||
                (!s->imu_wait && fabsf(s->yaw_progress_deg-old->yaw)>=RING_YAW_TURN_ASSIST_DEG));
            uint8_t lower_gate=(uint8_t)(old->y>=passage_row(f));
            /* A middle-distance target disappearing behind noise is not yet a
             * passage. A near-bottom target may pass while other corners remain. */
            if(old->missing>=RING_CORNER_CLEAR_FRAMES && progress && near_road(v) &&
               (lower_gate || clear_road(v))) old->passed=1U;
        }
    }
    if(s->corner_stage>=1U && s->corner_stage<=4U) {
        const ring_corner_consumed_t *old=&s->corner_consumed[s->corner_stage-1U];
        s->corner_x=old->x;s->corner_y=old->y;s->corner_missing=old->missing;s->corner_passed=old->passed;
    }
}
static uint8_t choose_360(const ring_control_state_t *s,const ring_corner_frame_t *f,uint8_t next)
{
    if(ambiguous(f) || f->rows==0U || f->cols==0U) return RING_GUIDE_NONE;
    if(next==1U && f->count[0] && f->count[1]) return RING_GUIDE_NONE;
    uint8_t base=next==1U || next==3U ? RING_GUIDE_A_LEFT :
                 (next==2U ? RING_GUIDE_B_BRANCH_LEFT : RING_GUIDE_B_MAIN_LEFT);
    uint8_t expected=next==1U || next==3U ? RING_CORNER_A : RING_CORNER_B;
    uint8_t selected=RING_GUIDE_NONE;
    for(unsigned side=0U;side<2U;++side) {
        uint8_t id=(uint8_t)(base+side);
        if(f->continued_mask&(1U<<side)) continue; /* Current support, not a new event vote. */
        if(!f->count[side] || !guide_valid(f,id)) continue;
        const ring_corner_guide_t *g=&f->guide[id];
        if(g->corner_side!=side || g->corner_kind!=expected || (next==1U &&
           g->corner_y<(float)f->rows*(float)RING_CORNER_ACTION_PERCENT/100.f)) continue;
        if(next==3U && side==s->corner_consumed[0].side) continue; /* A3 is A1's opposite-side mirror. */
        if(next>1U) {
            if(!progress_since_event(s)) continue;
            /* Seeing A1 and a farther B in one image must not turn early. The
             * first tip must reach the usable detector's near edge or pass. */
            if(next==2U && !s->corner_passed && s->corner_y<passage_row(f)) continue;
            uint8_t consumed=0U;
            for(unsigned event=0U;event<s->corner_stage && event<4U;++event) {
                const ring_corner_consumed_t *old=&s->corner_consumed[event];
                if(!old->passed && side==old->side && same_position(g->corner_x,g->corner_y,old->x,old->y,f))
                    consumed=1U;
            }
            if(consumed) continue;
        }
        if(selected!=RING_GUIDE_NONE) {
            /* Two equally near alternatives are ambiguous, not a direction vote. */
            if(fabsf(g->corner_y-f->guide[selected].corner_y)<(float)f->rows*0.05f)
                return RING_GUIDE_NONE;
            if(g->corner_y<f->guide[selected].corner_y) continue;
        }
        selected=id;
    }
    return selected;
}
static void complete(ring_control_state_t *s,uint32_t now)
{
    if(s->kind==TRACK_RING_270) {s->completed_mask|=RING_DONE_270;s->completed_270=saturating_increment(s->completed_270);}
    else {s->completed_mask|=RING_DONE_360;s->completed_360=saturating_increment(s->completed_360);}
    set_phase(s,RING_STATE_COOLDOWN,now);s->cooldown_until_ms=now+RING_COOLDOWN_MS;
    s->action_locked=0U;s->entry_seen=0U;s->line_active=0U;s->corner_guide_id=RING_GUIDE_NONE;
    s->corner_have_guide=0U;s->corner_guide_held=0U;
}
static void observe_270(ring_control_state_t *s,const track_ring_observation_t *v,uint32_t now)
{
    const ring_corner_frame_t *f=&v->corners;
    if(s->corner_stage==1U || s->corner_stage==4U) {
        if(pair_is(f,2U) && vote(s,f,RING_GUIDE_270_CONNECT)) {
            ++s->corner_stage;select_guide(s,f,RING_GUIDE_270_CONNECT,now);
            s->line_started_ms=now;s->line_started_counts=s->travel_counts;
            s->reset_tracking_history=1U;clear_vote(s);
        } else if(!pair_is(f,2U)) miss_270_vote(s,f,RING_GUIDE_270_CONNECT,2U);
        return;
    }
    if(s->corner_stage==2U || s->corner_stage==5U) {
        if(clear_road(v) && progress_since_event(s)) s->clear_frames=saturating_increment(s->clear_frames);
        else s->clear_frames=0U;
        if(s->clear_frames>=RING_CORNER_CLEAR_FRAMES) {
            if(s->corner_stage==5U) {complete(s,now);return;}
            s->corner_stage=3U;s->entry_passed=1U;
            s->corner_guide_id=RING_GUIDE_NONE;s->corner_have_guide=0U;s->line_active=0U;
            s->corner_event_travel=s->travel_counts;s->corner_event_yaw=s->yaw_progress_deg;
            set_phase(s,RING_STATE_INSIDE,now);
        }
        return;
    }
    if(s->corner_stage==3U) {
        if(pair_is(f,1U) && progress_since_event(s) && vote(s,f,RING_GUIDE_270_EXTEND)) {
            s->corner_stage=4U;s->exit_seen=1U;
            s->corner_event_travel=s->travel_counts;s->corner_event_yaw=s->yaw_progress_deg;
            set_phase(s,RING_STATE_EXITING,now);select_guide(s,f,RING_GUIDE_270_EXTEND,now);
            s->line_started_ms=now;s->line_started_counts=s->travel_counts;
        } else if(!pair_is(f,1U) && progress_since_event(s))
            miss_270_vote(s,f,RING_GUIDE_270_EXTEND,1U);
        else if(!progress_since_event(s)) clear_vote(s);
    }
}
static void observe_360(ring_control_state_t *s,const track_ring_observation_t *v,uint32_t now)
{
    track_consumed(s,v);
    if(s->corner_stage==4U) {
        if(s->corner_passed && clear_road(v)) s->clear_frames=saturating_increment(s->clear_frames);
        else s->clear_frames=0U;
        if(s->clear_frames>=RING_CORNER_CLEAR_FRAMES) complete(s,now);
        return;
    }
    uint8_t id=choose_360(s,&v->corners,(uint8_t)(s->corner_stage+1U));
    if(id==RING_GUIDE_NONE) {
        uint8_t expected=(uint8_t)(s->corner_stage==1U ? RING_GUIDE_B_BRANCH_LEFT :
                                   (s->corner_stage==2U ? RING_GUIDE_A_LEFT : RING_GUIDE_B_MAIN_LEFT));
        if(pair_is(&v->corners,0U) && s->corner_vote_frames && !s->corner_vote_misses &&
           s->last_frame_ms-s->corner_vote_ms<=RING_VISION_STALE_MS &&
           (s->corner_vote_guide==expected || s->corner_vote_guide==expected+1U))
            s->corner_vote_misses=1U;
        else clear_vote(s);
        return;
    }
    if(!vote(s,&v->corners,id)) return;
    ++s->corner_stage;
    if(s->corner_stage==2U) {s->entry_passed=1U;set_phase(s,RING_STATE_INSIDE,now);}
    else if(s->corner_stage==3U) set_phase(s,RING_STATE_EXIT_APPROACH,now);
    else {s->exit_seen=1U;set_phase(s,RING_STATE_EXITING,now);}
    consume_corner(s,&v->corners,id,now);
}
void ring_control_observe(ring_control_state_t *s,const track_ring_observation_t *v,uint32_t now,uint8_t running)
{
    if(s==NULL || v==NULL) return;
    if(!RING_FEATURE_ENABLED) {disabled_reset(s);return;}
    const ring_corner_frame_t *f=&v->corners;
    if(s->have_frame && ((int32_t)(f->frame_number-s->corner_frame_number)<=0 ||
       (int32_t)(now-s->last_frame_ms)<0)) return;
    s->corner_frame_number=f->frame_number;s->last_frame_ms=now;s->have_frame=1U;
    s->corner_counts[0]=f->count[0];s->corner_counts[1]=f->count[1];
    s->track_valid=v->path[0].valid;s->vision_candidate=(uint8_t)(f->count[0] || f->count[1]);
    if(!running && s->action_locked && !s->fault) reset_event(s,now);
    if(s->fault) return;
    if(s->phase==RING_STATE_COOLDOWN) {
        if(clear_road(v)) s->clear_frames=saturating_increment(s->clear_frames);else s->clear_frames=0U;
        if((int32_t)(now-s->cooldown_until_ms)>=0 && s->clear_frames>=RING_CORNER_CLEAR_FRAMES) reset_event(s,now);
        return;
    }
    if(s->action_locked) {
        if(ambiguous(f)) {clear_vote(s);return;}
        if(s->kind==TRACK_RING_270) observe_270(s,v,now);else observe_360(s,v,now);
        return;
    }
    /* Bilateral quantity evidence owns classification, independent of angle.
     * A far A is only a candidate; it cannot turn the car into the ring early. */
    if(pair_is(f,1U)) {
        s->type_vote=TRACK_RING_270;s->miss_frames=0U;
        s->kind=TRACK_RING_270;s->phase=RING_STATE_CANDIDATE;
        if(vote(s,f,RING_GUIDE_270_EXTEND) && running) start_action(s,TRACK_RING_270,f,RING_GUIDE_270_EXTEND,now);
        return;
    }
    if(f->count[0] && f->count[1]) {
        s->type_vote=TRACK_RING_270;s->miss_frames=0U;
        clear_vote(s);s->kind=TRACK_RING_UNKNOWN;s->phase=RING_STATE_CANDIDATE;return;
    }
    /* Do not reinterpret a brief one-sided occlusion of a bilateral candidate
     * as a 360 entry. This latch is evidence hysteresis, not an event timer. */
    if(s->type_vote==TRACK_RING_270 && s->miss_frames<RING_PREP_MISS_FRAMES) {
        ++s->miss_frames;
        miss_270_vote(s,f,RING_GUIDE_270_EXTEND,1U);
        return;
    }
    s->type_vote=TRACK_RING_UNKNOWN;
    uint8_t id=choose_360(s,f,1U);
    if(id!=RING_GUIDE_NONE) {
        s->kind=TRACK_RING_360;s->phase=RING_STATE_CANDIDATE;
        if(vote(s,f,id) && running) {
            start_action(s,TRACK_RING_360,f,id,now);
            s->direction=f->guide[id].corner_side==0U ? 1 : -1;
            consume_corner(s,f,id,now);
        }
    } else {
        /* One missed A observation may separate real votes. It adds no vote,
         * cannot change side/class, and cannot keep an old candidate alive. */
        uint8_t old=s->corner_vote_guide;
        if((old==RING_GUIDE_A_LEFT || old==RING_GUIDE_A_RIGHT) && s->corner_vote_frames &&
           !s->corner_vote_misses && now-s->corner_vote_ms<=RING_VISION_STALE_MS &&
           pair_is(f,0U)) {s->corner_vote_misses=1U;return;}
        clear_vote(s);s->kind=TRACK_RING_UNKNOWN;s->phase=s->vision_candidate ? RING_STATE_CANDIDATE : RING_STATE_IDLE;
    }
}
static uint8_t guide_matches_active(const ring_control_state_t *s,const ring_corner_frame_t *f)
{
    uint8_t id=s->corner_guide_id;
    if(!guide_valid(f,id)) return 0U;
    if(s->kind==TRACK_RING_270)
        return id==RING_GUIDE_270_EXTEND ? pair_is(f,1U) : pair_is(f,2U);
    const ring_corner_guide_t *g=&f->guide[id];
    return (uint8_t)(!s->corner_passed && g->corner_side==s->corner_consumed_side &&
                    g->corner_kind==s->corner_consumed_kind &&
                    same_position(g->corner_x,g->corner_y,s->corner_x,s->corner_y,f));
}
track_path_observation_t ring_control_select_path(ring_control_state_t *s,const track_ring_observation_t *v)
{
    track_path_observation_t invalid=invalid_path();
    if(s==NULL || v==NULL) return invalid;
    if(!RING_FEATURE_ENABLED) {disabled_reset(s);return v->path[TRACK_PATH_NORMAL];}
    s->control_refresh=0U;s->corner_guide_held=0U;
    if(s->fault) {s->selected_path=invalid;return invalid;}
    if(!s->action_locked) {
        uint8_t mode=TRACK_PATH_NORMAL;
        if(s->have_frame && v->corners.frame_number==s->corner_frame_number &&
           s->last_tick_ms-s->last_frame_ms<=RING_VISION_STALE_MS &&
           (v->approach_source==TRACK_PATH_LEFT || v->approach_source==TRACK_PATH_RIGHT) &&
           v->approach_path.valid && v->approach_path.confidence>=85U)
            mode=v->approach_source;
        if(mode!=s->path_mode) s->reset_tracking_history=1U;
        s->line_active=0U;s->path_mode=mode;
        s->selected_path=mode==TRACK_PATH_NORMAL ? v->path[TRACK_PATH_NORMAL] : v->approach_path;
        return s->selected_path;
    }
    uint32_t now=s->last_tick_ms;
    if(v->corners.frame_number!=s->corner_frame_number || now-s->last_frame_ms>RING_VISION_STALE_MS ||
       now-s->event_started_ms>RING_ACTIVE_TIMEOUT_MS) {fail(s);return invalid;}
    /* A confirmed 270 transaction already owns this crossing. Its next 1/1
     * pattern provides a CURRENT-frame straight boundary while exit-event
     * confirmation collects three frames; otherwise ordinary vision can fail
     * at the open mouth before the exit is ever admitted. No event is consumed. */
    if(s->kind==TRACK_RING_270 && s->corner_stage==3U && pair_is(&v->corners,1U) &&
       progress_since_event(s) && guide_valid(&v->corners,RING_GUIDE_270_EXTEND)) {
        if(s->corner_guide_id!=RING_GUIDE_270_EXTEND) {
            s->line_started_ms=now;s->line_started_counts=s->travel_counts;
            s->reset_tracking_history=1U;
        }
        select_guide(s,&v->corners,RING_GUIDE_270_EXTEND,s->last_frame_ms);
        s->line_active=1U;s->path_mode=TRACK_PATH_STRAIGHT;return s->selected_path;
    }
    /* A newly visible 360 corner already has a current-frame branch guide.
     * After two consistent votes use it for steering while the third vote
     * confirms the event. This avoids using the old ordinary midpoint at the
     * open mouth, without consuming the corner or changing the event order. */
    if(s->kind==TRACK_RING_360 && s->corner_stage<4U && s->corner_vote_frames>=2U &&
       !s->corner_vote_misses) {
        uint8_t id=choose_360(s,&v->corners,(uint8_t)(s->corner_stage+1U));
        if(id!=RING_GUIDE_NONE && id==s->corner_vote_guide) {
            if(s->corner_guide_id!=id) {
                s->line_started_ms=now;s->line_started_counts=s->travel_counts;
                s->reset_tracking_history=1U;
            }
            select_guide(s,&v->corners,id,s->last_frame_ms);
            s->line_active=1U;s->path_mode=TRACK_PATH_STRAIGHT;return s->selected_path;
        }
    }
    uint8_t normal_phase=(uint8_t)((s->kind==TRACK_RING_270 && s->corner_stage==3U) ||
                                  (s->kind==TRACK_RING_360 && s->corner_passed));
    /* A one-frame false clear cannot cancel a confirmed 360 boundary. Passage
     * already requires consecutive missing observations AND physical progress. */
    /* During the 270 exit vote, keep the provisional boundary through one
     * missing corner frame. Otherwise a single false clear swaps steering to
     * the junction-biased ordinary midpoint and back on the next image. */
    uint8_t provisional_270_miss=(uint8_t)(s->kind==TRACK_RING_270 && s->corner_stage==3U &&
        s->corner_guide_id==RING_GUIDE_270_EXTEND && s->corner_have_guide &&
        s->corner_vote_guide==RING_GUIDE_270_EXTEND && s->corner_vote_misses==1U);
    uint8_t provisional_360_miss=(uint8_t)(s->kind==TRACK_RING_360 && s->corner_stage<4U &&
        s->corner_have_guide && s->corner_guide_id==s->corner_vote_guide &&
        s->corner_vote_frames>=2U && s->corner_vote_misses==1U);
    if(!provisional_270_miss && !provisional_360_miss &&
       (normal_phase || (s->kind!=TRACK_RING_360 && clear_road(v))) && v->path[0].valid) {
        if(s->kind==TRACK_RING_270 && s->corner_stage==3U) {
            if(s->line_active) s->reset_tracking_history=1U;
            s->corner_guide_id=RING_GUIDE_NONE;s->corner_have_guide=0U;
        }
        s->line_active=0U;s->path_mode=TRACK_PATH_NORMAL;s->selected_path=v->path[0];return s->selected_path;
    }
    /* C14 bypasses automatic stopping, not the validity lifetime of a virtual
     * boundary. Never resurrect an expired line on the following ISR tick. */
    if(s->corner_guide_id!=RING_GUIDE_NONE &&
       (now-s->line_started_ms>RING_LINE_MAX_MS ||
        s->travel_counts-s->line_started_counts>RING_LINE_MAX_COUNTS)) {fail(s);return invalid;}
    if(s->corner_guide_id!=RING_GUIDE_NONE && guide_matches_active(s,&v->corners)) {
        select_guide(s,&v->corners,s->corner_guide_id,s->last_frame_ms);
        s->line_active=1U;s->path_mode=TRACK_PATH_STRAIGHT;return s->selected_path;
    }
    if(s->corner_have_guide && now-s->corner_last_guide_ms<=RING_CORNER_HOLD_MS) {
        const ring_corner_guide_t *g=&s->corner_selected_guide;
        s->selected_path=(track_path_observation_t){g->error,g->heading,1U,(uint8_t)(g->confidence/2U)};
        s->corner_guide_held=1U;s->line_active=1U;s->path_mode=TRACK_PATH_STRAIGHT;return s->selected_path;
    }
    fail(s);return invalid;
}
