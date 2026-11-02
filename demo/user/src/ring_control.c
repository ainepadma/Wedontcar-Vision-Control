#include "ring_control.h"
#include <math.h>
#include <string.h>
static track_path_observation_t invalid(void) {track_path_observation_t p={0,0,0,0};return p;}
void ring_control_init(ring_control_state_t *s)
{if(s) {memset(s,0,sizeof(*s));s->protection_enabled=1;}}
static void fail(ring_control_state_t *s)
{
    s->selected_path=invalid();s->line_active=0;
    if(s->protection_enabled) {s->fault_resume_phase=s->phase;s->phase=RING_STATE_FAULT;s->fault=1;}
}
void ring_control_set_protection(ring_control_state_t *s,uint8_t enabled)
{
    if(!s) return;
    s->protection_enabled=enabled;
    if(!enabled && s->fault) {s->phase=s->fault_resume_phase;s->fault=0;s->control_refresh=1;}
}
uint8_t ring_control_guidance_active(const ring_control_state_t *s)
{return (uint8_t)(RING_FEATURE_ENABLED && s && s->action_locked && !s->fault);}
float ring_control_line_x(const ring_control_state_t *s,float row)
{(void)row;return s ? s->selected_path.error : 0.f;}
void ring_control_360_motion(ring_control_state_t *s,float c,float l,float r,float speed,float *angle,float *out)
{(void)s;(void)c;(void)l;(void)r;(void)speed;(void)angle;(void)out;}
void ring_control_tick(ring_control_state_t *s,uint8_t running,uint8_t fresh,float rate,float distance,uint32_t now)
{
    if(!s) return;
    if(!RING_FEATURE_ENABLED) {ring_control_init(s);return;}
    unsigned dt=s->have_tick ? now-s->last_tick_ms : 0;s->last_tick_ms=now;s->have_tick=1;
    if(!running) {if(s->action_locked && !s->fault) ring_control_init(s);return;}
    /* Diagnostics only. No encoder or gyro thresholds advance visual states. */
    if(isfinite(distance) && distance>0) s->travel_counts+=distance;
    if(fresh && isfinite(rate) && dt<100) s->yaw_progress_deg+=rate*dt*.001f;
    if(s->action_locked && !s->fault &&
       (now-s->last_frame_ms>RING_VISION_STALE_MS || now-s->event_started_ms>RING_ACTIVE_TIMEOUT_MS)) fail(s);
}
static void stage(ring_control_state_t *s,unsigned value,uint32_t now)
{
    s->corner_stage=(uint8_t)value;s->transition_frames=0;s->clear_frames=0;s->phase_started_ms=now;
    s->reset_tracking_history=1;s->control_refresh=1;
    s->phase=value>=5 ? RING_STATE_INSIDE : RING_STATE_ENTERING;
    if(value==7) {s->phase=RING_STATE_EXITING;s->exit_seen=1;}
}
static int confirmed(ring_control_state_t *s,int condition)
{
    if(!condition) {s->transition_frames=0;return 0;}
    if(s->transition_frames<255) ++s->transition_frames;
    return s->transition_frames>=RING_CONFIRM_FRAMES;
}
static void complete(ring_control_state_t *s)
{
    s->completed_mask|=s->kind==TRACK_RING_360 ? RING_DONE_360 : RING_DONE_270;
    if(s->kind==TRACK_RING_360) ++s->completed_360;else ++s->completed_270;
    s->action_locked=0;s->line_active=0;s->corner_guide_id=0;s->corner_stage=0;
    s->phase=RING_STATE_COOLDOWN;s->clear_frames=0;s->vote_frames=0;s->reset_tracking_history=1;
}
void ring_control_observe(ring_control_state_t *s,const track_ring_observation_t *obs,uint32_t now,uint8_t running)
{
    if(!s || !obs || !RING_FEATURE_ENABLED || !running || s->fault) return;
    const ring_corner_frame_t *f=&obs->corners;
    if(s->have_frame && f->frame_number==s->corner_frame_number) return;
    s->have_frame=1;s->corner_frame_number=f->frame_number;s->last_frame_ms=now;
    if(s->phase==RING_STATE_COOLDOWN) {
        if(confirmed(s,f->road_clear)) {s->phase=RING_STATE_IDLE;s->transition_frames=0;}
        return;
    }
    if(!s->action_locked) {
        unsigned kind=f->entry[0] || f->entry[1] ? TRACK_RING_360 :
            (f->cross_entry && f->guide[RING_GUIDE_270_EXTEND].valid ? TRACK_RING_270 : 0);
        unsigned side=f->entry[1] ? 1 : 0;
        if(!kind) {s->vote_frames=0;return;}
        if(kind!=s->vote_kind || side!=s->vote_side) s->vote_frames=0;
        s->vote_kind=(uint8_t)kind;s->vote_side=(uint8_t)side;
        if(++s->vote_frames<RING_CONFIRM_FRAMES) return;
        s->kind=(uint8_t)kind;s->direction=side ? -1 : 1;s->action_locked=1;
        s->entry_passed=0;s->exit_seen=0;s->exit_gap_seen=0;
        s->corner_have_guide=0;s->corner_guide_id=0;s->selected_path=invalid();
        s->entry_seen=1;s->event_started_ms=now;++s->event_id;stage(s,1,now);return;
    }
    unsigned in=s->direction==1 ? 0 : 1,out=1-in;
    if(s->kind==TRACK_RING_270) {
        /* 270: repair entry, reacquire road, repair the later exit, reacquire.
         * An uninterrupted opening cannot count as both entry and exit. */
        switch(s->corner_stage) {
        case 1: if(confirmed(s,f->road_clear)) {stage(s,2,now);s->phase=RING_STATE_INSIDE;s->entry_passed=1;}break;
        case 2: if(confirmed(s,f->cross_entry && f->guide[RING_GUIDE_270_EXTEND].valid)) {stage(s,3,now);s->exit_seen=1;}break;
        case 3: if(confirmed(s,f->road_clear)) complete(s);break;
        default: break;
        }
        return;
    }
    /* PDF section 1, Figs 1.6--1.15. One switch dispatch per fresh frame:
     * cannot cascade or skip stages, cannot relabel direction mid-ring. */
    switch(s->corner_stage) {
    case 1: if(confirmed(s,!f->a[in].quality && f->bottom_open[in] && f->p[in].quality && f->continuous[out])) stage(s,2,now);break;
    case 2: if(confirmed(s,f->v[in].quality && !f->p[in].quality && f->continuous[out])) stage(s,3,now);break;
    case 3: if(confirmed(s,!f->v[in].quality && ((f->v[out].quality && f->v[out].y>f->rows*.35f) || f->paired))) stage(s,4,now);break;
    case 4: if(confirmed(s,f->paired && (!f->v[out].quality || f->v[out].y<f->rows*.25f))) {stage(s,5,now);s->entry_passed=1;}break;
    case 5:
        if(f->gap[out]) s->exit_gap_seen=1;
        if(confirmed(s,s->exit_gap_seen && f->bottom_open[in] && !f->bottom_open[out])) stage(s,6,now);
        break;
    case 6: if(confirmed(s,f->gap[in] && f->breakpoint[in].quality)) stage(s,7,now);break;
    case 7: if(confirmed(s,f->road_clear)) complete(s);break;
    default: break;
    }
}
track_path_observation_t ring_control_select_path(ring_control_state_t *s,const track_ring_observation_t *obs)
{
    if(!s || !obs) return invalid();
    s->control_refresh=0;s->corner_guide_held=0;
    if(!RING_FEATURE_ENABLED || !s->action_locked) {
        s->line_active=0;s->corner_guide_id=0;s->path_mode=TRACK_PATH_NORMAL;
        s->selected_path=obs->path[0];return s->selected_path;
    }
    if(s->fault || obs->corners.frame_number!=s->corner_frame_number) return invalid();
    const ring_corner_frame_t *f=&obs->corners;
    unsigned id=s->kind==TRACK_RING_360 ? ring_pdf_guide(s->corner_stage,s->direction==1 ? 0 : 1) :
        (s->corner_stage==2 ? RING_GUIDE_NONE : RING_GUIDE_270_EXTEND);
    const ring_corner_guide_t *g=&f->guide[id];
    if(id && g->valid) {
        s->corner_selected_guide=*g;s->corner_guide_id=(uint8_t)id;s->corner_have_guide=1;
        s->corner_guide_frame_number=f->frame_number;s->corner_last_guide_ms=s->last_frame_ms;
        s->line_active=(uint8_t)(g->boundary[0].valid || g->boundary[1].valid);
        s->path_mode=s->line_active ? TRACK_PATH_STRAIGHT : TRACK_PATH_NORMAL;
        s->selected_path=(track_path_observation_t){g->error,g->heading,g->valid,g->confidence};return s->selected_path;
    }
    /* Clear in-ring rows may use ordinary vision. Missing repair geometry must
     * not silently change branches or draw an old-frame line as a current one. */
    if((s->kind==TRACK_RING_270 && s->corner_stage==2) ||
       (s->kind==TRACK_RING_360 && (s->corner_stage==5 || s->corner_stage==6))) {
        s->line_active=0;s->corner_guide_id=0;s->path_mode=TRACK_PATH_NORMAL;
        s->selected_path=obs->path[0];return s->selected_path;
    }
    if(s->corner_have_guide && s->last_frame_ms-s->corner_last_guide_ms<=RING_CORNER_HOLD_MS) {
        s->corner_guide_held=1;return s->selected_path;
    }
    s->selected_path=invalid();return s->selected_path;
}
