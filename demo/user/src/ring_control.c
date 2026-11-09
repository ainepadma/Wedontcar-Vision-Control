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
uint8_t ring_control_path_source(const ring_control_state_t *s)
{
    if(!s || s->fault || !s->selected_path.valid) return RING_SOURCE_INVALID;
    if(s->corner_guide_held) return RING_SOURCE_HOLD;
    return s->line_active && s->corner_guide_id ? RING_SOURCE_GUIDE : RING_SOURCE_NORMAL;
}
const char *ring_control_stage_name(uint8_t value)
{
    static const char *const names[]={"IDLE","ENTRY","PREIN","INLOCK","JOIN","RING","EXITWAIT","EXIT"};
    return value<8U ? names[value] : "INVALID";
}
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
    if(value==7) {
        s->phase=RING_STATE_EXITING;s->exit_seen=1;
        s->corner_have_guide=0; /* S7 may only hold its own inner connection. */
        s->exit_road_pending=0;s->exit_road_frames=0;s->exit_lane_frames=0;
    }
}
static int confirmed(ring_control_state_t *s,int condition)
{
    if(!condition) {s->transition_frames=0;return 0;}
    if(s->transition_frames<255) ++s->transition_frames;
    return s->transition_frames>=RING_CONFIRM_FRAMES;
}
static int same_exit_tip(const ring_corner_frame_t *f,const ring_corner_point_t *tip,float x,float y)
{
    return fabsf(tip->x-x)<=f->cols*RING_EXIT_TIP_X_FRACTION &&
           fabsf(tip->y-y)<=f->rows*RING_EXIT_TIP_Y_FRACTION;
}
/* Called once per fresh observation. A distant vertex cannot replace an
 * active tip during its hold interval; after expiry it must confirm anew. */
static void observe_exit_tip(ring_control_state_t *s,const ring_corner_frame_t *f,unsigned in,uint32_t now)
{
    const ring_corner_point_t *tip=&f->exit_v[in];
    if(!tip->quality || !f->guide[ring_pdf_guide(7,in)].valid) {
        s->exit_candidate_frames=0;return;
    }
    if(s->exit_tip_valid && (uint32_t)(now-s->exit_tip_ms)<=RING_CORNER_HOLD_MS) {
        s->exit_candidate_frames=0;
        if(!same_exit_tip(f,tip,s->exit_tip_x,s->exit_tip_y)) return;
    } else {
        if(s->exit_candidate_frames && !same_exit_tip(f,tip,s->exit_candidate_x,s->exit_candidate_y))
            s->exit_candidate_frames=0;
        s->exit_candidate_x=tip->x;s->exit_candidate_y=tip->y;
        if(++s->exit_candidate_frames<RING_CONFIRM_FRAMES) return;
        s->exit_candidate_frames=0;
    }
    s->exit_tip_x=tip->x;s->exit_tip_y=tip->y;s->exit_tip_valid=1;
    s->exit_tip_frame=f->frame_number;s->exit_tip_ms=now;
}
static int exit_guide_matches(const ring_control_state_t *s,const ring_corner_frame_t *f,unsigned in)
{
    return s->exit_tip_valid && f->exit_v[in].quality && s->exit_tip_frame==f->frame_number;
}
static int exit_errors_conflict(int a,int b)
{
    return (a>RING_EXIT_NEAR_SIGN_DEADBAND && b<-RING_EXIT_NEAR_SIGN_DEADBAND) ||
           (a<-RING_EXIT_NEAR_SIGN_DEADBAND && b>RING_EXIT_NEAR_SIGN_DEADBAND);
}
static int exit_lane_visible(const track_ring_observation_t *obs)
{
    const ring_corner_frame_t *f=&obs->corners;
    const track_path_observation_t *p=&obs->path[TRACK_PATH_NORMAL];
    if(!f->road_clear || f->exit_white_band || f->entry[0] || f->entry[1] ||
       f->cross_entry || !p->valid || p->confidence<RING_EXIT_ROAD_MIN_CONFIDENCE) return 0;
    if(obs->handoff_near_valid && exit_errors_conflict(p->error,obs->handoff_near_error)) return 0;
    return 1;
}
/* Alignment permits releasing the ring speed limit. A visible lane can
 * already steer the car towards its centre before these bounds are met. */
static int exit_lane_aligned(const track_ring_observation_t *obs)
{
    const track_path_observation_t *p=&obs->path[TRACK_PATH_NORMAL];
    if(obs->handoff_near_valid) {
        int e=obs->handoff_near_error,h=obs->handoff_near_heading;
        if(e<-RING_EXIT_ROAD_MAX_ERROR || e>RING_EXIT_ROAD_MAX_ERROR ||
           h<-RING_EXIT_ROAD_MAX_HEADING || h>RING_EXIT_ROAD_MAX_HEADING) return 0;
    }
    if(obs->handoff_bottom_valid && (obs->handoff_bottom_error<-RING_EXIT_BOTTOM_MAX_ERROR ||
       obs->handoff_bottom_error>RING_EXIT_BOTTOM_MAX_ERROR)) return 0;
    return p->error>=-RING_EXIT_ROAD_MAX_ERROR && p->error<=RING_EXIT_ROAD_MAX_ERROR &&
        p->heading>=-RING_EXIT_ROAD_MAX_HEADING && p->heading<=RING_EXIT_ROAD_MAX_HEADING;
}
static void complete(ring_control_state_t *s,uint32_t now)
{
    s->completed_mask|=s->kind==TRACK_RING_360 ? RING_DONE_360 : RING_DONE_270;
    if(s->kind==TRACK_RING_360) ++s->completed_360;else ++s->completed_270;
    s->action_locked=0;s->line_active=0;s->corner_guide_id=0;s->corner_stage=0;
    s->phase=RING_STATE_COOLDOWN;s->clear_frames=0;s->vote_frames=0;s->transition_frames=0;
    /* S7 is already steering on the normal lane when it releases. */
    if(s->kind!=TRACK_RING_360) s->reset_tracking_history=1;
    s->phase_started_ms=now;s->exit_road_frames=0;s->exit_road_started_ms=0;
    s->corner_have_guide=0;s->corner_guide_held=0;
    s->control_refresh=(uint8_t)(s->kind!=TRACK_RING_360);
}
void ring_control_observe(ring_control_state_t *s,const track_ring_observation_t *obs,uint32_t now,uint8_t running)
{
    if(!s || !obs || !RING_FEATURE_ENABLED || !running || s->fault) return;
    const ring_corner_frame_t *f=&obs->corners;
    if(s->have_frame && f->frame_number==s->corner_frame_number) return;
    if(s->have_frame && (uint32_t)(now-s->last_frame_ms)>RING_VISION_STALE_MS) {
        s->exit_road_frames=0;s->clear_frames=0;s->transition_frames=0;s->bridge_clear_frames=0;
        s->exit_candidate_frames=0;s->exit_lane_frames=0;
    }
    s->have_frame=1;s->corner_frame_number=f->frame_number;s->last_frame_ms=now;
    if(s->phase==RING_STATE_COOLDOWN) {
        if(s->kind==TRACK_RING_360) {
            if(exit_lane_visible(obs) && exit_lane_aligned(obs)) {if(s->clear_frames<255U) ++s->clear_frames;}
            else s->clear_frames=0;
            if(s->clear_frames>=RING_360_REARM_FRAMES &&
               (uint32_t)(now-s->phase_started_ms)>=RING_360_REARM_MS) {
                s->phase=RING_STATE_IDLE;s->clear_frames=0;s->vote_frames=0;
            }
        } else if(confirmed(s,f->road_clear)) {s->phase=RING_STATE_IDLE;s->transition_frames=0;}
        return;
    }
    if(!s->action_locked) {
        unsigned kind=f->cross_entry && f->guide[RING_GUIDE_270_EXTEND].valid ? TRACK_RING_270 :
            (f->entry[0] || f->entry[1] ? TRACK_RING_360 : 0);
        unsigned side=f->entry[1] ? 1 : 0;
        if(!kind) {s->vote_frames=0;return;}
        if(kind!=s->vote_kind || side!=s->vote_side) s->vote_frames=0;
        s->vote_kind=(uint8_t)kind;s->vote_side=(uint8_t)side;
        if(++s->vote_frames<RING_CONFIRM_FRAMES) return;
        s->kind=(uint8_t)kind;s->direction=side ? -1 : 1;s->action_locked=1;
        s->entry_passed=0;s->exit_seen=0;s->exit_gap_seen=0;
        s->bridge_seen=0;s->bridge_passed=0;s->exit_armed=0;s->bridge_clear_frames=0;
        s->separator_seen=0;s->separator_frame=0;
        s->exit_tip_valid=0;s->exit_candidate_frames=0;
        s->corner_have_guide=0;s->corner_guide_id=0;s->selected_path=invalid();
        s->exit_road_frames=0;
        s->entry_seen=1;s->event_started_ms=now;++s->event_id;stage(s,1,now);return;
    }
    unsigned in=s->direction==1 ? 0 : 1,out=1-in;
    if(s->kind==TRACK_RING_270) {
        /* 270: repair entry, reacquire road, repair the later exit, reacquire.
         * An uninterrupted opening cannot count as both entry and exit. */
        switch(s->corner_stage) {
        case 1: if(confirmed(s,f->road_clear)) {stage(s,2,now);s->phase=RING_STATE_INSIDE;s->entry_passed=1;}break;
        case 2: if(confirmed(s,f->cross_entry && f->guide[RING_GUIDE_270_EXTEND].valid)) {stage(s,3,now);s->exit_seen=1;}break;
        case 3: if(confirmed(s,f->road_clear)) complete(s,now);break;
        default: break;
        }
        return;
    }
    /* PDF section 1, Figs 1.6--1.15. One switch dispatch per fresh frame:
     * cannot cascade or skip stages, cannot relabel direction mid-ring. */
    switch(s->corner_stage) {
    case 1: if(confirmed(s,!f->a[in].quality && f->bottom_open[in] && f->p[in].quality && f->continuous[out])) stage(s,2,now);break;
    case 2: if(confirmed(s,f->v[in].quality && !f->p[in].quality && f->continuous[out])) stage(s,3,now);break;
    case 3: if(confirmed(s,(!f->v[in].quality || (f->v_reacquired&(1U<<in))) && f->v[out].quality && !(f->v_reacquired&(1U<<out)) && f->v[out].y>f->rows*.25f)) stage(s,4,now);break;
    case 4: if(confirmed(s,f->paired && ((!f->v[out].quality || f->v[out].y<f->rows*.25f) ||
        (!f->v[in].quality && !f->a[in].quality && f->continuous[out] && f->bridge_tip[out].quality &&
         f->guide[ring_pdf_guide(5,in)].valid && f->guide[ring_pdf_guide(5,in)].boundary[out].valid)))) {
        stage(s,5,now);s->entry_passed=1;
    }break;
    case 5:
    case 6: {
        unsigned out_guide=ring_pdf_guide(5,in);
        int visible=f->guide[out_guide].valid && f->guide[out_guide].boundary[out].valid;
        if(visible) s->bridge_seen=1;
        /* A separator is remembered for this ring only AFTER third repair.
         * Its own frame always stays on the third repair, even with a V. */
        if(s->bridge_seen && !s->separator_seen && f->exit_separator) {
            s->separator_seen=1;s->separator_frame=f->frame_number;
            s->bridge_passed=1;s->exit_armed=1;
            if(s->corner_stage==5) stage(s,6,now);
            break;
        }
        if(s->corner_stage==6 && s->separator_seen && f->frame_number!=s->separator_frame) {
            observe_exit_tip(s,f,in,now);
            s->transition_frames=s->exit_candidate_frames;
            if(s->exit_tip_valid) stage(s,7,now);
        }
        break;
    }
    case 7:
        observe_exit_tip(s,f,in,now);
        /* Skipping path selection for a frame breaks continuous road following. */
        if(s->exit_road_pending) {s->exit_road_frames=0;s->transition_frames=0;s->exit_lane_frames=0;}
        s->exit_road_pending=1;
        break;
    default: break;
    }
}
static track_path_observation_t select_exit_path(ring_control_state_t *s,
                                                  const track_ring_observation_t *obs)
{
    unsigned inner=s->direction==1 ? 0U : 1U;
    unsigned id=ring_pdf_guide(7,inner);
    const ring_corner_frame_t *f=&obs->corners;
    const ring_corner_guide_t *guide=&f->guide[id];
    uint32_t now=s->last_frame_ms;
    int measured=guide->valid && exit_guide_matches(s,f,inner);
    if(measured) {
        s->corner_selected_guide=*guide;
        s->corner_have_guide=1;s->corner_last_guide_ms=now;
        s->corner_guide_frame_number=f->frame_number;
    }
    /* Keep the fourth line on the confirmation frame. During overlap, one
     * flash of straight road cannot displace a usable measured or held line. */
    int fresh=s->exit_road_pending;
    int recent=s->corner_have_guide && now-s->corner_last_guide_ms<=RING_CORNER_HOLD_MS;
    int following=s->exit_lane_frames && s->path_mode==TRACK_PATH_NORMAL;
    int both_near=obs->handoff_near_valid && obs->handoff_bottom_valid;
    int visible=exit_lane_visible(obs) && !(both_near &&
                exit_errors_conflict(obs->handoff_near_error,obs->handoff_bottom_error));
    int aligned=visible && exit_lane_aligned(obs);
    int can_follow=visible && (aligned || (both_near && (!measured || following))) &&
                   (following || !recent || both_near);
    if(fresh) {
        if(!can_follow) s->exit_lane_frames=0;
        else if(s->exit_lane_frames<RING_EXIT_LANE_CONFIRM_FRAMES) ++s->exit_lane_frames;
    }
    int use_road=s->exit_lane_frames && can_follow &&
                 (following || !recent || s->exit_lane_frames>=RING_EXIT_LANE_CONFIRM_FRAMES);
    s->exit_road_pending=0;
    if(use_road) {
        if(s->line_active) s->reset_tracking_history=1;
        s->line_active=0;s->corner_guide_id=0;s->path_mode=TRACK_PATH_NORMAL;
        s->selected_path=obs->path[TRACK_PATH_NORMAL];
        if(!aligned) {
            s->exit_road_frames=0;s->transition_frames=0;
        } else if(fresh) {
            if(!s->exit_road_frames) s->exit_road_started_ms=now;
            if(s->exit_road_frames<255U) ++s->exit_road_frames;
            s->transition_frames=s->exit_road_frames;
            if(s->exit_road_frames>=RING_EXIT_SETTLE_FRAMES &&
               now-s->exit_road_started_ms>=RING_EXIT_SETTLE_MS) {
                complete(s,now);
            }
        }
        return s->selected_path;
    }
    s->exit_road_frames=0;s->transition_frames=0;s->path_mode=TRACK_PATH_STRAIGHT;
    if(recent) {
        if(!s->line_active) s->reset_tracking_history=1;
        s->line_active=1;s->corner_guide_id=(uint8_t)id;
        s->corner_guide_held=(uint8_t)!measured;
        const ring_corner_guide_t *last=&s->corner_selected_guide;
        s->selected_path=(track_path_observation_t){last->error,last->heading,last->valid,last->confidence};
    } else {
        s->line_active=0;s->corner_guide_id=0;s->corner_have_guide=0;
        s->selected_path=invalid();
    }
    return s->selected_path;
}
track_path_observation_t ring_control_select_path(ring_control_state_t *s,const track_ring_observation_t *obs)
{
    if(!s || !obs) return invalid();
    s->control_refresh=0;s->corner_guide_held=0;
    if(!RING_FEATURE_ENABLED || !s->action_locked) {
        s->line_active=0;s->corner_guide_id=0;s->path_mode=TRACK_PATH_NORMAL;
        s->selected_path=obs->path[0];return s->selected_path;
    }
    if(s->fault || obs->corners.frame_number!=s->corner_frame_number) {
        s->selected_path=invalid();s->line_active=0;s->corner_guide_id=0;s->corner_have_guide=0;
        return s->selected_path;
    }
    const ring_corner_frame_t *f=&obs->corners;
    if(s->kind==TRACK_RING_360 && s->corner_stage==7) return select_exit_path(s,obs);
    unsigned id=s->kind==TRACK_RING_360 ? ring_pdf_guide(s->corner_stage,s->direction==1 ? 0 : 1) :
        (s->corner_stage==2 ? RING_GUIDE_NONE :
         (f->guide[RING_GUIDE_270_CONNECT].valid ? RING_GUIDE_270_CONNECT : RING_GUIDE_270_EXTEND));
    if(s->kind==TRACK_RING_360 && s->corner_stage==3 && !f->guide[id].valid &&
       f->v[s->direction==1 ? 1U : 0U].quality) {
        unsigned next=ring_pdf_guide(4,s->direction==1 ? 0U : 1U);
        if(f->guide[next].valid) id=next;
    }
    const ring_corner_guide_t *g=&f->guide[id];
    if(s->kind==TRACK_RING_360 && (s->corner_stage==5 || s->corner_stage==6) &&
       !g->boundary[0].valid && !g->boundary[1].valid) id=RING_GUIDE_NONE;
    if(id && g->valid) {
        s->corner_selected_guide=*g;s->corner_guide_id=(uint8_t)id;s->corner_have_guide=1;
        s->corner_guide_frame_number=f->frame_number;s->corner_last_guide_ms=s->last_frame_ms;
        s->line_active=(uint8_t)(g->boundary[0].valid || g->boundary[1].valid);
        s->path_mode=s->line_active ? TRACK_PATH_STRAIGHT : TRACK_PATH_NORMAL;
        s->selected_path=(track_path_observation_t){g->error,g->heading,g->valid,g->confidence};return s->selected_path;
    }
    /* S3 owns the entry branch until the existing S4 transition confirms.
     * A brief dropout may reuse ONLY a guide previously accepted in S3
     * (including its same-direction S4 continuation). Never reuse S2 or
     * silently steer down an ordinary fork. Do not refresh the hold deadline. */
    if(s->kind==TRACK_RING_360 && s->corner_stage==3) {
        unsigned side=s->direction==1 ? 0U : 1U;
        if(s->corner_have_guide && s->selected_path.valid &&
           (s->corner_guide_id==ring_pdf_guide(3,side) ||
            s->corner_guide_id==ring_pdf_guide(4,side)) &&
           (uint32_t)(s->last_frame_ms-s->corner_last_guide_ms)<=RING_CORNER_HOLD_MS) {
            s->corner_guide_held=1;s->line_active=1;s->path_mode=TRACK_PATH_STRAIGHT;
            return s->selected_path;
        }
        s->corner_have_guide=0;s->corner_guide_id=0;s->line_active=0;
        s->path_mode=TRACK_PATH_STRAIGHT;s->selected_path=invalid();
        return s->selected_path;
    }
    /* Once the third repair owns the fork, never fall back to the discarded
     * ordinary branch. A short dropout can hold only this third repair. */
    if(s->kind==TRACK_RING_360 && (s->corner_stage==5 || s->corner_stage==6) && s->bridge_seen) {
        unsigned side=s->direction==1 ? 0U : 1U;
        if(s->corner_have_guide && s->selected_path.valid &&
           (s->corner_guide_id==ring_pdf_guide(5,side) || s->corner_guide_id==ring_pdf_guide(6,side)) &&
           (uint32_t)(s->last_frame_ms-s->corner_last_guide_ms)<=RING_CORNER_HOLD_MS) {
            s->corner_guide_held=1;s->line_active=1;s->path_mode=TRACK_PATH_STRAIGHT;
            return s->selected_path;
        }
        s->corner_have_guide=0;s->corner_guide_id=0;s->line_active=0;
        s->path_mode=TRACK_PATH_STRAIGHT;s->selected_path=invalid();return s->selected_path;
    }
    /* Clear in-ring rows may use ordinary vision. Missing repair geometry must
     * not silently change branches or draw an old-frame line as a current one. */
    if((s->kind==TRACK_RING_270 && s->corner_stage==2) ||
       (s->kind==TRACK_RING_360 && (s->corner_stage==5 || s->corner_stage==6))) {
        s->line_active=0;s->corner_guide_id=0;s->path_mode=TRACK_PATH_NORMAL;
        s->selected_path=obs->path[0];return s->selected_path;
    }
    /* Never turn a visible main road into an invalid frame just because a
     * ring landmark went missing. Retain the ring phase and direction. */
    s->line_active=0;s->corner_guide_id=0;s->path_mode=TRACK_PATH_NORMAL;
    s->selected_path=obs->path[0];return s->selected_path;
}
