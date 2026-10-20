#include "ring_control.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

static float clampf(float x,float lo,float hi) {return fmaxf(lo,fminf(x,hi));}
static uint8_t elapsed(uint32_t now,uint32_t start,uint32_t duration)
{
    return (uint8_t)((int32_t)(now-start)>=(int32_t)duration);
}
static uint8_t kind_bit(uint8_t kind)
{
    return kind==TRACK_RING_270 ? RING_DONE_270 : (kind==TRACK_RING_360 ? RING_DONE_360 : 0U);
}
static void phase(ring_control_state_t *s,uint8_t p,uint32_t now)
{
    if(s->line_active || (s->action_locked && s->phase!=p)) s->reset_tracking_history=1U;
    s->phase=p;s->phase_started_ms=now;s->clear_frames=0U;s->near_frames=0U;
    s->miss_frames=0U;s->exit_frames=0U;s->turn_frames=0U;
    s->line_active=0U;s->line_locked=0U;s->control_refresh=1U;
    if(p==RING_STATE_ENTERING) s->action_locked=1U;
}
void ring_control_init(ring_control_state_t *s)
{
    if(s!=NULL) {memset(s,0,sizeof(*s));s->protection_enabled=1U;}
}
static void reset_event(ring_control_state_t *s,uint32_t now)
{
    uint32_t id=s->event_id;
    uint8_t remembered=s->remembered_line,reset=s->line_active || (s->action_locked && s->phase!=RING_STATE_COOLDOWN);
    uint8_t protection=s->protection_enabled,done=s->completed_mask;
    uint8_t count270=s->completed_270,count360=s->completed_360;
    float near=s->remembered_near,far=s->remembered_far;
    ring_control_init(s);s->event_id=id;s->remembered_line=remembered;
    s->remembered_near=near;s->remembered_far=far;s->last_tick_ms=now;s->have_tick=1U;
    s->reset_tracking_history=reset;s->protection_enabled=protection;
    s->completed_mask=done;s->completed_270=count270;s->completed_360=count360;
}
void ring_control_set_protection(ring_control_state_t *s,uint8_t enabled)
{
    if(s==NULL) return;
    s->protection_enabled=(uint8_t)(enabled!=0U);
    if(!s->protection_enabled && s->fault) {
        if(s->action_locked) {
            s->fault=0U;s->phase=s->fault_resume_phase;s->line_locked=s->fault_line_locked;
            s->control_refresh=1U;s->reset_tracking_history=1U;
        } else reset_event(s,s->last_tick_ms);
    }
}
static void fail(ring_control_state_t *s)
{
    if(!s->protection_enabled) {
        /* Once an action starts, a missing entrance image must not silently
         * clear T/I/X or abort the transaction. C13 still restarts the run. */
        if(!s->action_locked) reset_event(s,s->last_tick_ms);
        return;
    }
    s->fault_resume_phase=s->phase;s->fault_line_locked=s->line_locked;
    s->fault=1U;s->phase=RING_STATE_FAULT;s->line_active=0U;s->line_locked=0U;
    s->path_mode=TRACK_PATH_NORMAL;s->selected_path=(track_path_observation_t){0,0,0U,0U};
}
uint8_t ring_control_guidance_active(const ring_control_state_t *s)
{
    return (uint8_t)(s!=NULL && s->action_locked && !s->fault &&
        (s->line_active || s->phase==RING_STATE_INSIDE));
}
float ring_control_line_x(const ring_control_state_t *s,float row)
{
    if(s==NULL) return 0.f;
    float t=((float)RING_GUIDE_NEAR_PERCENT-row)/(float)(RING_GUIDE_NEAR_PERCENT-RING_GUIDE_FAR_PERCENT);
    return clampf(s->line_near+t*(s->line_far-s->line_near),-900.f,900.f);
}
void ring_control_tick(ring_control_state_t *s,uint8_t running,uint8_t fresh,
                       float rate,float distance,uint32_t now)
{
    if(s==NULL) return;
    uint32_t dt=s->have_tick ? now-s->last_tick_ms : 0U;
    if(dt>100U) dt=0U;
    s->last_tick_ms=now;s->have_tick=1U;
    uint8_t valid_yaw=(uint8_t)(fresh && isfinite(rate));
    s->yaw_rate_dps=valid_yaw ? rate : 0.f;
    if(valid_yaw) {s->have_yaw=1U;s->last_yaw_ms=now;}
    s->imu_wait=(uint8_t)(!s->have_yaw || now-s->last_yaw_ms>RING_360_IMU_WARN_MS);
    if(!running) {
        if(!s->fault && s->entry_seen && !s->action_locked) reset_event(s,now);
        return;
    }
    if(s->fault) return;
    if(isfinite(distance) && distance>0.f && distance<32768.f) s->travel_counts+=distance;
    if(s->phase==RING_STATE_COOLDOWN) {
        if((int32_t)(now-s->cooldown_until_ms)>=0) reset_event(s,now);
        return;
    }
    if(!s->entry_seen) return;
    if(s->have_frame && now-s->last_frame_ms>RING_VISION_STALE_MS) {fail(s);if(s->fault) return;}
    if(!s->action_locked && s->phase==RING_STATE_APPROACH && now-s->phase_started_ms>RING_PREP_TIMEOUT_MS) {
        reset_event(s,now);return;
    }
    uint8_t waiting_imu=(uint8_t)(s->kind==TRACK_RING_360 && s->phase==RING_STATE_INSIDE && s->imu_wait);
    if(waiting_imu && s->event_started_ms!=0U) s->event_started_ms+=dt;
    if(!waiting_imu && s->event_started_ms!=0U && now-s->event_started_ms>RING_ACTIVE_TIMEOUT_MS) {
        fail(s);if(s->fault) return;
    }
    if(s->line_locked && (now-s->line_started_ms>RING_LINE_MAX_MS ||
       s->travel_counts-s->line_started_counts>RING_LINE_MAX_COUNTS)) {fail(s);if(s->fault) return;}
    if(s->kind==TRACK_RING_360 && s->phase==RING_STATE_INSIDE) {
        /* Missing IMU freezes angle only. Keep the chosen turn until valid
         * calibrated samples return; never integrate an old/guessed rate. */
        if(valid_yaw) s->yaw_progress_deg+=(float)s->direction*rate*(float)dt/1000.f;
        if(s->yaw_progress_deg>=RING_360_TURN_DEG) {
            s->exit_seen=1U;phase(s,RING_STATE_EXITING,now);
        }
    } else if(s->kind==TRACK_RING_270 && valid_yaw && s->entry_passed) {
        if(s->direction==0 && fabsf(rate)>=RING_YAW_START_DPS) {
            int8_t motion=rate>0.f ? 1 : -1;
            if(motion!=s->yaw_vote) {s->yaw_vote=motion;s->yaw_motion_ticks=0U;}
            if(s->yaw_motion_ticks<5U) ++s->yaw_motion_ticks;
            if(s->yaw_motion_ticks>=5U) s->direction=motion;
        }
        if(s->direction!=0) s->yaw_progress_deg+=(float)s->direction*rate*(float)dt/1000.f;
        if(s->yaw_progress_deg>=RING_YAW_TURN_ASSIST_DEG) s->turn_seen=1U;
    }
}
static uint8_t entry_evidence(const track_ring_observation_t *v)
{
    if(v->type_hint==TRACK_RING_360 && v->direction_hint!=0 && v->zore360_match) return 1U;
    if(!v->junction || v->entry_score<70U || v->paired_road) return 0U;
    if(v->type_hint==TRACK_RING_270)
        return (uint8_t)(v->left_open && v->right_open && (v->wide_white || (v->topology[0] && v->topology[1])));
    if(v->type_hint==TRACK_RING_360)
        return (uint8_t)((v->direction_hint>0 && v->left_open && v->edge_line_valid[1]) ||
                         (v->direction_hint<0 && v->right_open && v->edge_line_valid[0]));
    return 0U;
}
static uint8_t forward_support(const track_ring_observation_t *v)
{
    return (uint8_t)(v->straight_support>=55U || v->path[TRACK_PATH_STRAIGHT].valid);
}
static uint8_t followable(const track_ring_observation_t *v)
{
    return (uint8_t)(v->path[0].valid || v->path[1].valid || v->path[2].valid);
}
static uint8_t mouth_near(const ring_control_state_t *s,const track_ring_observation_t *v)
{
    if(s->kind!=TRACK_RING_360) return (uint8_t)(v->junction && v->near_open && forward_support(v));
    return (uint8_t)(v->near_open || (v->junction && v->junction_near>=RING_360_ENTRY_POSITION_MIN));
}
static uint8_t turn_requested(const ring_control_state_t *s,const track_ring_observation_t *v)
{
    if(v->turn_hint==s->direction) return 1U;
    for(unsigned mode=0U;mode<TRACK_PATH_COUNT-1U;++mode) {
        const track_path_observation_t *p=&v->path[mode];
        if(p->valid && -(float)s->direction*((float)p->error+2.f*(float)p->heading)>=RING_360_TURN_TRIGGER_MIXED)
            return 1U;
    }
    return 0U;
}
static void complete(ring_control_state_t *s,uint32_t now)
{
    s->completed_mask|=kind_bit(s->kind);
    if(s->kind==TRACK_RING_360) s->completed_360=1U;else if(s->kind==TRACK_RING_270) s->completed_270=1U;
    phase(s,RING_STATE_COOLDOWN,now);s->cooldown_until_ms=now+RING_COOLDOWN_MS;
}
void ring_control_observe(ring_control_state_t *s,const track_ring_observation_t *v,uint32_t now,uint8_t running)
{
    if(s==NULL || v==NULL) return;
    if(s->have_frame && (int32_t)(now-s->last_frame_ms)<=0) return;
    s->last_frame_ms=now;s->have_frame=1U;s->vision_candidate=v->junction;s->track_valid=v->path[0].valid;
    if(s->fault || (s->action_locked && !running)) return;
    if(!s->entry_seen) {
        s->line_active=0U;s->line_locked=0U;s->path_mode=TRACK_PATH_NORMAL;s->reset_tracking_history=0U;
        if(v->reference_valid && !v->junction && v->paired_road) {
            s->remembered_near=v->reference_near;s->remembered_far=v->reference_far;s->remembered_line=1U;
        }
        if(s->completed_mask&kind_bit(v->type_hint)) {reset_event(s,now);return;}
        uint8_t current360=(uint8_t)(v->type_hint==TRACK_RING_360 && entry_evidence(v));
        uint8_t current270=(uint8_t)(v->type_hint==TRACK_RING_270 && entry_evidence(v));
        if((current360 || s->vote360_history!=0U) && !current270) {
            if(current360 && s->direction!=v->direction_hint) s->vote360_history=0U;
            s->vote360_history=(uint8_t)(((unsigned)s->vote360_history<<1U | current360)&((1U<<RING_360_CONFIRM_WINDOW)-1U));
            if(current360) {s->vote360_misses=0U;s->direction=v->direction_hint;}
            else if(s->vote360_misses<UINT8_MAX) ++s->vote360_misses;
            if(s->vote360_misses>RING_360_CANDIDATE_MISS_MAX || !s->vote360_history) {
                s->vote360_history=0U;s->type_frames=0U;s->type_vote=TRACK_RING_UNKNOWN;s->kind=TRACK_RING_UNKNOWN;
                s->near_frames=0U;s->phase=RING_STATE_IDLE;return;
            }
            unsigned hits=0U;for(unsigned bits=s->vote360_history;bits;bits>>=1U) hits+=bits&1U;
            s->type_vote=TRACK_RING_360;s->type_frames=(uint8_t)hits;s->phase=RING_STATE_CANDIDATE;
            if(current360 && (v->near_open || v->junction_near>=RING_360_ENTRY_POSITION_MIN)) {
                if(s->near_frames<RING_NEAR_CONFIRM_FRAMES) ++s->near_frames;
            } else s->near_frames=0U;
            if(hits<RING_360_CONFIRM_HITS) return;
            s->kind=TRACK_RING_360;if(!current360 || !running) return;
        } else {
            s->vote360_history=0U;s->vote360_misses=0U;
            if(!entry_evidence(v)) {
                s->type_frames=0U;s->type_vote=TRACK_RING_UNKNOWN;s->kind=TRACK_RING_UNKNOWN;
                s->near_frames=0U;s->phase=RING_STATE_IDLE;s->direction=0;return;
            }
            if(s->type_vote!=v->type_hint || (v->direction_hint!=0 && v->direction_hint!=s->direction)) {
                s->type_vote=v->type_hint;s->direction=v->direction_hint;s->type_frames=0U;s->near_frames=0U;
            }
            if(v->near_open && forward_support(v)) {if(s->near_frames<RING_NEAR_CONFIRM_FRAMES) ++s->near_frames;}
            else s->near_frames=0U;
            if(s->type_frames<RING_CONFIRM_FRAMES) ++s->type_frames;
            s->phase=RING_STATE_CANDIDATE;if(s->type_frames<RING_CONFIRM_FRAMES) return;
            s->kind=s->type_vote;if(!running) return;
        }
        s->entry_seen=1U;s->entry_passed=0U;s->exit_seen=0U;
        s->zore360_confirmed=(uint8_t)(s->kind==TRACK_RING_360);
        uint8_t near=s->near_frames;phase(s,RING_STATE_APPROACH,now);
        s->near_frames=near>0U ? (uint8_t)(near-1U) : 0U;
    }
    if(s->phase==RING_STATE_APPROACH) {
        if(mouth_near(s,v)) {if(s->near_frames<RING_NEAR_CONFIRM_FRAMES) ++s->near_frames;}
        else s->near_frames=0U;
        if(s->near_frames>=RING_NEAR_CONFIRM_FRAMES) {
            ++s->event_id;s->event_started_ms=now;phase(s,RING_STATE_ENTERING,now);
        } else if(!v->junction) {
            if(s->miss_frames<RING_PREP_MISS_FRAMES) ++s->miss_frames;
            if(s->miss_frames>=RING_PREP_MISS_FRAMES) reset_event(s,now);
        } else s->miss_frames=0U;
        return;
    }
    if(s->phase==RING_STATE_ENTERING) {
        if(s->kind==TRACK_RING_360) {
            if(!elapsed(now,s->phase_started_ms,RING_360_ENTRY_MIN_MS)) return;
            if(turn_requested(s,v)) {if(s->turn_frames<RING_360_TURN_CONFIRM_FRAMES) ++s->turn_frames;}
            else s->turn_frames=0U;
            if(s->turn_frames>=RING_360_TURN_CONFIRM_FRAMES) {
                s->entry_passed=1U;s->turn_seen=1U;s->yaw_progress_deg=0.f;
                phase(s,RING_STATE_INSIDE,now);
            }
        } else {
            if(!v->junction && !v->near_open && v->near_road && followable(v)) {
                if(s->clear_frames<RING_CLEAR_FRAMES) ++s->clear_frames;
            } else s->clear_frames=0U;
            if(s->clear_frames>=RING_CLEAR_FRAMES) {
                s->entry_passed=1U;s->revisit_armed=0U;s->yaw_progress_deg=0.f;phase(s,RING_STATE_INSIDE,now);
            }
        }
        return;
    }
    if(s->phase==RING_STATE_INSIDE && s->kind==TRACK_RING_360) return;
    if(s->phase==RING_STATE_INSIDE) {
        if(s->direction==0 && v->turn_hint!=0) s->direction=v->turn_hint;
        if(!v->junction && v->turn_hint!=0 && v->turn_hint==s->direction) {
            if(s->turn_frames<RING_TURN_CONFIRM_FRAMES) ++s->turn_frames;
            if(s->turn_frames>=RING_TURN_CONFIRM_FRAMES) s->turn_seen=1U;
        } else s->turn_frames=0U;
        if(!v->junction && v->near_road && followable(v)) {
            if(s->clear_frames<RING_REVISIT_CLEAR_FRAMES) ++s->clear_frames;
            if(s->turn_seen && s->clear_frames>=RING_REVISIT_CLEAR_FRAMES) s->revisit_armed=1U;
        } else s->clear_frames=0U;
        if(s->revisit_armed && s->entry_passed && v->junction && v->exit_score>=60U) {
            if(v->near_open && forward_support(v)) {if(s->near_frames<RING_NEAR_CONFIRM_FRAMES) ++s->near_frames;}
            else s->near_frames=0U;
            if(s->exit_frames<RING_CONFIRM_FRAMES) ++s->exit_frames;
            if(s->exit_frames>=RING_CONFIRM_FRAMES) {
                uint8_t reached=(uint8_t)(s->near_frames>=RING_NEAR_CONFIRM_FRAMES);
                s->exit_seen=1U;phase(s,reached ? RING_STATE_EXITING : RING_STATE_EXIT_APPROACH,now);
            }
        } else {s->exit_frames=0U;s->near_frames=0U;}
        return;
    }
    if(s->phase==RING_STATE_EXIT_APPROACH) {
        if(v->junction && v->near_open && forward_support(v)) {if(s->near_frames<RING_NEAR_CONFIRM_FRAMES) ++s->near_frames;}
        else s->near_frames=0U;
        if(s->near_frames>=RING_NEAR_CONFIRM_FRAMES) phase(s,RING_STATE_EXITING,now);
        else if(!v->junction) {
            if(s->miss_frames<RING_PREP_MISS_FRAMES) ++s->miss_frames;
            if(s->miss_frames>=RING_PREP_MISS_FRAMES) {s->exit_seen=0U;phase(s,RING_STATE_INSIDE,now);}
        } else s->miss_frames=0U;
        return;
    }
    if(s->phase==RING_STATE_EXITING) {
        if(s->kind==TRACK_RING_360 && !elapsed(now,s->phase_started_ms,RING_360_EXIT_MIN_MS)) return;
        if(!v->junction && !v->near_open && v->near_paired && v->path[0].valid &&
           (s->kind!=TRACK_RING_360 || abs(v->path[0].heading)<=RING_360_EXIT_RESTORE_HEADING)) {
            if(s->clear_frames<RING_EXIT_RESTORE_FRAMES) ++s->clear_frames;
        } else s->clear_frames=0U;
        if(s->clear_frames>=RING_EXIT_RESTORE_FRAMES) complete(s,now);
    }
}
static track_path_observation_t virtual_line(ring_control_state_t *s,const track_ring_observation_t *v)
{
    if(!s->line_locked) {
        uint8_t exiting=(uint8_t)(s->phase==RING_STATE_EXITING);
        unsigned edge=s->direction>0 ? 1U : 0U;
        float near=0.f,far=0.f;uint8_t source=0U;
        if(s->kind==TRACK_RING_360 && exiting) {
            near=RING_360_EXIT_CENTER;far=near; /* Current body forward, not the ring arc or entry heading. */
        } else if(s->kind==TRACK_RING_360 && v->edge_line_valid[edge]) {
            near=v->edge_near[edge];far=v->edge_far[edge];source=(uint8_t)(edge+1U);
        } else if(!exiting && s->remembered_line) {near=s->remembered_near;far=s->remembered_far;}
        else if(exiting && v->exit_reference_valid) {near=v->exit_near;far=v->exit_far;}
        else {near=v->near_center;far=near;}
        float nt=s->kind==TRACK_RING_360 ? (exiting ? RING_360_EXIT_NEAR_OFFSET : RING_360_ENTRY_NEAR_OFFSET) :
            (exiting ? RING_270_EXIT_NEAR_OFFSET : RING_270_ENTRY_NEAR_OFFSET);
        float ft=s->kind==TRACK_RING_360 ? (exiting ? RING_360_EXIT_FAR_OFFSET : RING_360_ENTRY_FAR_OFFSET) :
            (exiting ? RING_270_EXIT_FAR_OFFSET : RING_270_ENTRY_FAR_OFFSET);
        near+=nt;far+=ft;far=near+clampf(far-near,-RING_LINE_MAX_TILT,RING_LINE_MAX_TILT);
        s->line_near=clampf(near,-800.f,800.f);s->line_far=clampf(far,-850.f,850.f);
        s->line_started_ms=s->last_tick_ms;s->line_started_counts=s->travel_counts;
        s->line_source=source;s->line_locked=1U;s->reset_tracking_history=1U;
    }
    s->line_active=1U;
    float sum=0.f,ws=0.f,ns=0.f,fs=0.f,nc=0.f,fc=0.f;
    unsigned zone=(RING_GUIDE_NEAR_PERCENT-RING_GUIDE_FAR_PERCENT)/3U;
    for(unsigned row=RING_GUIDE_FAR_PERCENT;row<RING_GUIDE_NEAR_PERCENT;++row) {
        float x=ring_control_line_x(s,(float)row);
        float w=row>=RING_GUIDE_NEAR_PERCENT-zone ? 3.f : (row<RING_GUIDE_FAR_PERCENT+zone ? 1.f : 2.f);
        sum+=w*x;ws+=w;
        if(row>=RING_GUIDE_NEAR_PERCENT-zone) {ns+=x;nc+=1.f;}
        if(row<RING_GUIDE_FAR_PERCENT+zone) {fs+=x;fc+=1.f;}
    }
    track_path_observation_t p={(int16_t)clampf(sum/ws,-1000.f,1000.f),
        (int16_t)clampf(fs/fc-ns/nc,-1000.f,1000.f),1U,100U};
    return p;
}
static track_path_observation_t turn_path(ring_control_state_t *s,const track_ring_observation_t *v,unsigned *mode)
{
    track_path_observation_t p=s->last_turn_path;
    unsigned outside=s->direction>0 ? TRACK_PATH_RIGHT : TRACK_PATH_LEFT;
    unsigned order[3]={outside,TRACK_PATH_NORMAL,outside==TRACK_PATH_LEFT ? TRACK_PATH_RIGHT : TRACK_PATH_LEFT};
    for(unsigned i=0U;i<3U;++i) {
        track_path_observation_t candidate=v->path[order[i]];
        float amount=-(float)s->direction*((float)candidate.error+2.f*(float)candidate.heading);
        if(candidate.valid && amount>0.f) {p=candidate;*mode=order[i];break;}
    }
    if(!p.valid) {p=(track_path_observation_t){0,0,1U,100U};*mode=TRACK_PATH_NORMAL;}
    float amount=clampf(-(float)s->direction*((float)p.error+2.f*(float)p.heading),
        RING_360_TURN_MIN_MIXED,RING_360_TURN_MAX_MIXED);
    /* Feed the same desired mixed command through heading only. Holding the
     * synthetic lateral error at zero prevents the ordinary D term from
     * briefly cancelling a committed turn after a noisy boundary jump. */
    p.error=0;
    p.heading=(int16_t)clampf(-(float)s->direction*amount*0.5f,-1000.f,1000.f);
    p.valid=1U;p.confidence=100U;s->last_turn_path=p;return p;
}
track_path_observation_t ring_control_select_path(ring_control_state_t *s,const track_ring_observation_t *v)
{
    track_path_observation_t invalid={0,0,0U,0U};
    if(s==NULL || v==NULL) return invalid;
    s->control_refresh=0U;
    if(s->fault) {s->selected_path=invalid;return invalid;}
    unsigned mode=TRACK_PATH_NORMAL;track_path_observation_t p=v->path[0];
    if(s->action_locked && (s->phase==RING_STATE_ENTERING || s->phase==RING_STATE_EXITING)) {
        p=virtual_line(s,v);mode=TRACK_PATH_STRAIGHT;
    } else {
        s->line_active=0U;
        if(s->action_locked && s->kind==TRACK_RING_360 && s->phase==RING_STATE_INSIDE) p=turn_path(s,v,&mode);
        else if(s->entry_seen && (s->phase==RING_STATE_INSIDE || s->phase==RING_STATE_EXIT_APPROACH) && !p.valid) {
            mode=s->direction>0 ? TRACK_PATH_RIGHT : TRACK_PATH_LEFT;
            if(!v->path[mode].valid) mode=mode==TRACK_PATH_RIGHT ? TRACK_PATH_LEFT : TRACK_PATH_RIGHT;
            p=v->path[mode];
        } else if(!s->entry_seen) s->reset_tracking_history=0U;
    }
    s->path_mode=(uint8_t)mode;
    if(s->action_locked && s->phase!=RING_STATE_COOLDOWN) {
        if(p.valid) s->missing_path_frames=0U;
        else if(s->missing_path_frames<3U) ++s->missing_path_frames;
        if(s->missing_path_frames>=3U) {fail(s);if(s->fault) p=invalid;}
    }
    s->selected_path=p;return p;
}
