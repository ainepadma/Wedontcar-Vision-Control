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
static void ring_disabled_reset(ring_control_state_t *s)
{
    uint8_t protection=s->protection_enabled;
    ring_control_init(s);
    s->protection_enabled=protection;
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
    if(!RING_FEATURE_ENABLED) return 0U;
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
    if(!RING_FEATURE_ENABLED) {ring_disabled_reset(s);return;}
    uint32_t dt=s->have_tick ? now-s->last_tick_ms : 0U;
    if(dt>100U) dt=0U;
    s->last_tick_ms=now;s->have_tick=1U;
    uint8_t valid_yaw=(uint8_t)(fresh && isfinite(rate));
    s->yaw_rate_dps=valid_yaw ? rate : 0.f;
    if(valid_yaw) {s->have_yaw=1U;s->last_yaw_ms=now;}
    s->imu_wait=(uint8_t)(!s->have_yaw || now-s->last_yaw_ms>RING_360_YAW_STALE_MS);
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
    if(s->kind==TRACK_RING_360 && s->action_locked) return; /* Original frame timeouts, no IMU progress. */
    if(s->event_started_ms!=0U && now-s->event_started_ms>RING_ACTIVE_TIMEOUT_MS) {
        fail(s);if(s->fault) return;
    }
    if(s->line_locked && (now-s->line_started_ms>RING_LINE_MAX_MS ||
       s->travel_counts-s->line_started_counts>RING_LINE_MAX_COUNTS)) {fail(s);if(s->fault) return;}
    if(s->kind==TRACK_RING_270 && valid_yaw && s->entry_passed) {
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
    if(v->type_hint!=TRACK_RING_270 || !v->junction || v->entry_score<70U || v->paired_road) return 0U;
    return (uint8_t)(v->left_open && v->right_open && (v->wide_white || (v->topology[0] && v->topology[1])));
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
    (void)s;return (uint8_t)(v->junction && v->near_open && forward_support(v));
}
static void complete(ring_control_state_t *s,uint32_t now)
{
    s->completed_mask|=kind_bit(s->kind);
    if(s->kind==TRACK_RING_360) s->completed_360=1U;else if(s->kind==TRACK_RING_270) s->completed_270=1U;
    phase(s,RING_STATE_COOLDOWN,now);s->cooldown_until_ms=now+RING_COOLDOWN_MS;
}
/* ZORE handlers are evaluated on the image thread for each possible input
 * stage. The ISR commits one result per genuinely new frame. */
static void z360_advance(ring_control_state_t *s,const track_ring_observation_t *v,uint32_t now)
{
    if(!v->zore360_ready || s->zore360_stage<1U || s->zore360_stage>9U) return;
    uint8_t stage=s->zore360_stage;
    if(stage==s->zore360_last_stage) {
        ++s->zore360_timeout;
        unsigned limit=stage==7U ? RING_360_STAGE7_TIMEOUT_FRAMES : RING_360_TIMEOUT_FRAMES;
        if(s->zore360_timeout>limit) {reset_event(s,now);return;}
    } else s->zore360_timeout=0U;
    s->zore360_last_stage=stage;s->zore360_input_stage=stage;
    unsigned side=s->direction>0 ? 0U : 1U;
    uint8_t next=v->zore360_next[side][stage];
    if(v->zore360_done[side][stage]) {
        s->exit_seen=1U;s->completed_mask|=RING_DONE_360;
        if(s->completed_360<UINT8_MAX) ++s->completed_360;
    }
    if(next==0U) {reset_event(s,now);return;}
    s->zore360_stage=next;
    s->entry_passed=(uint8_t)(next>=5U);
    uint8_t mapped=next<=2U ? RING_STATE_ENTERING : (next>=8U ? RING_STATE_EXITING : RING_STATE_INSIDE);
    if(mapped!=s->phase) {s->phase=mapped;s->phase_started_ms=now;s->reset_tracking_history=1U;}
    s->line_active=s->line_locked=0U;s->path_mode=TRACK_PATH_NORMAL;
    s->zore360_off=v->zore360_off[side][stage];
    s->zore360_top=v->zore360_top;s->zore360_end=v->zore360_end;
    s->zore360_native_rows=v->zore360_native_rows;s->zore360_native_cols=v->zore360_native_cols;
    s->zore360_center_x=v->zore360_center_x;s->zore360_width_near=v->zore360_width_near;
    if(v->zore360_det_valid[side][stage]) {
        s->selected_path=v->zore360_path[side][stage];
        s->zore360_det=v->zore360_det[side][stage];
        s->zore360_error_scale=v->zore360_error_scale[side][stage];
    } /* Keep the last deviation if the reference has no valid view. */
    if(v->zore360_center_stage==stage && v->zore360_center_direction==s->direction)
        memcpy(s->zore360_centers,v->zore360_center,sizeof(s->zore360_centers));
}

/* Private copy of upstream fuzzy membership/gain math; ordinary PD is untouched.
 * Source PWM offset is mapped to this car's already calibrated servo endpoints.
 * Motor counts use the current car's base-speed scale, not STC encoder units. */
static int z360_mul_div(int value,int numerator,int denominator)
{
    if(denominator==0) return 0;
    int64_t n=(int64_t)value*numerator;
    n=(n>=0 ? n+denominator/2 : n-denominator/2)/denominator;
    return (int)clampf((float)n,-32768.f,32767.f);
}
static void z360_membership(int x,int max,int *left,int *right,int *ml,int *mr)
{
    if(x<=-max) {*left=*right=0;*ml=16;*mr=0;return;}
    if(x>=max) {*left=*right=6;*ml=16;*mr=0;return;}
    int q=(x+max)*6*16/(2*max),index=q/16,frac=q-index*16;
    *left=index;*right=index+1;*ml=16-frac;*mr=frac;
}
void ring_control_360_motion(ring_control_state_t *s,float center,float left,float right,
                             float base_speed,float *angle,float *speed)
{
    if(!RING_FEATURE_ENABLED) {if(s!=NULL) ring_disabled_reset(s);return;}
    static const int table[7][7]={
        {215,215,185,185,175,130,130},{215,215,185,175,175,130,175},
        {185,185,185,175,130,175,175},{185,185,175,130,175,185,185},
        {175,175,130,175,175,185,185},{175,130,175,185,185,215,215},
        {130,130,185,185,185,215,215}};
    if(s==NULL || angle==NULL || speed==NULL || !s->action_locked || s->kind!=TRACK_RING_360 || s->fault) return;
    /* Only numerical units change: retain the existing ZORE fuzzy/rate math. */
    float scale=s->zore360_error_scale>0.f ? s->zore360_error_scale : 1.f;
    int error=(int)lroundf((s->zore360_center_x-(float)s->zore360_det)/scale);
    int ec=s->zore360_previous_error-error;
    s->zore360_previous_error=(int16_t)error;
    int el,er,eml,emr,cl,cr,cml,cmr;
    z360_membership(error,30,&el,&er,&eml,&emr);z360_membership(ec,20,&cl,&cr,&cml,&cmr);
    int gain=(eml*cml*table[el][cl]+eml*cmr*table[el][cr]+emr*cml*table[er][cl]+emr*cmr*table[er][cr]+128)>>8;
    int p=z360_mul_div(error,gain+RING_360_KP_BASE,5);
    float gyro=s->zore360_stage>=6U && s->zore360_stage<=7U ? RING_360_GYRO_INSIDE : RING_360_GYRO_NORMAL;
    /* Upstream casts gyro to integer before its integer multiplication. */
    int g=z360_mul_div((int)s->yaw_rate_dps,(int)lroundf(gyro*10.f),10);
    float offset=clampf((float)(p+g),-550.f,650.f);
    s->zore360_servo_offset=offset;
    *angle=center+(offset>=0.f ? (left-center)*offset/650.f : (right-center)*(-offset)/550.f);
    if(s->zore360_speed==0.f && base_speed>0.f) s->zore360_speed=clampf(*speed/base_speed*RING_360_SPEED_BASE,0.f,RING_360_SPEED_BASE);
    float desired=RING_360_SPEED_BASE-RING_360_SPEED_DROP-(float)z360_mul_div((int)fabsf(offset),1,(int)RING_360_TURN_SLOW_DIV);
    float step=desired>s->zore360_speed ? RING_360_SPEED_ACC_STEP : RING_360_SPEED_DEC_STEP;
    s->zore360_speed+=clampf(desired-s->zore360_speed,-step,step);
    *speed=fmaxf(0.f,s->zore360_speed)*base_speed/RING_360_SPEED_BASE;
}

void ring_control_observe(ring_control_state_t *s,const track_ring_observation_t *v,uint32_t now,uint8_t running)
{
    if(!RING_FEATURE_ENABLED) {if(s!=NULL) ring_disabled_reset(s);return;}
    if(s==NULL || v==NULL) return;
    if(s->have_frame && (int32_t)(now-s->last_frame_ms)<=0) return;
    s->last_frame_ms=now;s->have_frame=1U;s->vision_candidate=v->junction;s->track_valid=v->path[0].valid;
    if(s->fault || (s->action_locked && !running)) return;
    if(s->action_locked && s->kind==TRACK_RING_360) {z360_advance(s,v,now);return;}
    if(!s->entry_seen && v->zore360_ready && v->zore360_match && v->direction_hint!=0) {
        s->kind=TRACK_RING_360;s->direction=v->direction_hint;
        if(!running) {s->phase=RING_STATE_CANDIDATE;return;}
        s->entry_seen=s->action_locked=1U;s->zore360_confirmed=1U;s->zore360_stage=1U;
        s->event_started_ms=now;++s->event_id;
        s->phase=RING_STATE_ENTERING;s->phase_started_ms=now;s->reset_tracking_history=1U;
        z360_advance(s,v,now);return;
    }
    if(!s->entry_seen) {
        s->line_active=0U;s->line_locked=0U;s->path_mode=TRACK_PATH_NORMAL;s->reset_tracking_history=0U;
        if(v->reference_valid && !v->junction && v->paired_road) {
            s->remembered_near=v->reference_near;s->remembered_far=v->reference_far;s->remembered_line=1U;
        }
        if(v->type_hint==TRACK_RING_270 && (s->completed_mask&RING_DONE_270)) {reset_event(s,now);return;}
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
            if(!v->junction && !v->near_open && v->near_road && followable(v)) {
                if(s->clear_frames<RING_CLEAR_FRAMES) ++s->clear_frames;
            } else s->clear_frames=0U;
            if(s->clear_frames>=RING_CLEAR_FRAMES) {
                s->entry_passed=1U;s->revisit_armed=0U;s->yaw_progress_deg=0.f;phase(s,RING_STATE_INSIDE,now);
            }
        return;
    }
    if(s->phase==RING_STATE_INSIDE && s->kind==TRACK_RING_270) {
        if(s->direction==0 && v->turn_hint!=0) s->direction=v->turn_hint;
        if(!v->junction && v->turn_hint!=0 && v->turn_hint==s->direction) {
            if(s->turn_frames<RING_TURN_CONFIRM_FRAMES) ++s->turn_frames;
            if(s->turn_frames>=RING_TURN_CONFIRM_FRAMES) s->turn_seen=1U;
        } else s->turn_frames=0U;
        /* ENTERING already confirms that the first mouth has disappeared on
         * RING_CLEAR_FRAMES genuine near-road frames. Only then accept another
         * unchanged 270 entrance pattern as the exit; never count a persistent
         * first mouth twice. No additional turn, yaw or exit-score prerequisite. */
        if(s->entry_passed) s->revisit_armed=1U;
        if(s->revisit_armed && v->type_hint==TRACK_RING_270 && entry_evidence(v)) {
            if(s->exit_frames<RING_270_EXIT_CONFIRM_FRAMES) ++s->exit_frames;
            if(s->exit_frames>=RING_270_EXIT_CONFIRM_FRAMES) {
                s->exit_seen=1U;phase(s,RING_STATE_EXITING,now);
            }
        } else s->exit_frames=0U;
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
        if(s->kind==TRACK_RING_270 && !elapsed(now,s->phase_started_ms,RING_270_EXIT_MIN_MS)) return;
        unsigned restore_frames=s->kind==TRACK_RING_270 ? RING_270_EXIT_RESTORE_FRAMES : RING_EXIT_RESTORE_FRAMES;
        if(!v->junction && !v->near_open && v->near_paired && v->path[0].valid) {
            if(s->clear_frames<restore_frames) ++s->clear_frames;
        } else s->clear_frames=0U;
        if(s->clear_frames>=restore_frames) complete(s,now);
    }
}
static track_path_observation_t virtual_line(ring_control_state_t *s,const track_ring_observation_t *v)
{
    if(!s->line_locked) {
        uint8_t exiting=(uint8_t)(s->phase==RING_STATE_EXITING);
        float near=0.f,far=0.f;uint8_t source=0U;
        if(exiting) {near=RING_270_EXIT_CENTER;far=near;
        } else if(!exiting && s->remembered_line) {near=s->remembered_near;far=s->remembered_far;}
        else if(exiting && v->exit_reference_valid) {near=v->exit_near;far=v->exit_far;}
        else {near=v->near_center;far=near;}
        float nt=exiting ? RING_270_EXIT_NEAR_OFFSET : RING_270_ENTRY_NEAR_OFFSET;
        float ft=exiting ? RING_270_EXIT_FAR_OFFSET : RING_270_ENTRY_FAR_OFFSET;
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
track_path_observation_t ring_control_select_path(ring_control_state_t *s,const track_ring_observation_t *v)
{
    track_path_observation_t invalid={0,0,0U,0U};
    if(!RING_FEATURE_ENABLED) {
        if(s!=NULL) ring_disabled_reset(s);
        return v!=NULL ? v->path[TRACK_PATH_NORMAL] : invalid;
    }
    if(s==NULL || v==NULL) return invalid;
    s->control_refresh=0U;
    if(s->fault) {s->selected_path=invalid;return invalid;}
    if(s->kind==TRACK_RING_360 && s->action_locked) {s->line_active=0U;return s->selected_path;}
    unsigned mode=TRACK_PATH_NORMAL;track_path_observation_t p=v->path[0];
    if(s->action_locked && (s->phase==RING_STATE_ENTERING || s->phase==RING_STATE_EXITING)) {
        p=virtual_line(s,v);mode=TRACK_PATH_STRAIGHT;
    } else {
        s->line_active=0U;
        if(s->entry_seen && (s->phase==RING_STATE_INSIDE || s->phase==RING_STATE_EXIT_APPROACH) && !p.valid) {
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
