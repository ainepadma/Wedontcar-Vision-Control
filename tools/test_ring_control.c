#include "ring_control.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#define CHECK(x) do {if(!(x)){fprintf(stderr,"FAIL %d: %s\n",__LINE__,#x);exit(1);}}while(0)
static uint32_t now;
static track_ring_observation_t road,mouth,arc;
static void setup(uint8_t type,int direction)
{
 now=100U;memset(&road,0,sizeof(road));
 road.model_ready=road.near_road=road.near_paired=road.paired_road=road.road_clear=1U;
 road.reference_valid=1U;road.straight_support=100U;road.left_quality=road.right_quality=100U;
 for(unsigned i=0U;i<TRACK_PATH_COUNT;++i) road.path[i]=(track_path_observation_t){0,0,1U,100U};
 mouth=road;mouth.near_paired=mouth.paired_road=mouth.road_clear=0U;
 mouth.junction=1U;mouth.type_hint=type;mouth.entry_score=90U;mouth.exit_score=80U;
 mouth.near_open=1U;mouth.direction_hint=(int8_t)(type==TRACK_RING_360 ? direction : 0);
 mouth.left_open=(uint8_t)(type==TRACK_RING_270 || direction>0);
 mouth.right_open=(uint8_t)(type==TRACK_RING_270 || direction<0);
 mouth.wide_white=(uint8_t)(type==TRACK_RING_270);mouth.zore360_match=(uint8_t)(type==TRACK_RING_360);
 mouth.edge_line_valid[0]=mouth.edge_line_valid[1]=1U;
 mouth.path[0]=(track_path_observation_t){(int16_t)(-direction*500),(int16_t)(-direction*200),1U,100U};
 arc=road;arc.turn_hint=(int8_t)direction;
 arc.path[0]=(track_path_observation_t){(int16_t)(-direction*100),(int16_t)(-direction*80),1U,100U};
}
static track_path_observation_t frame(ring_control_state_t *s,const track_ring_observation_t *v,uint8_t fresh,float yaw)
{
 s->reset_tracking_history=0U;now+=10U;
 ring_control_tick(s,1U,fresh,yaw,0.f,now);ring_control_observe(s,v,now,1U);
 return ring_control_select_path(s,v);
}
static void enter360(ring_control_state_t *s,int direction)
{
 setup(TRACK_RING_360,direction);ring_control_init(s);ring_control_set_protection(s,0U);
 mouth.near_open=0U;
 for(unsigned i=0U;i<RING_360_CONFIRM_HITS;++i) {
  track_path_observation_t p=frame(s,&mouth,1U,0.f);
  CHECK(!s->line_active && p.error==mouth.path[0].error && !ring_control_guidance_active(s));
 }
 CHECK(s->entry_seen && s->phase==RING_STATE_APPROACH);
 mouth.near_open=1U;mouth.straight_support=0U;mouth.path[TRACK_PATH_STRAIGHT].valid=0U;
 for(unsigned i=0U;i<RING_NEAR_CONFIRM_FRAMES;++i) (void)frame(s,&mouth,1U,0.f);
 CHECK(s->kind==TRACK_RING_360 && s->phase==RING_STATE_ENTERING && s->line_active && s->action_locked);
 track_ring_observation_t blank={0};
 for(unsigned i=0U;i<RING_360_ENTRY_MIN_MS/10U;++i) {
  track_path_observation_t p=frame(s,&blank,1U,(float)direction*100.f);
  CHECK(p.valid && s->line_active && s->entry_seen && !s->exit_seen && s->kind==TRACK_RING_360);
  CHECK(s->yaw_progress_deg==0.f);
 }
 for(unsigned i=0U;i<RING_360_TURN_CONFIRM_FRAMES;++i) (void)frame(s,&arc,1U,0.f);
 CHECK(s->phase==RING_STATE_INSIDE && s->entry_passed && s->yaw_progress_deg==0.f);
}
static void cooldown(ring_control_state_t *s)
{
 for(unsigned i=0U;i<RING_COOLDOWN_MS/10U+1U;++i) (void)frame(s,&road,1U,0.f);
 CHECK(s->phase==RING_STATE_IDLE && !s->entry_seen && !s->exit_seen && !s->action_locked);
}
static void route360(int direction)
{
 ring_control_state_t s;enter360(&s,direction);
 track_ring_observation_t wrong=road;
 for(unsigned i=0U;i<TRACK_PATH_COUNT;++i) wrong.path[i]=(track_path_observation_t){(int16_t)(direction*500),(int16_t)(direction*200),1U,100U};
 for(unsigned i=0U;i<50U;++i) {
  track_path_observation_t p=frame(&s,&wrong,0U,NAN);
  CHECK(-(float)direction*((float)p.error+2.f*(float)p.heading)>=RING_360_TURN_MIN_MIXED-1.f);
  CHECK(s.phase==RING_STATE_INSIDE && s.kind==TRACK_RING_360 && !s.exit_seen && s.yaw_progress_deg==0.f);
 }
 CHECK(s.imu_wait);
 for(unsigned i=0U;i<279U;++i) (void)frame(&s,&mouth,1U,(float)direction*100.f);
 CHECK(s.phase==RING_STATE_INSIDE && fabsf(s.yaw_progress_deg-279.f)<0.01f && !s.exit_seen);
 float before=s.yaw_progress_deg;(void)frame(&s,&arc,1U,(float)-direction*40.f);
 CHECK(s.yaw_progress_deg<before);
 while(s.yaw_progress_deg<279.5f) (void)frame(&s,&arc,1U,(float)direction*100.f);
 /* Angle threshold hands off without a new camera frame. */
 now+=10U;ring_control_tick(&s,1U,1U,(float)direction*100.f,0.f,now);
 CHECK(s.phase==RING_STATE_EXITING && s.control_refresh && s.exit_seen && s.kind==TRACK_RING_360);
 ring_control_observe(&s,&road,now-1U,1U);CHECK(s.clear_frames==0U);
 track_ring_observation_t curved=arc;curved.edge_near[0]=curved.edge_near[1]=700.f;
 curved.edge_far[0]=curved.edge_far[1]=-300.f;curved.edge_line_valid[0]=curved.edge_line_valid[1]=1U;
 track_path_observation_t p=ring_control_select_path(&s,&curved);
 CHECK(s.line_active && p.valid && p.error==0 && p.heading==0 && s.reset_tracking_history && !s.control_refresh);
 for(unsigned i=0U;i<RING_360_EXIT_MIN_MS/10U;++i) {
  p=frame(&s,&mouth,1U,(float)-direction*100.f);CHECK(p.error==0 && p.heading==0 && s.line_active);
 }
 for(unsigned i=0U;i<RING_EXIT_RESTORE_FRAMES;++i) (void)frame(&s,&road,1U,0.f);
 CHECK(s.phase==RING_STATE_COOLDOWN && s.completed_360==1U && s.completed_mask==RING_DONE_360 && !s.line_active);
 cooldown(&s);CHECK(s.completed_360==1U);
 for(unsigned i=0U;i<10U;++i) {
  p=frame(&s,&mouth,1U,0.f);CHECK(!s.entry_seen && !s.line_active && p.error==mouth.path[0].error);
 }
 /* A 270 remains eligible after completing 360. */
 uint32_t saved_now=now;setup(TRACK_RING_270,direction);now=saved_now;
 for(unsigned i=0U;i<RING_CONFIRM_FRAMES;++i) (void)frame(&s,&mouth,1U,0.f);
 CHECK(s.kind==TRACK_RING_270 && s.line_active);
 for(unsigned i=0U;i<RING_CLEAR_FRAMES;++i) (void)frame(&s,&road,1U,0.f);
 for(unsigned i=0U;i<RING_REVISIT_CLEAR_FRAMES;++i) (void)frame(&s,&arc,1U,0.f);
 CHECK(s.phase==RING_STATE_INSIDE && s.revisit_armed);
 for(unsigned i=0U;i<RING_CONFIRM_FRAMES;++i) (void)frame(&s,&mouth,1U,0.f);
 CHECK(s.phase==RING_STATE_EXITING && s.exit_seen && s.line_active);
 for(unsigned i=0U;i<RING_EXIT_RESTORE_FRAMES;++i) (void)frame(&s,&road,1U,0.f);
 CHECK(s.completed_mask==(RING_DONE_360|RING_DONE_270) && s.completed_270==1U && s.completed_360==1U);
 cooldown(&s);
 for(unsigned i=0U;i<10U;++i) (void)frame(&s,&mouth,1U,0.f);
 CHECK(!s.entry_seen && !s.line_active);
 ring_control_init(&s);CHECK(!s.completed_mask && !s.completed_270 && !s.completed_360);
 for(unsigned i=0U;i<RING_CONFIRM_FRAMES;++i) (void)frame(&s,&mouth,1U,0.f);
 CHECK(s.entry_seen && s.line_active); /* C13 starts a fresh execution record. */
}
int main(void)
{
 route360(1);route360(-1);
 setup(TRACK_RING_360,1);ring_control_state_t s;ring_control_init(&s);mouth.near_open=0U;
 (void)frame(&s,&mouth,1U,0.f);(void)frame(&s,&road,1U,0.f);(void)frame(&s,&mouth,1U,0.f);
 CHECK(s.entry_seen && s.kind==TRACK_RING_360 && !s.line_active);
 uint32_t stamp=now;ring_control_observe(&s,&mouth,stamp,1U);ring_control_observe(&s,&mouth,stamp-1U,1U);
 CHECK(!s.line_active);
 enter360(&s,1);float angle=s.yaw_progress_deg;
 now+=1000U;ring_control_tick(&s,1U,1U,100.f,0.f,now);CHECK(s.yaw_progress_deg==angle && !s.fault);
 (void)frame(&s,&arc,1U,100.f);CHECK(s.yaw_progress_deg==angle+1.f);
 /* Missing images in FREE mode keep the transaction and its original line. */
 setup(TRACK_RING_360,1);ring_control_init(&s);ring_control_set_protection(&s,0U);
 for(unsigned i=0U;i<3U;++i) (void)frame(&s,&mouth,1U,0.f);
 track_ring_observation_t blank={0};
 for(unsigned i=0U;i<400U;++i) (void)frame(&s,&blank,0U,NAN);
 CHECK(s.action_locked && s.line_active && !s.fault && s.kind==TRACK_RING_360 && !s.completed_mask);
 ring_control_set_protection(&s,1U);now+=RING_VISION_STALE_MS+1U;
 ring_control_tick(&s,1U,0U,NAN,0.f,now);CHECK(s.fault && !s.completed_mask);
 uint8_t locked_kind=s.kind,resume=s.fault_resume_phase;
 float saved_angle=s.yaw_progress_deg;
 ring_control_set_protection(&s,0U);
 CHECK(!s.fault && s.action_locked && s.kind==locked_kind && s.phase==resume && s.yaw_progress_deg==saved_angle);
 CHECK(ring_control_select_path(&s,&blank).valid);
 /* Stopped type preview must never own motion. */
 setup(TRACK_RING_360,1);ring_control_init(&s);
 for(unsigned i=0U;i<20U;++i) {
  now+=10U;ring_control_tick(&s,0U,1U,0.f,0.f,now);ring_control_observe(&s,&mouth,now,0U);
  track_path_observation_t p=ring_control_select_path(&s,&mouth);
  CHECK(!s.entry_seen && !s.action_locked && !s.line_active && p.error==mouth.path[0].error);
 }
 CHECK(s.kind==TRACK_RING_360);
 ring_control_init(&s);now=UINT32_MAX-20U;
 for(unsigned i=0U;i<3U;++i) (void)frame(&s,&mouth,1U,0.f);
 CHECK(s.entry_seen && s.action_locked && s.line_active);
 puts("PASS action chain: locked entry despite missing mouth, visual turn + sustained same-direction lower bound, signed net 280deg, IMU loss holds turn/freezes angle, immediate no-camera exit refresh, body-forward exit, 270 legacy route, one execution per type, C13 clears records, preview/duplicate/wrap/protection");
 return 0;
}
