#include "ring_control.h"
#include "track_control.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x) do {if(!(x)){fprintf(stderr,"FAIL %d: %s\n",__LINE__,#x);exit(1);}}while(0)
static uint32_t now;
static track_ring_observation_t road,mouth;
static track_path_observation_t frame(ring_control_state_t *s,const track_ring_observation_t *v)
{
 now+=10U;ring_control_tick(s,1U,0U,0.f,0.f,now);
 ring_control_observe(s,v,now,1U);return ring_control_select_path(s,v);
}
static void setup(ring_control_state_t *s)
{
 now=100U;ring_control_init(s);ring_control_set_protection(s,0U);
 memset(&road,0,sizeof(road));
 road.near_road=road.near_paired=road.paired_road=road.reference_valid=1U;
 road.reference_near=20.f;road.reference_far=20.f;road.straight_support=100U;
 for(unsigned i=0U;i<TRACK_PATH_COUNT;++i) road.path[i]=(track_path_observation_t){20,0,1U,100U};
 (void)frame(s,&road);
 mouth=road;mouth.paired_road=mouth.near_paired=0U;
 mouth.type_hint=TRACK_RING_270;mouth.junction=mouth.left_open=mouth.right_open=mouth.wide_white=1U;
 mouth.near_open=1U;mouth.entry_score=90U;mouth.exit_score=0U;
 for(unsigned i=0U;i<RING_CONFIRM_FRAMES;++i) (void)frame(s,&mouth);
 CHECK(s->phase==RING_STATE_ENTERING && s->kind==TRACK_RING_270 && s->line_active && !s->exit_seen);
}
static void leave(ring_control_state_t *s)
{
 for(unsigned i=0U;i<RING_CLEAR_FRAMES;++i) (void)frame(s,&road);
 CHECK(s->entry_passed && s->phase==RING_STATE_INSIDE && !s->line_active && !s->turn_seen);
}
int main(void)
{
 for(unsigned topology=0U;topology<2U;++topology) {
  ring_control_state_t s;setup(&s);
  /* First persistent white cross-band may not be counted again. */
  for(unsigned i=0U;i<100U;++i) (void)frame(&s,&mouth);
  CHECK(!s.entry_passed && !s.exit_seen && s.phase==RING_STATE_ENTERING);
  for(unsigned i=0U;i<RING_CLEAR_FRAMES-1U;++i) (void)frame(&s,&road);
  (void)frame(&s,&mouth);CHECK(!s.entry_passed);
  leave(&s);
  /* No turn hint or IMU required; same unchanged entrance feature is enough.
   * Deliberately no near-open/forward/exit-score on the second far mouth. */
  mouth.near_open=0U;mouth.straight_support=0U;mouth.path[TRACK_PATH_STRAIGHT].valid=0U;
  mouth.exit_reference_valid=1U;mouth.exit_near=600.f;mouth.exit_far=680.f;
  mouth.path[0]=(track_path_observation_t){-500,-200,1U,100U};
  if(topology) {mouth.wide_white=0U;mouth.topology[0]=mouth.topology[1]=1U;}
  for(unsigned i=0U;i<RING_270_EXIT_CONFIRM_FRAMES-1U;++i) (void)frame(&s,&mouth);
  CHECK(!s.exit_seen && !s.line_active);
  uint32_t stamp=now;ring_control_observe(&s,&mouth,stamp,1U);
  CHECK(!s.exit_seen); /* A duplicate camera frame cannot complete the vote. */
  (void)frame(&s,&road);CHECK(s.exit_frames==0U);
  for(unsigned i=0U;i<RING_270_EXIT_CONFIRM_FRAMES;++i) (void)frame(&s,&mouth);
  CHECK(s.phase==RING_STATE_EXITING && s.exit_seen && s.entry_seen && s.line_active);
  CHECK(s.selected_path.valid && s.selected_path.error==0 && s.selected_path.heading==0);
  /* With the actual unchanged controller, a noisy old entrance/arc direction
   * is replaced by body-forward and the servo centres through original slew. */
  track_control_config_t config={87.f,98.5f,75.5f,120.f,80.f,0.02f,0.0005f,120.f};
  track_control_state_t control;track_control_init(&control,&config);
  track_control_set_loss_stop(&control,0U);track_control_toggle(&control,now);
  for(unsigned i=0U;i<RING_270_EXIT_MIN_MS/10U-1U;++i) {
   track_path_observation_t p=frame(&s,&road);
   CHECK(s.phase==RING_STATE_EXITING && s.line_active && !s.completed_270);
   track_control_observe(&control,p.error,p.heading,p.confidence,p.valid,now);
   CHECK(track_control_tick(&control,now).servo_angle==87.f);
  }
  track_ring_observation_t blank={0};
  (void)frame(&s,&blank);CHECK(s.line_active && s.clear_frames==0U);
  for(unsigned i=0U;i<RING_270_EXIT_RESTORE_FRAMES-1U;++i) (void)frame(&s,&road);
  (void)frame(&s,&mouth);CHECK(s.clear_frames==0U && s.line_active);
  for(unsigned i=0U;i<RING_270_EXIT_RESTORE_FRAMES;++i) (void)frame(&s,&road);
  CHECK(s.phase==RING_STATE_COOLDOWN && s.completed_270 && !s.line_active);
  for(unsigned i=0U;i<RING_COOLDOWN_MS/10U+1U;++i) (void)frame(&s,&road);
  CHECK(s.phase==RING_STATE_IDLE && !s.kind && !s.entry_seen && s.completed_270);
 }
 /* A 360 mouth or a white patch without unchanged 270 evidence is rejected. */
 ring_control_state_t s;setup(&s);leave(&s);
 track_ring_observation_t bad=mouth;bad.type_hint=TRACK_RING_360;bad.zore360_match=1U;bad.direction_hint=1;
 for(unsigned i=0U;i<10U;++i) (void)frame(&s,&bad);
 CHECK(!s.exit_seen && s.phase==RING_STATE_INSIDE);
 bad=mouth;bad.wide_white=0U;bad.topology[0]=bad.topology[1]=0U;
 for(unsigned i=0U;i<10U;++i) (void)frame(&s,&bad);
 CHECK(!s.exit_seen && s.phase==RING_STATE_INSIDE);
 bad=mouth;bad.paired_road=1U;
 for(unsigned i=0U;i<10U;++i) (void)frame(&s,&bad);
 CHECK(!s.exit_seen);
 puts("PASS 270 exit: unchanged entrance, leave-first debounce, same white-band/topology admission, no turn/IMU/exit-score/near gate, duplicate rejection, immediate body-forward actual-servo control, minimum hold + normal-road restoration, 360/nonstructured rejection");
 return 0;
}
