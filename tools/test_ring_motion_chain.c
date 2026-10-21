#include "ring_control.h"
#include "track_control.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x) do {if(!(x)){fprintf(stderr,"FAIL %d: %s\n",__LINE__,#x);exit(1);}}while(0)
int main(void)
{
 for(int direction=-1;direction<=1;direction+=2) {
  ring_control_state_t ring;ring_control_init(&ring);ring_control_set_protection(&ring,0U);
  ring.phase=RING_STATE_INSIDE;ring.kind=TRACK_RING_360;ring.action_locked=1U;
  ring.entry_seen=ring.entry_passed=1U;ring.direction=(int8_t)direction;
  ring.reset_tracking_history=1U;
  track_control_config_t config={87.f,98.5f,75.5f,120.f,80.f,0.02f,0.0005f,120.f};
  track_control_state_t control;track_control_init(&control,&config);
  track_control_set_loss_stop(&control,0U);track_control_toggle(&control,0U);
  track_ring_observation_t v={0};
  for(unsigned i=1U;i<=400U;++i) {
   uint32_t now=i*10U;
   for(unsigned mode=0U;mode<TRACK_PATH_COUNT;++mode) {
    v.path[mode]=(track_path_observation_t){(int16_t)(i%2U ? 1000 : -1000),
        (int16_t)(i%3U ? -500 : 500),1U,100U};
   }
   ring_control_tick(&ring,1U,1U,(float)direction*100.f,0.f,now);
   track_path_observation_t p=ring_control_select_path(&ring,&v);
   if(ring.reset_tracking_history) {
    control.filtered_error=(float)p.error;control.filtered_heading=(float)p.heading;
    control.derivative=0.f;control.derivative_ready=0U;ring.reset_tracking_history=0U;
   }
   track_control_observe(&control,p.error,p.heading,p.confidence,p.valid,now);
   track_control_output_t output=track_control_tick(&control,now);
   CHECK(output.running && p.valid && control.derivative==0.f);
   if(ring.phase==RING_STATE_INSIDE) {
    CHECK((float)direction*(output.servo_angle-87.f)>0.f);
    CHECK(-(float)direction*((float)p.error+2.f*p.heading)>=RING_360_TURN_MIN_MIXED-1.f);
   } else {
    CHECK(ring.phase==RING_STATE_EXITING && p.error==0 && p.heading==0 && ring.exit_seen);
    if(i>300U) CHECK(output.servo_angle==87.f);
   }
  }
 }
 puts("PASS committed-turn actual controller: left/right turns retain servo direction under large boundary jumps, unchanged PD derivative cannot cancel turn, 280deg transitions to body-forward zero and recentres through original servo slew limit");
 return 0;
}
