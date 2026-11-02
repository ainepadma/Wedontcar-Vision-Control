#include "ring_control.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static ring_control_state_t state;
static track_ring_observation_t obs;
static unsigned frame;
static void fresh(unsigned count) {
 for(unsigned i=0;i<count;++i) {
  obs.corners.frame_number=++frame;
  ring_control_tick(&state,1,0,0,0,frame*20);
  ring_control_observe(&state,&obs,frame*20,1);
 }
}
static void start(void) {ring_control_init(&state);memset(&obs,0,sizeof(obs));frame=0;obs.corners.rows=154;}
int main(void) {
 start();obs.corners.entry[0]=1;fresh(3);assert(state.kind==TRACK_RING_360 && state.corner_stage==1);
 /* Duplicate frames cannot vote. Later-stage evidence cannot skip state 2. */
 obs.corners.v[0].quality=90;fresh(3);assert(state.corner_stage==1);
 for(unsigned i=0;i<10;++i) ring_control_observe(&state,&obs,frame*20,1);
 assert(state.corner_stage==1);
 obs.corners.entry[0]=0;obs.corners.entry[1]=1;
 obs.corners.bottom_open[0]=1;obs.corners.p[0].quality=90;obs.corners.continuous[1]=1;
 fresh(3);assert(state.corner_stage==2 && state.direction==1);
 obs.corners.p[0].quality=0;fresh(3);assert(state.corner_stage==3);
 obs.corners.v[0].quality=0;obs.corners.v[1].quality=90;obs.corners.v[1].y=100;
 fresh(3);assert(state.corner_stage==4);
 obs.corners.v[1].quality=0;obs.corners.paired=1;fresh(3);assert(state.corner_stage==5);
 obs.corners.gap[1]=1;obs.corners.bottom_open[0]=1;obs.corners.bottom_open[1]=0;
 fresh(3);assert(state.corner_stage==6);
 obs.corners.gap[0]=1;obs.corners.breakpoint[0].quality=90;fresh(3);assert(state.corner_stage==7);
 obs.corners.road_clear=1;fresh(3);assert(!state.action_locked && state.completed_360==1);
 start();obs.corners.cross_entry=1;obs.corners.guide[RING_GUIDE_270_EXTEND].valid=1;
 fresh(3);assert(state.kind==TRACK_RING_270 && state.corner_stage==1);
 fresh(20);assert(state.corner_stage==1 && state.completed_270==0);
 obs.corners.cross_entry=0;obs.corners.road_clear=1;fresh(3);assert(state.corner_stage==2);
 obs.corners.cross_entry=1;obs.corners.road_clear=0;obs.corners.p[0].quality=0;fresh(3);assert(state.corner_stage==3);
 obs.corners.cross_entry=0;obs.corners.road_clear=1;fresh(3);assert(state.completed_270==1);
 /* Late camera data must still trigger the existing stop input. */
 start();obs.corners.entry[0]=1;fresh(3);ring_control_tick(&state,1,0,0,0,frame*20+151);assert(state.fault);
 puts("PASS PDF order, direction lock, duplicate frames, 270 identical crossings, stale vision");
 return 0;
}

