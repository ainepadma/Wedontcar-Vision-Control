#include "ring_control.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static unsigned char image[154*304];
static track_vision_result_t result;
static track_ring_vision_state_t model;
static ring_control_state_t state;
int main(int argc,char **argv) {
 assert(argc==2);unsigned frame=0;unsigned expected[]={1,1,2,3,4,5,5,5};
 track_ring_vision_init(&model);ring_control_init(&state);
 for(unsigned k=1;k<=11;++k) {
  char path[1024];snprintf(path,sizeof(path),"%s/%02u.bin",argv[1],k);FILE *f=fopen(path,"rb");assert(f);
  assert(fread(image,1,sizeof(image),f)==sizeof(image));fclose(f);
  if(k==9) ring_control_init(&state);
  for(unsigned repeat=0;repeat<3;++repeat) {
   track_vision_process(image,154,304,&result);result.frame_number=++frame;
   track_ring_vision_process(&model,image,154,304,&result);
   ring_control_tick(&state,1,0,0,0,frame*20);
   ring_control_observe(&state,&result.ring,frame*20,1);
   track_path_observation_t path_result=ring_control_select_path(&state,&result.ring);
   if(repeat==2) {assert(path_result.valid);assert(!state.fault);
    assert(state.kind==(k<9 ? TRACK_RING_360 : TRACK_RING_270));
    assert(state.corner_stage==(k<9 ? expected[k-1] : 1));
    printf("PASS photo %u: kind %u stage %u E %d H %d\n",k,state.kind,state.corner_stage,path_result.error,path_result.heading);
   }
  }
 }
 puts("NOTE: three repeated stills verify debounce and geometry, not real temporal/vehicle dynamics.");
 return 0;
}
