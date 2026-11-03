#include "ring_control.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static uint8_t image[154*304],saved[154*304];
static track_vision_result_t result;
static track_ring_vision_state_t model;
static ring_control_state_t state;
static unsigned frame;
static const char *directory;
static void reset(void){frame=0;ring_control_init(&state);track_ring_vision_init(&model);}
static void load(const char *name){char path[1024];snprintf(path,sizeof(path),"%s/%s.bin",directory,name);FILE*f=fopen(path,"rb");assert(f);assert(fread(image,1,sizeof(image),f)==sizeof(image));fclose(f);}
static void process(unsigned count){for(unsigned i=0;i<count;++i){
 memcpy(saved,image,sizeof(image));track_vision_process(image,154,304,&result);result.frame_number=++frame;
 track_ring_vision_process(&model,image,154,304,&result);
 ring_control_observe(&state,&result.ring,frame*20,1);ring_control_select_path(&state,&result.ring);
 assert(!memcmp(image,saved,sizeof(image)));
}}
static void step(const char *name,unsigned stage){load(name);process(3);assert(state.corner_stage==stage);assert(state.selected_path.valid);}
static void clipped(void){
 const ring_corner_guide_t *g=&state.corner_selected_guide;
 assert(g->clip_upper && g->active_top>0 && g->active_end==154);
 for(unsigned i=0;i<16;++i) if(i*153U/15U<g->active_top) assert(!(g->center_valid&(1U<<i)));
 assert(g->boundary[1].valid && !g->boundary[0].valid);
}
int main(int argc,char **argv){assert(argc==2);directory=argv[1];
 const char *ordinary[]={"ordinary_p1","ordinary_p2","ordinary_p9","ordinary_p10","ordinary_p1_alternate","ordinary_p2_alternate","ordinary_p9_alternate","ordinary_p10_alternate"};
 for(unsigned n=0;n<8;++n){reset();load(ordinary[n]);process(10);
  assert(result.valid && state.selected_path.valid && result.valid_rows>60);
  assert(!state.action_locked && state.kind==0 && state.corner_guide_id==0);
  printf("PASS %s: %u middle rows, no ring repair\n",ordinary[n],result.valid_rows);
 }
 reset();state.action_locked=1;state.kind=TRACK_RING_360;state.direction=1;state.corner_stage=3;load("p2");
 for(unsigned i=0;i<120;++i){process(1);assert(state.selected_path.valid);assert(state.corner_stage==3 || state.corner_stage==4);assert(state.corner_guide_id==13 || state.corner_guide_id==15);clipped();}
 puts("PASS p2:120 frames keep right entry continuation; all top-straight centre samples excluded");
 reset();step("entry",1);assert(state.corner_selected_guide.boundary[0].x0==112 && state.selected_path.error==-82);
 step("island",2);step("first_v",3);clipped();step("outer_v",4);clipped();step("bridge",5);process(3);assert(state.corner_stage==6);
 load("p1");process(3);assert(state.corner_stage==6 && state.corner_guide_id==19);
 for(unsigned i=0;i<60;++i){
  result.ring.corners.v[0].quality=90;result.ring.corners.v[0].x=60;result.ring.corners.v[0].y=70;result.ring.corners.v_reacquired=0;
  result.ring.corners.frame_number=++frame;ring_control_observe(&state,&result.ring,frame*20,1);ring_control_select_path(&state,&result.ring);
  assert(state.corner_stage==6 && !state.exit_armed && state.corner_selected_guide.boundary[1].valid && !state.corner_selected_guide.boundary[0].valid);
 }
 puts("PASS p1 remains PRE-final right-mouth turn even with repeated false inner V; never S7/G21");
 memset(image,0,sizeof(image));for(unsigned y=0;y<154;++y) memset(image+y*304+90,255,124);process(5);
 assert(state.corner_stage==6 && state.exit_armed);
 step("first_v",7);assert(state.corner_guide_id==21 && state.corner_selected_guide.boundary[0].valid);
 puts("PASS final left exit requires previous right mouth, low-corner passage, clear paired road, then a new V");
 reset();step("cross",1);assert(state.kind==TRACK_RING_270 && state.corner_guide_id==2);
 puts("PASS 270 bilateral recognition unchanged; initial entry geometry unchanged");return 0;}
