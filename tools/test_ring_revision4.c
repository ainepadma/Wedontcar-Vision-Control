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
 assert(result.ring.handoff_near_error==result.near_error && result.ring.handoff_near_heading==result.near_heading);
 assert(result.ring.handoff_near_valid==result.near_geometry_valid);
 assert(result.ring.handoff_bottom_error==result.bottom_error && result.ring.handoff_bottom_valid==result.bottom_valid);
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
 step("island",2);step("first_v",3);clipped();step("outer_v",4);clipped();step("bridge",5);
 load("timing_p3");process(30);assert(state.corner_stage==5 && state.corner_guide_id==0 && !state.separator_seen);
 load("timing_p4");process(20);assert(state.corner_stage==5 && state.corner_guide_id==17);
 assert(state.corner_selected_guide.boundary[1].x1==result.ring.corners.a[1].x);
 assert(state.corner_selected_guide.boundary[1].y1==result.ring.corners.a[1].y);
 load("timing_p1");process(20);assert(state.corner_stage==5 && state.corner_guide_id==17 && !state.separator_seen);
 load("timing_p2");process(1);assert(state.corner_stage==6 && state.corner_guide_id==19 && state.separator_seen);
 load("timing_p3");process(30);assert(state.corner_stage==6 && state.corner_guide_id!=21);
 load("latest_p3");process(3);assert(state.corner_stage==7 && state.corner_guide_id==21);
 assert(state.corner_selected_guide.boundary[0].valid && !state.corner_selected_guide.boundary[1].valid);
 load("latest_p4");process(30);assert(state.corner_stage==7 && state.corner_guide_id==21);
 load("latest_p5");process(30);assert(state.corner_stage==7 && state.corner_guide_id==21);
 memset(image,0,sizeof(image));for(unsigned y=0;y<154;++y) memset(image+y*304+90,255,124);
 process(3);assert(state.completed_360==0 && state.corner_stage==7);
 assert(state.selected_path.valid && state.path_mode==TRACK_PATH_STRAIGHT && !state.corner_guide_id);
 process(8);assert(state.completed_360==1 && state.phase==RING_STATE_COOLDOWN);
 puts("PASS third mouth stays repaired; one-frame separator then directional inner V connects left boundary without timed steering; then aligned road settles 200ms");
 reset();step("cross",1);assert(state.kind==TRACK_RING_270 && state.corner_guide_id==2);
 puts("PASS 270 bilateral recognition unchanged; initial entry geometry unchanged");return 0;}
