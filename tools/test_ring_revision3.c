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
static void process(void){
 memcpy(saved,image,sizeof(image));track_vision_process(image,154,304,&result);result.frame_number=++frame;
 track_ring_vision_process(&model,image,154,304,&result);
 ring_control_observe(&state,&result.ring,frame*20,1);ring_control_select_path(&state,&result.ring);
 assert(!memcmp(image,saved,sizeof(image)));
 for(unsigned y=0;y<154;++y) if(result.row_valid[y]) {assert(result.center[y]<304);assert(image[y*304+result.center[y]]);}
}
static void load(const char *name){char path[1024];snprintf(path,sizeof(path),"%s/%s.bin",directory,name);FILE*f=fopen(path,"rb");assert(f);assert(fread(image,1,sizeof(image),f)==sizeof(image));fclose(f);}
int main(int argc,char **argv){
 assert(argc==2);directory=argv[1];
 const char *cases[]={"p1","p2","p9","p10","p1_alternate","p2_alternate","p9_alternate","p10_alternate"};
 for(unsigned n=0;n<8;++n){reset();load(cases[n]);for(unsigned i=0;i<10;++i) process();
  assert(result.valid && state.selected_path.valid && result.valid_rows>60);
  assert(!state.action_locked && state.kind==0 && state.corner_guide_id==0);
  printf("PASS %s: %u middle rows, ordinary E=%d H=%d, no ring repair\n",cases[n],result.valid_rows,state.selected_path.error,state.selected_path.heading);
 }
 reset();load("p1");state.action_locked=1;state.kind=TRACK_RING_360;state.direction=1;state.corner_stage=3;
 for(unsigned i=0;i<12;++i) {process();}
 assert(state.corner_stage==3 && state.selected_path.valid);
 puts("PASS lost entry landmark cannot advance from paired-road evidence alone; current ordinary curve remains available");
 reset();load("p7");state.action_locked=1;state.kind=TRACK_RING_360;state.direction=1;state.corner_stage=6;process();
 const ring_corner_line_t *line=&result.ring.corners.guide[ring_pdf_guide(6,0)].boundary[1];
 assert(line->valid && line->x1==result.ring.corners.a[1].x && line->y1==result.ring.corners.a[1].y);
 assert(!state.corner_selected_guide.boundary[0].valid && state.selected_path.valid);
 puts("PASS p7 right bridge begins at the displayed physical A vertex");
 state.corner_stage=5;state.transition_frames=0;state.have_frame=0;
 result.ring.corners.paired=1;result.ring.corners.v[0].quality=90;result.ring.corners.v_reacquired=1;
 for(unsigned i=0;i<6;++i) {result.ring.corners.frame_number=++frame;ring_control_observe(&state,&result.ring,frame*20,1);}
 assert(state.corner_stage==5);
 result.ring.corners.v_reacquired=0;
 for(unsigned i=0;i<6;++i) {result.ring.corners.frame_number=++frame;ring_control_observe(&state,&result.ring,frame*20,1);}
 assert(state.corner_stage==5);
 puts("PASS locally recovered first V is neither a new event nor disappearance evidence");
 reset();memset(image,0,sizeof(image));for(unsigned y=0;y<154;++y) memset(image+y*304+90,255,124);
 memset(image+80*304,0,3*304);process();assert(result.valid && result.row_valid[79] && !result.row_valid[81]);
 puts("PASS local 3-row black seam no longer discards the visible road above it");
 reset();memset(image,0,sizeof(image));for(unsigned y=75;y<154;++y) memset(image+y*304+90,255,124);
 for(unsigned y=20;y<24;++y) memset(image+y*304+30,255,240);
 process();assert(result.valid && !result.row_valid[22]);
 puts("PASS isolated upper white stripe does not replace the main road");
 reset();memset(image,0,sizeof(image));for(unsigned y=140;y<154;++y) memset(image+y*304+145,255,13);
 process();assert(!result.valid);
 puts("PASS isolated 13x14 near white speck does not establish a road");
 reset();memset(image,0,sizeof(image));process();assert(!result.valid && !state.selected_path.valid);
 puts("PASS genuinely absent white road remains invalid; no invented midline");return 0;
}
