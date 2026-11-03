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
static void load(const char *name) {
 char path[1024];snprintf(path,sizeof(path),"%s/%s.bin",directory,name);
 FILE *file=fopen(path,"rb");assert(file);assert(fread(image,1,sizeof(image),file)==sizeof(image));fclose(file);memcpy(saved,image,sizeof(image));
}
static void process(unsigned n) {
 for(unsigned i=0;i<n;++i) {
  track_vision_process(image,154,304,&result);result.frame_number=++frame;
  track_ring_vision_process(&model,image,154,304,&result);
  ring_control_observe(&state,&result.ring,frame*20,1);
  (void)ring_control_select_path(&state,&result.ring);
  assert(memcmp(image,saved,sizeof(image))==0);
 }
}
static void step(const char *name,unsigned expected) {
 load(name);process(3);assert(state.corner_stage==expected);assert(state.selected_path.valid);
}
static void start(void){frame=0;ring_control_init(&state);track_ring_vision_init(&model);}
static void to_inside(void) {
 start();step("entry",1);
 const ring_corner_line_t *line=&result.ring.corners.guide[ring_pdf_guide(1,0)].boundary[0];
 assert(line->valid && line->x0==112 && line->y0==20 && line->x1==52 && line->y1==110);
 assert(state.selected_path.error==-82 && state.selected_path.heading==3);
 process(12);assert(state.corner_stage==1);
 step("island",2);step("junction",3);
 assert(!state.corner_selected_guide.boundary[0].valid && state.corner_selected_guide.boundary[1].valid);
 process(12);assert(state.corner_stage==3); /* Visible first V is not a second visit. */
 step("outer_v",4);step("exit_bridge",5);
 assert(!state.corner_selected_guide.boundary[0].valid && state.corner_selected_guide.boundary[1].valid);
 process(3);assert(state.corner_stage==6);
}
int main(int argc,char **argv) {
 assert(argc==2);directory=argv[1];to_inside();step("junction",7);
 assert(state.corner_selected_guide.boundary[0].valid && !state.corner_selected_guide.boundary[1].valid);
 puts("PASS p2 unchanged; first/second identical V choose opposite repaired sides; visible first V cannot be consumed twice");
 to_inside();step("failed_exit",7);assert(state.corner_selected_guide.boundary[0].valid && !state.corner_selected_guide.boundary[1].valid);
 puts("PASS actual failed-exit p3 now selects inner-side exit connection");
 start();state.action_locked=1;state.kind=TRACK_RING_360;state.direction=1;state.corner_stage=5;
 load("p1");process(1);assert(!state.corner_selected_guide.boundary[0].valid && state.corner_selected_guide.boundary[1].valid);
 puts("PASS p1 right-only bridge, no left repair");
 start();step("cross",1);assert(state.kind==TRACK_RING_270 && state.corner_guide_id==RING_GUIDE_270_CONNECT);
 assert(state.corner_selected_guide.boundary[0].valid && state.corner_selected_guide.boundary[1].valid);
 assert(state.selected_path.error>-50 && state.selected_path.error<50);
 assert(state.selected_path.heading>-100 && state.selected_path.heading<100);
 puts("PASS 270 bilateral sharp-tip connections, near-straight middle line, no 360 classification");
 start();load("cross");memset(image+100*304,0,3*304);memcpy(saved,image,sizeof(image));process(3);
 assert(!result.ring.corners.paired_mouth && state.kind!=TRACK_RING_270);
 puts("PASS a black separator prevents disconnected far crossing takeover");
 return 0;
}
