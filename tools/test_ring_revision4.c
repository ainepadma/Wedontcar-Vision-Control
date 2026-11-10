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
static void observe(unsigned now)
{
 ring_control_observe(&state,&result.ring,now,1);
 ring_control_select_path(&state,&result.ring);
}
static void entry_handoff(unsigned inner)
{
 reset();memset(&result.ring,0,sizeof(result.ring));
 ring_corner_frame_t *f=&result.ring.corners;
 f->rows=154;f->cols=304;
 unsigned outer=1U-inner,first=ring_pdf_guide(1,inner),second=ring_pdf_guide(3,inner);
 ring_corner_guide_t *a=&f->guide[first],*v=&f->guide[second];
 a->valid=1;a->confidence=90;a->boundary[inner].valid=1;
 v->valid=1;v->confidence=90;v->boundary[outer].valid=1;
 result.ring.path[0]=(track_path_observation_t){0,0,1,90};
 f->v[inner]=(ring_corner_point_t){.x=inner?250.f:53.f,.y=115.f,.quality=90};
 f->frame_number=1;observe(20);
 assert(!state.action_locked && !state.corner_guide_id);
 state.action_locked=1;state.kind=TRACK_RING_360;state.direction=inner?-1:1;state.corner_stage=1;
 f->p[inner].quality=90;f->bottom_open[inner]=1;f->continuous[outer]=1;
 for(unsigned frame_number=2;frame_number<=5;++frame_number) {
     f->frame_number=frame_number;observe(frame_number*20);
     assert(state.corner_stage==1 && state.corner_guide_id==first);
 }
 a->valid=0;
 f->frame_number=6;observe(120);
 assert(state.corner_stage==1 && state.corner_guide_id==first && state.corner_guide_held);
 a->valid=1;
 f->entry_v[inner]=(ring_corner_point_t){.x=inner?174.f:129.f,.y=35.f,.quality=90};
 v->corner_kind=RING_CORNER_B;v->corner_x=f->entry_v[inner].x;v->corner_y=35;
 f->frame_number=7;observe(140);
 assert(state.corner_stage==3 && state.corner_guide_id==second && !state.corner_guide_held);
 f->entry_v[inner].quality=0;v->corner_y=115;
 f->frame_number=8;observe(160);
 assert(state.corner_stage==3 && state.corner_guide_id==second && state.corner_guide_held);
 assert(state.corner_selected_guide.corner_y==35);
 f->entry_v[inner].quality=90;v->corner_y=36;
 f->frame_number=9;observe(180);
 assert(state.corner_stage==3 && state.corner_guide_id==second && !state.corner_guide_held);

 reset();state.action_locked=1;state.kind=TRACK_RING_360;
 state.direction=inner?-1:1;state.corner_stage=1;
 ring_entry_v_min_y_percent=30;ring_entry_v_pull_frames=2;
 f->frame_number=1;observe(20);assert(state.corner_guide_id==first);
 f->entry_v[inner].y=60;v->corner_y=60;
 f->frame_number=2;observe(40);assert(state.corner_guide_id==first && state.entry_v_frames==1);
 observe(41);assert(state.entry_v_frames==1);
 f->entry_v[inner].x+=30;v->corner_x=f->entry_v[inner].x;
 f->frame_number=3;observe(60);assert(state.corner_guide_id==first && state.entry_v_frames==1);
 f->frame_number=4;observe(80);assert(state.corner_stage==3 && state.corner_guide_id==second);
 ring_entry_v_min_y_percent=20;ring_entry_v_pull_frames=1;
}
int main(int argc,char **argv){assert(argc==2);directory=argv[1];
 entry_handoff(0);entry_handoff(1);
 puts("PASS mirrored entry handoff: first line until upper tip, configurable votes, no return to low corner");
 ring_third_left_lean=.06f; /* Calibration used to capture these historical fixtures. */
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
 load("island");process(3);
 assert(state.corner_stage==1 && state.corner_guide_id==9 && state.corner_guide_held);
 load("first_v");process(1);assert(state.corner_stage==3 && state.corner_guide_id==13);clipped();
 assert(state.corner_selected_guide.boundary[1].x0==result.ring.corners.entry_v[0].x);
 assert(state.corner_selected_guide.boundary[1].x1==result.ring.corners.border[1][152]);
 assert(state.corner_selected_guide.boundary[1].y1==153);
 process(2);step("outer_v",4);clipped();step("bridge",5);
 load("timing_p3");process(1);assert(!state.third_passed && ring_control_path_source(&state)==RING_SOURCE_HOLD);
 process(29);assert(state.corner_stage==5 && state.corner_guide_id==0 && !state.separator_seen && !state.third_passed);
 load("timing_p4");process(20);assert(state.corner_stage==5 && state.corner_guide_id==17);
 assert(state.corner_selected_guide.boundary[1].x1==result.ring.corners.a[1].x);
 assert(state.corner_selected_guide.boundary[1].y1==result.ring.corners.a[1].y);
 load("timing_p1");process(20);assert(state.corner_stage==5 && state.corner_guide_id==17 && !state.separator_seen);
 load("timing_p2");process(1);assert(state.corner_stage==6 && state.corner_guide_id==19 && state.separator_seen);
 load("timing_p3");process(6);assert(state.corner_stage==6 && !state.third_passed);
 assert(ring_control_path_source(&state)==RING_SOURCE_HOLD);
 process(1);assert(state.corner_stage==6 && !state.corner_guide_id && state.third_passed);
 assert(ring_control_path_source(&state)==RING_SOURCE_NORMAL);
 process(23);assert(state.corner_stage==6 && !state.corner_guide_id);
 load("latest_p3");process(3);assert(state.corner_stage==7 && state.corner_guide_id==21);
 assert(state.corner_selected_guide.boundary[0].valid && !state.corner_selected_guide.boundary[1].valid);
 load("latest_p4");process(30);assert(state.corner_stage==7 && state.corner_guide_id==21);
 load("latest_p5");process(30);assert(state.corner_stage==7 && state.corner_guide_id==21);
 memset(image,0,sizeof(image));for(unsigned y=0;y<154;++y) memset(image+y*304+90,255,124);
 process(2);assert(ring_control_path_source(&state)==RING_SOURCE_HOLD && state.tip_missing_frames==2);
 process(1);assert(state.completed_360==1 && state.phase==RING_STATE_COOLDOWN);
 assert(state.selected_path.valid && state.path_mode==TRACK_PATH_NORMAL && !state.corner_guide_id);
 process(9);assert(state.completed_360==1 && state.phase==RING_STATE_COOLDOWN);
 puts("PASS third repair, ordinary handoff, fourth repair and ordinary tracking on exit");
 reset();step("cross",1);assert(state.kind==TRACK_RING_270 && state.corner_guide_id==2);
 puts("PASS 270 bilateral recognition unchanged; initial entry geometry unchanged");return 0;}
