#include "ring_control.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x) do {if(!(x)){fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x);return 1;}}while(0)
static uint8_t original[154*304],image[154*304];
static track_vision_result_t result;
static ring_corner_frame_t raw;
static ring_corner_tracker_t raw_tracker;
static track_ring_vision_state_t model;
static ring_control_state_t control;
int main(int argc,char **argv)
{
 CHECK(argc==2);char path[1024];snprintf(path,sizeof(path),"%s/outer_v.bin",argv[1]);
 FILE *file=fopen(path,"rb");CHECK(file && fread(original,1,sizeof(original),file)==sizeof(original));fclose(file);
 ring_corner_guide_t baseline;
 unsigned added_frames=0;int previous_heading=0;
 ring_control_init(&control);control.action_locked=1;control.kind=TRACK_RING_360;control.direction=1;control.corner_stage=4;
 for(unsigned frame=0;frame<120;++frame) {
  int dx=(int)(frame%3)-1,dy=(int)(frame/3%3)-1;
  memset(image,0,sizeof(image));
  for(int y=0;y<154;++y) for(int x=0;x<304;++x)
   if(x-dx>=0 && x-dx<304 && y-dy>=0 && y-dy<154)image[y*304+x]=original[(y-dy)*304+x-dx];
  track_vision_process(image,154,304,&result);
  int error=result.error,heading=result.heading,valid=result.valid;
  ring_corner_process(image,154,304,0,&raw);ring_corner_stabilize(image,0,&raw_tracker,&raw);
  baseline=raw.guide[ring_pdf_guide(4,0)];CHECK(baseline.valid);
  result.frame_number=frame+1;track_ring_vision_process(&model,image,154,304,&result);
  const ring_corner_guide_t *g=&result.ring.corners.guide[ring_pdf_guide(4,0)];
  CHECK(g->valid && result.error==error && result.heading==heading && result.valid==valid);
  CHECK(!memcmp(g->boundary,baseline.boundary,sizeof(g->boundary)));
  CHECK((g->center_valid&baseline.center_valid)==baseline.center_valid);
  unsigned added=0;
  for(unsigned i=0;i<16;++i) if(g->center_valid&(1U<<i)) {
   unsigned y=i*153U/15U,x=g->center[i];CHECK(image[y*304+x]);
   if(baseline.center_valid&(1U<<i)) CHECK(x==baseline.center[i]);
   else {CHECK(result.row_valid[y] && x==result.center[y] && x<220U);++added;}
   if(i && (g->center_link_valid&(1U<<i)) && !(baseline.center_link_valid&(1U<<i))) {
    unsigned y0=(i-1U)*153U/15U,x0=g->center[i-1U];
    for(unsigned row=y0;row<=y;++row) {
     unsigned col=(x0*(y-row)+x*(row-y0))/(y-y0);CHECK(image[row*304+col]);
    }
   }
  }
  CHECK(added>=5U && g->active_top<baseline.active_top);
  CHECK((g->center_link_valid & (1U<<8U)) && (g->center_link_valid & (1U<<12U)));
  ++added_frames;
  for(unsigned stage=1;stage<=7;++stage)
   CHECK(!memcmp(&raw.guide[ring_pdf_guide(stage,1)],&result.ring.corners.guide[ring_pdf_guide(stage,1)],sizeof(*g)));
  ring_control_observe(&control,&result.ring,(frame+1)*20,1);
  track_path_observation_t selected=ring_control_select_path(&control,&result.ring);
  CHECK(control.corner_stage==4 && control.corner_guide_id==15 && !control.corner_guide_held);
  CHECK(selected.error==g->error && selected.heading==g->heading);
  if(frame)CHECK(abs(selected.heading-previous_heading)<100);
  previous_heading=selected.heading;control.reset_tracking_history=0;
 }
 /* The white road does not disappear when the ordinary detector misses
  * two rows. The already pixel-checked connection must remain drawable. */
 memcpy(image,original,sizeof(image));track_ring_vision_init(&model);
 track_vision_process(image,154,304,&result);
 result.row_valid[80]=0;result.row_valid[81]=0;
 track_ring_vision_process(&model,image,154,304,&result);
 const ring_corner_guide_t *gap=&result.ring.corners.guide[ring_pdf_guide(4,0)];
 CHECK(gap->valid && (gap->center_valid&(1U<<8U)));
 CHECK((gap->center_link_valid&(1U<<8U)) && (gap->center_link_valid&(1U<<9U)));
 CHECK(!result.row_valid[80] && !result.row_valid[81]);
 puts("PASS two ordinary detector misses preserve the verified white-road connection");
 /* A black separator must stop the continuation even if ordinary row
  * candidates exist in the white region on its far side. */
 memcpy(image,original,sizeof(image));memset(image+80U*304U,0,304U);
 memset(&raw_tracker,0,sizeof(raw_tracker));track_ring_vision_init(&model);
 track_vision_process(image,154,304,&result);
 track_ring_vision_process(&model,image,154,304,&result);
 const ring_corner_guide_t *cut=&result.ring.corners.guide[ring_pdf_guide(4,0)];
 CHECK(cut->valid);
 for(unsigned i=0;i<16;++i) if(i*153U/15U<=80U) CHECK(!(cut->center_valid&(1U<<i)));
 /* An upper straight connected through a white cap cannot replace a left
  * entry's already selected far center. */
 snprintf(path,sizeof(path),"%s/latest_p3.bin",argv[1]);file=fopen(path,"rb");
 CHECK(file && fread(image,1,sizeof(image),file)==sizeof(image));fclose(file);
 memset(&raw_tracker,0,sizeof(raw_tracker));track_ring_vision_init(&model);
 track_vision_process(image,154,304,&result);ring_corner_process(image,154,304,0,&raw);
 ring_corner_stabilize(image,0,&raw_tracker,&raw);
 track_ring_vision_process(&model,image,154,304,&result);
 baseline=raw.guide[ring_pdf_guide(3,0)];CHECK(baseline.valid);
 const ring_corner_guide_t *left=&result.ring.corners.guide[ring_pdf_guide(3,0)];
 CHECK(left->center_valid==baseline.center_valid && left->heading==baseline.heading);
 puts("PASS black separator and connected upper-straight rejection");
 printf("PASS %u translated S4 frames: ordinary left-road continuation, original blue/near center preserved, right branch rejected, control uses completed path without hold/dropout\n",added_frames);
 return 0;
}
