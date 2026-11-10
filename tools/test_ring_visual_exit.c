#include "ring_control.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static ring_control_state_t s;
static track_ring_observation_t o;
static unsigned frame;
static void step(void){o.corners.frame_number=++frame;ring_control_observe(&s,&o,frame*20,1);ring_control_select_path(&s,&o);}
static void guide(unsigned stage,unsigned in,int error){ring_corner_guide_t*g=&o.corners.guide[ring_pdf_guide(stage,in)];g->valid=1;g->confidence=90;g->error=(int16_t)error;g->boundary[stage==7?in:1U-in].valid=1;}
int main(void){for(unsigned in=0;in<2;++in){
 ring_control_init(&s);memset(&o,0,sizeof(o));frame=0;s.action_locked=1;s.kind=TRACK_RING_360;s.direction=in?-1:1;s.corner_stage=5;
 o.corners.rows=154;o.corners.cols=304;o.path[0]=(track_path_observation_t){100,20,1,90};
 guide(7,in,150);o.corners.exit_v[in].quality=90;
 for(unsigned i=0;i<20;++i){step();}assert(s.corner_stage==5&&!s.separator_seen);
 o.corners.exit_separator=1;step();assert(!s.separator_seen); /* cannot arm before third */
 o.corners.exit_separator=0;guide(5,in,-200);guide(6,in,-200);step();assert(s.bridge_seen);
 for(unsigned i=0;i<20;++i){step();}assert(s.corner_stage==5&&!s.separator_seen);
 o.corners.exit_separator=1;step();assert(s.corner_stage==6&&s.separator_seen&&s.corner_guide_id==ring_pdf_guide(6,in));
 unsigned separator=s.separator_frame;
 ring_control_observe(&s,&o,frame*20+1,1);ring_control_select_path(&s,&o);assert(s.separator_frame==separator&&s.corner_guide_id==ring_pdf_guide(6,in));
 o.corners.exit_separator=0;o.corners.exit_v[in].quality=0;
 for(unsigned i=0;i<30;++i){step();}assert(s.corner_stage==6&&s.separator_seen); /* one frame is enough */
 o.corners.exit_v[in].quality=90;
 /* Same-side tips at opposite positions in the exit mouth cannot cast
  * consecutive votes for one vertex. The third line is gone, so this also
  * checks that the first visible fourth line is not borrowed during S6. */
 for(unsigned stage=5;stage<=6;++stage)memset(&o.corners.guide[ring_pdf_guide(stage,in)],0,sizeof(ring_corner_guide_t));
 for(unsigned i=0;i<6;++i){
     float x=i%2U ? 125.f : 40.f;
     o.corners.exit_v[in].x=in ? 303.f-x : x;
     o.corners.exit_v[in].y=80.f;
     step();
     assert(s.corner_stage==6 && s.corner_guide_id!=ring_pdf_guide(7,in));
 }
 o.corners.exit_v[in].quality=0;step(); /* Restart the confirmation sequence. */
 o.corners.exit_v[in].quality=90;
 o.corners.road_clear=1; /* Even a ready straight must not steal the switch frame. */
 for(unsigned i=0;i<3;++i){
     float x=60.f+i*2.f;
     o.corners.exit_v[in].x=in ? 303.f-x : x;
     o.corners.exit_v[in].y=80.f+i;
     step();
     if(i<2) assert(s.corner_stage==6 && s.corner_guide_id!=ring_pdf_guide(7,in));
 }
 assert(s.corner_stage==7 && s.selected_path.error==150 && !s.tip_missing_frames);
 assert(ring_control_path_source(&s)==RING_SOURCE_GUIDE);
 o.corners.road_clear=0;
 guide(5,in,-800);guide(6,in,-800);step();assert(s.corner_stage==7&&s.selected_path.error==150);
 /* A competing tip on the same side must not pull the fourth repair
  * across the image. Within the old tip's lifetime we can still recover it. */
 o.corners.exit_v[in].x=in ? 178.f : 125.f;
 guide(7,in,-400);step();
 assert(ring_control_path_source(&s)==RING_SOURCE_HOLD && s.selected_path.error==150);
 o.corners.exit_v[in].x=in ? 237.f : 66.f;
 guide(7,in,165);step();
 assert(ring_control_path_source(&s)==RING_SOURCE_GUIDE && s.selected_path.error==165);
 memset(&o.corners.guide[ring_pdf_guide(7,in)],0,sizeof(ring_corner_guide_t));
 o.corners.exit_v[in].quality=0;step();assert(s.corner_guide_held);
 step();assert(s.corner_guide_held);
 step();assert(s.completed_360==1 && s.phase==RING_STATE_COOLDOWN);
 assert(ring_control_path_source(&s)==RING_SOURCE_NORMAL && s.selected_path.error==100);
 /* A competing vertex cannot re-enter after the exit has finished. */
 o.corners.exit_v[in].quality=90;
 o.corners.exit_v[in].x=in ? 178.f : 125.f;guide(7,in,-400);
 for(unsigned i=0;i<3;++i) {
     step();assert(!s.action_locked && !s.corner_guide_id && s.selected_path.error==100);
 }
 ring_control_init(&s);assert(!s.separator_seen&&!s.separator_frame);
 }puts("PASS mirrored separator order, same-tip confirmation, fourth-line hold and reacquisition");return 0;}
