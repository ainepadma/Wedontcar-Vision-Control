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
 for(unsigned stage=5;stage<=6;++stage)memset(&o.corners.guide[ring_pdf_guide(stage,in)],0,sizeof(ring_corner_guide_t));
 for(unsigned i=0;i<3;++i){step();}assert(s.corner_stage==7&&s.selected_path.error==150);
 assert(ring_control_path_source(&s)==RING_SOURCE_GUIDE);
 guide(5,in,-800);guide(6,in,-800);step();assert(s.corner_stage==7&&s.selected_path.error==150);
 memset(&o.corners.guide[ring_pdf_guide(7,in)],0,sizeof(ring_corner_guide_t));step();assert(s.corner_guide_held);
 for(unsigned i=0;i<7;++i){step();}assert(!s.selected_path.valid);
 ring_control_init(&s);assert(!s.separator_seen&&!s.separator_frame);
 }puts("PASS separator only after third; own frame stays right; duplicate/new-frame distinction; one-frame latch; V still required; mirrored order, bounded hold and reset");return 0;}
