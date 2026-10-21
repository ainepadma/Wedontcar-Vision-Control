#include "ring_control.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#define CHECK(x) do {if(!(x)){fprintf(stderr,"FAIL %d %ux%u: %s\n",__LINE__,height,width,#x);exit(1);}}while(0)
static uint8_t pixels[480*752];
static unsigned height,width;
static track_vision_result_t result;
static track_ring_vision_state_t model;
static uint32_t now;
static void image(int kind,int mirror,int curve,int hood)
{
 memset(pixels,0,height*width);
 for(unsigned y=0U;y<height;++y) {
  float yf=(float)y/(float)height;
  float wf=0.16f+0.12f*(yf-0.40f)/0.28f;if(wf<0.07f) wf=0.07f;
  float c=(float)(width-1U)*0.5f+(float)curve*(float)width*0.04f*(0.68f-yf)*(0.68f-yf)/(0.28f*0.28f);
  int l=(int)(c-(float)width*wf*0.5f),r=(int)(c+(float)width*wf*0.5f);
  uint8_t far=(uint8_t)(kind==1 || kind==3 || kind==5);
  float lo=far ? 0.35f : 0.53f,hi=far ? 0.43f : 0.62f;
  if(kind!=0 && yf>=lo && yf<=hi) {
   l=(kind==1 || kind==2) ? (int)width/40 : 0;
   if(kind>=3) r=(int)width-1;
  }
  if(l<0) l=0;
  if(r>=(int)width) r=(int)width-1;
  for(int x=l;x<=r;++x) pixels[y*width+(mirror ? width-1U-(unsigned)x : (unsigned)x)]=255U;
  if((kind==1 || kind==2) && y>=height*RING_SEARCH_START_PERCENT/100U && y<height*RING_SEARCH_END_PERCENT/100U) {
   unsigned start=height*RING_SEARCH_START_PERCENT/100U,end=height*RING_SEARCH_END_PERCENT/100U;
   float zy=(float)(y-start)*59.f/(float)(end-start-1U),cy=kind==1 ? 20.f : 30.f;
   for(unsigned x=0U;x<width;++x) {
    float zx=(float)x*79.f/(float)(width-1U),dx=zx-10.f,dy=zy-cy;
    float outside=dx*dx/(25.f*25.f)+dy*dy/(19.f*19.f);
    float inside=dx*dx/(15.f*15.f)+dy*dy/(11.f*11.f);
    uint8_t white=(uint8_t)((zx>=30.f && zx<=50.f) || (outside<=1.f && inside>=1.f));
    pixels[y*width+(mirror ? width-1U-x : x)]=white ? 255U : 0U;
   }
  }
  if(hood && yf>=0.70f) for(unsigned x=width/4U;x<width*3U/4U;++x) pixels[y*width+x]=0U;
 }
}
static void process(void)
{
 track_vision_process(pixels,(uint16_t)height,(uint16_t)width,&result);
 track_ring_vision_process(&model,pixels,(uint16_t)height,(uint16_t)width,&result);
}
static void observe_image(ring_control_state_t *s)
{
 process();now+=10U;
 ring_control_tick(s,1U,0U,0.f,0.f,now);ring_control_observe(s,&result.ring,now,1U);
 (void)ring_control_select_path(s,&result.ring);
 CHECK(!s->fault);
}
static void frame(ring_control_state_t *s,int kind,int mirror)
{
 image(kind,mirror,0,1);observe_image(s);
}
static void arc_frame(ring_control_state_t *s,int mirror)
{
 image(0,mirror,-2,1);process();now+=10U;
 ring_control_tick(s,1U,0U,0.f,0.f,now);ring_control_observe(s,&result.ring,now,1U);
 (void)ring_control_select_path(s,&result.ring);CHECK(!s->fault);
}
static void yaw_arc_frame(ring_control_state_t *s,int mirror)
{
 image(0,mirror,-2,1);process();now+=10U;
 ring_control_tick(s,1U,1U,mirror ? -150.f : 150.f,0.f,now);
 ring_control_observe(s,&result.ring,now,1U);
 (void)ring_control_select_path(s,&result.ring);CHECK(!s->fault);
}
static void sequence(int mirror)
{
 ring_control_state_t ring;ring_control_init(&ring);track_ring_vision_init(&model);now=100U;
 for(unsigned i=0U;i<10U;++i) frame(&ring,0,mirror);
 CHECK(model.ready && ring.remembered_line);
 for(unsigned cycle=0U;cycle<2U;++cycle) {
  int far=cycle%2U==0U ? 1 : 3,near=far+1;
  for(unsigned i=0U;i<RING_CONFIRM_FRAMES;++i) frame(&ring,far,mirror);
  CHECK(ring.entry_seen && ring.phase==RING_STATE_APPROACH && !ring.line_active);
  CHECK(ring.kind==(far==1 ? TRACK_RING_360 : TRACK_RING_270));
  for(unsigned i=0U;i<RING_NEAR_CONFIRM_FRAMES;++i) frame(&ring,near,mirror);
  CHECK(ring.phase==RING_STATE_ENTERING && ring.line_active);
  CHECK(abs(ring.selected_path.error)<100 && abs(ring.selected_path.heading)<=RING_LINE_MAX_TILT+1.f);
  for(unsigned i=0U;i<30U;++i) frame(&ring,near,mirror);
  CHECK(!ring.exit_seen && ring.kind==(far==1 ? TRACK_RING_360 : TRACK_RING_270));
  if(far==1) {
   for(unsigned i=0U;i<10U && ring.phase!=RING_STATE_INSIDE;++i) arc_frame(&ring,mirror);
   CHECK(ring.entry_passed && ring.phase==RING_STATE_INSIDE);
   for(unsigned i=0U;i<400U && ring.phase==RING_STATE_INSIDE;++i) yaw_arc_frame(&ring,mirror);
   CHECK(ring.exit_seen && ring.yaw_progress_deg>=RING_360_TURN_DEG);
  } else {
   for(unsigned i=0U;i<RING_CLEAR_FRAMES;++i) frame(&ring,0,mirror);
   for(unsigned i=0U;i<RING_REVISIT_CLEAR_FRAMES;++i) arc_frame(&ring,mirror);
   CHECK(ring.entry_passed && ring.revisit_armed && ring.phase==RING_STATE_INSIDE);
   for(unsigned i=0U;i<RING_CONFIRM_FRAMES;++i) frame(&ring,far,mirror);
   CHECK(ring.exit_seen && ring.phase==RING_STATE_EXITING && ring.line_active);
   for(unsigned i=0U;i<RING_NEAR_CONFIRM_FRAMES;++i) frame(&ring,near,mirror);
  }
  CHECK(ring.phase==RING_STATE_EXITING && ring.line_active);
  for(unsigned i=0U;i<30U;++i) frame(&ring,near,mirror);
  CHECK(ring.phase==RING_STATE_EXITING && ring.line_active);
  for(unsigned i=0U;i<RING_EXIT_RESTORE_FRAMES;++i) frame(&ring,0,mirror);
  CHECK(ring.phase==RING_STATE_COOLDOWN && !ring.line_active);
  for(unsigned i=0U;i<RING_COOLDOWN_MS/10U+1U;++i) frame(&ring,0,mirror);
  CHECK(ring.phase==RING_STATE_IDLE && ring.event_id==cycle+1U);
 }
 CHECK(ring.completed_360==1U && ring.completed_270==1U);
 for(unsigned i=0U;i<10U;++i) frame(&ring,2,mirror);
 CHECK(ring.phase==RING_STATE_IDLE && !ring.entry_seen && ring.event_id==2U);
 for(unsigned i=0U;i<10U;++i) frame(&ring,4,mirror);
 CHECK(ring.phase==RING_STATE_IDLE && !ring.entry_seen && ring.event_id==2U);
}
int main(void)
{
 for(unsigned scale=1U;scale<=4U;scale*=2U) {
  height=120U*scale;width=188U*scale;
  for(int mirror=0;mirror<2;++mirror) {
   track_ring_vision_init(&model);image(0,mirror,0,1);
   for(unsigned i=0U;i<8U;++i) process();
   CHECK(model.ready && result.ring.paired_road && !result.ring.junction);
   image(1,mirror,0,1);process();
   printf("%ux%u mirror%d far360: type%u top%u/%u edge%u/%u near%u SS%u\n",height,width,mirror,result.ring.type_hint,result.ring.topology[0],result.ring.topology[1],result.ring.edge_line_valid[0],result.ring.edge_line_valid[1],result.ring.near_open,result.ring.straight_support);
   CHECK(result.ring.type_hint==TRACK_RING_360 && !result.ring.near_open);
   CHECK(result.ring.direction_hint==(mirror ? -1 : 1));
   image(2,mirror,0,1);process();CHECK(result.ring.near_open && result.ring.type_hint==TRACK_RING_360);
   image(3,mirror,0,1);process();CHECK(result.ring.type_hint==TRACK_RING_270 && !result.ring.near_open);
   image(4,mirror,0,1);process();CHECK(result.ring.type_hint==TRACK_RING_270 && result.ring.near_open);
   sequence(mirror);
   for(int curve=-3;curve<=3;++curve) {
    image(0,mirror,curve,1);process();CHECK(result.ring.type_hint==TRACK_RING_UNKNOWN);
   }
   /* Changing only pixels below the hood cut cannot change ring recognition. */
   image(0,mirror,0,0);process();track_ring_observation_t clear=result.ring;
   image(0,mirror,0,1);process();CHECK(result.ring.type_hint==clear.type_hint && !result.ring.junction);
   memset(pixels,255,height*width);process();CHECK(result.ring.type_hint==TRACK_RING_UNKNOWN);
   memset(pixels,0,height*width);process();CHECK(result.ring.type_hint==TRACK_RING_UNKNOWN && !result.ring.path[3].valid);
  }
 }
 height=240U;width=376U;track_ring_vision_init(&model);image(0,0,0,1);for(unsigned i=0U;i<8U;++i) process();
 /* A dark island must have white flanks, not just be the external black floor. */
 image(2,0,0,1);
 for(unsigned y=height*31U/100U;y<height*39U/100U;++y) {
  int r=(int)((float)width*(0.50f+(0.16f+0.12f*((float)y/(float)height-0.40f)/0.28f)*0.5f));
  for(unsigned x=width*8U/100U;x<(unsigned)r;++x) pixels[y*width+x]=255U;
  for(unsigned x=width*18U/100U;x<width*32U/100U;++x) pixels[y*width+x]=0U;
 }
 process();CHECK(result.ring.island_left && result.ring.type_hint==TRACK_RING_360);
 /* One structured candidate followed by blank white is never a new entry. */
 ring_control_state_t ring;ring_control_init(&ring);track_ring_vision_init(&model);now=100U;
 for(unsigned i=0U;i<8U;++i) frame(&ring,0,0);
 frame(&ring,4,0);
 for(unsigned i=0U;i<4U;++i) {
  memset(pixels,255,height*width);observe_image(&ring);
  CHECK(!ring.entry_seen && !ring.line_active && !ring.selected_path.valid);
 }
 /* A persistent first mouth cannot count as an exit. After leaving it,
  * saturated white still fails the unchanged entrance predicate; a structured
  * second 270 cross-band must be seen to start the exit. */
 for(unsigned i=0U;i<8U;++i) frame(&ring,0,0);
 for(unsigned i=0U;i<RING_CONFIRM_FRAMES;++i) frame(&ring,4,0);
 CHECK(ring.entry_seen && ring.line_active && ring.kind==TRACK_RING_270);
 for(unsigned i=0U;i<100U;++i) {
  memset(pixels,255,height*width);observe_image(&ring);
  CHECK(ring.line_active && !ring.entry_passed && !ring.exit_seen);
 }
 for(unsigned i=0U;i<RING_CLEAR_FRAMES;++i) frame(&ring,0,0);
 for(unsigned i=0U;i<RING_REVISIT_CLEAR_FRAMES;++i) arc_frame(&ring,0);
 CHECK(ring.revisit_armed);
 /* Test admission rejection without the independent optional loss-stop. */
 ring_control_set_protection(&ring,0U);
 for(unsigned i=0U;i<RING_CONFIRM_FRAMES;++i) {
  memset(pixels,255,height*width);observe_image(&ring);
 }
 CHECK(!ring.exit_seen && ring.phase==RING_STATE_INSIDE && !ring.line_active);
 for(unsigned i=0U;i<RING_270_EXIT_CONFIRM_FRAMES;++i) frame(&ring,4,0);
 CHECK(ring.exit_seen && ring.phase==RING_STATE_EXITING && ring.line_active);
 for(unsigned i=0U;i<100U;++i) {
  memset(pixels,255,height*width);observe_image(&ring);CHECK(ring.line_active);
 }
 for(unsigned i=0U;i<RING_EXIT_RESTORE_FRAMES;++i) frame(&ring,0,0);
 CHECK(ring.phase==RING_STATE_COOLDOWN);
 /* Stored straight endpoints are evaluated at 40% and 68%, not band averages. */
 image(0,0,-1,1);process();CHECK(result.ring.reference_valid);
 double sy=0.,syy=0.,sx=0.,sxy=0.,n=0.;
 for(unsigned y=height*RING_GUIDE_FAR_PERCENT/100U;y<height*RING_GUIDE_NEAR_PERCENT/100U;++y) {
  if(!result.row_valid[y]) continue;
  sy+=y;syy+=(double)y*y;sx+=result.center[y];sxy+=(double)y*result.center[y];n+=1.;
 }
 double k=(n*sxy-sy*sx)/(n*syy-sy*sy),b=(sx-k*sy)/n;
 double expected_near=(b+k*height*RING_GUIDE_NEAR_PERCENT/100.)*2000./(width-1U)-1000.;
 double expected_far=(b+k*height*RING_GUIDE_FAR_PERCENT/100.)*2000./(width-1U)-1000.;
 CHECK(fabs(result.ring.reference_near-expected_near)<0.1 && fabs(result.ring.reference_far-expected_far)<0.1);
 uint32_t rng=12345U;
 for(unsigned f=0U;f<200U;++f) {
  for(unsigned i=0U;i<height*width;++i) {rng=rng*1664525U+1013904223U;pixels[i]=(rng>>31U) ? 255U : 0U;}
  process();CHECK(result.ring.entry_score==0U);
 }
 puts("PASS ZORE geometry: 3 resolutions/mirrors, upper detection vs near takeover, one360+one270 actual-pixel routes, 280deg action-chain exit, completed-type suppression, normal curves, hood exclusion, island, black/white/noise rejection");
 return 0;
}
