#include "ring_control.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x) do {if(!(x)){fprintf(stderr,"FAIL %d %ux%u: %s\n",__LINE__,h,w,#x);exit(1);}}while(0)
static uint8_t image[480*752],saved[480*752];
static track_vision_result_t result,baseline;
static track_ring_vision_state_t model;
static unsigned h,w;
static void road(void)
{
 memset(image,0,h*w);
 for(unsigned y=0U;y<h;++y) {
  float yf=(float)y/(float)h,width=0.16f+0.12f*(yf-0.40f)/0.28f;
  if(width<0.07f) width=0.07f;
  unsigned half=(unsigned)((float)w*width*0.5f);
  memset(image+y*w+w/2U-half,255,half*2U+1U);
 }
}
static void process(void)
{
 memcpy(saved,image,h*w);track_vision_process(image,(uint16_t)h,(uint16_t)w,&result);
 track_ring_vision_process(&model,image,(uint16_t)h,(uint16_t)w,&result);
 CHECK(memcmp(saved,image,h*w)==0);
}
static void no_far_guidance(unsigned end)
{
 for(unsigned y=0U;y<end;++y) {
  CHECK(!result.row_valid[y]);
  for(unsigned mode=0U;mode<TRACK_PATH_COUNT-1U;++mode) CHECK(!result.guide_valid[mode][y]);
 }
 CHECK(!result.ring.path[TRACK_PATH_STRAIGHT].valid && result.ring.entry_score==0U);
}
int main(void)
{
 for(unsigned scale=1U;scale<=4U;scale*=2U) {
  h=120U*scale;w=188U*scale;track_ring_vision_init(&model);
  road();for(unsigned n=0U;n<8U;++n) process();baseline=result;
  /* A second road is present in every row, but does not touch the ego road. */
  for(unsigned y=0U;y<h*68U/100U;++y) memset(image+y*w+w/12U,255,w/10U);
  process();CHECK(result.valid && result.error==baseline.error && result.heading==baseline.heading);
  CHECK(memcmp(result.center,baseline.center,sizeof(result.center))==0);
  CHECK(memcmp(result.guide_valid,baseline.guide_valid,sizeof(result.guide_valid))==0);
  CHECK(memcmp(result.guide_center,baseline.guide_center,sizeof(result.guide_center))==0);
  /* Even an aligned far road cannot be reacquired after a black band. */
  road();memset(image+h*52U/100U*w,0,(h*55U/100U-h*52U/100U)*w);
  process();no_far_guidance(h*52U/100U);CHECK(!result.valid);
  /* No near road: a large distant horizontal white route is not a seed. */
  memset(image,0,h*w);
  for(unsigned y=h*30U/100U;y<h*51U/100U;++y) memset(image+y*w+w/5U,255,w*3U/5U);
  process();CHECK(!result.valid && result.valid_rows==0U && !result.ring.junction);
  no_far_guidance(h);
  /* Near ego road + a separate upper wide road: no false 270/exit or centre. */
  road();memset(image,0,h*55U/100U*w);
  for(unsigned y=h*30U/100U;y<h*51U/100U;++y) memset(image+y*w,255,w);
  process();no_far_guidance(h*55U/100U);CHECK(!result.ring.junction);
 }
 puts("PASS nearest-route ownership: 3 resolutions, separate parallel road, aligned upper road behind black band, far-only scene, remote white crossing; ordinary and ring guides reject remote components, shared binary immutable");
 return 0;
}
