#include "ring_control.h"
#include "track_control.h"
#include "pid.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#define CHECK(x) do {if(!(x)){fprintf(stderr,"FAIL %d: %s\n",__LINE__,#x);exit(1);}}while(0)
#define H 240U
#define W 376U
void baseline_track_vision_process(const uint8_t *,uint16_t,uint16_t,track_vision_result_t *);
static uint8_t image[H*W],saved[H*W];
static track_vision_result_t baseline,result,ordinary_copy;
static track_ring_vision_state_t model;
static ring_control_state_t ring;
static track_control_state_t before,after;
static pid_param_t pid_before[2],pid_after[2];
static uint32_t now=100U;
static void picture(int shift,int curve)
{
 memset(image,0,sizeof(image));
 for(unsigned y=0U;y<H;++y) {
  float yf=(float)y/(float)H;
  float wf=0.16f+0.12f*(yf-0.40f)/0.28f;if(wf<0.07f) wf=0.07f;
  float c=(float)(W-1U)*0.5f+(float)shift+(float)curve*(float)W*0.025f*(0.68f-yf)*(0.68f-yf)/(0.28f*0.28f);
  int l=(int)(c-(float)W*wf*0.5f),r=(int)(c+(float)W*wf*0.5f);
  if(l<0) l=0;
  if(r>=(int)W) r=(int)W-1;
  for(int x=l;x<=r;++x) image[y*W+(unsigned)x]=255U;
  if(y>=H*70U/100U) for(unsigned x=W/4U;x<W*3U/4U;++x) image[y*W+x]=0U;
 }
}
static void ordinary_equal(const track_vision_result_t *a,const track_vision_result_t *b)
{
 CHECK(memcmp(a->left,b->left,sizeof(a->left))==0);
 CHECK(memcmp(a->right,b->right,sizeof(a->right))==0);
 CHECK(memcmp(a->center,b->center,sizeof(a->center))==0);
 CHECK(memcmp(a->row_valid,b->row_valid,sizeof(a->row_valid))==0);
#define SAME(field) CHECK(a->field==b->field)
 SAME(valid);SAME(confidence);SAME(reject_flags);SAME(valid_rows);SAME(roi_rows);
 SAME(near_valid_rows);SAME(far_valid_rows);SAME(no_candidate_rows);SAME(ambiguous_rows);
 SAME(disconnected_rows);SAME(near_width);SAME(far_width);SAME(width_growth_percent);
 SAME(frame_number);SAME(error);SAME(heading);
#undef SAME
}
static void compare_path(const track_ring_observation_t *v,uint8_t duplicate)
{
 if(!duplicate) now+=10U;
 ring_control_tick(&ring,after.running,0U,NAN,80.f,now);
 ring_control_observe(&ring,v,now,after.running);
 track_path_observation_t selected=ring_control_select_path(&ring,v);
 CHECK(!ring.line_active && !ring.reset_tracking_history && !ring.fault && !ring_control_guidance_active(&ring));
 CHECK(selected.error==v->path[0].error && selected.heading==v->path[0].heading &&
       selected.valid==v->path[0].valid && selected.confidence==v->path[0].confidence);
 track_control_observe(&before,v->path[0].error,v->path[0].heading,v->path[0].confidence,v->path[0].valid,now);
 track_control_observe(&after,selected.error,selected.heading,selected.confidence,selected.valid,now);
 track_control_output_t a=track_control_tick(&before,now),b=track_control_tick(&after,now);
 CHECK(memcmp(&before,&after,sizeof(before))==0);
 CHECK(a.running==b.running && a.ready==b.ready && a.stop_reason==b.stop_reason &&
       a.servo_angle==b.servo_angle && a.target_speed==b.target_speed);
 float da=Speed_Differential_Delta(a.target_speed,(a.servo_angle-87.f)*-1.f);
 float db=Speed_Differential_Delta(b.target_speed,(b.servo_angle-87.f)*-1.f);CHECK(da==db);
 for(unsigned wheel=0U;wheel<2U;++wheel) {
  float target_a=a.target_speed+(wheel==0U ? da : -da);
  float target_b=b.target_speed+(wheel==0U ? db : -db);
  CHECK(target_a==target_b);
  float measured=(float)((now/10U+wheel*7U)%110U);
  float pa=PidLocCtrl(&pid_before[wheel],target_a-measured,1.f);
  float pb=PidLocCtrl(&pid_after[wheel],target_b-measured,1.f);
  CHECK(pa==pb && memcmp(&pid_before[wheel],&pid_after[wheel],sizeof(pid_param_t))==0);
 }
}
static void process(void)
{
 memcpy(saved,image,sizeof(image));
 baseline_track_vision_process(image,H,W,&baseline);
 track_vision_process(image,H,W,&result);ordinary_equal(&baseline,&result);
 ordinary_copy=result;
 track_ring_vision_process(&model,image,H,W,&result);
 ordinary_equal(&ordinary_copy,&result);CHECK(memcmp(saved,image,sizeof(image))==0);
 compare_path(&result.ring,0U);
}
int main(void)
{
 track_ring_vision_init(&model);ring_control_init(&ring);
 track_control_init(&before,NULL);track_control_init(&after,NULL);
 for(unsigned i=0U;i<2U;++i) {
  Pid_Param_Init(&pid_before[i],50.f,0.2f,0.f,5000.f,2000.f);
  Pid_Set_Output_Limits(&pid_before[i],0.f,2000.f);pid_after[i]=pid_before[i];
 }
 for(unsigned i=0U;i<15U;++i) {memset(image,0,sizeof(image));process();}
 for(unsigned i=0U;i<15U;++i) {memset(image,255,sizeof(image));process();}
 picture(0,0);for(unsigned i=0U;i<8U;++i) process();
 track_control_toggle(&before,now);track_control_toggle(&after,now);
 CHECK(before.running && after.running);
 track_control_set_loss_stop(&before,0U);track_control_set_loss_stop(&after,0U);
 for(int f=0;f<2200;++f) {
  picture((f/10)%71-35,(f/50)%7-3);
  if(f%113==0) memset(image,0,sizeof(image));
  if(f%127==0) memset(image,255,sizeof(image));
  process();
  if(f%97==0) compare_path(&result.ring,1U);
 }
 picture(0,0);process();
 track_ring_observation_t ordinary=result.ring,special=ordinary;
 special.junction=1U;special.paired_road=0U;special.left_open=1U;special.right_open=0U;
 special.type_hint=TRACK_RING_360;special.direction_hint=1;special.entry_score=90U;
 special.edge_line_valid[1]=1U;special.straight_support=100U;special.near_open=1U;
 /* Unverified broad-mouth hints cannot enter the private ZORE transaction. */
 special.zore360_match=0U;
 for(unsigned i=0U;i<100U;++i) {compare_path(&special,0U);compare_path(&ordinary,0U);}
 CHECK(!ring.entry_seen && !ring.line_active);
 puts("PASS ordinary isolation vs pre-edit fixture: 2200 straight/bend/loss/white frames, hood, immutable binary and ordinary rows/scalars, duplicate frames, candidate glitches/far prep, exact controller/differential/dual PID target/state/PWM equality");
 return 0;
}
