/* Compile the actual private adapter and compare its admission to ZORE's
 * original functions. The reference oracle is host-test-only. */
#include <stdio.h>
#include <stdlib.h>
#include "../demo/user/src/track_vision.c"
#define CHECK(x) do {if(!(x)){fprintf(stderr,"FAIL %d: %s\n",__LINE__,#x);exit(1);}}while(0)
typedef struct {
 int LeftBorder,RightBorder,Wide,LeftBoundary_First,RightBoundary_First,LeftBoundary,RightBoundary;
 char IsLeftFind,IsRightFind;
} oracle_row_t;
static oracle_row_t ImageDeal[70];
static struct {int Right_Line,Left_Line,OFFLine,WhiteLine,Road_type;} ImageStatus;
static struct {int image_element_rings_flag,image_element_rings,ring_big_small;} ImageFlag;
static int Ysite,Ring_Help_Flag,Global_Straight_R,Global_Straight_L;
static int Left_RingsFlag_Point1_Ysite,Left_RingsFlag_Point2_Ysite;
static int Right_RingsFlag_Point1_Ysite,Right_RingsFlag_Point2_Ysite;
#define LEFT_RING_LOSS 19
#define RIGHT_RING_LOSS 18
#define safe_wide_thresh 40
#define LeftCirque 1
#define RightCirque 2
#define BEEP_PIN 0
#define gpio_set_level(pin,value) ((void)(pin),(void)(value))
#include "fixtures/zore_original_360_admission.inc"

static uint32_t random_state=12345U;
static unsigned random_value(unsigned maximum)
{
 random_state=random_state*1664525U+1013904223U;return (random_state>>8U)%maximum;
}
static unsigned oracle_compare(void)
{
 unsigned accepted=0U,relaxed_count=0U;
 for(unsigned trial=0U;trial<20000U;++trial) {
  zore360_frame_t f={0};
  f.off=10+(int)random_value(16U);f.white=(int)random_value(5U);
  unsigned favored=random_value(2U);
  f.lost[favored]=15+(int)random_value(20U);f.lost[1U-favored]=(int)random_value(4U);
  f.variance[0]=(int)random_value(230U);f.variance[1]=(int)random_value(230U);
  for(unsigned y=0U;y<60U;++y) {
   f.border[0][y]=5+(int)random_value(20U);f.border[1][y]=55+(int)random_value(20U);
   f.found[0][y]=f.found[1][y]='T';
   f.first[1][y]=f.last[1][y]=79;
   if(y>=26U && y<=49U) for(unsigned side=0U;side<2U;++side) {
    f.first[side][y]=(int)random_value(79U);f.last[side][y]=(int)random_value(79U);
   }
  }
  for(int y=59;y>=54;--y) if(f.border[1][y]-f.border[0][y]<40) {
   ++f.narrow[1];if(y<=57) ++f.narrow[0];
  }
  for(unsigned side=0U;side<2U;++side) {
   memset(&ImageFlag,0,sizeof(ImageFlag));memset(ImageDeal,0,sizeof(ImageDeal));
   ImageStatus.Left_Line=f.lost[0];ImageStatus.Right_Line=f.lost[1];
   ImageStatus.WhiteLine=f.white;ImageStatus.OFFLine=f.off;
   Global_Straight_L=f.variance[0];Global_Straight_R=f.variance[1];Ring_Help_Flag=0;
   for(unsigned y=0U;y<60U;++y) {
    ImageDeal[y].LeftBorder=f.border[0][y];ImageDeal[y].RightBorder=f.border[1][y];
    ImageDeal[y].Wide=f.border[1][y]-f.border[0][y];
    ImageDeal[y].IsLeftFind=f.found[0][y];ImageDeal[y].IsRightFind=f.found[1][y];
    ImageDeal[y].LeftBoundary_First=f.first[0][y];ImageDeal[y].RightBoundary_First=f.first[1][y];
    ImageDeal[y].LeftBoundary=f.last[0][y];ImageDeal[y].RightBoundary=f.last[1][y];
   }
   if(side==0U) Element_Judgment_Left_Rings();else Element_Judgment_Right_Rings();
   int expected=ImageFlag.image_element_rings_flag!=0;
   int valid_first=side==0U ? Left_RingsFlag_Point1_Ysite : Right_RingsFlag_Point1_Ysite;
   int relaxed=z360_decide(&f,side);
   if(expected && valid_first>0) {CHECK(relaxed);++accepted;}
   relaxed_count+=(unsigned)relaxed;
  }
 }
 CHECK(accepted>50U && relaxed_count>accepted);
 printf("PASS relaxed360: %u valid two-corner reference positives retained; %u relaxed positives in 40000 feature trials\n",accepted,relaxed_count);
 return accepted;
}

void before360_ring_vision_init(track_ring_vision_state_t *);
void before360_ring_vision_process(track_ring_vision_state_t *,const uint8_t *,uint16_t,uint16_t,track_vision_result_t *);
static uint8_t image[480*752],copy_image[480*752];
static track_vision_result_t current,previous;
static void strip360(track_ring_observation_t *v)
{
 v->zore360_match=v->zore360_both_lost=0U;
 memset(v->zore360_loss,0,sizeof(v->zore360_loss));
 memset(v->zore360_point1,0,sizeof(v->zore360_point1));
 memset(v->zore360_point2,0,sizeof(v->zore360_point2));
 memset(v->zore360_help,0,sizeof(v->zore360_help));
 memset(v->zore360_variance,0,sizeof(v->zore360_variance));
}
static unsigned compare270(void)
{
 unsigned count=0U;
 for(unsigned scale=1U;scale<=4U;scale*=2U) {
  unsigned h=120U*scale,w=188U*scale;
  track_ring_vision_state_t old_model,new_model;
  before360_ring_vision_init(&old_model);track_ring_vision_init(&new_model);
  for(unsigned frame=0U;frame<220U;++frame) {
   memset(image,0,h*w);
   int shift=(int)(frame%19U)-9;
   for(unsigned y=0U;y<h;++y) {
    float yf=(float)y/(float)h,wf=0.16f+0.12f*(yf-0.40f)/0.28f;
    if(wf<0.07f) wf=0.07f;
    int c=(int)w/2+shift,l=c-(int)((float)w*wf*0.5f),r=c+(int)((float)w*wf*0.5f);
    if(frame>=10U) {
     float low=frame%2U ? 0.35f : 0.53f,high=frame%2U ? 0.43f : 0.62f;
     if(yf>=low && yf<=high) {l=0;r=(int)w-1;}
    }
    l=z360_limit(l,0,(int)w-1);r=z360_limit(r,0,(int)w-1);
    for(int x=l;x<=r;++x) image[y*w+(unsigned)x]=255U;
   }
   memcpy(copy_image,image,h*w);
   track_vision_process(image,(uint16_t)h,(uint16_t)w,&current);previous=current;
   before360_ring_vision_process(&old_model,image,(uint16_t)h,(uint16_t)w,&previous);
   track_ring_vision_process(&new_model,image,(uint16_t)h,(uint16_t)w,&current);
   CHECK(!current.ring.zore360_match);
   strip360(&current.ring);strip360(&previous.ring);
   CHECK(memcmp(&current,&previous,sizeof(current))==0);
   CHECK(memcmp(copy_image,image,h*w)==0);
   if(previous.ring.type_hint==TRACK_RING_270) ++count;
  }
 }
 CHECK(count>500U);return count;
}
int main(void)
{
 unsigned accepted=oracle_compare(),unchanged=compare270();
 printf("PASS 360 admission: 40000 reference/relaxed comparisons (%u reference positives retained); 660 normal/270 frames exactly unchanged (%u confirmed270), immutable shared binary\n",accepted,unchanged);
 return 0;
}
