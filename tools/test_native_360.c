#include <stdio.h>
#include <stdlib.h>
#include "../demo/user/src/track_vision.c"
#define CHECK(x) do {if(!(x)){fprintf(stderr,"FAIL %d: %s\n",__LINE__,#x);exit(1);}}while(0)
static struct {int Road_type;} ImageStatus;
static struct {int image_element_rings_flag;} ImageFlag;
#define LeftCirque 1
#define RightCirque 2
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#include "fixtures/zore_original_360_servo.inc"
#pragma GCC diagnostic pop
static uint8_t picture[TRACK_VISION_MAX_ROWS*TRACK_VISION_MAX_COLS],saved_picture[sizeof(picture)];
static track_vision_result_t result,raw_copy;
static track_ring_vision_state_t model;
static void straight(unsigned h,unsigned w)
{
    memset(picture,0,h*w);
    for(unsigned y=0U;y<h;++y) {
        float yf=(float)y/(float)h;
        float width=(float)w*(0.16f+0.12f*(yf-0.40f)/0.28f);
        if(width<(float)w*0.07f) width=(float)w*0.07f;
        int l=(int)((float)(w-1U)*0.5f-width*0.5f),r=(int)((float)(w-1U)*0.5f+width*0.5f);
        for(int x=l;x<=r;++x) if(x>=0 && x<(int)w) picture[y*w+(unsigned)x]=255U;
    }
}
static unsigned test_height,test_width;
static void ring_picture(int kind,int mirror,int curve,int hood)
{
 memset(picture,0,test_height*test_width);
 for(unsigned y=0U;y<test_height;++y) {
  float yf=(float)y/(float)test_height;
  float wf=0.16f+0.12f*(yf-0.40f)/0.28f;if(wf<0.07f) wf=0.07f;
  float c=(float)(test_width-1U)*0.5f+(float)curve*(float)test_width*0.04f*(0.68f-yf)*(0.68f-yf)/(0.28f*0.28f);
  int l=(int)(c-(float)test_width*wf*0.5f),r=(int)(c+(float)test_width*wf*0.5f);
  uint8_t far=(uint8_t)(kind==1 || kind==3 || kind==5);
  float lo=far ? 0.35f : 0.53f,hi=far ? 0.43f : 0.62f;
  if(kind!=0 && yf>=lo && yf<=hi) {
   l=(kind==1 || kind==2) ? (int)test_width/40 : 0;
   if(kind>=3) r=(int)test_width-1;
  }
  if(l<0) l=0;
  if(r>=(int)test_width) r=(int)test_width-1;
  for(int x=l;x<=r;++x) picture[y*test_width+(mirror ? test_width-1U-(unsigned)x : (unsigned)x)]=255U;
  if((kind==1 || kind==2) && y>=test_height*RING_SEARCH_START_PERCENT/100U && y<test_height*RING_SEARCH_END_PERCENT/100U) {
   unsigned start=test_height*RING_SEARCH_START_PERCENT/100U,end=test_height*RING_SEARCH_END_PERCENT/100U;
   float zy=(float)(y-start)*59.f/(float)(end-start-1U),cy=kind==1 ? 20.f : 30.f;
   for(unsigned x=0U;x<test_width;++x) {
    float zx=(float)x*79.f/(float)(test_width-1U),dx=zx-10.f,dy=zy-cy;
    float outside=dx*dx/(25.f*25.f)+dy*dy/(19.f*19.f);
    float inside=dx*dx/(15.f*15.f)+dy*dy/(11.f*11.f);
    uint8_t white=(uint8_t)((zx>=30.f && zx<=50.f) || (outside<=1.f && inside>=1.f));
    picture[y*test_width+(mirror ? test_width-1U-x : x)]=white ? 255U : 0U;
   }
  }
  if(hood && yf>=0.70f) for(unsigned x=test_width/4U;x<test_width*3U/4U;++x) picture[y*test_width+x]=0U;
 }
}
static void actual_ring_images(void)
{
 for(unsigned factor=1U;factor<=4U;factor*=2U) for(int mirror=0;mirror<2;++mirror) {
  test_height=120U*factor;test_width=188U*factor;
  track_ring_vision_init(&model);track_ring_360_set_active(0U,0);
  ring_picture(0,mirror,0,1);
  for(unsigned n=0U;n<8U;++n) {track_vision_process(picture,test_height,test_width,&result);track_ring_vision_process(&model,picture,test_height,test_width,&result);}
  CHECK(result.ring.zore360_width_ready);
  for(int kind=1;kind<=4;++kind) {
   ring_picture(kind,mirror,0,1);memcpy(saved_picture,picture,test_height*test_width);
   track_vision_process(picture,test_height,test_width,&result);
   track_ring_vision_process(&model,picture,test_height,test_width,&result);
   CHECK(memcmp(saved_picture,picture,test_height*test_width)==0);
   if(kind<=2) {CHECK(result.ring.type_hint==TRACK_RING_360 && result.ring.zore360_match);CHECK(result.ring.direction_hint==(mirror ? -1 : 1));}
   else CHECK(result.ring.type_hint==TRACK_RING_270 && !result.ring.zore360_match);
  }
 }
 puts("PASS actual native binary images: far/near left/right 360 admission and unchanged 270 classification at 3 camera resolutions, no image mutation");
}
static void native_pipeline(void)
{
    for(unsigned factor=1U;factor<=4U;factor*=2U) {
        unsigned h=120U*factor,w=188U*factor;
        track_ring_vision_init(&model);track_ring_360_set_active(0U,0);straight(h,w);
        memcpy(saved_picture,picture,h*w);
        for(unsigned f=0U;f<8U;++f) {track_vision_process(picture,(uint16_t)h,(uint16_t)w,&result);track_ring_vision_process(&model,picture,(uint16_t)h,(uint16_t)w,&result);}
        CHECK(memcmp(saved_picture,picture,h*w)==0 && result.ring.zore360_width_ready && result.ring.zore360_ready);
        CHECK(result.ring.zore360_native_rows==h && result.ring.zore360_native_cols==w);
        CHECK(!result.ring.zore360_match && result.ring.type_hint==TRACK_RING_UNKNOWN);
        raw_copy=result;
        (void)ring_360_zore(picture,h,w,&model,&result);
        CHECK(memcmp(raw_copy.raw_left,result.raw_left,sizeof(result.raw_left))==0);
        CHECK(memcmp(raw_copy.raw_right,result.raw_right,sizeof(result.raw_right))==0);
        CHECK(memcmp(raw_copy.raw_flags,result.raw_flags,sizeof(result.raw_flags))==0);
        CHECK(memcmp(raw_copy.left,result.left,sizeof(result.left))==0 && memcmp(raw_copy.center,result.center,sizeof(result.center))==0);
        CHECK(memcmp(raw_copy.row_valid,result.row_valid,sizeof(result.row_valid))==0);
        int native_y=z360_y(43)+1,native_x=(int)w/2+1;
        CHECK(route_member((unsigned)native_y,(unsigned)native_x));
        picture[(unsigned)native_y*w+(unsigned)native_x]=0U;CHECK(z360_px(native_y,native_x)==0);
        picture[(unsigned)native_y*w+(unsigned)native_x]=255U;CHECK(z360_px(native_y,native_x)==1);
        CHECK(z360_binary==picture); /* Direct original-pixel access, no copied image. */
        float learned=z360_width_near;
        track_ring_360_set_active(5U,1);model.width_near*=1.2f;
        (void)ring_360_zore(picture,h,w,&model,&result);CHECK(z360_width_near==learned);
        track_ring_360_set_active(0U,0);
        printf("PASS native %ux%u: direct source pixels, original image/raw/ordinary arrays immutable, width %.1fpx and frozen model\n",h,w,learned);
    }
}
static void native_raw(unsigned h,unsigned w)
{
    z360_h=h;z360_w=w;z360_top=(int)(h*30U/100U);z360_end=(int)(h*68U/100U);
    z360_sy=(float)(z360_end-z360_top-1)/59.f;
    z360_width_near=(float)w*0.28f;z360_width_slope=0.f;z360_width_ready=1U;
    z360_off=z360_y(10);z360_lost[0]=z360_lost[1]=z360_white=0;
    memset(z360_rows,0,sizeof(z360_rows));memset(picture,0,h*w);z360_binary=picture;
    for(int y=z360_top;y<z360_end;++y) {
        z360_rows[y].left=(int)((float)w*0.36f);z360_rows[y].right=(int)((float)w*0.64f);
        z360_rows[y].center=(z360_rows[y].left+z360_rows[y].right)/2;
        z360_rows[y].found[0]=z360_rows[y].found[1]='T';
        z360_rows[y].first[0]=z360_rows[y].last[0]=z360_rows[y].left;
        z360_rows[y].first[1]=z360_rows[y].last[1]=z360_rows[y].right;
    }
}
static void geometric_rules(void)
{
    for(unsigned factor=1U;factor<=4U;factor*=2U) for(unsigned side=0U;side<2U;++side) {
        unsigned h=120U*factor,w=188U*factor;native_raw(h,w);
        z360_lost[side]=(int)ceilf(22.f*z360_sy);
        int a=z360_y(35),b=z360_y(45),one=z360_step(1),p1=0,p2=0;
        int jump=z360_dx(7.f,a);
        z360_rows[a].first[side]+=side==0U ? jump : -jump;
        z360_rows[b+one].last[side]+=side==0U ? z360_dx(7.f,b) : -z360_dx(7.f,b);
        CHECK(z360_admit(side,&p1,&p2));
        CHECK(p1>=z360_top && p2>p1 && p2<z360_end);
        int keep=z360_lost[side];z360_lost[side]=0;CHECK(!z360_admit(side,&p1,&p2));z360_lost[side]=keep;
        z360_lost[1U-side]=(int)ceilf(5.f*z360_sy);CHECK(!z360_admit(side,&p1,&p2));z360_lost[1U-side]=0;
        uint8_t done=0U;
        for(int y=z360_y(31);y<=z360_y(55);++y) z360_rows[y].found[side]='W';
        CHECK(z360_handle(side,1U,&done)==2U && !done);
        native_raw(h,w);
        CHECK(z360_handle(side,2U,&done)==5U);
        native_raw(h,w);z360_lost[1U-side]=(int)ceilf(13.f*z360_sy);
        CHECK(z360_handle(side,5U,&done)==6U);
        native_raw(h,w);
        for(int y=z360_top;y<z360_end;++y) {
            if(side==0U) z360_rows[y].right=2*y;else z360_rows[y].left=2*y;
        }
        CHECK(z360_handle(side,6U,&done)==7U);
        native_raw(h,w);
        /* Flat recovered outside edge, with own side still missing, accepts
         * original 8->9 exit without angle or a timed straight prerequisite. */
        for(int y=z360_y(11);y<=z360_y(40);++y) z360_rows[y].found[side]='W';
        CHECK(z360_handle(side,8U,&done)==9U && done);
        native_raw(h,w);done=0U;CHECK(z360_handle(side,9U,&done)==0U);
        native_raw(h,w);z360_off=z360_y(10);
        (void)z360_handle(side,1U,&done);
        int y=z360_y(50),center=z360_rows[y].center;
        float expected=side==0U ? (float)z360_rows[y].right-z360_width(y)*0.5f+RING_360_LEFT_ENTRY_TRIM*z360_sx(y) : (float)z360_rows[y].left+z360_width(y)*0.5f;
        CHECK(abs(center-(int)lroundf(expected))<=1);
    }
    puts("PASS 3 resolutions / both sides: scaled strict admission, original 1/2/5/6/7/8/9/0 conditions and native perspective-width center repair");
}
static void transaction(void)
{
    static track_ring_observation_t v;
    for(int direction=-1;direction<=1;direction+=2) {
        ring_control_state_t s;ring_control_init(&s);ring_control_set_protection(&s,0U);
        memset(&v,0,sizeof(v));v.zore360_ready=v.zore360_match=1U;v.direction_hint=(int8_t)direction;
        v.type_hint=TRACK_RING_360;v.zore360_top=72U;v.zore360_end=163U;v.zore360_native_rows=240U;v.zore360_native_cols=376U;
        v.zore360_center_x=187.5f;unsigned side=direction>0 ? 0U : 1U;
        const unsigned stages[]={1U,2U,5U,6U,7U,8U,9U,0U};
        for(unsigned i=0U;i<7U;++i) {
            unsigned stage=stages[i],next=stages[i+1U];
            v.zore360_next[side][stage]=(uint8_t)next;v.zore360_done[side][stage]=(uint8_t)(next==9U);
            v.zore360_det[side][stage]=(int16_t)(direction>0 ? 160 : 215);v.zore360_det_valid[side][stage]=1U;
            v.zore360_error_scale[side][stage]=2.f;v.zore360_path[side][stage]=(track_path_observation_t){(int16_t)(-direction*150),0,1U,100U};
            uint32_t now=100U+i*10U;ring_control_tick(&s,1U,1U,(float)direction*1000.f,0.f,now);
            ring_control_observe(&s,&v,now,1U);(void)ring_control_select_path(&s,&v);CHECK(s.yaw_progress_deg==0.f);
            if(next) {
                CHECK(s.zore360_stage==next && s.action_locked && !s.line_active);
                ring_control_observe(&s,&v,now,1U);CHECK(s.zore360_stage==next);
                float angle=87.f,speed=120.f;ring_control_360_motion(&s,87.f,98.5f,75.5f,120.f,&angle,&speed);
                CHECK(angle>=75.5f && angle<=98.5f && speed>=0.f && speed<=102.f);
            } else CHECK(!s.action_locked && s.completed_360==1U);
        }
        ring_control_observe(&s,&v,500U,1U);CHECK(s.action_locked && s.kind==TRACK_RING_360);
    }
    puts("PASS native ISR transaction: same stage chain, no yaw integration, duplicate rejection, completion and repeated rings");
}
static void motion_units(void)
{
 unsigned checks=0U;
 for(int stage=1;stage<=9;++stage) for(int error=-42;error<=37;++error)
  for(int previous=-40;previous<=40;previous+=10) for(int rate=-200;rate<=200;rate+=50) {
   ring_control_state_t state;ring_control_init(&state);state.action_locked=1U;state.kind=TRACK_RING_360;
   state.zore360_stage=(uint8_t)stage;state.zore360_center_x=188.f;state.zore360_error_scale=2.f;
   state.zore360_det=(int16_t)(188-error*2);state.zore360_previous_error=(int16_t)previous;state.yaw_rate_dps=(float)rate;
   float angle=87.f,speed=120.f;ring_control_360_motion(&state,87.f,98.5f,75.5f,120.f,&angle,&speed);
   PID_t original={(int16_t)previous,0,0,-550,650};
   ImageStatus.Road_type=LeftCirque;ImageFlag.image_element_rings_flag=stage;imu660rb_gyro_z=(int16_t)rate;
   PID_Update_Servo(&original,(int16_t)error);CHECK((int)state.zore360_servo_offset==original.Out);++checks;
  }
 printf("PASS %u native-unit conversion / original ZORE fuzzy steering comparisons\n",checks);
}
int main(void) {native_pipeline();actual_ring_images();geometric_rules();transaction();motion_units();return 0;}
