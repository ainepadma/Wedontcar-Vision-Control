#include "ring_control.h"
#include "track_control.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define H 240U
#define W 376U
#define CHECK(c) do {if(!(c)){fprintf(stderr,"FAIL %d: %s (stage=%u counts=%u,%u guide=%u fault=%u)\n",__LINE__,#c,ring.corner_stage,result.ring.corners.count[0],result.ring.corners.count[1],ring.corner_guide_id,ring.fault);exit(1);}}while(0)
static uint8_t pixels[H*W],saved[H*W];
static track_vision_result_t result;
static track_ring_vision_state_t model;
static ring_control_state_t ring;
static track_control_state_t controller;
static uint32_t clock_ms,frame_id;
static track_path_observation_t selected;
static track_control_output_t output;

static void road(void)
{
    memset(pixels,0,sizeof(pixels));
    for(unsigned y=0;y<H;++y) for(unsigned x=W*29U/100U;x<=W*70U/100U;++x) pixels[y*W+x]=255U;
}
static void crossing(unsigned start,unsigned end)
{
    road();for(unsigned y=start;y<end;++y) memset(pixels+y*W,255,W);
}
static void acute(uint8_t kind,unsigned tip,uint8_t mirror)
{
    road();int edge=(int)(W*29U/100U);
    for(unsigned y=0;y<H;++y) for(int x=0;x<edge;++x) {
        float diagonal=kind==RING_CORNER_A ? (float)tip+.5f*(float)(edge-x) : (float)tip-.5f*(float)(edge-x);
        if((kind==RING_CORNER_A && (float)y<diagonal) || (kind==RING_CORNER_B && (float)y>diagonal)) pixels[y*W+(unsigned)x]=255U;
    }
    if(mirror) for(unsigned y=0;y<H;++y) for(unsigned x=0;x<W/2U;++x) {
        uint8_t p=pixels[y*W+x];pixels[y*W+x]=pixels[y*W+W-1U-x];pixels[y*W+W-1U-x]=p;
    }
}
static void frame_expect(uint8_t expected_fault)
{
    memcpy(saved,pixels,sizeof(pixels));clock_ms+=20U;
    track_vision_process(pixels,H,W,&result);result.frame_number=++frame_id;
    int16_t error=result.error,heading=result.heading;
    track_ring_vision_process(&model,pixels,H,W,&result);
    CHECK(result.error==error && result.heading==heading && !memcmp(saved,pixels,sizeof(pixels)));
    CHECK(result.ring.corners.frame_number==frame_id);
    ring_control_tick(&ring,controller.running,0U,0.f,25.f,clock_ms);
    ring_control_observe(&ring,&result.ring,clock_ms,controller.running);
    selected=ring_control_select_path(&ring,&result.ring);
    if(ring.reset_tracking_history) {
        if(selected.valid) {controller.filtered_error=(float)selected.error;controller.filtered_heading=(float)selected.heading;}
        controller.derivative=0.f;controller.derivative_ready=0U;ring.reset_tracking_history=0U;
    }
    track_control_observe(&controller,selected.error,selected.heading,selected.confidence,selected.valid,clock_ms);
    if(ring.fault) track_control_stop(&controller,TRACK_STOP_RING);
    output=track_control_tick(&controller,clock_ms);
    CHECK(isfinite(output.servo_angle) && output.servo_angle>=75.5f && output.servo_angle<=98.5f);
    if(ring.fault!=expected_fault) fprintf(stderr,"DETAIL frame=%u ms=%u passed=%u vote=%u/%u type=%u y=%.2f guides A=%u/%u B2=%u/%u B4=%u/%u clear=%u path=%u near=%u\n",
        frame_id,clock_ms,ring.corner_passed,ring.corner_vote_guide,ring.corner_vote_frames,
        result.ring.corners.point[0][0].kind,(double)result.ring.corners.point[0][0].y,
        result.ring.corners.guide[RING_GUIDE_A_LEFT].valid,result.ring.corners.guide[RING_GUIDE_A_LEFT].confidence,
        result.ring.corners.guide[RING_GUIDE_B_BRANCH_LEFT].valid,result.ring.corners.guide[RING_GUIDE_B_BRANCH_LEFT].confidence,
        result.ring.corners.guide[RING_GUIDE_B_MAIN_LEFT].valid,result.ring.corners.guide[RING_GUIDE_B_MAIN_LEFT].confidence,
        result.ring.corners.road_clear,result.ring.path[0].valid,result.ring.near_road);
    CHECK(ring.fault==expected_fault);
    if(ring.line_active && !ring.corner_guide_held) {
        CHECK(ring.corner_guide_frame_number==frame_id);
        CHECK(selected.error==result.ring.corners.guide[ring.corner_guide_id].error);
    }
}
static void frame(void) {frame_expect(0U);}
static void frames(unsigned count) {for(unsigned i=0;i<count;++i) frame();}
static void setup(void)
{
    clock_ms=0U;frame_id=0U;track_ring_vision_init(&model);ring_control_init(&ring);
    const track_control_config_t config={87.f,98.5f,75.5f,TRACK_SPEED_STRAIGHT,TRACK_SPEED_CURVE_MIN,.02f,.0005f,120.f};
    track_control_init(&controller,&config);road();frames(8U);
    CHECK(model.ready && output.ready);track_control_toggle(&controller,clock_ms);CHECK(controller.running);
}
static void test_270(void)
{
    setup();
    for(unsigned pass=0U;pass<2U;++pass) {
        crossing(0U,125U);frames(3U);
        CHECK(ring.kind==TRACK_RING_270 && ring.corner_stage==(pass==0U ? 1U : 4U));
        CHECK(ring.corner_guide_id==RING_GUIDE_270_EXTEND && selected.valid);
        crossing(98U,132U);frames(3U);
        CHECK(ring.corner_stage==(pass==0U ? 2U : 5U));
        CHECK(ring.corner_guide_id==RING_GUIDE_270_CONNECT && abs(selected.error)<30);
        road();frames(3U);
        CHECK(pass==0U ? ring.corner_stage==3U : ring.completed_270==1U);
    }
    CHECK(!ring.action_locked);
}
static void test_360(uint8_t mirror)
{
    setup();acute(RING_CORNER_A,125U,mirror);frames(3U);
    CHECK(ring.kind==TRACK_RING_360 && ring.corner_stage==1U);
    CHECK(ring.corner_guide_id==RING_GUIDE_A_LEFT+mirror);
    road();frames(4U);
    acute(RING_CORNER_B,105U,mirror);frames(3U);
    CHECK(ring.corner_stage==2U && ring.corner_guide_id==RING_GUIDE_B_BRANCH_LEFT+mirror);
    CHECK(mirror ? selected.error>0 : selected.error<0);
    CHECK(mirror ? output.servo_angle<87.f : output.servo_angle>87.f);
    road();frames(4U);
    acute(RING_CORNER_A,125U,(uint8_t)!mirror);frames(3U);
    CHECK(ring.corner_stage==3U && ring.corner_guide_id==RING_GUIDE_A_LEFT+(unsigned)!mirror);
    road();frames(4U);
    acute(RING_CORNER_B,105U,(uint8_t)!mirror);frames(3U);
    CHECK(ring.corner_stage==4U && ring.corner_guide_id==RING_GUIDE_B_MAIN_LEFT+(unsigned)!mirror);
    CHECK(abs(selected.error)<90); /* Same B shape now follows main boundary. */
    const ring_corner_guide_t *branch=&result.ring.corners.guide[RING_GUIDE_B_BRANCH_LEFT+(unsigned)!mirror];
    CHECK(branch->valid && abs(branch->error)>abs(selected.error)+100);
    road();frames(6U);CHECK(ring.completed_360==1U && !ring.action_locked);
}
static void test_invalid_b_corridor(uint8_t mirror)
{
    setup();acute(RING_CORNER_A,125U,mirror);frames(3U);road();frames(4U);
    /* Obstruct only the selected branch, leaving the main street white.
     * Reject it rather than substitute B4's easy but wrong straight path. */
    acute(RING_CORNER_B,105U,mirror);
    for(unsigned y=135U;y<H;++y) for(unsigned x=0U;x<W*29U/100U;++x)
        pixels[y*W+(mirror ? W-1U-x : x)]=0U;
    frames(2U);
    CHECK(result.ring.corners.count[mirror]>0U);
    CHECK(!result.ring.corners.guide[RING_GUIDE_B_BRANCH_LEFT+mirror].valid);
    /* The main street remains white, but an invalid branch must never advance
     * state 2 or select the state-4 main-edge policy as a replacement. */
    CHECK(pixels[150U*W+W/2U]!=0U);
    CHECK(ring.corner_stage==1U && ring.corner_guide_id==RING_GUIDE_A_LEFT+mirror);
    frame_expect(1U);
    CHECK(ring.corner_stage==1U && !selected.valid && !output.running && output.target_speed==0.f);
}
int main(void)
{
    test_270();test_360(0U);test_360(1U);test_invalid_b_corridor(0U);test_invalid_b_corridor(1U);
    puts("PASS binary->native vision->corner geometry->ring FSM->ordinary PD: 270 entry/exit, 360 four events, mirrors, steering signs, same-frame guides, immutable input");
    return 0;
}
