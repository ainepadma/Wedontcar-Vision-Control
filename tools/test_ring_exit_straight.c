#include "ring_control.h"
#include "track_control.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static ring_control_state_t ring;
static track_ring_observation_t obs;
static unsigned frame;
static void setup(unsigned inner,uint32_t now)
{
    ring_control_init(&ring);memset(&obs,0,sizeof(obs));frame=0;
    ring.action_locked=1;ring.kind=TRACK_RING_360;ring.direction=inner ? -1 : 1;
    ring.corner_stage=7;ring.phase=RING_STATE_EXITING;ring.exit_armed=1;
    ring.event_started_ms=now;
    obs.corners.rows=154;obs.corners.cols=304;
    obs.corners.border_angle_valid=(uint8_t)(1U<<(1U-inner));
    obs.corners.border_angle10[1U-inner]=(int16_t)(inner ? 1800-RING_EXIT_BORDER_TARGET10 : RING_EXIT_BORDER_TARGET10);
    obs.path[0]=(track_path_observation_t){250,100,1,90};
}
static void fresh(uint32_t now)
{
    obs.corners.frame_number=++frame;
    ring_control_observe(&ring,&obs,now,1);
    ring_control_select_path(&ring,&obs);
}
static uint32_t trigger(unsigned inner,uint32_t start)
{
    setup(inner,start);
    fresh(start);assert(!ring.exit_straight_active && ring.exit_angle_frames==1);
    ring_control_observe(&ring,&obs,start+1U,1);
    assert(!ring.exit_straight_active && ring.exit_angle_frames==1);
    fresh(start+20U);assert(!ring.exit_straight_active && ring.exit_angle_frames==2);
    fresh(start+40U);assert(ring_control_straight_active(&ring,start+40U));
    assert(ring.selected_path.valid && ring.selected_path.error==0 && ring.selected_path.heading==0 && ring.selected_path.confidence==100);
    assert(ring.path_mode==TRACK_PATH_STRAIGHT && !ring.line_active && !ring.corner_guide_id);
    assert(ring_control_path_source(&ring)==RING_SOURCE_STRAIGHT);
    assert(ring.exit_straight_started_ms==start+40U && !ring.exit_straight_done);
    return start+40U;
}
static void angle_gate_tests(void)
{
    for(unsigned inner=0;inner<2;++inner) {
        trigger(inner,1000);
        for(unsigned s=0;s<7;++s) {
            setup(inner,1000);ring.corner_stage=(uint8_t)s;
            fresh(1000);fresh(1020);fresh(1040);
            assert(!ring.exit_straight_active && !ring.exit_angle_frames);
        }
        for(unsigned s=1;s<=3;++s) {
            setup(inner,1000);ring.kind=TRACK_RING_270;ring.corner_stage=(uint8_t)s;
            fresh(1000);fresh(1020);fresh(1040);
            assert(!ring.exit_straight_active && !ring.exit_straight_done);
        }
        setup(inner,1000);ring.exit_armed=0;
        fresh(1000);fresh(1020);fresh(1040);assert(!ring.exit_straight_active);
        setup(inner,1000);obs.corners.border_angle_valid=(uint8_t)(1U<<inner);
        obs.corners.border_angle10[inner]=obs.corners.border_angle10[1U-inner];
        fresh(1000);fresh(1020);fresh(1040);assert(!ring.exit_straight_active);
        setup(inner,1000);obs.corners.border_angle10[1U-inner]=(int16_t)(inner ? 600 : 1200);
        fresh(1000);fresh(1020);fresh(1040);assert(!ring.exit_straight_active); /* wrong slope */
        setup(inner,1000);obs.corners.border_angle10[1U-inner]=-600;
        fresh(1000);fresh(1020);fresh(1040);assert(!ring.exit_straight_active);
        for(int d=-1;d<=1;d+=2) {
            int angle=RING_EXIT_BORDER_TARGET10+d*RING_EXIT_BORDER_TOLERANCE10;
            setup(inner,1000);obs.corners.border_angle10[1U-inner]=(int16_t)(inner ? 1800-angle : angle);
            fresh(1000);fresh(1020);fresh(1040);assert(ring.exit_straight_active);
            angle+=d;
            setup(inner,1000);obs.corners.border_angle10[1U-inner]=(int16_t)(inner ? 1800-angle : angle);
            fresh(1000);fresh(1020);fresh(1040);assert(!ring.exit_straight_active);
        }
        setup(inner,1000);fresh(1000);fresh(1020);
        obs.corners.border_angle_valid=0;fresh(1040);assert(!ring.exit_angle_frames);
        obs.corners.border_angle_valid=(uint8_t)(1U<<(1U-inner));fresh(1060);fresh(1080);
        assert(!ring.exit_straight_active);fresh(1100);assert(ring.exit_straight_active);
        setup(inner,1000);fresh(1000);fresh(1020);fresh(1171);
        assert(ring.exit_angle_frames==1 && !ring.exit_straight_active); /* new sequence after frame gap */
    }
    puts("PASS mirrored real-outer angle, S7/armed-only, 3 distinct frames, inclusive thresholds, sign/side/dropout rejection, 270 unchanged");
}
static void lifetime_tests(void)
{
    for(unsigned inner=0;inner<2;++inner) {
        uint32_t start=trigger(inner,1000);
        for(unsigned dt=20;dt<300;dt+=20) {
            fresh(start+dt);
            assert(ring_control_straight_active(&ring,start+dt) && ring.exit_straight_started_ms==start);
        }
        assert(ring_control_straight_active(&ring,start+299));
        assert(!ring_control_straight_active(&ring,start+300));
        ring_control_tick(&ring,1,0,0,0,start+300);
        assert(!ring.exit_straight_active && ring.exit_straight_done && ring.control_refresh);
        for(unsigned dt=320;dt<520;dt+=20) fresh(start+dt);
        assert(!ring.exit_straight_active && ring.exit_straight_done); /* cannot refresh/retrigger */
        obs.corners.road_clear=1;obs.path[0]=(track_path_observation_t){0,0,1,100};
        for(unsigned dt=520;dt<=720;dt+=20) fresh(start+dt);
        assert(ring.phase==RING_STATE_COOLDOWN && ring.completed_360==1);
        for(unsigned dt=740;dt<=1320;dt+=20) fresh(start+dt);
        assert(ring.phase==RING_STATE_IDLE);
        obs.corners.entry[inner]=1;
        fresh(start+1340);fresh(start+1360);fresh(start+1380);
        assert(ring.corner_stage==1 && ring.action_locked && !ring.exit_straight_done && !ring.exit_straight_active);
    }
    uint32_t start=trigger(0,UINT32_MAX-100U);
    for(unsigned dt=20;dt<300;dt+=20) fresh(start+dt);
    assert(ring_control_straight_active(&ring,start+299));
    fresh(start+300);assert(!ring.exit_straight_active && ring.exit_straight_done);
    start=trigger(0,1000);
    ring_control_tick(&ring,0,0,0,0,start+1);
    assert(!ring_control_straight_active(&ring,start+1) && !ring.action_locked);
    assert(!ring_control_straight_active(0,0));
    puts("PASS one-shot fixed 300ms, no deadline refresh, next-ring reset, uint32 wrap, manual stop");
}
static void stale_tests(void)
{
    for(unsigned protect=0;protect<=1;++protect) {
        uint32_t start=trigger(0,1000);ring_control_set_protection(&ring,(uint8_t)protect);
        assert(ring_control_straight_active(&ring,start+150));
        ring_control_tick(&ring,1,0,0,0,start+150);
        assert(ring.exit_straight_active && !ring.fault);
        assert(!ring_control_straight_active(&ring,start+151));
        ring_control_tick(&ring,1,0,0,0,start+151);
        assert(!ring.exit_straight_active && ring.exit_straight_done);
        assert(ring.fault==protect);
        if(protect) assert(ring.phase==RING_STATE_FAULT);
        else {fresh(start+160);fresh(start+180);fresh(start+200);assert(!ring.exit_straight_active);}
        start=trigger(1,1000);ring_control_set_protection(&ring,(uint8_t)protect);
        fresh(start+151);assert(!ring.exit_straight_active && ring.exit_straight_done);
    }
    puts("PASS 150/151ms freshness boundary cancels straight with protection on/off and new-frame resume");
}
static void visible_road_tests(void)
{
    for(unsigned inner=0;inner<2;++inner) {
        setup(inner,1000);obs.path[0].valid=0;
        fresh(1000);fresh(1020);fresh(1040);
        assert(!ring.exit_straight_active && !ring.exit_angle_frames && !ring.selected_path.valid);
        setup(inner,1000);obs.path[0].confidence=44;
        fresh(1000);fresh(1020);fresh(1040);assert(!ring.exit_straight_active);
        obs.path[0].confidence=45;
        fresh(1060);fresh(1080);fresh(1100);assert(ring.exit_straight_active);

        setup(inner,1000);obs.path[0].valid=0;
        ring_corner_guide_t *g=&obs.corners.guide[ring_pdf_guide(7,inner)];
        g->valid=1;g->confidence=44;g->error=-220;g->heading=-250;g->boundary[1U-inner].valid=1;
        fresh(1000);fresh(1020);fresh(1040);assert(!ring.exit_straight_active);
        g->confidence=45;
        fresh(1060);fresh(1080);fresh(1100);assert(ring.exit_straight_active);
        fresh(1120);assert(ring_control_straight_active(&ring,1120)); /* valid guide alone is enough */

        setup(inner,1000);obs.path[0].valid=0;
        g=&obs.corners.guide[ring_pdf_guide(7,1U-inner)];g->valid=1;g->confidence=90;
        fresh(1000);fresh(1020);fresh(1040);assert(!ring.exit_straight_active); /* wrong-side guide */
        memset(obs.corners.guide,0,sizeof(obs.corners.guide));
        g=&obs.corners.guide[ring_pdf_guide(6,inner)];g->valid=1;g->confidence=90;
        fresh(1060);fresh(1080);fresh(1100);assert(!ring.exit_straight_active); /* wrong-stage guide */

        uint32_t began=trigger(inner,1000);
        obs.path[0].valid=0; /* fresh all-black observation: no valid normal or S7 guide */
        fresh(began+20);
        assert(!ring.exit_straight_active && ring.exit_straight_done && !ring.selected_path.valid);
        assert(ring_control_path_source(&ring)==RING_SOURCE_INVALID);
        obs.path[0].valid=1;obs.path[0].confidence=90;
        fresh(began+40);fresh(began+60);fresh(began+80);
        assert(!ring.exit_straight_active && ring.exit_straight_done); /* one shot stays consumed */
        began=trigger(inner,1000);obs.path[0].confidence=44;
        fresh(began+20);assert(!ring.exit_straight_active && ring.exit_straight_done);

        /* Known handback: the timer does not forbid an existing white-band
         * G21/G22 from steering again once the bounded straight ends. */
        began=trigger(inner,1000);obs.path[0].valid=0;obs.corners.exit_white_band=1;
        g=&obs.corners.guide[ring_pdf_guide(7,inner)];
        g->valid=1;g->confidence=90;g->error=-220;g->heading=-250;g->boundary[1U-inner].valid=1;
        for(unsigned dt=20;dt<300;dt+=20) {
            fresh(began+dt);assert(ring.exit_straight_active && ring_control_path_source(&ring)==RING_SOURCE_STRAIGHT);
        }
        fresh(began+300);
        assert(!ring.exit_straight_active && ring.exit_straight_done);
        assert(ring.corner_stage==7 && ring.corner_guide_id==ring_pdf_guide(7,inner));
        assert(ring_control_path_source(&ring)==RING_SOURCE_GUIDE && ring.selected_path.error==-220);
    }
    puts("PASS real-path visibility Q44/45, fresh all-black cancellation, guide-only validity, no retrigger; expired straight may return to G21/G22");
}
static void output_tests(void)
{
    track_control_state_t control;
    const track_control_config_t cfg={87.f,100.f,74.f,120.f,90.f,.030f,.0005f,180.f};
    track_control_init(&control,&cfg);
    track_control_observe(&control,400,200,100,1,1000);
    track_control_observe(&control,400,200,100,1,1020);
    track_control_observe(&control,400,200,100,1,1040);
    track_control_toggle(&control,1040);assert(control.running);
    (void)track_control_tick(&control,1040);
    track_control_observe_near(&control,400,200,350,150,1,100,1,1100);
    control.bottom_active=1;control.bottom_start_ms=1100;control.bottom_pulse_deg=2.f;
    control.bottom_votes=2;control.bottom_center_votes=1;
    track_control_output_t output=track_control_tick(&control,1100);
    assert(output.running && output.servo_angle<87.f);
    float speed=output.target_speed;uint8_t ready=output.ready,reason=output.stop_reason;
    uint8_t state_reason=control.stop_reason;
    track_control_force_straight(&control,&output);
    assert(output.servo_angle==87.f && control.servo_angle==87.f);
    assert(output.target_speed==speed && output.ready==ready && output.stop_reason==reason && output.running);
    assert(control.running && control.stop_reason==state_reason);
    assert(control.filtered_error==0.f && control.filtered_heading==0.f && control.filtered_near_heading==0.f);
    assert(control.derivative==0.f && !control.derivative_ready && !control.near_geometry_valid);
    assert(!control.bottom_active && !control.bottom_votes && !control.bottom_center_votes);
    assert(!memcmp(&control.config,&cfg,sizeof(cfg)));

    /* After explicit release, ordinary current observations may steer again. */
    track_control_observe(&control,-300,-100,100,1,1120);
    output=track_control_tick(&control,1120);
    assert(output.running && output.servo_angle>87.f && output.servo_angle<=100.f);
    track_control_stop(&control,TRACK_STOP_MANUAL);
    float stopped_angle=control.servo_angle,old_output_angle=output.servo_angle;
    track_control_force_straight(&control,&output);
    assert(control.servo_angle==stopped_angle && output.servo_angle==old_output_angle);
    assert(control.stop_reason==TRACK_STOP_MANUAL && !control.running);
    control.running=1;output.running=0;output.servo_angle=74.f;output.stop_reason=TRACK_STOP_ENCODER;
    track_control_force_straight(&control,&output);
    assert(output.servo_angle==74.f && output.stop_reason==TRACK_STOP_ENCODER && !output.running);
    track_control_force_straight(0,&output);track_control_force_straight(&control,0);
    puts("PASS actual track tick then immediate calibrated centre, speed/stop unchanged, history cleared, release and stop guards");
}
int main(void)
{
    angle_gate_tests();lifetime_tests();stale_tests();visible_road_tests();output_tests();return 0;
}
