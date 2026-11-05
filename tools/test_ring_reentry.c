#include "ring_control.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static ring_control_state_t state;
static track_ring_observation_t observation;
static unsigned frame;
static void fresh(uint32_t now)
{
    observation.corners.frame_number=++frame;
    ring_control_observe(&state,&observation,now,1);
}
static void good_road(void)
{
    memset(&observation,0,sizeof(observation));
    observation.corners.rows=154;observation.corners.cols=304;
    observation.corners.road_clear=1;
    observation.path[0]=(track_path_observation_t){50,30,1,90};
    observation.handoff_near_valid=1;observation.handoff_near_error=55;
    observation.handoff_near_heading=25;
    observation.handoff_bottom_valid=1;observation.handoff_bottom_error=20;
}
static void start(unsigned kind)
{
    ring_control_init(&state);good_road();frame=0;
    state.kind=(uint8_t)kind;state.direction=1;state.action_locked=1;
    state.phase=RING_STATE_EXITING;state.corner_stage=kind==TRACK_RING_360 ? 7 : 3;
}
static uint32_t finish360(uint32_t begin)
{
    start(TRACK_RING_360);
    for(unsigned i=0;i<=10;++i) {
        fresh(begin+i*20U);
        if(i<10) assert(state.action_locked && state.corner_stage==7 && state.completed_360==0);
    }
    assert(state.phase==RING_STATE_COOLDOWN && state.completed_360==1 && !state.action_locked);
    assert(state.transition_frames==0 && state.vote_frames==0 && state.clear_frames==0 && state.exit_road_frames==0);
    return begin+200U;
}
static void settle_tests(void)
{
    start(TRACK_RING_360);
    for(unsigned i=0;i<8;++i) fresh(1000U+i);
    assert(state.exit_road_frames==8 && state.corner_stage==7); /* frames alone insufficient */
    ring_control_observe(&state,&observation,1100,1);
    assert(state.exit_road_frames==8 && state.last_frame_ms==1007); /* duplicate frame */
    fresh(1100);fresh(1199);assert(state.corner_stage==7);
    fresh(1200);assert(state.phase==RING_STATE_COOLDOWN && state.completed_360==1);

    start(TRACK_RING_360);
    for(unsigned i=0;i<7;++i) fresh(1000U+i*40U);
    assert(state.corner_stage==7 && state.exit_road_frames==7); /* elapsed alone insufficient */
    fresh(1280);assert(state.phase==RING_STATE_COOLDOWN && state.completed_360==1);

    start(TRACK_RING_360);fresh(1000);fresh(1020);fresh(1040);
    observation.corners.road_clear=0;fresh(1060);
    assert(state.exit_road_frames==0 && state.transition_frames==0);
    good_road();fresh(1080);assert(state.exit_road_frames==1 && state.exit_road_started_ms==1080);
    fresh(1231);assert(state.exit_road_frames==1 && state.exit_road_started_ms==1231); /* >150ms */
    fresh(1381);assert(state.exit_road_frames==2 && state.exit_road_started_ms==1231); /* exactly150ms */
    assert(state.corner_stage==7);
    puts("PASS S7 8-frame AND 200ms gate, duplicates, interruption, 150ms gap boundaries");
}
static void make_bad(unsigned id)
{
    switch(id) {
    case 0: observation.corners.road_clear=0;break;
    case 1: observation.corners.exit_white_band=1;break;
    case 2: observation.corners.entry[0]=1;break;
    case 3: observation.corners.entry[1]=1;break;
    case 4: observation.corners.cross_entry=1;break;
    case 5: observation.path[0].valid=0;break;
    case 6: observation.path[0].confidence=64;break;
    case 7: observation.path[0].error=181;break;
    case 8: observation.path[0].error=-181;break;
    case 9: observation.path[0].heading=121;break;
    case 10: observation.path[0].heading=-121;break;
    case 11: observation.handoff_near_error=181;break;
    case 12: observation.handoff_near_error=-181;break;
    case 13: observation.handoff_near_heading=121;break;
    case 14: observation.handoff_near_heading=-121;break;
    case 15: observation.path[0].error=41;observation.handoff_near_error=-41;break;
    case 16: observation.path[0].error=-41;observation.handoff_near_error=41;break;
    case 17: observation.handoff_bottom_error=81;break;
    case 18: observation.handoff_bottom_error=-81;break;
    default: assert(0);break;
    }
}
static void road_gate_tests(void)
{
    for(unsigned i=0;i<19;++i) {
        start(TRACK_RING_360);make_bad(i);
        for(unsigned j=0;j<16;++j) fresh(1000+j*20U);
        assert(state.corner_stage==7 && state.exit_road_frames==0 && state.completed_360==0);
        assert(!ring_control_select_path(&state,&observation).valid);
    }
    for(int sign=-1;sign<=1;sign+=2) {
        start(TRACK_RING_360);observation.path[0].confidence=65;
        observation.path[0].error=(int16_t)(sign*180);observation.path[0].heading=(int16_t)(sign*120);
        observation.handoff_near_error=(int16_t)(sign*180);observation.handoff_near_heading=(int16_t)(sign*120);
        observation.handoff_bottom_error=(int16_t)(sign*80);fresh(1000);
        assert(state.exit_road_frames==1);
    }
    start(TRACK_RING_360);observation.path[0].error=40;observation.handoff_near_error=-180;fresh(1000);
    assert(state.exit_road_frames==1); /* one E at deadband boundary */
    start(TRACK_RING_360);observation.path[0].error=180;observation.handoff_near_error=-40;fresh(1000);
    assert(state.exit_road_frames==1);
    start(TRACK_RING_360);observation.handoff_near_valid=0;observation.handoff_bottom_valid=0;
    observation.handoff_near_error=-1000;observation.handoff_near_heading=1000;observation.handoff_bottom_error=1000;
    fresh(1000);assert(state.exit_road_frames==1); /* unavailable auxiliary observations ignored */
    puts("PASS exit-road gates, near/global sign conflict, bottom error, inclusive thresholds and optional observations");
}
static void path_handoff_tests(void)
{
    start(TRACK_RING_360);fresh(1000);
    track_path_observation_t p=ring_control_select_path(&state,&observation);
    assert(p.valid && p.error==50 && p.heading==30 && state.corner_stage==7);
    assert(state.path_mode==TRACK_PATH_STRAIGHT && !state.line_active && !state.corner_guide_id);
    assert(ring_control_path_source(&state)==RING_SOURCE_NORMAL);
    observation.path[0].error=300;fresh(1020);
    assert(!ring_control_select_path(&state,&observation).valid); /* cannot hold the accepted normal path as guide */
    good_road();observation.corners.exit_white_band=1;
    ring_corner_guide_t *g=&observation.corners.guide[ring_pdf_guide(7,0)];
    g->valid=1;g->confidence=95;g->error=-220;g->heading=-250;g->boundary[1].valid=1;
    fresh(1040);p=ring_control_select_path(&state,&observation);
    assert(p.valid && p.error==-220 && ring_control_path_source(&state)==RING_SOURCE_GUIDE);
    memset(observation.corners.guide,0,sizeof(observation.corners.guide));
    fresh(1160);assert(ring_control_select_path(&state,&observation).valid);
    assert(ring_control_path_source(&state)==RING_SOURCE_HOLD);
    fresh(1161);assert(!ring_control_select_path(&state,&observation).valid);
    assert(ring_control_path_source(&state)==RING_SOURCE_INVALID && state.corner_stage==7);
    puts("PASS aligned normal path under S7 ownership, no near/bottom override, G/H/U retained for unready road");
}
static void rearm_tests(void)
{
    uint32_t done=finish360(1000);
    for(unsigned i=1;i<30;++i) {fresh(done+i*20U);assert(state.phase==RING_STATE_COOLDOWN);}
    fresh(done+600);assert(state.phase==RING_STATE_IDLE && state.vote_frames==0 && state.completed_360==1);
    observation.corners.entry[0]=1;
    fresh(done+620);assert(!state.action_locked && state.vote_frames==1);
    ring_control_observe(&state,&observation,done+621,1);assert(state.vote_frames==1);
    fresh(done+640);assert(!state.action_locked && state.vote_frames==2);
    fresh(done+660);assert(state.action_locked && state.corner_stage==1 && state.kind==TRACK_RING_360);
    assert(state.completed_360==1); /* next legitimate lap allowed */

    done=finish360(1000);
    for(unsigned i=1;i<=9;++i) fresh(done+i*80U);
    assert(state.phase==RING_STATE_COOLDOWN && state.clear_frames==9); /* elapsed600 alone insufficient */
    fresh(done+800);assert(state.phase==RING_STATE_IDLE && state.completed_360==1);

    done=finish360(1000);fresh(done+20);fresh(done+40);
    fresh(done+191);assert(state.clear_frames==1); /* >150 resets cooldown */
    observation.corners.entry[0]=1;fresh(done+211);
    assert(state.clear_frames==0 && state.vote_frames==0 && state.phase==RING_STATE_COOLDOWN);
    good_road();fresh(done+231);assert(state.clear_frames==1);
    observation.handoff_bottom_error=81;fresh(done+251);assert(state.clear_frames==0);
    assert(state.completed_360==1);

    done=finish360(UINT32_MAX-100U);
    assert(done==99U);
    for(unsigned i=1;i<=30;++i) fresh(done+i*20U);
    assert(state.phase==RING_STATE_IDLE && state.completed_360==1);
    done=finish360(UINT32_MAX-700U);
    for(unsigned i=1;i<=30;++i) fresh(done+i*20U);
    assert(state.phase==RING_STATE_IDLE && state.completed_360==1);
    ring_control_init(&state);
    assert(!state.action_locked && state.phase==RING_STATE_IDLE && !state.completed_360 && !state.clear_frames && !state.exit_road_frames);
    puts("PASS 600ms AND 10-frame rearm, old-exit rejection, next lap, gaps, uint32 wrap and manual reset");
}
static void original270_tests(void)
{
    start(TRACK_RING_270);
    /* New 360-only confidence/heading restrictions must not alter 270. */
    observation.path[0].valid=0;observation.handoff_bottom_error=900;
    fresh(1000);assert(state.transition_frames==1 && state.action_locked);
    fresh(1020);assert(state.transition_frames==2 && state.action_locked);
    fresh(1040);assert(state.phase==RING_STATE_COOLDOWN && state.completed_270==1);
    assert(!state.transition_frames && !state.vote_frames && !state.clear_frames);
    ring_control_observe(&state,&observation,1041,1);assert(!state.transition_frames);
    fresh(1060);assert(state.phase==RING_STATE_COOLDOWN && state.transition_frames==1);
    fresh(1080);assert(state.phase==RING_STATE_COOLDOWN && state.transition_frames==2);
    observation.corners.road_clear=0;fresh(1100);assert(!state.transition_frames);
    observation.corners.road_clear=1;
    fresh(1120);fresh(1140);assert(state.phase==RING_STATE_COOLDOWN);
    fresh(1160);assert(state.phase==RING_STATE_IDLE && state.completed_270==1 && !state.completed_360);
    for(unsigned i=0;i<12;++i) fresh(1180+i*20U);
    assert(state.completed_270==1);
    puts("PASS 270 original 3-frame exit/cooldown, cleared votes and single completion count");
}
int main(void)
{
    settle_tests();road_gate_tests();path_handoff_tests();rearm_tests();original270_tests();
    return 0;
}
