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
    ring_control_select_path(&state,&observation);
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
    for(unsigned i=0;i<RING_CONFIRM_FRAMES;++i) {
        fresh(begin+i*20U);
        if(i+1U<RING_CONFIRM_FRAMES) assert(state.action_locked && state.corner_stage==7);
    }
    assert(state.phase==RING_STATE_COOLDOWN && state.completed_360==1 && !state.action_locked);
    assert(!state.transition_frames && !state.vote_frames && !state.clear_frames);
    return begin+(RING_CONFIRM_FRAMES-1U)*20U;
}
static void release_without_alignment(void)
{
    start(TRACK_RING_360);
    observation.corners.road_clear=0;
    observation.path[0]=(track_path_observation_t){520,-280,1,60};
    observation.handoff_bottom_error=810;
    fresh(1000);
    ring_control_observe(&state,&observation,1001,1);
    ring_control_select_path(&state,&observation);
    assert(state.tip_missing_frames==1 && state.action_locked);
    fresh(1020);fresh(1040);
    assert(state.completed_360==1 && !ring_control_guidance_active(&state));
    assert(state.path_mode==TRACK_PATH_NORMAL && state.selected_path.error==520);
    assert(state.selected_path.heading==-280 && state.selected_path.confidence==60);
    for(unsigned i=0;i<40;++i) {
        fresh(1060+i*20U);
        assert(!state.action_locked && state.selected_path.error==520 && state.completed_360==1);
    }
    puts("PASS tip loss releases directly to ordinary tracking, independent of road alignment");
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

    done=finish360(UINT32_MAX-20U);
    assert(done==19U);
    for(unsigned i=1;i<=30;++i) fresh(done+i*20U);
    assert(state.phase==RING_STATE_IDLE && state.completed_360==1);
    done=finish360(UINT32_MAX-700U);
    for(unsigned i=1;i<=30;++i) fresh(done+i*20U);
    assert(state.phase==RING_STATE_IDLE && state.completed_360==1);
    ring_control_init(&state);
    assert(!state.action_locked && state.phase==RING_STATE_IDLE && !state.completed_360 && !state.clear_frames && !state.tip_missing_frames);
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
    release_without_alignment();
    rearm_tests();
    original270_tests();
    return 0;
}
