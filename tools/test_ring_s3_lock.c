#include "ring_control.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static ring_control_state_t state;
static track_ring_observation_t obs;
static unsigned frame;

static track_path_observation_t step(uint32_t now)
{
    obs.corners.frame_number=++frame;
    ring_control_observe(&state,&obs,now,1);
    return ring_control_select_path(&state,&obs);
}
static void reset(unsigned side)
{
    ring_control_init(&state);memset(&obs,0,sizeof(obs));frame=0;
    state.action_locked=1;state.kind=TRACK_RING_360;state.direction=side ? -1 : 1;
    state.corner_stage=3;state.phase=RING_STATE_ENTERING;
    obs.corners.rows=154;obs.corners.cols=304;
    obs.path[TRACK_PATH_NORMAL]=(track_path_observation_t){500,400,1,100};
}
static void guide(unsigned stage,unsigned side,int error)
{
    ring_corner_guide_t *g=&obs.corners.guide[ring_pdf_guide(stage,side)];
    g->valid=1;g->confidence=95;g->error=(int16_t)error;g->heading=-30;
    g->boundary[1U-side].valid=1;
}
int main(void)
{
    for(unsigned side=0;side<2;++side) {
        reset(side);guide(3,side,-200);
        obs.corners.v[side].quality=90;
        assert(step(100).error==-200 && ring_control_path_source(&state)==RING_SOURCE_GUIDE);
        memset(obs.corners.guide,0,sizeof(obs.corners.guide));
        obs.corners.v[side].quality=0;
        assert(step(120).error==-200 && state.corner_guide_held);
        assert(state.corner_guide_frame_number==1 && state.corner_last_guide_ms==100);

        guide(4,side,-150);
        obs.corners.v[1U-side].quality=90;obs.corners.v[1U-side].y=70;
        assert(step(140).error==-150 && state.corner_stage==3);
        assert(step(160).error==-150 && state.corner_stage==3);
        assert(step(180).error==-150 && state.corner_stage==4);

        obs.corners.v[1U-side].quality=0;
        obs.corners.v[side].quality=90; /* A stale inner V cannot restart the second line. */
        guide(3,side,-900);
        assert(step(200).error==-150 && state.corner_guide_held);
        assert(step(220).error==-150 && state.corner_guide_held);
        assert(step(240).error==500 && ring_control_path_source(&state)==RING_SOURCE_NORMAL);
        assert(state.corner_stage==5 && !state.corner_guide_id && !state.line_active);
        obs.corners.v[1U-side].quality=90;
        assert(step(260).error==500 && state.corner_stage==5);
        guide(5,side,-100);
        assert(step(280).error==-100 && ring_control_path_source(&state)==RING_SOURCE_GUIDE);

        reset(side);guide(3,side,-200);
        assert(step(20).error==-200);
        memset(obs.corners.guide,0,sizeof(obs.corners.guide));
        step(40);
        assert(step(60).error==500 && state.corner_stage==5); /* Opposite V was never visible. */
    }
    reset(0);state.corner_have_guide=1;state.corner_guide_id=ring_pdf_guide(2,0);
    state.selected_path=(track_path_observation_t){-700,-700,1,100};
    assert(step(20).error==500); /* No S2 inheritance. */

    reset(0);guide(3,0,-200);obs.corners.v[0].quality=90;
    assert(step(UINT32_MAX-59U).error==-200);
    memset(obs.corners.guide,0,sizeof(obs.corners.guide));
    assert(step(40).error==-200 && state.corner_guide_held);
    assert(step(61).error==500 && ring_control_path_source(&state)==RING_SOURCE_NORMAL);
    puts("PASS mirrored S3/S4 dropout, passage, ordinary tracking, third repair and timer wrap");
    return 0;
}
