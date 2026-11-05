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
    obs.path[0]=(track_path_observation_t){500,400,1,100};
}
static void guide(unsigned stage,unsigned side,int error)
{
    ring_corner_guide_t *g=&obs.corners.guide[ring_pdf_guide(stage,side)];
    g->valid=1;g->confidence=95;g->error=(int16_t)error;g->heading=-30;
    g->boundary[1-side].valid=1;
}
int main(void)
{
    for(unsigned side=0;side<2;++side) {
        reset(side);guide(3,side,-200);
        assert(step(100).error==-200 && ring_control_path_source(&state)==RING_SOURCE_GUIDE);
        memset(obs.corners.guide,0,sizeof(obs.corners.guide));
        assert(step(120).error==-200 && state.corner_guide_held);
        assert(state.corner_guide_id==ring_pdf_guide(3,side));
        assert(state.corner_guide_frame_number==1 && state.corner_last_guide_ms==100);
        assert(step(220).valid && ring_control_path_source(&state)==RING_SOURCE_HOLD);
        assert(!step(221).valid && ring_control_path_source(&state)==RING_SOURCE_INVALID);
        assert(state.corner_stage==3 && !state.corner_guide_id && !state.line_active);
        assert(state.path_mode!=TRACK_PATH_NORMAL); /* no near/bottom branch override */
        guide(3,side,-180);assert(step(240).error==-180 && !state.corner_guide_held);
        memset(obs.corners.guide,0,sizeof(obs.corners.guide));guide(4,side,-150);
        obs.corners.v[1-side].quality=90;obs.corners.v[1-side].y=70;
        assert(step(260).error==-150 && state.corner_stage==3);
        assert(step(280).error==-150 && state.corner_stage==3);
        assert(step(300).error==-150 && state.corner_stage==4);
        assert(state.corner_guide_id==ring_pdf_guide(4,side) && !state.corner_guide_held);
        /* S4 and 270 retain their existing ordinary-path behaviour. */
        memset(obs.corners.guide,0,sizeof(obs.corners.guide));
        assert(step(320).error==500 && ring_control_path_source(&state)==RING_SOURCE_NORMAL);
        reset(side);state.kind=TRACK_RING_270;assert(step(100).error==500);
    }
    reset(0);state.corner_have_guide=1;state.corner_guide_id=ring_pdf_guide(2,0);
    state.selected_path=(track_path_observation_t){-700,-700,1,100};
    assert(!step(20).valid); /* cannot inherit S2 */
    reset(0);guide(3,0,-200);assert(step(UINT32_MAX-59U).valid);
    memset(obs.corners.guide,0,sizeof(obs.corners.guide));
    assert(step(40).valid && state.corner_guide_held);
    assert(!step(61).valid); /* 121ms across uint32 wrap */
    reset(0);guide(3,0,-200);step(100);state.fault=1;
    assert(!step(120).valid && !state.corner_have_guide);
    ring_control_init(&state);assert(!state.corner_guide_held && !state.corner_have_guide);
    assert(!strcmp(ring_control_stage_name(3),"INLOCK"));
    assert(!strcmp(ring_control_stage_name(5),"RING"));
    assert(!strcmp(ring_control_stage_name(6),"EXITWAIT"));
    puts("PASS S3 mirrored hold, non-refreshing deadline, recovery, S4 transition, no S2 leakage, wrap, reset and source labels");
    return 0;
}
