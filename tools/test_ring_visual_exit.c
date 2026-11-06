#include "ring_control.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static ring_control_state_t state;
static track_ring_observation_t obs;
static unsigned frame;
static void setup(unsigned inner,unsigned stage)
{
    ring_control_init(&state);memset(&obs,0,sizeof(obs));frame=0;
    state.action_locked=1;state.kind=TRACK_RING_360;state.direction=inner ? -1 : 1;
    state.corner_stage=(uint8_t)stage;state.phase=RING_STATE_INSIDE;
    obs.corners.rows=154;obs.corners.cols=304;
    obs.path[0]=(track_path_observation_t){300,200,1,90};
}
static void step(uint32_t now)
{
    obs.corners.frame_number=++frame;
    ring_control_observe(&state,&obs,now,1);
    ring_control_select_path(&state,&obs);
}
static void guide(unsigned stage,unsigned inner,int error)
{
    ring_corner_guide_t *g=&obs.corners.guide[ring_pdf_guide(stage,inner)];
    g->valid=1;g->confidence=90;g->error=(int16_t)error;g->heading=80;
    g->boundary[stage==7 ? inner : 1U-inner].valid=1;
}
int main(void)
{
    for(unsigned inner=0;inner<2;++inner) {
        unsigned outer=1U-inner;
        setup(inner,5);guide(5,inner,-200);guide(6,inner,-200);
        obs.corners.paired=1;obs.corners.continuous[outer]=1;
        obs.corners.bridge_tip[outer].quality=90;obs.corners.bridge_tip[outer].y=55;
        obs.corners.exit_white_band=1; /* Full white must NOT advance a visible third mouth. */
        guide(7,inner,150);
        for(unsigned i=1;i<=60;++i) {
            step(i*20);assert(state.corner_stage==5 && state.corner_guide_id==ring_pdf_guide(5,inner));
        }
        memset(&obs.corners.guide[ring_pdf_guide(5,inner)],0,sizeof(ring_corner_guide_t));
        memset(&obs.corners.guide[ring_pdf_guide(6,inner)],0,sizeof(ring_corner_guide_t));
        obs.corners.exit_white_band=0;
        step(1220);assert(state.corner_stage==5);
        ring_control_observe(&state,&obs,1221,1);assert(state.bridge_clear_frames==1); /* duplicate frame */
        step(1240);assert(state.corner_stage==5);
        step(1260);assert(state.corner_stage==6 && state.exit_armed && state.corner_guide_id==ring_pdf_guide(7,inner));
        step(1280);step(1300);step(1320);
        assert(state.corner_stage==7 && state.line_active && state.selected_path.error==150);
        /* Real angle is diagnostic only; it must never replace the measured route with zero. */
        obs.corners.border_angle_valid=(uint8_t)(1U<<outer);
        obs.corners.border_angle10[outer]=(int16_t)(outer ? 600 : 1200);
        for(unsigned i=0;i<30;++i) {step(1340+i*20);assert(state.corner_stage==7 && state.selected_path.error==150);}
        assert(ring_control_path_source(&state)==RING_SOURCE_GUIDE);
        /* Returning old third geometry can never re-select it after final-exit commitment. */
        guide(5,inner,-800);guide(6,inner,-800);step(2000);
        assert(state.corner_stage==7 && state.selected_path.error==150);
        memset(&obs.corners.guide[ring_pdf_guide(7,inner)],0,sizeof(ring_corner_guide_t));
        step(2020);assert(state.corner_guide_held && state.selected_path.error==150);
        step(2120);assert(state.corner_guide_held);
        step(2121);assert(!state.selected_path.valid && !state.corner_guide_held);
        /* Reacquired road settles visually before completion and cannot instantly re-enter. */
        memset(&obs.corners,0,sizeof(obs.corners));obs.corners.road_clear=1;
        obs.path[0]=(track_path_observation_t){0,0,1,100};
        for(unsigned i=0;i<10;++i) {step(2140+i*20);assert(state.corner_stage==7);}
        step(2340);assert(state.phase==RING_STATE_COOLDOWN && state.completed_360==1);
        step(2920);assert(state.phase==RING_STATE_COOLDOWN);
        for(unsigned i=0;i<10;++i) step(2940+i*20);
        assert(state.phase==RING_STATE_IDLE);
        /* A final-looking frame and a white band cannot skip an unseen third mouth. */
        setup(inner,5);guide(7,inner,150);obs.corners.continuous[outer]=1;obs.corners.exit_white_band=1;
        for(unsigned i=0;i<30;++i) step(1000+i*20);
        assert(state.corner_stage==5 && !state.exit_armed);
    }
    puts("PASS visual-only exit: mirrored third-mouth retention, ordered/new-frame transitions, white band/angle non-trigger, current guide errors preserved, no regression, 120ms hold, visual completion/rearm");
    return 0;
}
