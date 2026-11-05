#include "ring_control.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static uint8_t image[480U*752U],saved[480U*752U];
static ring_corner_frame_t frame;
static track_ring_observation_t obs;
static ring_control_state_t state;
static unsigned sequence;
static void detect(unsigned h,unsigned w)
{
    memcpy(saved,image,h*w);
    ring_corner_process(image,(uint16_t)h,(uint16_t)w,0,&frame);
    assert(!memcmp(saved,image,h*w));
}
static void band(unsigned w,unsigned start,unsigned count)
{for(unsigned y=start;y<start+count;++y) memset(image+y*w,255,w);}
static void detection_tests(void)
{
    const unsigned sizes[][2]={{154,304},{240,376},{480,752},{20,32}};
    for(unsigned i=0;i<4;++i) {
        unsigned h=sizes[i][0],w=sizes[i][1],n=(h*RING_EXIT_WHITE_BAND_PERCENT+99U)/100U;
        memset(image,0,h*w);band(w,2,n-1);detect(h,w);assert(!frame.exit_white_band);
        band(w,2,n);detect(h,w);
        assert(frame.exit_white_band && frame.exit_white_top==2 && frame.exit_white_bottom==n+1);
        /* Even the first/last pixel is part of a row. A black pixel breaks it. */
        image[(n+1)*w]=0;detect(h,w);assert(!frame.exit_white_band);
        image[(n+1)*w]=255;image[(n+2)*w-1]=0;detect(h,w);assert(!frame.exit_white_band);
        memset(image,0,h*w);band(w,h-n,n);detect(h,w);
        assert(frame.exit_white_band && frame.exit_white_bottom==h-1);
        memset(image,0,h*w);band(w,0,n);detect(h,w);assert(frame.exit_white_top==0);
        memset(image,0,h*w);band(w,1,n);band(w,h-n,n);detect(h,w);
        assert(frame.exit_white_top==h-n); /* nearest qualifying band */
    }
    memset(image,0,154*304);for(unsigned y=0;y<154;y+=2) band(304,y,1);
    detect(154,304);assert(!frame.exit_white_band);
    /* A nearly full-width road is not an all-white row. */
    memset(image,255,154*304);for(unsigned y=0;y<154;++y) image[y*304+150]=0;
    detect(154,304);assert(!frame.exit_white_band);
    puts("PASS white band: full width, thickness, disjoint rows, edges, sizes, immutable input");
}
static void lane(void)
{
    memset(image,0,154*304);
    for(unsigned y=0;y<154;++y) memset(image+y*304+90,255,124);
}
static void observe(uint32_t now)
{
    obs.corners.frame_number=++sequence;
    ring_control_observe(&state,&obs,now,1);
    ring_control_select_path(&state,&obs);
}
static void geometry_and_state_tests(void)
{
    for(unsigned inner=0;inner<2;++inner) {
        lane();band(304,60,40);detect(154,304);
        unsigned side=1U-inner;
        for(unsigned s=6;s<=7;++s) {
            const ring_corner_guide_t *g=&frame.guide[ring_pdf_guide(s,inner)];
            const ring_corner_line_t *l=&g->boundary[side];
            assert(g->valid && l->valid && !g->boundary[inner].valid);
            assert(fabsf(fabsf(l->x1-l->x0)-fabsf(l->y1-l->y0)*tanf(RING_EXIT_LINE_ANGLE_DEG*.01745329252f))<.01f);
            assert(l->y1<l->y0 && (side ? l->x1<l->x0 : l->x1>l->x0));
            assert(l->y0==frame.a[side].y && l->x0==frame.a[side].x);
            assert(g->error*(inner ? -1 : 1)<0);
        }
        /* Stabilization may recover V but must not rewrite the new S7 line. */
        ring_corner_tracker_t tracker;memset(&tracker,0,sizeof(tracker));
        ring_corner_guide_t original=frame.guide[ring_pdf_guide(7,inner)];
        tracker.rows=154;tracker.cols=304;tracker.v_valid[inner]=1;
        tracker.v[inner].x=100;tracker.v[inner].y=70;
        ring_corner_stabilize(image,0,&tracker,&frame);
        assert(!memcmp(&original,&frame.guide[ring_pdf_guide(7,inner)],sizeof(original)));
        ring_control_init(&state);memset(&obs,0,sizeof(obs));sequence=0;
        state.action_locked=1;state.kind=TRACK_RING_360;state.direction=inner ? -1 : 1;
        state.corner_stage=6;state.phase=RING_STATE_INSIDE;
        obs.corners=frame;obs.path[0]=(track_path_observation_t){0,0,1,100};
        obs.handoff_near_valid=1;obs.handoff_near_error=0;obs.handoff_near_heading=0;
        obs.handoff_bottom_valid=1;obs.handoff_bottom_error=0;
        observe(20);assert(state.corner_stage==6 && state.corner_guide_id==ring_pdf_guide(6,inner));
        ring_control_observe(&state,&obs,21,1);assert(state.transition_frames==1);
        observe(40);assert(state.corner_stage==6);
        observe(60);assert(state.corner_stage==7 && state.exit_armed);
        assert(state.corner_guide_id==ring_pdf_guide(7,inner));
        /* Visible band cannot complete even if road_clear is erroneously set. */
        obs.corners.road_clear=1;
        observe(80);observe(100);observe(120);assert(state.corner_stage==7);
        obs.corners.exit_white_band=0;obs.corners.road_clear=0;
        memset(obs.corners.guide,0,sizeof(obs.corners.guide));
        observe(140);assert(state.corner_guide_held && state.selected_path.valid);
        observe(240);assert(state.corner_guide_held && state.selected_path.valid);
        observe(241);assert(!state.selected_path.valid && state.corner_stage==7);
        assert(!state.corner_guide_id && state.path_mode!=TRACK_PATH_NORMAL);
        obs.corners.road_clear=1;
        obs.corners.entry[0]=obs.corners.entry[1]=obs.corners.cross_entry=0;
        observe(260);observe(280);assert(state.corner_stage==7);
        observe(300);assert(state.completed_360==0 && state.corner_stage==7);
        assert(state.selected_path.valid && state.selected_path.error==0 && state.path_mode==TRACK_PATH_STRAIGHT);
        for(unsigned i=0;i<7;++i) {observe(320+i*20);assert(state.corner_stage==7 && state.completed_360==0);}
        observe(460);assert(state.completed_360==1 && state.phase==RING_STATE_COOLDOWN);
        /* Old V/bridge gate alone does not advance S6. */
        ring_control_init(&state);state.action_locked=1;state.kind=TRACK_RING_360;
        state.direction=inner ? -1 : 1;state.corner_stage=6;
        state.bridge_seen=state.bridge_passed=state.exit_armed=1;
        obs.corners.continuous[side]=1;obs.corners.v[inner].quality=90;
        for(unsigned i=0;i<8;++i) observe(600+i*20);
        assert(state.corner_stage==6);
        /* A band whose generated path is unusable must not advance either. */
        obs.corners.exit_white_band=1;
        for(unsigned i=0;i<4;++i) observe(800+i*20);
        assert(state.corner_stage==6);
    }
    memset(image,255,154*304);detect(154,304);
    for(unsigned inner=0;inner<2;++inner) {
        const ring_corner_line_t *l=&frame.guide[ring_pdf_guide(7,inner)].boundary[1-inner];
        assert(!l->valid); /* White alone cannot invent an A at the image edge. */
    }
    puts("PASS mirrored A-anchored exit, no image-edge fallback, S6 confirmation, S7 bounded hold/completion, old-V rejection");
}
int main(void)
{
    detection_tests();
    geometry_and_state_tests();
    return 0;
}
