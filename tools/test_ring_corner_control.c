#include "ring_control.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static ring_control_state_t state;
static track_ring_observation_t frame;
static uint32_t clock_ms,frame_id;

static void fresh_frame(void)
{
    memset(&frame,0,sizeof(frame));
    frame.corners.rows=240U;frame.corners.cols=376U;
    frame.path[0]=(track_path_observation_t){0,0,1U,95U};
    frame.near_road=frame.near_paired=1U;
}
static void setup(void)
{
    ring_control_init(&state);clock_ms=0U;frame_id=0U;fresh_frame();
}
static void guide(unsigned id,unsigned side,unsigned kind,float y,int error)
{
    ring_corner_guide_t *g=&frame.corners.guide[id];
    g->valid=1U;g->confidence=90U;g->error=(int16_t)error;g->heading=(int16_t)(error/2);
    g->corner_side=(uint8_t)side;g->corner_kind=(uint8_t)kind;
    g->corner_x=side==0U ? 80.f : 296.f;g->corner_y=y;
    g->boundary[0]=(ring_corner_line_t){80.f,160.f,100.f,96.f,1U};
    g->boundary[1]=(ring_corner_line_t){296.f,160.f,276.f,96.f,1U};
}
static void paired(unsigned count)
{
    fresh_frame();frame.corners.count[0]=frame.corners.count[1]=(uint8_t)count;
    for(unsigned side=0;side<2U;++side) for(unsigned i=0;i<count && i<RING_CORNER_MAX_PER_SIDE;++i) {
        ring_corner_point_t *p=&frame.corners.point[side][i];
        p->x=side==0U ? 80.f : 296.f;p->y=150.f-30.f*(float)i;p->quality=90U;
        p->kind=i==0U ? RING_CORNER_A : RING_CORNER_B;
    }
    if(count==1U) guide(RING_GUIDE_270_EXTEND,0U,RING_CORNER_NONE,150.f,35);
    if(count==2U) guide(RING_GUIDE_270_CONNECT,0U,RING_CORNER_NONE,150.f,15);
}
static void corner(unsigned side,unsigned kind,float y)
{
    fresh_frame();frame.corners.count[side]=1U;
    ring_corner_point_t *p=&frame.corners.point[side][0];
    p->x=side==0U ? 80.f : 296.f;p->y=y;p->kind=(uint8_t)kind;p->quality=90U;
    int sign=side==0U ? -1 : 1;
    if(kind==RING_CORNER_A) guide(RING_GUIDE_A_LEFT+side,side,kind,y,sign*120);
    else {
        guide(RING_GUIDE_B_BRANCH_LEFT+side,side,kind,y,sign*300);
        guide(RING_GUIDE_B_MAIN_LEFT+side,side,kind,y,sign*50);
    }
}
static void clear(void)
{
    fresh_frame();frame.corners.road_clear=1U;
}
static track_path_observation_t feed_motion(float travel)
{
    clock_ms+=20U;frame.corners.frame_number=++frame_id;
    ring_control_tick(&state,1U,0U,0.f,travel,clock_ms);
    ring_control_observe(&state,&frame,clock_ms,1U);
    return ring_control_select_path(&state,&frame);
}
static track_path_observation_t feed(void) {return feed_motion(25.f);}
static void repeat(unsigned n) {for(unsigned i=0;i<n;++i) (void)feed();}

#if RING_FEATURE_ENABLED
static void test_270(void)
{
    setup();paired(1U);repeat(2U);assert(!state.action_locked);
    track_path_observation_t p=feed();
    assert(state.kind==TRACK_RING_270 && state.corner_stage==1U && state.action_locked);
    assert(p.valid && p.error==35 && state.corner_guide_id==RING_GUIDE_270_EXTEND);
    paired(2U);repeat(3U);assert(state.corner_stage==2U && state.selected_path.error==15);
    /* A persistent entrance cannot become the exit or complete a lap. */
    repeat(12U);assert(state.corner_stage==2U && !state.exit_seen);
    paired(1U);repeat(2U);assert(state.corner_stage==2U && !state.exit_seen);
    clear();repeat(3U);assert(state.corner_stage==3U && state.entry_passed && !state.line_active);
    paired(1U);frame.path[0].valid=0U;(void)feed();
    assert(state.corner_stage==3U && !state.exit_seen && state.line_active && state.selected_path.valid);
    assert(state.corner_guide_frame_number==state.corner_frame_number);
    clear();(void)feed();assert(state.corner_stage==3U && !state.line_active);
    assert(state.corner_guide_id==RING_GUIDE_NONE && !state.corner_have_guide);
    paired(1U);repeat(3U);assert(state.corner_stage==4U && state.exit_seen);
    assert(state.selected_path.error==35);
    paired(2U);repeat(3U);assert(state.corner_stage==5U && state.selected_path.error==15);
    clear();repeat(3U);assert(state.completed_270==1U && !state.action_locked);
    assert(state.phase==RING_STATE_COOLDOWN);
    /* No once-per-boot ban: a later physical ring may be recognized. */
    repeat(10U);assert(state.phase==RING_STATE_IDLE);
    paired(1U);repeat(3U);assert(state.action_locked && state.kind==TRACK_RING_270);
}
static void test_360(unsigned first_side)
{
    setup();unsigned other=1U-first_side;int first_sign=first_side==0U ? -1 : 1;
    corner(first_side,RING_CORNER_A,100.f);repeat(4U);
    assert(!state.action_locked); /* Distant A cannot take steering ownership. */
    corner(first_side,RING_CORNER_A,128.f);repeat(3U);
    assert(state.kind==TRACK_RING_360 && state.corner_stage==1U);
    assert(state.selected_path.error==first_sign*120);
    repeat(10U);assert(state.corner_stage==1U); /* One corner, many frames. */
    corner(first_side,RING_CORNER_A,145.f);(void)feed();
    corner(first_side,RING_CORNER_B,105.f);repeat(3U);
    assert(state.corner_stage==2U && state.corner_guide_id==RING_GUIDE_B_BRANCH_LEFT+first_side);
    assert(state.selected_path.error==first_sign*300);
    corner(other,RING_CORNER_A,128.f);repeat(3U);
    assert(state.corner_stage==3U && state.corner_guide_id==RING_GUIDE_A_LEFT+other);
    assert(state.direction==(first_side==0U ? 1 : -1)); /* Target side is not ring direction. */
    corner(other,RING_CORNER_B,157.f);repeat(3U);
    assert(state.corner_stage==4U && state.corner_guide_id==RING_GUIDE_B_MAIN_LEFT+other);
    assert(state.selected_path.error==-first_sign*50); /* B4 does not reuse B2 branch. */
    clear();repeat(5U);assert(state.completed_360==1U && !state.action_locked);
}
static void test_quantity_priority_and_order(void)
{
    setup();paired(1U);
    /* Even A-shaped arms on BOTH sides are quantity evidence for 270. */
    guide(RING_GUIDE_A_LEFT,0U,RING_CORNER_A,150.f,-120);
    guide(RING_GUIDE_A_RIGHT,1U,RING_CORNER_A,150.f,120);
    repeat(3U);assert(state.kind==TRACK_RING_270);
    setup();paired(2U);repeat(5U);assert(!state.action_locked); /* Cannot skip 1->2. */
    setup();paired(1U);repeat(1U);
    corner(0U,RING_CORNER_A,128.f);repeat(4U);assert(!state.action_locked);
    setup();paired(3U);repeat(5U);assert(!state.action_locked);
    setup();corner(0U,RING_CORNER_B,150.f);repeat(5U);assert(!state.action_locked);
    setup();corner(0U,RING_CORNER_A,128.f);repeat(3U);
    ring_control_set_protection(&state,0U);
    corner(1U,RING_CORNER_A,130.f);repeat(4U);assert(state.corner_stage==1U);
}
static void test_dedup_and_static_car(void)
{
    setup();corner(0U,RING_CORNER_A,128.f);(void)feed();
    for(unsigned i=0;i<8U;++i) ring_control_observe(&state,&frame,clock_ms,1U);
    assert(!state.action_locked && state.corner_vote_frames==1U);
    repeat(2U);assert(state.corner_stage==1U);
    /* A/B classification jitter at one spatial point cannot count twice. */
    corner(0U,RING_CORNER_B,128.f);repeat(8U);assert(state.corner_stage==1U);
    setup();corner(0U,RING_CORNER_A,145.f);repeat(3U);
    corner(0U,RING_CORNER_B,105.f);
    for(unsigned i=0;i<4U;++i) (void)feed_motion(0.f);
    assert(state.corner_stage==1U);
    /* A unique new target can advance without a frame where all corners vanish. */
    repeat(3U);assert(state.corner_stage==2U);
    /* Two corners may remain in one image. A1 must not be reused as A3. */
    setup();corner(0U,RING_CORNER_A,128.f);
    frame.corners.count[0]=2U;
    frame.corners.point[0][1]=frame.corners.point[0][0];
    frame.corners.point[0][1].kind=RING_CORNER_B;frame.corners.point[0][1].y=105.f;
    guide(RING_GUIDE_B_BRANCH_LEFT,0U,RING_CORNER_B,105.f,-300);
    guide(RING_GUIDE_B_MAIN_LEFT,0U,RING_CORNER_B,105.f,-50);
    repeat(6U);assert(state.corner_stage==1U); /* Farther B cannot turn before A1 reaches entrance. */
    frame.corners.point[0][0].y=145.f;frame.corners.guide[RING_GUIDE_A_LEFT].corner_y=145.f;
    repeat(3U);assert(state.corner_stage==2U);
    repeat(12U);assert(state.corner_stage==2U && !state.completed_360);
    /* Even a new same-side A cannot substitute for the opposite-side A3. */
    corner(0U,RING_CORNER_A,140.f);repeat(3U);assert(state.corner_stage==2U);
    setup();corner(0U,RING_CORNER_A,145.f);repeat(3U);
    corner(0U,RING_CORNER_B,105.f);
    frame.corners.guide[RING_GUIDE_B_BRANCH_LEFT].valid=0U;
    repeat(3U);assert(state.corner_stage==1U);
    assert(state.corner_guide_id!=RING_GUIDE_B_MAIN_LEFT); /* Valid B4 cannot replace invalid B2. */
}
static void test_missing_stop_and_stale(void)
{
    setup();paired(1U);repeat(3U);
    ring_control_set_protection(&state,0U);
    fresh_frame();frame.path[0].valid=0U;frame.near_road=frame.near_paired=0U;
    track_path_observation_t p=feed();assert(p.valid && state.corner_guide_held && p.confidence<55U);
    assert(state.corner_guide_frame_number!=state.corner_frame_number);
    repeat(7U);assert(!state.selected_path.valid && !state.fault && state.corner_stage==1U);
    ring_control_set_protection(&state,1U);(void)feed();assert(state.fault);
    ring_control_set_protection(&state,0U);assert(!state.fault && !state.corner_have_guide);
    ring_control_tick(&state,0U,0U,0.f,0.f,clock_ms+10U);
    assert(!state.action_locked && state.corner_stage==0U && !state.fault);
    setup();corner(0U,RING_CORNER_A,128.f);repeat(3U);
    clock_ms+=RING_VISION_STALE_MS+1U;
    ring_control_tick(&state,1U,0U,0.f,25.f,clock_ms);assert(state.fault);
    assert(!ring_control_select_path(&state,&frame).valid);
    ring_control_tick(&state,0U,0U,0.f,0.f,clock_ms+10U);assert(state.fault);
}
static void test_transaction_and_timeout(void)
{
    setup();corner(0U,RING_CORNER_A,128.f);repeat(3U);
    ring_corner_guide_t g=state.corner_selected_guide;
    assert(state.corner_guide_frame_number==state.corner_frame_number);
    /* Obsolete frame data is never applied to the current state. */
    --frame.corners.frame_number;assert(!ring_control_select_path(&state,&frame).valid);
    assert(state.fault);assert(state.corner_selected_guide.error==g.error);
    setup();paired(1U);repeat(3U);
    clock_ms+=RING_ACTIVE_TIMEOUT_MS+1U;
    ring_control_tick(&state,1U,0U,0.f,25.f,clock_ms);
    assert(state.fault && state.corner_stage==1U); /* Timeout never completes a stage. */
    setup();paired(1U);repeat(3U);ring_control_set_protection(&state,0U);
    repeat(RING_LINE_MAX_MS/20U+2U);
    assert(!state.fault && !state.selected_path.valid && state.corner_stage==1U);
    (void)feed();assert(!state.selected_path.valid); /* No line resurrection with protection off. */
    setup();frame_id=UINT32_MAX-2U;paired(1U);repeat(3U);
    assert(state.action_locked && frame.corners.frame_number==0U); /* Wrap is supported. */
    float angle=91.f,speed=80.f;
    ring_control_360_motion(&state,87.f,98.5f,75.5f,120.f,&angle,&speed);
    assert(angle==91.f && speed==80.f); /* No legacy actuator override survives. */
}
#endif
int main(void)
{
#if RING_FEATURE_ENABLED
    test_270();test_360(0U);test_360(1U);test_quantity_priority_and_order();
    test_dedup_and_static_car();test_missing_stop_and_stale();test_transaction_and_timeout();
    puts("PASS corner ring control: quantities, A/B sequence, mirrors, dedup, handoff, stale and protection");
#else
    setup();paired(1U);repeat(5U);assert(!state.action_locked);
    corner(0U,RING_CORNER_A,128.f);repeat(5U);assert(!state.action_locked);
    clear();assert(feed().valid);puts("PASS disabled corner rings preserve ordinary path");
#endif
    return 0;
}
