/* Test the production implementation directly, including runtime-invalid
 * angle configurations. Do not link ring_corner.c a second time. */
#include "ring_corner.h"
static const float configured_angle = RING_EXIT_LINE_ANGLE_DEG;
static float test_angle = RING_EXIT_LINE_ANGLE_DEG;
#undef RING_EXIT_LINE_ANGLE_DEG
#define RING_EXIT_LINE_ANGLE_DEG test_angle
#include "../demo/user/src/ring_corner.c"
#include "ring_control.h"
#include <assert.h>
#include <stdio.h>

#define TEST_H 154U
#define TEST_W 304U
static uint8_t test_image[TEST_H*TEST_W],test_saved[TEST_H*TEST_W];
static ring_corner_frame_t test_frame;
static ring_corner_guide_t test_guide;

static void setup(void)
{
    memset(test_image,255,sizeof(test_image));
    memset(&test_frame,0,sizeof(test_frame));
    memset(&test_guide,0,sizeof(test_guide));
    memset(flags,0,sizeof(flags));
    memset(length,0,sizeof(length));
    for(unsigned y=0;y<TEST_H;++y) {
        flags[y]=3U;b[0][y]=30;b[1][y]=273;
    }
    test_frame.rows=TEST_H;test_frame.cols=TEST_W;
    test_frame.exit_white_band=1;
    test_frame.exit_white_top=90;test_frame.exit_white_bottom=110;
    test_angle=60.f;
}
static void set_anchor(unsigned inner,float x,float y,unsigned quality)
{
    test_frame.a[1U-inner]=point(x,y,RING_CORNER_A);
    test_frame.a[1U-inner].quality=(uint8_t)quality;
}
static void generate(unsigned inner)
{
    memcpy(test_saved,test_image,sizeof(test_image));
    memset(&test_guide,0,sizeof(test_guide));
    exit_diagonal(test_image,TEST_H,TEST_W,inner,&test_frame,&test_guide);
    assert(!memcmp(test_saved,test_image,sizeof(test_image)));
}
static void assert_geometry(unsigned inner,float x,float y)
{
    const ring_corner_line_t *line=&test_guide.boundary[1U-inner];
    float expected=tanf(test_angle*3.14159265358979323846f/180.f);
    assert(line->valid && !test_guide.boundary[inner].valid);
    assert(line->x0==x && line->y0==y); /* Never relocate the observed A. */
    assert(line->y1<line->y0);
    assert(inner ? line->x1>line->x0 : line->x1<line->x0);
    assert(line->x1>=0.f && line->x1<=TEST_W-1.f);
    assert(line->y1>=0.f && line->y1<=TEST_H-1.f);
    assert(fabsf(fabsf(line->x1-x)/(y-line->y1)-expected)<.0001f);
    assert(test_guide.corner_x==x && test_guide.corner_y==y);
    assert(test_guide.corner_side==1U-inner && test_guide.corner_kind==RING_CORNER_A);
}
static void geometry_tests(void)
{
    assert(configured_angle==60.f);
    for(unsigned inner=0;inner<2;++inner) {
        float x=inner ? 73.f : 230.f;
        setup();set_anchor(inner,x,80.f,RING_CORNER_MIN_QUALITY);
        test_angle=45.f;generate(inner);assert_geometry(inner,x,80.f);
        float dx45=fabsf(test_guide.boundary[1U-inner].x1-x);
        assert(fabsf(dx45-80.f)<.001f);
        test_angle=60.f;generate(inner);assert_geometry(inner,x,80.f);
        assert(test_guide.valid);
        float dx60=fabsf(test_guide.boundary[1U-inner].x1-x);
        assert(fabsf(dx60/dx45-sqrtf(3.f))<.0001f);
        assert(test_guide.boundary[1U-inner].y1==0.f);

        /* Side clipping must shorten dx and dy proportionally, not clamp a
         * full-height endpoint and silently change the requested angle. */
        x=inner ? 283.f : 20.f;
        setup();set_anchor(inner,x,100.f,90);generate(inner);assert_geometry(inner,x,100.f);
        const ring_corner_line_t *line=&test_guide.boundary[1U-inner];
        assert(fabsf(line->x1-(inner ? 303.f : 0.f))<.0001f);
        assert(fabsf(line->y1-(100.f-20.f/sqrtf(3.f)))<.0001f);
        assert(line->y0!=test_frame.exit_white_bottom);

        /* The exact last image row/column is an allowed real coordinate. */
        x=inner ? 0.f : 303.f;
        setup();set_anchor(inner,x,153.f,90);generate(inner);assert_geometry(inner,x,153.f);
    }
    puts("PASS A anchor unchanged, 45/60 comparison, mirrored slope, top/side clipping, image bounds");
}
static void rejection_tests(void)
{
    const float invalid_angles[]={0.f,90.f,-1.f,91.f,INFINITY,NAN};
    const float invalid_points[][2]={{-.01f,80.f},{303.01f,80.f},{100.f,-.01f},
        {100.f,153.01f},{NAN,80.f},{100.f,NAN},{INFINITY,80.f},{100.f,-INFINITY}};
    for(unsigned inner=0;inner<2;++inner) {
        setup();test_frame.a[inner]=point(150,80,RING_CORNER_A);generate(inner);
        assert(!test_guide.valid && !test_guide.boundary[1U-inner].valid); /* wrong side only */
        set_anchor(inner,inner ? 73.f : 230.f,80.f,0);generate(inner);
        assert(!test_guide.boundary[1U-inner].valid);
        set_anchor(inner,inner ? 73.f : 230.f,80.f,RING_CORNER_MIN_QUALITY-1U);generate(inner);
        assert(!test_guide.valid && !test_guide.boundary[1U-inner].valid);
        set_anchor(inner,inner ? 73.f : 230.f,80.f,90);
        test_frame.a[1U-inner].kind=RING_CORNER_B;generate(inner);
        assert(!test_guide.valid && !test_guide.boundary[1U-inner].valid);
        test_frame.a[1U-inner].kind=RING_CORNER_NONE;generate(inner);
        assert(!test_guide.valid && !test_guide.boundary[1U-inner].valid);
        for(unsigned i=0;i<sizeof(invalid_angles)/sizeof(invalid_angles[0]);++i) {
            setup();set_anchor(inner,inner ? 73.f : 230.f,80.f,90);
            test_angle=invalid_angles[i];generate(inner);
            assert(!test_guide.valid && !test_guide.boundary[1U-inner].valid);
        }
        for(unsigned i=0;i<sizeof(invalid_points)/sizeof(invalid_points[0]);++i) {
            setup();set_anchor(inner,invalid_points[i][0],invalid_points[i][1],90);generate(inner);
            assert(!test_guide.valid && !test_guide.boundary[1U-inner].valid);
        }
        setup();set_anchor(inner,inner ? 73.f : 230.f,80.f,90);
        test_frame.exit_white_band=0;generate(inner);
        assert(!test_guide.valid && !test_guide.boundary[1U-inner].valid);
        setup();set_anchor(inner,inner ? 73.f : 230.f,0.f,90);generate(inner);
        assert(!test_guide.boundary[1U-inner].valid);
        setup();set_anchor(inner,inner ? 302.f : 1.f,80.f,90);generate(inner);
        assert(!test_guide.boundary[1U-inner].valid); /* <2 vertical pixels */
    }
    puts("PASS no wrong-side/image-edge fallback; missing/weak/nonfinite/outside A, invalid angle/band/span rejected");
}
static void reacquire_tests(void)
{
    for(unsigned inner=0;inner<2;++inner) {
        setup();set_anchor(inner,inner ? 73.f : 230.f,80.f,90);generate(inner);
        test_frame.guide[ring_pdf_guide(7,inner)]=test_guide;
        ring_corner_tracker_t tracker;memset(&tracker,0,sizeof(tracker));
        tracker.rows=TEST_H;tracker.cols=TEST_W;
        tracker.v_valid[inner]=1;tracker.v[inner]=point(150,70,RING_CORNER_B);
        /* Real production V reacquisition branch, not merely a no-op call. */
        length[inner]=7;
        for(unsigned i=0;i<7;++i) {
            edge[inner][i][0]=(uint16_t)(147U+i);
            edge[inner][i][1]=(uint16_t)(70-abs((int)i-3));
        }
        for(unsigned y=66;y<=68;++y) for(unsigned x=149;x<=151;++x)
            test_image[y*TEST_W+x]=0;
        ring_corner_stabilize(test_image,0,&tracker,&test_frame);
        assert(test_frame.v[inner].quality && (test_frame.v_reacquired&(1U<<inner)));
        assert(!memcmp(&test_guide,&test_frame.guide[ring_pdf_guide(7,inner)],sizeof(test_guide)));
    }
    puts("PASS actual V reacquisition cannot overwrite A-based S7 exit");
}
static void state_tests(void)
{
    for(unsigned inner=0;inner<2;++inner) {
        setup();
        track_ring_observation_t obs;memset(&obs,0,sizeof(obs));
        ring_control_state_t state;ring_control_init(&state);
        state.action_locked=1;state.kind=TRACK_RING_360;state.direction=inner ? -1 : 1;
        state.corner_stage=6;state.phase=RING_STATE_INSIDE;
        obs.corners=test_frame;
        for(unsigned i=1;i<=4;++i) {
            obs.corners.frame_number=i;ring_control_observe(&state,&obs,i*20,1);
        }
        assert(state.corner_stage==6 && !state.exit_armed); /* white band alone */
        set_anchor(inner,inner ? 73.f : 230.f,80.f,90);generate(inner);
        assert(test_guide.valid);
        obs.corners.guide[ring_pdf_guide(6,inner)]=test_guide;
        obs.corners.guide[ring_pdf_guide(7,inner)]=test_guide;
        for(unsigned i=5;i<=7;++i) {
            obs.corners.frame_number=i;ring_control_observe(&state,&obs,i*20,1);
            ring_control_select_path(&state,&obs);
        }
        assert(state.corner_stage==7 && state.exit_armed);
        assert(state.corner_guide_id==ring_pdf_guide(7,inner));
        assert(state.corner_selected_guide.boundary[1U-inner].x0==(inner ? 73.f : 230.f));
    }
    puts("PASS S6 requires usable A route; S7 keeps correct mirrored A guide");
}
int main(void)
{
    geometry_tests();rejection_tests();reacquire_tests();state_tests();
    return 0;
}
