#include "track_stripe.h"
#include "track_control.h"
#include "track_stop_diagnostics.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#define H 154U
#define W 304U
static uint8_t pixels[H*W];
static track_stripe_state_t state;
static uint32_t clock_ms;

static track_stripe_observation_t pattern(unsigned count,unsigned top,unsigned bottom,unsigned width,unsigned gap)
{
    track_stripe_observation_t o;
    unsigned span=count*width+(count-1U)*gap;
    memset(pixels,0,sizeof(pixels));
    for(unsigned y=0;y<H;++y) {
        /* White road and exterior black, including a curved edge. */
        unsigned left=20U+(H-y)/6U;
        memset(pixels+y*W+left,255,W-left-20U);
        if(y<top || y>bottom) continue;
        for(unsigned i=0;i<count;++i)
            memset(pixels+y*W+(W-span)/2U+i*(width+gap),0,width);
    }
    track_stripe_detect(pixels,H,W,&o);
    return o;
}
static uint8_t observe(track_stripe_observation_t o,uint8_t running)
{
    clock_ms+=20U;
    return track_stripe_observe(&state,&o,clock_ms,clock_ms,running);
}
static void reset(void) { track_stripe_init(&state);clock_ms=0U; }
int main(int argc,char **argv)
{
    track_stripe_observation_t p1=pattern(7,38,51,3,3);
    track_stripe_observation_t p2=pattern(7,55,76,4,4);
    track_stripe_observation_t p3=pattern(7,72,110,7,6);
    track_stripe_observation_t p4=pattern(7,99,153,9,8);
    track_stripe_observation_t clear=pattern(1,60,120,25,5);
    assert(p1.complete && p2.complete && p3.complete && p4.complete);
    assert(!p1.near && !p2.near && !p3.near && p4.near);
    assert(!pattern(7,100,130,5,5).near); /* large but too far from bottom */
    assert(!pattern(7,140,153,3,3).near); /* bottom but too short */
    assert(!pattern(7,100,102,4,4).visible); /* horizontal speckle */
    assert(!pattern(8,90,153,7,6).visible); /* cannot select seven of eight */
    assert(!pattern(1,90,153,80,5).visible); /* solid transverse bar */
    reset();
    for(unsigned i=0;i<4;++i) assert(!observe(p1,1));
    assert(state.armed && !state.stopped);
    for(unsigned i=0;i<4;++i) assert(!observe(p2,1));
    for(unsigned i=0;i<4;++i) assert(!observe(p3,1));
    assert(!observe(p4,1));assert(observe(p4,1));
    assert(state.stopped && observe(clear,1));
    puts("PASS P1/P2/P3 arm only; P4 needs size AND bottom; stop latches");

    reset();
    track_stripe_observation_t partial=pattern(6,99,153,9,8);
    assert(partial.visible && !partial.complete && partial.near);
    for(unsigned i=0;i<10;++i) assert(!observe(partial,1));
    assert(!state.armed);
    for(unsigned i=0;i<3;++i) assert(!observe(p1,1));
    assert(!observe(partial,1));assert(observe(partial,1));
    puts("PASS partial group cannot arm; partial near group can finish armed approach");

    reset();assert(!observe(p1,1));
    for(unsigned i=0;i<20;++i)
        assert(!track_stripe_observe(&state,&p1,clock_ms,clock_ms,1));
    assert(!state.armed && state.arm_votes==1U);
    track_stripe_observation_t bad=p4;bad.reliable=0U;
    observe(bad,1);assert(!state.arm_votes);
    for(unsigned i=0;i<3;++i) observe(p1,1);
    observe(p4,1);observe(bad,1);assert(!observe(p4,1));
    clock_ms+=200U;
    assert(!track_stripe_observe(&state,&p4,clock_ms-180U,clock_ms,1));
    assert(!state.near_votes);
    assert(!observe(p4,1));assert(observe(p4,1));
    puts("PASS duplicate timestamps, unreliable frames and stale data cannot confirm stop");

    track_stripe_restart(&state,1);
    assert(state.bypass && !state.stopped);
    for(unsigned i=0;i<30;++i) assert(!observe(p4,1));
    for(unsigned i=0;i<30;++i) assert(!observe(clear,0));
    assert(state.bypass); /* stopped preview must not clear bypass */
    for(unsigned i=0;i<30;++i) assert(!observe(bad,1));
    assert(state.bypass); /* unreliable is not clear */
    for(unsigned i=0;i<TRACK_STRIPE_CLEAR_FRAMES;++i) assert(!observe(clear,1));
    assert(!state.bypass);
    for(unsigned i=0;i<3;++i) assert(!observe(p1,1));
    assert(!observe(p4,1));assert(observe(p4,1));
    puts("PASS C13 restart bypasses current marker; reliable moving clear frames rearm next marker");

    track_control_state_t control;
    track_stop_snapshot_t record={0};
    track_control_init(&control,NULL);
    track_control_set_loss_stop(&control,0);
    track_control_toggle(&control,0);
    assert(control.running);
    track_control_stop(&control,TRACK_STOP_STRIPE);
    track_control_output_t output=track_control_tick(&control,10);
    assert(!output.running && output.target_speed==0.0f && output.stop_reason==TRACK_STOP_STRIPE);
    track_stop_record(&record,1,0,output.stop_reason,0,0,10,90,90,120,600,600);
    assert(record.sequence==1 && record.reason==TRACK_STOP_STRIPE);
    track_control_set_loss_stop(&control,1);
    track_control_observe(&control,0,0,100,1,20);
    assert(!track_control_tick(&control,20).running);
    puts("PASS C14-independent stripe reason and zero-speed latch survive observations/protection toggle");

    /* Optional raw binary replay: no dependency on image libraries. */
    for(int i=1;i<argc;++i) {
        FILE *f=fopen(argv[i],"rb");assert(f);
        assert(fread(pixels,1,sizeof(pixels),f)==sizeof(pixels));fclose(f);
        track_stripe_observation_t o;
        track_stripe_detect(pixels,H,W,&o);
        printf("REPLAY %s: visible=%u complete=%u near=%u top=%u bottom=%u span=%u\n",
            argv[i],o.visible,o.complete,o.near,o.top,o.bottom,o.span);
    }
    return 0;
}
