#include "track_stripe_brake.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

#define ROWS 154U
#define COLS 304U
static uint8_t image[ROWS][COLS];

static stripe_brake_observation_t marker(unsigned top, unsigned bottom,
                                        unsigned count, unsigned width)
{
    stripe_brake_observation_t observation;
    memset(image, 255, sizeof(image));
    for(unsigned y=top;y<=bottom && y<ROWS;++y)
        for(unsigned k=0;k<count;++k)
            for(unsigned x=70U+k*(width+8U);x<70U+k*(width+8U)+width;++x)
                image[y][x]=0U;
    stripe_brake_detect(&image[0][0],ROWS,COLS,&observation);
    return observation;
}

static void arm(stripe_brake_state_t *state, uint32_t start)
{
    stripe_brake_observation_t observation=marker(40U,70U,7U,6U);
    stripe_brake_init(state);
    assert(observation.visible && observation.complete && !observation.near);
    for(uint32_t i=0U;i<3U;++i)
        assert(!stripe_brake_observe(state,&observation,start+i*20U,start+i*20U));
    assert(state->armed && !state->stopped);
}

int main(void)
{
    stripe_brake_state_t state;
    stripe_brake_observation_t observation;
    arm(&state,0U);
    observation=marker(98U,145U,7U,6U);
    assert(observation.near);
    assert(stripe_brake_observe(&state,&observation,60U,60U));
    assert(state.stopped);
    puts("PASS: hand-push arming and ONE P4 observation latch stop");

    arm(&state,0U);
    memset(&observation,0,sizeof(observation));
    observation.reliable=1U;
    for(uint32_t t=60U;t<4000U;t+=20U)
        assert(!stripe_brake_observe(&state,&observation,t,t));
    assert(state.armed);
    observation=marker(98U,145U,5U,6U);
    assert(observation.visible && !observation.complete && observation.near);
    assert(stripe_brake_observe(&state,&observation,4000U,4000U));
    puts("PASS: hundreds of misses keep ARMED; partial stripes stop at P4");

    arm(&state,0U);
    observation=marker(80U,130U,7U,6U);
    assert(observation.large && !observation.near);
    assert(!stripe_brake_observe(&state,&observation,60U,60U));
    observation=marker(142U,153U,5U,10U);
    assert(observation.visible && observation.clipped && !observation.large);
    assert(stripe_brake_observe(&state,&observation,80U,80U));
    puts("PASS: established size survives bottom cropping, including short wide remnants");

    for(unsigned shift=0U;shift<=40U;shift+=4U) {
        arm(&state,0U);
        observation=marker(80U,130U,7U,6U);
        assert(!stripe_brake_observe(&state,&observation,60U,60U));
        observation=marker(98U+shift,145U+shift,7U,6U);
        assert(observation.visible);
        assert(stripe_brake_observe(&state,&observation,80U,80U));
    }
    puts("PASS: P4 moved down 0..40 pixels never loses stopping condition");

    stripe_brake_init(&state);
    observation=marker(98U,145U,7U,6U);
    assert(!stripe_brake_observe(&state,&observation,0U,0U));
    assert(!stripe_brake_observe(&state,&observation,20U,20U));
    assert(stripe_brake_observe(&state,&observation,40U,40U));
    puts("PASS: initially near marker stops on the frame completing confirmation");

    stripe_brake_init(&state);
    observation=marker(40U,70U,7U,6U);
    assert(!stripe_brake_observe(&state,&observation,10U,10U));
    for(unsigned i=0U;i<20U;++i)
        assert(!stripe_brake_observe(&state,&observation,10U,10U));
    assert(state.arm_votes==1U);
    assert(!stripe_brake_observe(&state,&observation,20U,200U));
    assert(!state.armed && state.arm_votes==0U);
    assert(!stripe_brake_observe(&state,&observation,10U,200U));
    observation.reliable=0U;
    assert(!stripe_brake_observe(&state,&observation,200U,200U));
    assert(!state.armed);
    arm(&state,UINT32_MAX-39U);
    observation=marker(98U,145U,7U,6U);
    assert(stripe_brake_observe(&state,&observation,20U,20U));
    puts("PASS: duplicate/stale/unreliable frames rejected; timer wrap accepted");

    for(unsigned count=5U;count<=8U;++count) {
        if(count==7U) continue;
        stripe_brake_init(&state);
        observation=marker(98U,145U,count,6U);
        assert(!observation.complete);
        for(uint32_t t=0U;t<200U;t+=20U)
            assert(!stripe_brake_observe(&state,&observation,t,t));
        assert(!state.armed && !state.stopped);
    }
    stripe_brake_detect(NULL,ROWS,COLS,&observation);
    assert(!observation.reliable && !observation.visible);
    memset(image,0,sizeof(image));
    stripe_brake_detect(&image[0][0],ROWS,COLS,&observation);
    assert(!observation.visible);
    observation=marker(110U,153U,7U,6U);
    memset(image[120],255,COLS);
    stripe_brake_detect(&image[0][0],ROWS,COLS,&observation);
    assert(observation.visible);
    puts("PASS: five/six/eight stripes, black field, invalid input do not arm");

    arm(&state,0U);
    observation=marker(98U,145U,7U,6U);
    assert(stripe_brake_observe(&state,&observation,60U,60U));
    assert(stripe_brake_hold(&state,0U,60,1U)==-2100);
    assert(state.position_error[0]==0);
    assert(stripe_brake_hold(&state,0U,10,0U)==-470);
    assert(stripe_brake_hold(&state,0U,0,0U)==-120);
    assert(stripe_brake_hold(&state,1U,-10,0U)==470);
    assert(stripe_brake_hold(&state,1U,0,0U)==120);
    for(unsigned i=0U;i<1000U;++i) {
        int16_t left=stripe_brake_hold(&state,0U,32767,0U);
        int16_t right=stripe_brake_hold(&state,1U,-32768,0U);
        assert(left==-STRIPE_BRAKE_PWM_LIMIT && right==STRIPE_BRAKE_PWM_LIMIT);
    }
    assert(stripe_brake_hold(&state,2U,10,0U)==0);
    memset(&observation,0,sizeof(observation));
    for(unsigned i=0U;i<100U;++i)
        assert(stripe_brake_observe(&state,&observation,80U+i*20U,80U+i*20U));
    stripe_brake_init(&state);
    assert(!state.armed && !state.stopped);
    assert(stripe_brake_hold(&state,0U,100,0U)==0);
    puts("PASS: signed opposing torque, stationary position resistance, bounds, reset");
    return 0;
}
