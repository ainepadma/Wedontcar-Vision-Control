#include "track_stripe_brake.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

#define ROWS 154U
#define COLS 304U
static uint8_t image[ROWS][COLS];
static uint8_t road[ROWS][COLS];

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

static uint8_t observe_and_cross(stripe_brake_state_t *state,
    const stripe_brake_observation_t *observation, uint32_t observed, uint32_t now)
{
    if(state->stopped) return stripe_brake_observe(state,observation,observed,now);
    assert(!stripe_brake_observe(state,observation,observed,now));
    assert(state->crossing && !state->stopped);
    assert(!stripe_brake_advance(state,1000,1000));
    assert(!stripe_brake_advance(state,2800,2799));
    assert(!stripe_brake_advance(state,100,100));
    return stripe_brake_advance(state,0,1);
}

int main(void)
{
    stripe_brake_state_t state;
    stripe_brake_start_gate_t gate;
    stripe_brake_observation_t observation;
    stripe_brake_start_gate_init(&gate);
    observation=marker(98U,145U,7U,6U);
    assert(!stripe_brake_start_gate_tick(&gate,0U,1U,&observation,100,100));
    stripe_brake_start_gate_begin(&gate);
    assert(!stripe_brake_start_gate_tick(&gate,1U,1U,&observation,100,100));
    assert(gate.clear_counts[0]==0 && gate.clear_frames==0U);
    observation.visible=0U;
    for(unsigned i=0U;i<14U;++i)
        assert(!stripe_brake_start_gate_tick(&gate,1U,1U,&observation,100,99));
    assert(!stripe_brake_start_gate_tick(&gate,1U,1U,&observation,100,100));
    assert(gate.clear_counts[0]==STRIPE_BRAKE_START_CLEAR_COUNTS &&
           gate.clear_counts[1]<STRIPE_BRAKE_START_CLEAR_COUNTS);
    assert(stripe_brake_start_gate_tick(&gate,1U,1U,&observation,0,15));
    observation.visible=1U;
    assert(stripe_brake_start_gate_tick(&gate,1U,1U,&observation,0,0));
    stripe_brake_start_gate_begin(&gate);
    assert(gate.enabled);
    stripe_brake_start_gate_init(&gate);
    stripe_brake_start_gate_begin(&gate);
    observation.visible=0U;
    assert(!stripe_brake_start_gate_tick(&gate,0U,1U,&observation,1500,1500));
    assert(!stripe_brake_start_gate_tick(&gate,1U,0U,&observation,1500,1500));
    assert(gate.clear_frames==0U);
    assert(!stripe_brake_start_gate_tick(&gate,1U,1U,&observation,0,0));
    assert(!stripe_brake_start_gate_tick(&gate,1U,1U,&observation,0,0));
    observation.reliable=0U;
    assert(!stripe_brake_start_gate_tick(&gate,1U,1U,&observation,0,0));
    assert(gate.clear_frames==0U);
    observation.reliable=1U;
    assert(!stripe_brake_start_gate_tick(&gate,1U,1U,&observation,0,0));
    assert(!stripe_brake_start_gate_tick(&gate,1U,1U,&observation,0,0));
    assert(stripe_brake_start_gate_tick(&gate,1U,1U,&observation,0,0));
    puts("PASS: first launch ignores starting zebra until both wheels clear it; later zebra remains enabled");

    arm(&state,0U);
    observation=marker(98U,145U,7U,6U);
    assert(observation.near);
    assert(observe_and_cross(&state,&observation,60U,60U));
    assert(state.stopped);
    puts("PASS: near image enters measured crossing; both wheels pass before brake");

    arm(&state,0U);
    memset(&observation,0,sizeof(observation));
    observation.reliable=1U;
    for(uint32_t t=60U;t<4000U;t+=20U)
        assert(!stripe_brake_observe(&state,&observation,t,t));
    assert(state.armed);
    observation=marker(98U,145U,5U,6U);
    assert(observation.visible && !observation.complete && observation.near);
    assert(observe_and_cross(&state,&observation,4000U,4000U));
    puts("PASS: distant misses keep ARMED; partial stripes can start crossing");

    arm(&state,0U);
    observation=marker(80U,110U,7U,6U);
    assert(observation.approaching && !observation.near);
    assert(!stripe_brake_observe(&state,&observation,60U,60U));
    memset(&observation,0,sizeof(observation));
    observation.reliable=1U;
    assert(!stripe_brake_observe(&state,&observation,80U,80U));
    assert(!state.crossing);
    assert(!stripe_brake_observe(&state,&observation,100U,100U));
    assert(state.crossing && !state.stopped);
    assert(!stripe_brake_advance(&state,1700,1000));
    assert(!stripe_brake_advance(&state,0,700));
    assert(!stripe_brake_advance(&state,1100,1100));
    assert(stripe_brake_advance(&state,1100,1100));
    puts("PASS: approaching marker lost for two fresh frames still triggers measured crossing");

    stripe_brake_init(&state);
    observation=marker(112U,124U,3U,10U);
    assert(observation.partial && observation.near && !observation.large);
    assert(!stripe_brake_observe(&state,&observation,0U,0U));
    assert(!stripe_brake_observe(&state,&observation,20U,20U));
    assert(state.armed && state.crossing);
    assert(!stripe_brake_advance(&state,1699,1700));
    assert(!stripe_brake_advance(&state,1,0));
    assert(!stripe_brake_advance(&state,1100,1100));
    assert(stripe_brake_advance(&state,1100,1100));
    puts("PASS: short half-visible bars cross without the old height gate");

    observation=marker(40U,70U,7U,6U);
    for(unsigned y=112U;y<=124U;++y)
        for(unsigned k=0U;k<3U;++k)
            for(unsigned x=70U+k*18U;x<80U+k*18U;++x) image[y][x]=0U;
    stripe_brake_detect(&image[0][0],ROWS,COLS,&observation);
    assert(observation.top==112U && observation.near && observation.partial);
    puts("PASS: near short fragment wins over taller distant bars");

    arm(&state,0U);
    observation=marker(40U,70U,7U,6U);
    assert(observation.large && !observation.near);
    assert(!stripe_brake_observe(&state,&observation,60U,60U));
    observation=marker(142U,153U,5U,10U);
    assert(observation.visible && observation.clipped && !observation.large);
    assert(observe_and_cross(&state,&observation,80U,80U));
    puts("PASS: established size survives bottom cropping, including short wide remnants");

    for(unsigned shift=0U;shift<=40U;shift+=4U) {
        arm(&state,0U);
        observation=marker(40U,70U,7U,6U);
        assert(!stripe_brake_observe(&state,&observation,60U,60U));
        observation=marker(98U+shift,145U+shift,7U,6U);
        assert(observation.visible);
        assert(observe_and_cross(&state,&observation,80U,80U));
    }
    puts("PASS: near marker moved down 0..40 pixels starts crossing");

    stripe_brake_init(&state);
    observation=marker(98U,145U,7U,6U);
    assert(!stripe_brake_observe(&state,&observation,0U,0U));
    assert(!stripe_brake_observe(&state,&observation,20U,20U));
    assert(observe_and_cross(&state,&observation,40U,40U));
    puts("PASS: initially near marker starts crossing on confirmation frame");

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
    assert(observe_and_cross(&state,&observation,20U,20U));
    puts("PASS: duplicate/stale/unreliable frames rejected; timer wrap accepted");

    for(unsigned count=3U;count<=8U;++count) {
        if(count==7U) continue;
        stripe_brake_init(&state);
        observation=marker(98U,145U,count,6U);
        assert(!observation.complete);
        for(uint32_t t=0U;t<60U;t+=20U)
            assert(!stripe_brake_observe(&state,&observation,t,t));
        assert(state.armed==(uint8_t)(count<8U));
        assert(state.crossing==(uint8_t)(count<8U));
    }
    observation=marker(40U,70U,3U,6U);
    assert(observation.partial && !observation.complete);
    stripe_brake_road_view(&image[0][0],&road[0][0],ROWS,COLS,&observation);
    assert(image[55][70]==0U && road[55][70]==255U);
    assert(image[55][20]==255U && road[55][20]==255U);
    memset(image,255,sizeof(image));
    for(unsigned y=40U;y<70U;++y)
        for(unsigned x=110U;x<145U;++x) image[y][x]=0U;
    stripe_brake_detect(&image[0][0],ROWS,COLS,&observation);
    assert(!observation.visible && !observation.partial);
    stripe_brake_road_view(&image[0][0],&road[0][0],ROWS,COLS,&observation);
    assert(road[55][120]==0U && memcmp(image,road,sizeof(image))==0);
    stripe_brake_detect(NULL,ROWS,COLS,&observation);
    assert(!observation.reliable && !observation.visible);
    memset(image,0,sizeof(image));
    stripe_brake_detect(&image[0][0],ROWS,COLS,&observation);
    assert(!observation.visible);
    observation=marker(110U,153U,7U,6U);
    memset(image[120],255,COLS);
    stripe_brake_detect(&image[0][0],ROWS,COLS,&observation);
    assert(observation.visible);
    puts("PASS: three-to-six partial bars arm and are removed only from road view; solid obstacle and eight bars rejected");

    arm(&state,0U);
    observation=marker(98U,145U,7U,6U);
    assert(observe_and_cross(&state,&observation,60U,60U));
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
