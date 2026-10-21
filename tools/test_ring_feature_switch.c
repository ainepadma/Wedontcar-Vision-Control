#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ring_control.h"
#include "track_control.h"
#define CHECK(c) do{if(!(c)){fprintf(stderr,"FAIL %d: %s\n",__LINE__,#c);exit(1);}}while(0)
static uint8_t image[240U*376U];
static track_vision_result_t result;
static void dirty(ring_control_state_t *s)
{
    memset(s,0,sizeof(*s));s->protection_enabled=1;s->fault=1;s->action_locked=1;
    s->line_active=1;s->kind=TRACK_RING_360;s->phase=RING_STATE_INSIDE;
    s->reset_tracking_history=1;s->control_refresh=1;s->completed_270=2;s->completed_360=2;
}
static void clear(const ring_control_state_t *s)
{
    CHECK(!s->fault && !s->action_locked && !s->line_active && !s->entry_seen);
    CHECK(!s->reset_tracking_history && !s->completed_270 && !s->completed_360);
    CHECK(s->kind==TRACK_RING_UNKNOWN && s->path_mode==TRACK_PATH_NORMAL && s->protection_enabled);
}
int main(void)
{
    CHECK(RING_FEATURE_ENABLED==0);
    for(unsigned y=0;y<240;++y) memset(image+y*376U+126,255,124);
    track_vision_process(image,240,376,&result);CHECK(result.valid && !result.ring_candidate);
    int16_t error=result.error,heading=result.heading;uint8_t confidence=result.confidence;
    track_ring_vision_state_t vision_state;memset(&vision_state,0xFF,sizeof(vision_state));
    track_ring_360_set_active(5,1);
    track_ring_vision_process(&vision_state,image,240,376,&result);
    CHECK(result.valid && result.error==error && result.heading==heading && result.confidence==confidence);
    CHECK(result.ring.path[TRACK_PATH_NORMAL].valid && !result.ring.junction && !result.ring.zore360_ready);
    CHECK(!result.ring_candidate && !result.ring_direction && !vision_state.ready);
    ring_control_state_t state;dirty(&state);
    ring_control_tick(&state,1,1,100.f,100.f,1000);clear(&state);
    for(unsigned kind=TRACK_RING_270;kind<=TRACK_RING_360;++kind) {
        track_ring_observation_t v;memset(&v,0,sizeof(v));
        v.path[0]=(track_path_observation_t){123,-45,1,90};v.type_hint=(uint8_t)kind;
        v.junction=v.left_open=v.right_open=v.zore360_ready=v.zore360_match=1;v.entry_score=100;v.direction_hint=1;
        dirty(&state);ring_control_observe(&state,&v,1000,1);clear(&state);
        dirty(&state);track_path_observation_t p=ring_control_select_path(&state,&v);clear(&state);
        CHECK(p.error==123 && p.heading==-45 && p.valid && p.confidence==90);
        CHECK(!ring_control_guidance_active(&state));
        dirty(&state);float angle=88.f,speed=123.f;
        ring_control_360_motion(&state,87.f,99.f,75.f,120.f,&angle,&speed);clear(&state);
        CHECK(angle==88.f && speed==123.f);
        track_control_state_t direct,through;track_control_init(&direct,NULL);track_control_init(&through,NULL);
        for(unsigned t=20;t<=100;t+=20) {
            track_control_observe(&direct,123,-45,90,1,t);
            track_control_observe(&through,p.error,p.heading,p.confidence,p.valid,t);
        }
        CHECK(memcmp(&direct,&through,sizeof(direct))==0);
    }
    puts("PASS rings OFF: 270/360 admission blocked, dirty state cleared, ordinary observation/control identical, motion untouched");
    return 0;
}
