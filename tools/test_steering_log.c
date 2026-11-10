#include "steering_log.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
int main(void) {
    steering_log_record_t r={0}, out;
    assert(sizeof(r)==72U && STEERING_LOG_CAPACITY==14563U);
    assert(!steering_log_info().count);
    r.running=1U;
    steering_log_tick(&r);
    assert(!steering_log_info().count);
    steering_log_start();
    r.ms=100U; steering_log_tick(&r);
    r.ms=110U; steering_log_tick(&r);
    assert(steering_log_info().count==1U);
    r.ms=120U; steering_log_tick(&r);
    assert(!steering_log_read(1U,0U,&out));
    r.ms=130U; r.running=0U; r.stop_reason=6U;
    r.ring_stage=6;r.ring_guide=20;r.ring_source=2;r.ring_direction=-1;
    r.bridge_seen=1;r.bridge_passed=1;r.exit_armed=1;r.transition_frames=2;
    steering_log_tick(&r);
    assert(steering_log_info().count==3U && !steering_log_info().active);
    assert(steering_log_read(1U,2U,&out) && out.stop_reason==6U && out.ms==130U);
    assert(out.ring_stage==6 && out.ring_guide==20 && out.ring_source==2 && out.ring_direction==-1);
    assert(out.bridge_seen && out.bridge_passed && out.exit_armed && out.transition_frames==2);
    r.ms=140U; steering_log_tick(&r); assert(steering_log_info().count==3U);
    steering_log_start(); r.running=1U;
    for(uint32_t i=0;i<STEERING_LOG_CAPACITY+10U;++i) { r.ms=i*20U; steering_log_tick(&r); }
    r.ms+=10U; r.running=0U; steering_log_tick(&r);
    assert(steering_log_info().overwritten==11U);
    assert(!steering_log_read(1U,0U,&out));
    assert(steering_log_read(2U,0U,&out) && out.ms==220U);
    assert(steering_log_read(2U,STEERING_LOG_CAPACITY-1U,&out) && out.ms==r.ms);
    assert(!steering_log_read(2U,STEERING_LOG_CAPACITY,&out));
    steering_log_start(); r.running=1U; r.ms=UINT32_MAX-9U; steering_log_tick(&r);
    r.ms=0U; steering_log_tick(&r); assert(steering_log_info().count==1U);
    r.ms=10U; steering_log_tick(&r); assert(steering_log_info().count==2U);
    r.running=0U; steering_log_tick(&r);
    char csv[512]; int n=steering_log_csv(csv,sizeof(csv),&r);
    assert(n>0 && (size_t)n==strlen(csv));
    unsigned commas=0; for(char *p=csv;*p;++p) if(*p==',')++commas;
    assert(commas==41U);
    assert(strstr(csv,",6,20,2,-1,1,1,1,2,0,0,0,0,0,0,0,0,0\r\n"));
    assert(steering_log_csv(csv,4U,&r)>=4);
    puts("PASS: 50Hz, start/stop, freeze, wrap/overwrite, session isolation, timestamp overflow, CSV");
    return 0;
}
