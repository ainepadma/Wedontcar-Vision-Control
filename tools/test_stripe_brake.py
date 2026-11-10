"""Test detector/hold and actual ISR brake integration using hardware mocks."""
from pathlib import Path
import subprocess

root=Path(__file__).resolve().parents[1]
dest=root/'demo/mdk/Objects/stripe-brake-tests'
dest.mkdir(parents=True,exist_ok=True)
s=(root/'demo/user/src/isr.c').read_text(encoding='utf-8')
button=s[s.index('        if (button_pressed && !stripe_brake.stopped)'):s.index('        // 获取编码器读数')]
observe=s[s.index('        /* C14 enables stripe arming'):s.index('        if (protection_enabled && ring_control.fault)')]
stop=s[s.index('        /* This runs every control tick'):s.index('        if (ring_control_guidance_active')]
apply=s[s.index('static void stripe_brake_apply'):s.index('static volatile uint32_t control_clock_ms')]
drive=s[s.index('        if (stripe_brake.stopped)\n        {'):s.index('        else if (motor_enabled)')]
fixture=r'''
#include "track_stripe_brake.h"
#include "track_control.h"
#include "track_stop_diagnostics.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#define SERVO_CENTER_TEST_MODE 0
#define SPEED_BENCH_MODE 0
#define MOTOR1_PWM 0
#define MOTOR2_PWM 1
#define SERVO_MOTOR_PWM 2
#define MOTOR1_DIR 0
#define MOTOR2_DIR 1
#define MOTOR1_FORWARD_DIR_LEVEL 0
#define MOTOR2_FORWARD_DIR_LEVEL 0
#define SERVO_MOTOR_DUTY(x) ((x)*10.0f)
static stripe_brake_state_t stripe_brake;
static track_control_state_t track_control;
static struct { stripe_brake_observation_t stripe; uint32_t observed_ms; } track_mailbox;
static uint32_t control_clock_ms;
static uint8_t track_stripe_brake_status,button_pressed,motor_feedback_fault,protection_enabled;
static float stripe_brake_servo_angle,speed_pwm_l,speed_pwm_r,speed_pwm,speed_target;
static int16_t encoder_data_1,encoder_data_2,track_servo_tenths;
static unsigned duty[3],direction[2],resets,starts,ring_control;
static int speed_pid_l,speed_pid_r;
static track_stop_snapshot_t record;
static void pwm_set_duty(unsigned channel,unsigned value) {duty[channel]=value;}
static void gpio_set_level(unsigned pin,unsigned value) {
    assert(duty[pin]==0U);direction[pin]=value;
}
static void Pid_Reset(int *pid) {*pid=0;++resets;}
static void ring_control_init(unsigned *ring) {++*ring;}
static void steering_log_start(void) {++starts;}
''' + apply + r'''
static void tick(uint8_t fresh_ring_frame) {
    uint8_t was_running=track_control.running;
    control_clock_ms+=10U;
    if(fresh_ring_frame)track_mailbox.observed_ms=control_clock_ms;
''' + button + observe + stop + r'''
    track_stop_record(&record,(uint8_t)(was_running || stripe_just_latched),output.running,
        output.stop_reason,0U,0U,control_clock_ms,encoder_data_1,encoder_data_2,
        speed_target,speed_pwm_l,speed_pwm_r);
''' + drive + r'''
    assert(!stripe_brake.stopped || (!output.running && output.target_speed==0.0f));
    button_pressed=0U;
}
static void initialize(void) {
    track_control_init(&track_control,NULL);
    track_control_set_loss_stop(&track_control,0U);
    stripe_brake_init(&stripe_brake);
    protection_enabled=1U;
    memset(&record,0,sizeof(record));
    memset(duty,0,sizeof(duty));
    encoder_data_1=encoder_data_2=0;
    track_mailbox.stripe=(stripe_brake_observation_t){1,1,1,0,0,1,40,100,150,100,60};
}
int main(void) {
    initialize();
    protection_enabled=0U;
    track_mailbox.stripe.near=1U;
    for(unsigned i=0U;i<5U;++i)tick(1U);
    assert(!stripe_brake.armed && !stripe_brake.stopped && !stripe_brake.arm_votes);
    protection_enabled=1U;
    track_mailbox.stripe.near=0U;
    tick(1U);tick(1U);assert(stripe_brake.arm_votes==2U);
    protection_enabled=0U;tick(0U);
    assert(!stripe_brake.arm_votes && !stripe_brake.armed);
    protection_enabled=1U;
    tick(1U);for(unsigned i=0U;i<10U;++i)tick(0U);
    assert(!stripe_brake.armed && stripe_brake.arm_votes==1U);
    tick(1U);tick(1U);assert(track_stripe_brake_status==1U);
    assert(!track_control.running && duty[0]==0U && duty[1]==0U);
    for(unsigned i=0U;i<100U;++i) {track_mailbox.stripe.visible=0U;tick(1U);}
    assert(track_stripe_brake_status==1U);
    track_mailbox.stripe.visible=track_mailbox.stripe.near=1U;
    encoder_data_1=20;encoder_data_2=10;
    tick(1U);
    assert(track_stripe_brake_status==2U && track_control.stop_reason==TRACK_STOP_STRIPE);
    assert(record.sequence==1U && record.reason==TRACK_STOP_STRIPE);
    assert(duty[0]==700U && duty[1]==350U && direction[0]==1U && direction[1]==1U);
    assert(duty[2]==(unsigned)(track_control.servo_angle*10.0f));
    assert(resets==2U);
    encoder_data_1=encoder_data_2=0;
    protection_enabled=0U;tick(0U);assert(stripe_brake.stopped);
    button_pressed=1U;tick(0U);
    assert(!track_control.running && starts==0U && ring_control==0U);
    encoder_data_1=10;encoder_data_2=-10;tick(0U);
    assert(duty[0]==470U && direction[0]==1U && duty[1]==470U && direction[1]==0U);
    encoder_data_1=encoder_data_2=0;tick(0U);
    assert(duty[0]==120U && duty[1]==120U);
    for(unsigned i=0U;i<100U;++i)tick(0U);
    assert(record.sequence==1U && track_stripe_brake_status==2U);
    puts("PASS actual ISR: C13 never pressed, misses, single P4, LAST, opposing signed PWM, held servo, C13 blocked, no-frame latch");

    initialize();
    track_control.servo_angle=91.0f;
    button_pressed=1U;tick(1U);assert(track_control.running);
    tick(1U);tick(1U);assert(track_stripe_brake_status==1U);
    track_mailbox.stripe.near=1U;tick(1U);
    assert(!track_control.running && record.reason==TRACK_STOP_STRIPE);
    unsigned held=duty[2];
    track_control.servo_angle=74.0f;tick(0U);
    assert(duty[2]==held);
    button_pressed=1U;tick(0U);assert(!track_control.running);
    puts("PASS actual ISR: normal C13 still starts before stripe lock; driving stop freezes servo and rejects restart");
    return 0;
}
'''
(dest/'stripe_brake_isr.c').write_text(fixture,encoding='utf-8')
for name,test in [('stripe_brake_unit',root/'tools/test_stripe_brake.c'),
                  ('stripe_brake_isr',dest/'stripe_brake_isr.c')]:
    exe=dest/(name+'.exe')
    subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror','-Wconversion',
        '-Wshadow','-pedantic','-O2','-I',str(root/'demo/user/inc'),str(test),
        str(root/'demo/user/src/track_stripe_brake.c'),
        str(root/'demo/user/src/track_control.c'),'-lm','-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
