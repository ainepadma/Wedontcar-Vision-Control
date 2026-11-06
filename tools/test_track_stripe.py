"""Run detector tests and the actual ISR stripe/PWM-clear blocks with host mocks."""
from pathlib import Path
import subprocess

root = Path(__file__).resolve().parents[1]
dest = root / 'demo/mdk/Objects/stripe-tests'
dest.mkdir(parents=True, exist_ok=True)
source = (root / 'demo/user/src/isr.c').read_text(encoding='utf-8')
stripe = source[source.index('        /* Seven-stripe stop'):source.index('        if (protection_enabled && ring_control.fault')]
clear = source[source.index('        static float ramped_counts;'):source.index('        float log_target_left')]
disabled = source[source.index('            // 停机是撤销驱动 PWM'):source.index('        /* Capture applied outputs')]
# Include just the stopped body, excluding the outer branch's closing brace.
disabled = disabled.rsplit('        }', 1)[0]
fixture = r'''
#include "track_stripe.h"
#include "track_control.h"
#include "motor_switch.h"
#include <assert.h>
#include <stdio.h>
#define SERVO_CENTER_TEST_MODE 0
#define SPEED_BENCH_MODE 0
#define MOTOR1_PWM 0
#define MOTOR2_PWM 1
#define SERVO_MOTOR_PWM 2
static track_stripe_state_t stripe_control;
static track_control_state_t track_control;
static struct {track_stripe_observation_t stripe; uint32_t observed_ms;} track_mailbox;
static uint32_t control_clock_ms;
static uint8_t track_stripe_status,motor_enabled,button_pressed;
static float speed_target,speed_pwm_l,speed_pwm_r,speed_pwm;
static unsigned duty[3],resets;
static int speed_pid_l,speed_pid_r;
static motor_feedback_watchdog_t motor_feedback_watchdog;
static motor_overspeed_watchdog_t motor_overspeed_watchdog;
static void pwm_set_duty(unsigned channel,unsigned value) {duty[channel]=value;}
static void Pid_Reset(int *pid) {*pid=0;++resets;}
static void tick(uint8_t fresh_ring_frame) {
 uint8_t was_running=motor_enabled;
 control_clock_ms+=10;
 if(fresh_ring_frame)track_mailbox.observed_ms=control_clock_ms;
''' + stripe + r'''
 track_control_output_t output=track_control_tick(&track_control,control_clock_ms);
 motor_enabled=output.running;
 const float requested_counts=output.target_speed;
''' + clear + r'''
 if(!motor_enabled) {
''' + disabled + r'''
 }
}
int main(void) {
 track_control_init(&track_control,NULL);track_control_set_loss_stop(&track_control,0);
 track_stripe_init(&stripe_control);track_control_toggle(&track_control,0);
 motor_enabled=1;speed_pid_l=speed_pid_r=1;
 speed_pwm_l=speed_pwm_r=speed_pwm=600;
 duty[0]=duty[1]=600;
 track_mailbox.stripe=(track_stripe_observation_t){1,1,1,0,38,51,150,42};
 tick(1);assert(stripe_control.arm_votes==1 && motor_enabled);
 for(unsigned i=0;i<10;++i)tick(0);
 assert(stripe_control.arm_votes==1 && !stripe_control.armed);
 tick(1);tick(1);assert(stripe_control.armed && motor_enabled && speed_target>0);
 track_mailbox.stripe.near=1;
 tick(1);assert(motor_enabled);tick(0);assert(motor_enabled);
 tick(1);
 assert(!motor_enabled && track_control.stop_reason==TRACK_STOP_STRIPE);
 assert(speed_target==0 && speed_pwm_l==0 && speed_pwm_r==0 && speed_pwm==0);
 assert(duty[0]==0 && duty[1]==0 && duty[2]==0);
 assert(resets==2 && speed_pid_l==0 && speed_pid_r==0);
 for(unsigned i=0;i<20;++i)tick(0);
 assert(!motor_enabled && speed_target==0 && duty[0]==0 && duty[1]==0);
 puts("PASS actual ISR: new-frame votes only, C14 off, latched zero target/PWM, both PID resets");
 return 0;
}
'''
(dest / 'stripe_isr.c').write_text(fixture, encoding='utf-8')
for name, test in [('stripe_unit', root / 'tools/test_track_stripe.c'),
                   ('stripe_isr', dest / 'stripe_isr.c')]:
    exe = dest / (name + '.exe')
    subprocess.run(['gcc', '-std=c99', '-Wall', '-Wextra', '-Werror', '-Wconversion',
                    '-Wshadow', '-pedantic', '-O2', '-I', str(root / 'demo/user/inc'),
                    str(test), str(root / 'demo/user/src/track_stripe.c'),
                    str(root / 'demo/user/src/track_control.c'), '-lm', '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
