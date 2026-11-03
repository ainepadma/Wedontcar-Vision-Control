from pathlib import Path
import subprocess
r=Path(__file__).resolve().parent.parent
dest=r/'demo/mdk/Objects/stop-diagnostics-test'
dest.mkdir(parents=True,exist_ok=True)
s=(r/'demo/user/src/isr.c').read_text(encoding='utf-8')
checks=s[s.index('        static uint8_t reverse_ticks_l'):s.index('        // Ring decision selects')]
test=r'''
#include "track_control.h"
#include "motor_switch.h"
#include "track_stop_diagnostics.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static track_control_state_t track_control;
static motor_feedback_watchdog_t motor_feedback_watchdog;
static motor_overspeed_watchdog_t motor_overspeed_watchdog;
static track_stop_snapshot_t record;
static int16_t encoder_data_1,encoder_data_2;
static float speed_pwm_l,speed_pwm_r,speed_target;
static uint8_t motor_feedback_fault;
static uint32_t now;
static void sample(void){
 uint8_t was_running=track_control.running;
 now+=10;
'''+checks+r'''
 track_stop_record(&record,was_running,track_control.running,track_control.stop_reason,
  motor_feedback_fault,0,now,encoder_data_1,encoder_data_2,speed_target,speed_pwm_l,speed_pwm_r);
 if(!track_control.running){motor_feedback_watchdog_reset(&motor_feedback_watchdog);motor_overspeed_watchdog_reset(&motor_overspeed_watchdog);speed_pwm_l=speed_pwm_r=0;}
}
static void start(void){
 const track_control_config_t config={87.f,98.5f,75.5f,TRACK_SPEED_STRAIGHT,TRACK_SPEED_CURVE_MIN,.03f,.0005f,180.f};
 track_control_init(&track_control,&config);track_control_set_loss_stop(&track_control,0);
 motor_feedback_watchdog_reset(&motor_feedback_watchdog);motor_overspeed_watchdog_reset(&motor_overspeed_watchdog);
 sample();memset(&record,0,sizeof(record));now=0;motor_feedback_fault=0;
 track_control_toggle(&track_control,0);assert(track_control.running);
 speed_target=90;speed_pwm_l=speed_pwm_r=600;
}
int main(void){
 start();
 for(unsigned i=0;i<20;++i){track_control_observe(&track_control,0,0,0,0,i*20);assert(track_control_tick(&track_control,i*20).running);}
 assert(track_control_tick(&track_control,5000).running);
 puts("PASS protection=0: repeated lost frames and camera timeout do not stop controller");
 start();encoder_data_1=-3;encoder_data_2=100;
 sample();sample();assert(track_control.running);sample();
 assert(!track_control.running && record.reason==TRACK_STOP_ENCODER && record.wheels==1 && record.ms==30);
 puts("PASS actual ISR: 30ms negative encoder feedback stops with protection=0 (reason5)");
 start();encoder_data_1=0;encoder_data_2=100;
 for(unsigned i=0;i<49;++i)sample();assert(track_control.running);sample();
 assert(!track_control.running && record.reason==TRACK_STOP_NO_FEEDBACK && record.wheels==1 && record.ms==500);
 assert(record.left==0 && record.right==100 && record.target==90 && record.left_pwm==600);
 track_control_toggle(&track_control,now);sample();assert(record.sequence==1 && record.reason==TRACK_STOP_NO_FEEDBACK);
 track_control_stop(&track_control,TRACK_STOP_MANUAL);sample();assert(record.sequence==1);
 puts("PASS actual ISR: 500ms no feedback stops with protection=0 (reason6); pre-clear evidence survives restart/manual stop");
 start();encoder_data_1=201;encoder_data_2=100;
 for(unsigned i=0;i<9;++i)sample();assert(track_control.running);sample();
 assert(!track_control.running && record.reason==TRACK_STOP_OVERSPEED && record.ms==100);
 puts("PASS actual ISR: 100ms feedback>2*target+20 stops with protection=0 (reason7)");
 start();track_control_stop(&track_control,TRACK_STOP_RING);assert(track_control.stop_reason==TRACK_STOP_RING);
 track_stop_record(&record,1,0,TRACK_STOP_RING,0,1,now,0,0,90,600,600);assert(record.reason==TRACK_STOP_RING);
 puts("PASS ring reason8 preserved instead of incorrectly reported as manual stop");
 return 0;
}
'''
(dest/'stop_diagnose_test.c').write_text(test,encoding='utf-8')
subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror','-Wno-misleading-indentation','-pedantic','-I',str(r/'demo/user/inc'),str(dest/'stop_diagnose_test.c'),str(r/'demo/user/src/track_control.c'),'-lm','-o',str(dest/'stop_diagnose_test.exe')],check=True)
p=subprocess.run([str(dest/'stop_diagnose_test.exe')],check=True,text=True,capture_output=True)
print(p.stdout,end='')
