#ifndef _PID_H
#define _PID_H

/* Shared 100-Hz wheel/button/tracking tick; do not change this in isolation.
 * PID t is measured in multiples of this period: the firmware passes 1.0f.
 * Targets and encoder feedback are counts per 10-ms sample, not physical speed.
 */
#define SPEED_CONTROL_PERIOD_MS (10U)

typedef struct
{
    float                kp;                    //P
    float                ki;                    //I
    float                kd;                    //D
    float                imax;                  // integral state limit: sum(error * t), not PWM

    float                out_p;                 //KP输出
    float                out_i;                 //KI输出
    float                out_d;                 //KD输出
    float                out;                   //pid输出
    float                outmin;                // actual actuator lower bound (must include zero)
    float                outmax;                // actual actuator upper bound

    float                integrator;            //< 积分值
    float                last_error;            //< 上次误差
    float                last_derivative;       //< 上次误差与上上次误差之差
    unsigned long        last_t;                //< 上次时间
}pid_param_t;

extern pid_param_t speed_pid_l;  // 电机PID
extern pid_param_t speed_pid_r;  // 电机PID
extern float speed_KP, speed_KI,speed_KD, speed_IMAX, speed_OUTMAX;
extern volatile float speed_target;             // common target, counts per 10-ms sample
extern volatile float speed_real_l, speed_real_r;
extern volatile float speed_pwm_l, speed_pwm_r;  // applied forward PWM, updated by the ISR
extern volatile float speed_real;               // compatibility view: right-wheel measurement
extern volatile float speed_pwm;                // compatibility view: right-wheel applied PWM


void My_Pid_Init(void);                         // power-on only; initializes both wheel parameters
void Pid_Reset(pid_param_t *pid);               // clear history/output, preserve tuning and limits
void Pid_Param_Init(pid_param_t * pid, float kp, float ki, float kd, float imax, float outmax);
/* Finite min <= 0 <= max required; an invalid interval disables output (0..0).
 * The original initializer remains symmetric; wheels explicitly use 0..outmax.
 */
void Pid_Set_Output_Limits(pid_param_t *pid, float minimum, float maximum);

float constrain_float(float amt, float low, float high);
short constrain_short(short amt, short low, short high);


/* Position form: I state = sum(error*t), D = delta(error)/t.
 * Invalid input/state or arithmetic overflow resets history and returns zero.
 * Ki == 0 discards integral history. Gains must be retuned if sample units change.
 */
float PidLocCtrl(pid_param_t * pid, float error, float t);
/* Incremental form keeps accumulated output at the same actual actuator limits.
 * Its finite-difference D assumes a fixed sample interval; the wheel loop uses
 * position form and does not enable incremental control automatically.
 */
float PidIncCtrl(pid_param_t * pid, float error , float t);


#endif
