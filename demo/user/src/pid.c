#include "pid.h"
#include <math.h>

// 10ms 位置式 PI：小幅积分补偿持续负载，停止/故障时清除历史。
// imax 是积分状态限幅；Ki=0.2 时积分最多贡献 1000，即满量程的10%。
// 保留原 P、D、50%总输出上限；无反馈保护见 PIT 中断。
float speed_KP = 50.0f, speed_KI = 0.2f, speed_KD = 0.0f;
float speed_IMAX = 5000.0f, speed_OUTMAX = 5000.0f;
volatile float speed_target = 0.0f;
volatile float speed_real_l = 0.0f, speed_real_r = 0.0f;
volatile float speed_pwm_l = 0.0f, speed_pwm_r = 0.0f;
volatile float speed_real = 0.0f, speed_pwm = 0.0f;

pid_param_t speed_pid_l;  // 电机PID
pid_param_t speed_pid_r;  // 电机PID

/*******************************************************************************
* 函 数 名         : My_Pid_Init
* 函数功能         : PID初始化	
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void My_Pid_Init(void)
{
    Pid_Param_Init(&speed_pid_l, speed_KP, speed_KI,speed_KD, speed_IMAX, speed_OUTMAX);
    Pid_Param_Init(&speed_pid_r, speed_KP, speed_KI,speed_KD, speed_IMAX, speed_OUTMAX);
    Pid_Set_Output_Limits(&speed_pid_l, 0.0f, speed_OUTMAX);
    Pid_Set_Output_Limits(&speed_pid_r, 0.0f, speed_OUTMAX);
    speed_target = 0.0f;
    speed_real_l = speed_real_r = speed_real = 0.0f;
    speed_pwm_l = speed_pwm_r = speed_pwm = 0.0f;
}

void Pid_Reset(pid_param_t *pid)
{
    if (!pid) return;
    pid->out_p = 0.0f;
    pid->out_i = 0.0f;
    pid->out_d = 0.0f;
    pid->out = 0.0f;
    pid->integrator = 0.0f;
    pid->last_error = 0.0f;
    pid->last_derivative = 0.0f;
    pid->last_t = 0;
}


/*******************************************************************************
* 函 数 名         : Pid_Param_Init
* 函数功能         : PID参数初始化	
* 输    入         : imax: sum(error * t)积分状态限幅，不是PWM积分输出限幅
* 输    出         : 无
*******************************************************************************/
void Pid_Param_Init(pid_param_t * pid, float kp, float ki, float kd, float imax, float outmax)
{
    if (!pid) return;
    pid->kp        = kp;
    pid->ki        = ki;
    pid->kd        = kd;
    pid->imax      = fabsf(imax);
    pid->outmin    = -fabsf(outmax);
    pid->outmax    = fabsf(outmax);
    Pid_Reset(pid);
    Pid_Set_Output_Limits(pid, pid->outmin, pid->outmax);
}

void Pid_Set_Output_Limits(pid_param_t *pid, float minimum, float maximum)
{
    if (!pid) return;
    if (!isfinite(minimum) || !isfinite(maximum) || minimum > 0.0f || maximum < 0.0f)
    {
        pid->outmin = pid->outmax = 0.0f;
        Pid_Reset(pid);
        return;
    }
    pid->outmin = minimum;
    pid->outmax = maximum;
    if (!isfinite(pid->out)) Pid_Reset(pid);
    pid->out = constrain_float(pid->out, minimum, maximum);
}

static int pid_is_valid(const pid_param_t *pid, float error, float t)
{
    return pid && isfinite(error) && isfinite(t) && t > 0.0f &&
           isfinite(pid->kp) && isfinite(pid->ki) && isfinite(pid->kd) &&
           isfinite(pid->imax) && pid->imax >= 0.0f &&
           isfinite(pid->outmin) && pid->outmin <= 0.0f &&
           isfinite(pid->outmax) && pid->outmax >= 0.0f &&
           isfinite(pid->out) && isfinite(pid->integrator) &&
           isfinite(pid->last_error) && isfinite(pid->last_derivative);
}

static float pid_fail_safe(pid_param_t *pid)
{
    Pid_Reset(pid);
    return 0.0f;
}

/*******************************************************************************
* 函 数 名         : PidLocCtrl
* 函数功能	   : 位置式PID控制
* 输    入         : pid, error, t
* 输    出         : float
*******************************************************************************/
float PidLocCtrl(pid_param_t * pid, float error, float t)
{
    float previous_integral, candidate_integral, delta_integral, raw_output;
    float proportional, integral, derivative;
    if (!pid_is_valid(pid, error, t)) return pid_fail_safe(pid);

    previous_integral = pid->ki == 0.0f ? 0.0f :
                        constrain_float(pid->integrator, -pid->imax, pid->imax);
    candidate_integral = previous_integral;
    if (pid->ki != 0.0f)
    {
        candidate_integral += error * t;
        if (!isfinite(candidate_integral)) return pid_fail_safe(pid);
        candidate_integral = constrain_float(candidate_integral, -pid->imax, pid->imax);
    }
    proportional = pid->kp * error;
    integral = pid->ki * candidate_integral;
    derivative = pid->kd * ((error - pid->last_error) / t);
    delta_integral = pid->ki * (candidate_integral - previous_integral);
    raw_output = proportional + integral + derivative;
    if (!isfinite(proportional) || !isfinite(integral) || !isfinite(derivative) ||
        !isfinite(delta_integral) || !isfinite(raw_output)) return pid_fail_safe(pid);

    /* Conditional integration: freeze a step that pushes an already saturated
     * actuator farther out. If this step first crosses the boundary, keep the
     * portion that reaches it; rejecting the whole step would starve a pure-I
     * controller whose first error is larger than the actuator range.
     * The opposite step is allowed to unwind immediately.
     */
    if ((raw_output > pid->outmax && delta_integral > 0.0f) ||
        (raw_output < pid->outmin && delta_integral < 0.0f))
    {
        float previous_output = proportional + pid->ki * previous_integral + derivative;
        float boundary = raw_output > pid->outmax ? pid->outmax : pid->outmin;
        if (!isfinite(previous_output)) return pid_fail_safe(pid);
        if ((delta_integral > 0.0f && previous_output < boundary) ||
            (delta_integral < 0.0f && previous_output > boundary))
        {
            float boundary_integral = previous_integral + (boundary - previous_output) / pid->ki;
            float minimum = previous_integral < candidate_integral ? previous_integral : candidate_integral;
            float maximum = previous_integral > candidate_integral ? previous_integral : candidate_integral;
            if (!isfinite(boundary_integral)) return pid_fail_safe(pid);
            candidate_integral = constrain_float(boundary_integral, minimum, maximum);
        }
        else
        {
            candidate_integral = previous_integral;
        }
        integral = pid->ki * candidate_integral;
        raw_output = proportional + integral + derivative;
        if (!isfinite(integral) || !isfinite(raw_output)) return pid_fail_safe(pid);
    }
    pid->integrator = candidate_integral;
    pid->out_p = proportional;
    pid->out_i = integral;
    pid->out_d = derivative;
    pid->last_derivative = error - pid->last_error;
    pid->last_error = error;
    pid->out = constrain_float(raw_output, pid->outmin, pid->outmax);
    return pid->out;
}


/*******************************************************************************
* 函 数 名         : PidIncCtrl
* 函数功能	   : 增量式PID控制
* 输    入         : pid, error, t
* 输    出         : float
*******************************************************************************/
float PidIncCtrl(pid_param_t * pid, float error, float t)
{
    float delta_error, proportional, integral, derivative, raw_output;
    if (!pid_is_valid(pid, error, t)) return pid_fail_safe(pid);
    delta_error = error - pid->last_error;
    proportional = pid->kp * delta_error;
    integral = pid->ki * error * t;
    derivative = pid->kd * ((delta_error - pid->last_derivative) / t);
    raw_output = constrain_float(pid->out, pid->outmin, pid->outmax) +
                 proportional + integral + derivative;
    if (!isfinite(delta_error) || !isfinite(proportional) || !isfinite(integral) ||
        !isfinite(derivative) || !isfinite(raw_output)) return pid_fail_safe(pid);

    pid->integrator = 0.0f; /* Incremental control has no separate I accumulator. */
    pid->out_p = proportional;
    pid->out_i = integral;
    pid->out_d = derivative;
    pid->last_derivative = delta_error;
    pid->last_error = error;
    /* No hidden +/-outmax accumulator behind a later forward-only PWM clamp. */
    pid->out = constrain_float(raw_output, pid->outmin, pid->outmax);
    return pid->out;
}

/*******************************************************************************
* 函 数 名         : constrain_float
* 函数功能         : 浮点型数限幅
* 输    入         : amt,low,high
* 输    出         : float
*******************************************************************************/
float constrain_float(float amt, float low, float high)
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

/*******************************************************************************
* 函 数 名         : constrain_short
* 函数功能         : 短整型数限幅
* 输    入         : amt,low,high
* 输    出         : short
*******************************************************************************/
short constrain_short(short amt, short low, short high)
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

