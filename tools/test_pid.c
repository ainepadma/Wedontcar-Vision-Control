#include "pid.h"
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
    exit(EXIT_FAILURE); } } while (0)

static void near_value(float actual, float expected)
{
    CHECK(isfinite(actual));
    CHECK(fabsf(actual - expected) < 0.001f);
}

static void reset_state_is_zero(const pid_param_t *pid)
{
    near_value(pid->out_p, 0.0f);
    near_value(pid->out_i, 0.0f);
    near_value(pid->out_d, 0.0f);
    near_value(pid->out, 0.0f);
    near_value(pid->integrator, 0.0f);
    near_value(pid->last_error, 0.0f);
    near_value(pid->last_derivative, 0.0f);
    CHECK(pid->last_t == 0UL);
}

static void test_reset_preserves_parameters(void)
{
    pid_param_t pid;
    Pid_Param_Init(&pid, 37.0f, 0.5f, 0.25f, 77.0f, 3000.0f);
    Pid_Set_Output_Limits(&pid, 0.0f, 2500.0f);
    (void)PidLocCtrl(&pid, 3.0f, 1.0f);
    pid.last_t = 123UL;
    Pid_Reset(&pid);
    reset_state_is_zero(&pid);
    near_value(pid.kp, 37.0f);
    near_value(pid.ki, 0.5f);
    near_value(pid.kd, 0.25f);
    near_value(pid.imax, 77.0f);
    near_value(pid.outmin, 0.0f);
    near_value(pid.outmax, 2500.0f);
    Pid_Reset(NULL);
}

static void test_power_on_and_independent_wheels(void)
{
    My_Pid_Init();
    CHECK(SPEED_CONTROL_PERIOD_MS == 10U);
    near_value(speed_target, 0.0f);
    near_value(speed_pid_l.kp, 50.0f);
    near_value(speed_pid_r.kp, 50.0f);
    near_value(speed_pid_l.ki, 0.2f);
    near_value(speed_pid_r.ki, 0.2f);
    near_value(speed_pid_l.kd, 0.0f);
    near_value(speed_pid_r.kd, 0.0f);
    near_value(speed_pid_l.imax, 5000.0f);
    near_value(speed_pid_r.imax, 5000.0f);
    near_value(speed_pid_l.outmin, 0.0f);
    near_value(speed_pid_r.outmax, 2000.0f);
    near_value(speed_real_l, 0.0f);
    near_value(speed_real_r, 0.0f);
    near_value(speed_pwm_l, 0.0f);
    near_value(speed_pwm_r, 0.0f);
    near_value(speed_real, 0.0f);
    near_value(speed_pwm, 0.0f);
    speed_pid_l.kp = 37.0f;
    speed_pid_r.kp = 61.0f;
    speed_pid_l.ki = speed_pid_r.ki = 0.0f;
    near_value(PidLocCtrl(&speed_pid_l, 2.0f, 1.0f), 74.0f);
    near_value(PidLocCtrl(&speed_pid_r, 3.0f, 1.0f), 183.0f);
    near_value(speed_pid_l.last_error, 2.0f);
    near_value(speed_pid_r.last_error, 3.0f);
    Pid_Reset(&speed_pid_l);
    near_value(speed_pid_l.kp, 37.0f);
    near_value(speed_pid_r.kp, 61.0f);
    near_value(speed_pid_r.out, 183.0f);
    near_value(speed_pid_r.last_error, 3.0f);
}

static void test_integral_state_limit_and_dt(void)
{
    pid_param_t pid;
    unsigned iteration;
    Pid_Param_Init(&pid, 0.0f, 2.0f, 0.0f, 5.0f, 1000.0f);
    for (iteration = 0U; iteration < 100U; ++iteration)
        (void)PidLocCtrl(&pid, 2.0f, 1.0f);
    near_value(pid.integrator, 5.0f);
    near_value(pid.out_i, 10.0f);
    for (iteration = 0U; iteration < 100U; ++iteration)
        (void)PidLocCtrl(&pid, -2.0f, 1.0f);
    near_value(pid.integrator, -5.0f);
    near_value(pid.out_i, -10.0f);
    Pid_Param_Init(&pid, 0.0f, 2.0f, 0.0f, 100.0f, 1000.0f);
    near_value(PidLocCtrl(&pid, 4.0f, 0.5f), 4.0f);
    near_value(PidLocCtrl(&pid, 4.0f, 2.0f), 20.0f);
    near_value(pid.integrator, 10.0f);
    Pid_Param_Init(&pid, 0.0f, 0.0f, 2.0f, 100.0f, 1000.0f);
    near_value(PidLocCtrl(&pid, 4.0f, 0.5f), 16.0f);
    near_value(PidLocCtrl(&pid, 6.0f, 2.0f), 2.0f);
}

static void test_conditional_integration_and_release(void)
{
    pid_param_t pid;
    unsigned iteration;
    Pid_Param_Init(&pid, 1.0f, 1.0f, 0.0f, 1000.0f, 10.0f);
    near_value(PidLocCtrl(&pid, 2.0f, 1.0f), 4.0f);
    for (iteration = 0U; iteration < 100U; ++iteration)
        near_value(PidLocCtrl(&pid, 20.0f, 1.0f), 10.0f);
    near_value(pid.integrator, 2.0f);
    near_value(PidLocCtrl(&pid, -1.0f, 1.0f), 0.0f);
    near_value(pid.integrator, 1.0f);
    for (iteration = 0U; iteration < 100U; ++iteration)
        near_value(PidLocCtrl(&pid, -20.0f, 1.0f), -10.0f);
    near_value(pid.integrator, 1.0f);
    near_value(PidLocCtrl(&pid, 1.0f, 1.0f), 3.0f);
    near_value(pid.integrator, 2.0f);

    Pid_Reset(&pid);
    Pid_Set_Output_Limits(&pid, 0.0f, 10.0f);
    for (iteration = 0U; iteration < 100U; ++iteration)
        near_value(PidLocCtrl(&pid, -20.0f, 1.0f), 0.0f);
    near_value(pid.integrator, 0.0f);
    near_value(PidLocCtrl(&pid, 1.0f, 1.0f), 2.0f);
}

static void test_p_only_matches_existing_drive(void)
{
    pid_param_t pid;
    int error;
    My_Pid_Init();
    pid = speed_pid_l;
    pid.ki = 0.0f; /* Preserve the former P-only law as an explicit algorithm regression. */
    for (error = -200; error <= 200; ++error)
    {
        float expected = constrain_float(50.0f * (float)error, 0.0f, 2000.0f);
        near_value(PidLocCtrl(&pid, (float)error, 1.0f), expected);
        near_value(pid.integrator, 0.0f);
    }
    pid.integrator = 123.0f;
    near_value(PidLocCtrl(&pid, 2.0f, 1.0f), 100.0f);
    near_value(pid.integrator, 0.0f);
    pid.ki = 1.0f;
    near_value(PidLocCtrl(&pid, 0.0f, 1.0f), 0.0f);
}

static void test_production_traction_pi(void)
{
    unsigned iteration;
    float previous_output;
    My_Pid_Init();
    /* Use the real production initializer, not test-only substitute gains.
     * Straight/curve targets are 120/80 counts per 10-ms sample. */
    near_value(speed_pid_l.kp, 50.0f);
    near_value(speed_pid_l.ki, 0.2f);
    near_value(speed_pid_l.kd, 0.0f);
    near_value(speed_pid_l.imax, 5000.0f);
    near_value(speed_pid_l.outmin, 0.0f);
    near_value(speed_pid_l.outmax, 2000.0f);
    near_value(PidLocCtrl(&speed_pid_l, 120.0f, 1.0f), 2000.0f);
    near_value(PidLocCtrl(&speed_pid_r, 80.0f, 1.0f), 2000.0f);
    near_value(speed_pid_l.integrator, 0.0f);
    near_value(speed_pid_r.integrator, 0.0f);
    near_value(PidLocCtrl(&speed_pid_l, 2.0f, 1.0f), 100.4f);
    near_value(speed_pid_r.out, 2000.0f);
    near_value(speed_pid_r.integrator, 0.0f);
    Pid_Reset(&speed_pid_l);
    reset_state_is_zero(&speed_pid_l);
    near_value(speed_pid_l.ki, 0.2f);
    near_value(speed_pid_r.out, 2000.0f);

    /* A small sustained tracking error builds bounded load compensation. */
    previous_output = 0.0f;
    for (iteration = 0U; iteration < 3000U; ++iteration)
    {
        float output = PidLocCtrl(&speed_pid_l, 2.0f, 1.0f);
        CHECK(isfinite(output));
        CHECK(output >= previous_output);
        CHECK(output <= 1100.0f);
        CHECK(speed_pid_l.integrator <= 5000.0f);
        CHECK(speed_pid_l.out_i <= 1000.0f);
        previous_output = output;
    }
    near_value(speed_pid_l.integrator, 5000.0f);
    near_value(speed_pid_l.out_i, 1000.0f);
    near_value(speed_pid_l.out, 1100.0f);
    /* Reaching target keeps the accumulated load compensation, unlike P-only. */
    near_value(PidLocCtrl(&speed_pid_l, 0.0f, 1.0f), 1000.0f);
    /* Overspeed reduces PWM immediately, with no permanent minimum-PWM floor. */
    near_value(PidLocCtrl(&speed_pid_l, -10.0f, 1.0f), 498.0f);
    near_value(PidLocCtrl(&speed_pid_l, -30.0f, 1.0f), 0.0f);
    Pid_Reset(&speed_pid_l);
    reset_state_is_zero(&speed_pid_l);
    near_value(speed_pid_l.kp, 50.0f);
    near_value(speed_pid_l.ki, 0.2f);
    near_value(PidLocCtrl(&speed_pid_l, 0.0f, 1.0f), 0.0f);
    near_value(speed_pid_r.out, 2000.0f);
}

static void test_large_pure_integral_step_reaches_boundary(void)
{
    pid_param_t pid;
    unsigned iteration;
    Pid_Param_Init(&pid, 0.0f, 1.0f, 0.0f, 1000.0f, 10.0f);
    Pid_Set_Output_Limits(&pid, 0.0f, 10.0f);
    near_value(PidLocCtrl(&pid, 20.0f, 1.0f), 10.0f);
    near_value(pid.integrator, 10.0f);
    for (iteration = 0U; iteration < 100U; ++iteration)
        near_value(PidLocCtrl(&pid, 20.0f, 1.0f), 10.0f);
    near_value(pid.integrator, 10.0f);
    near_value(PidLocCtrl(&pid, -3.0f, 1.0f), 7.0f);
    near_value(pid.integrator, 7.0f);
    near_value(PidLocCtrl(&pid, -20.0f, 1.0f), 0.0f);
    near_value(pid.integrator, 0.0f);
    near_value(PidLocCtrl(&pid, 2.0f, 1.0f), 2.0f);

    Pid_Param_Init(&pid, 0.0f, 2.0f, 0.0f, 1000.0f, 10.0f);
    near_value(PidLocCtrl(&pid, -20.0f, 1.0f), -10.0f);
    near_value(pid.integrator, -5.0f);
    near_value(PidLocCtrl(&pid, 2.0f, 1.0f), -6.0f);

    /* Conditional integration follows I contribution, including a negative Ki. */
    Pid_Param_Init(&pid, 0.0f, -2.0f, 0.0f, 1000.0f, 10.0f);
    near_value(PidLocCtrl(&pid, -20.0f, 1.0f), 10.0f);
    near_value(pid.integrator, -5.0f);
    near_value(PidLocCtrl(&pid, 2.0f, 1.0f), 6.0f);
}

static void test_invalid_input_and_finite_outputs(void)
{
    const float invalid_times[] = { 0.0f, -1.0f, NAN, INFINITY, -INFINITY };
    const float invalid_errors[] = { NAN, INFINITY, -INFINITY };
    float (*const controllers[])(pid_param_t *, float, float) = { PidLocCtrl, PidIncCtrl };
    size_t controller, index;
    pid_param_t pid;
    for (controller = 0U; controller < sizeof(controllers) / sizeof(controllers[0]); ++controller)
    {
        for (index = 0U; index < sizeof(invalid_times) / sizeof(invalid_times[0]); ++index)
        {
            Pid_Param_Init(&pid, 37.0f, 0.5f, 0.25f, 77.0f, 3000.0f);
            Pid_Set_Output_Limits(&pid, 0.0f, 2500.0f);
            (void)controllers[controller](&pid, 2.0f, 1.0f);
            near_value(controllers[controller](&pid, 2.0f, invalid_times[index]), 0.0f);
            reset_state_is_zero(&pid);
            near_value(pid.kp, 37.0f);
            near_value(pid.outmax, 2500.0f);
            near_value(pid.outmin, 0.0f);
        }
        for (index = 0U; index < sizeof(invalid_errors) / sizeof(invalid_errors[0]); ++index)
        {
            Pid_Param_Init(&pid, 37.0f, 0.5f, 0.25f, 77.0f, 3000.0f);
            near_value(controllers[controller](&pid, invalid_errors[index], 1.0f), 0.0f);
            reset_state_is_zero(&pid);
        }
        Pid_Param_Init(&pid, FLT_MAX, 0.0f, 0.0f, 77.0f, 3000.0f);
        near_value(controllers[controller](&pid, FLT_MAX, 1.0f), 0.0f);
        reset_state_is_zero(&pid);
        Pid_Param_Init(&pid, 1.0f, 0.5f, 0.25f, 77.0f, 3000.0f);
        pid.integrator = NAN;
        near_value(controllers[controller](&pid, 1.0f, 1.0f), 0.0f);
        reset_state_is_zero(&pid);
        pid.kp = NAN;
        near_value(controllers[controller](&pid, 1.0f, 1.0f), 0.0f);
        reset_state_is_zero(&pid);
        near_value(controllers[controller](NULL, 1.0f, 1.0f), 0.0f);
    }
}

static void test_output_limits_and_incremental_state(void)
{
    pid_param_t pid;
    unsigned iteration;
    Pid_Param_Init(&pid, 0.0f, 1.0f, 0.0f, 1000.0f, 10.0f);
    near_value(pid.outmin, -10.0f);
    Pid_Set_Output_Limits(&pid, 0.0f, 10.0f);
    for (iteration = 0U; iteration < 100U; ++iteration)
        near_value(PidIncCtrl(&pid, 20.0f, 1.0f), 10.0f);
    near_value(pid.out, 10.0f);
    near_value(PidIncCtrl(&pid, -1.0f, 1.0f), 9.0f);
    for (iteration = 0U; iteration < 100U; ++iteration)
        near_value(PidIncCtrl(&pid, -20.0f, 1.0f), 0.0f);
    near_value(pid.out, 0.0f);
    near_value(PidIncCtrl(&pid, 1.0f, 1.0f), 1.0f);
    near_value(pid.integrator, 0.0f);
    Pid_Set_Output_Limits(&pid, 0.0f, 0.5f);
    near_value(pid.out, 0.5f);
    Pid_Set_Output_Limits(&pid, 1.0f, 10.0f);
    reset_state_is_zero(&pid);
    near_value(pid.outmin, 0.0f);
    near_value(pid.outmax, 0.0f);
    near_value(PidIncCtrl(&pid, 100.0f, 1.0f), 0.0f);
    Pid_Set_Output_Limits(&pid, -10.0f, -1.0f);
    reset_state_is_zero(&pid);
    Pid_Set_Output_Limits(&pid, NAN, 10.0f);
    reset_state_is_zero(&pid);
    Pid_Set_Output_Limits(&pid, 0.0f, INFINITY);
    reset_state_is_zero(&pid);
}

int main(void)
{
    test_reset_preserves_parameters();
    test_power_on_and_independent_wheels();
    test_integral_state_limit_and_dt();
    test_conditional_integration_and_release();
    test_p_only_matches_existing_drive();
    test_production_traction_pi();
    test_large_pure_integral_step_reaches_boundary();
    test_invalid_input_and_finite_outputs();
    test_output_limits_and_incremental_state();
    puts("PID host tests passed: production traction PI/reset/independent wheels/dt/integral bounds/anti-windup/invalid input/legacy P-only/incremental actuator limits.");
    return EXIT_SUCCESS;
}
