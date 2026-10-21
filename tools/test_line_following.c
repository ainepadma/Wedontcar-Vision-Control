#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "motor_switch.h"
#include "pid.h"
#include "track_control.h"
#include "track_vision.h"

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); \
        exit(EXIT_FAILURE); \
    } \
} while (0)

typedef struct
{
    track_control_state_t control;
    motor_switch_t button;
    motor_feedback_watchdog_t feedback_watchdog;
    track_vision_result_t vision;
    track_control_output_t output;
    pid_param_t pid_left;
    pid_param_t pid_right;
    int16_t feedback_left;
    int16_t feedback_right;
    float measured_left;
    float measured_right;
    float pwm_left;
    float pwm_right;
    uint8_t feedback_fault;
    uint32_t pid_calls;
    uint32_t pid_resets;
    uint32_t now_ms;
    uint32_t button_events;
    uint16_t rows;
    uint16_t cols;
} simulation_t;

static void check_pid_tuning(const simulation_t *sim)
{
    CHECK(sim->pid_left.kp == 37.0f && sim->pid_right.kp == 61.0f);
    CHECK(sim->pid_left.ki == 0.5f && sim->pid_right.ki == 0.75f);
    CHECK(sim->pid_left.kd == 0.25f && sim->pid_right.kd == 0.5f);
    CHECK(sim->pid_left.imax == 2000.0f && sim->pid_right.imax == 1500.0f);
    CHECK(sim->pid_left.outmin == 0.0f && sim->pid_right.outmin == 0.0f);
    CHECK(sim->pid_left.outmax == 5000.0f && sim->pid_right.outmax == 5000.0f);
}

static void check_pid_history_zero(const pid_param_t *pid)
{
    CHECK(pid->out_p == 0.0f && pid->out_i == 0.0f && pid->out_d == 0.0f);
    CHECK(pid->out == 0.0f && pid->integrator == 0.0f);
    CHECK(pid->last_error == 0.0f && pid->last_derivative == 0.0f);
    CHECK(pid->last_t == 0UL);
}

static void simulation_init(simulation_t *sim, uint16_t rows, uint16_t cols)
{
    memset(sim, 0, sizeof(*sim));
    sim->rows = rows;
    sim->cols = cols;
    sim->feedback_left = 4;
    sim->feedback_right = 9;
    Pid_Param_Init(&sim->pid_left, 37.0f, 0.5f, 0.25f, 2000.0f, 5000.0f);
    Pid_Param_Init(&sim->pid_right, 61.0f, 0.75f, 0.5f, 1500.0f, 5000.0f);
    Pid_Set_Output_Limits(&sim->pid_left, 0.0f, 5000.0f);
    Pid_Set_Output_Limits(&sim->pid_right, 0.0f, 5000.0f);
    check_pid_tuning(sim);
    track_control_init(&sim->control, NULL);
    sim->output = track_control_tick(&sim->control, 0U);
    CHECK(!sim->output.running && !sim->output.ready);
    CHECK(sim->output.target_speed == 0.0f);
}

/* Match ISR ordering: image, C13 edge, measured encoders/reverse/watchdog,
 * control tick,
 * reset on transition, then independently gated position PID for each wheel.
 * Every call advances 10 ms; non-NULL images are published only every 20 ms.
 */
static void step(simulation_t *sim, const uint8_t *image, uint8_t pressed)
{
    const uint8_t was_running = sim->output.running;
    uint8_t button_event;
    CHECK(SPEED_CONTROL_PERIOD_MS == 10U);
    sim->now_ms += SPEED_CONTROL_PERIOD_MS;
    if (image != NULL && sim->now_ms % 20U == 0U)
    {
        track_vision_process(image, sim->rows, sim->cols, &sim->vision);
        track_control_observe(&sim->control, sim->vision.error, sim->vision.heading,
                              sim->vision.confidence, sim->vision.valid, sim->now_ms);
    }
    button_event = motor_switch_sample(&sim->button, pressed);
    if (button_event)
    {
        ++sim->button_events;
        sim->feedback_fault = 0U;
        if (was_running)
            track_control_stop(&sim->control, TRACK_STOP_MANUAL);
        else
            track_control_toggle(&sim->control, sim->now_ms);
    }
    sim->measured_left = sim->feedback_left;
    sim->measured_right = sim->feedback_right;
    if (sim->control.running &&
        motor_feedback_fault_mask(sim->feedback_left, sim->feedback_right))
    {
        sim->feedback_fault = motor_feedback_fault_mask(sim->feedback_left,
                                                        sim->feedback_right);
        track_control_stop(&sim->control, TRACK_STOP_ENCODER);
    }
    const uint8_t no_feedback = motor_feedback_watchdog_sample(
        &sim->feedback_watchdog, (uint8_t)(was_running && sim->control.running),
        sim->feedback_left, sim->feedback_right,
        (uint32_t)sim->pwm_left, (uint32_t)sim->pwm_right);
    if (no_feedback)
    {
        sim->feedback_fault = no_feedback;
        track_control_stop(&sim->control, TRACK_STOP_NO_FEEDBACK);
    }
    sim->output = track_control_tick(&sim->control, sim->now_ms);
    sim->button.enabled = sim->output.running;
    if (button_event || was_running != sim->output.running)
    {
        motor_feedback_watchdog_reset(&sim->feedback_watchdog);
        sim->pwm_left = sim->pwm_right = 0.0f;
        Pid_Reset(&sim->pid_left);
        Pid_Reset(&sim->pid_right);
        ++sim->pid_resets;
        check_pid_history_zero(&sim->pid_left);
        check_pid_history_zero(&sim->pid_right);
    }
    if (sim->output.running)
    {
        const pid_param_t right_before = sim->pid_right;
        sim->pwm_left = PidLocCtrl(&sim->pid_left,
            sim->output.target_speed - sim->measured_left, 1.0f);
        CHECK(memcmp(&right_before, &sim->pid_right, sizeof(right_before)) == 0);
        CHECK(sim->pwm_left >= 0.0f && sim->pwm_left <= 5000.0f);
        sim->pwm_left = constrain_float(sim->pwm_left, 0.0f, 5000.0f);
        sim->pid_left.out = sim->pwm_left;

        const pid_param_t left_before = sim->pid_left;
        sim->pwm_right = PidLocCtrl(&sim->pid_right,
            sim->output.target_speed - sim->measured_right, 1.0f);
        CHECK(memcmp(&left_before, &sim->pid_left, sizeof(left_before)) == 0);
        CHECK(sim->pwm_right >= 0.0f && sim->pwm_right <= 5000.0f);
        sim->pwm_right = constrain_float(sim->pwm_right, 0.0f, 5000.0f);
        sim->pid_right.out = sim->pwm_right;
        sim->pid_calls += 2U;
    }
    else
    {
        sim->pwm_left = sim->pwm_right = 0.0f;
        check_pid_history_zero(&sim->pid_left);
        check_pid_history_zero(&sim->pid_right);
    }
    check_pid_tuning(sim); /* Reset must never reload shared wheel gains. */
    CHECK(sim->measured_left == (float)sim->feedback_left);
    CHECK(sim->measured_right == (float)sim->feedback_right);
    CHECK(isfinite(sim->pwm_left) && isfinite(sim->pwm_right));
    CHECK(sim->pwm_left >= 0.0f && sim->pwm_left <= 5000.0f);
    CHECK(sim->pwm_right >= 0.0f && sim->pwm_right <= 5000.0f);
    CHECK(isfinite(sim->output.servo_angle));
    CHECK(isfinite(sim->output.target_speed));
    CHECK(sim->output.servo_angle >= fminf(sim->control.config.servo_left,
                                          sim->control.config.servo_right));
    CHECK(sim->output.servo_angle <= fmaxf(sim->control.config.servo_left,
                                          sim->control.config.servo_right));
    CHECK(sim->output.target_speed >= 0.0f);
    if (!sim->output.running)
    {
        CHECK(sim->output.target_speed == 0.0f);
        CHECK(sim->pwm_left == 0.0f && sim->pwm_right == 0.0f);
    }
}

static void advance(simulation_t *sim, const uint8_t *image,
                     uint8_t pressed, unsigned int ticks)
{
    for (unsigned int i = 0; i < ticks; ++i)
        step(sim, image, pressed);
}

static void press(simulation_t *sim, const uint8_t *image, uint8_t expected_running)
{
    const uint8_t was_running = sim->output.running;
    const uint32_t old_events = sim->button_events;
    CHECK(sim->button.armed);
    for (unsigned int i = 0; i < 3U; ++i)
    {
        step(sim, image, 1U);
        CHECK(sim->button_events == old_events);
        CHECK(sim->output.running == was_running);
    }
    step(sim, image, 1U);
    CHECK(sim->button_events == old_events + 1U);
    CHECK(sim->output.running == expected_running);
}

static void draw_lane(uint8_t *image, uint16_t rows, uint16_t cols, int32_t bend)
{
    memset(image, 0, (size_t)rows * cols);
    const int32_t first = (int32_t)((uint32_t)rows * TRACK_VISION_ROI_START_PERCENT / 100U);
    const int32_t end = (int32_t)((uint32_t)rows * TRACK_VISION_ROI_END_PERCENT / 100U);
    const int32_t width = cols / 3;
    for (int32_t y = first; y < end; ++y)
    {
        const int32_t center = cols / 2 + bend * (end - 1 - y) / (end - 1 - first);
        const int32_t left = center - width / 2;
        CHECK(left > 0 && left + width < cols);
        memset(image + (uint32_t)y * cols + (uint32_t)left, 255, (size_t)width);
    }
}

static void prepare_running(simulation_t *sim, const uint8_t *image)
{
    /* 20 ms and 40 ms publish two valid frames; a third arrives at 60 ms. */
    advance(sim, image, 0U, 5U);
    CHECK(sim->control.ready_frames == 2U && !sim->output.ready);
    CHECK(!sim->output.running);
    step(sim, image, 0U);
    CHECK(sim->control.ready_frames == 3U && sim->output.ready);
    CHECK(!sim->output.running); /* Readiness must never start the car. */
    press(sim, image, 1U);
    CHECK(sim->output.stop_reason == TRACK_STOP_NONE);
    CHECK(sim->now_ms == 100U);
}

static void check_three_new_frames_while_stopped(simulation_t *sim,
                                                const uint8_t *image)
{
    const uint32_t old_events = sim->button_events;
    const uint32_t old_pid_calls = sim->pid_calls;
    const uint32_t old_pid_resets = sim->pid_resets;
    const uint8_t old_feedback_fault = sim->feedback_fault;
    CHECK(sim->now_ms % 20U == 0U);
    for (unsigned int frame = 1U; frame <= 3U; ++frame)
    {
        advance(sim, image, 1U, 2U);
        CHECK(sim->control.ready_frames == frame);
        CHECK(sim->output.ready == (frame == 3U));
        CHECK(!sim->output.running && sim->output.target_speed == 0.0f);
        CHECK(sim->button_events == old_events);
        CHECK(sim->pid_calls == old_pid_calls);
        CHECK(sim->feedback_fault == old_feedback_fault);
    }
    advance(sim, image, 1U, 10U);
    CHECK(sim->output.ready && !sim->output.running);
    CHECK(sim->button_events == old_events);
    CHECK(sim->pid_calls == old_pid_calls);
    CHECK(sim->feedback_fault == old_feedback_fault);
    advance(sim, image, 0U, 4U);
    press(sim, image, 1U);
    CHECK(sim->pid_calls == old_pid_calls + 2U);
    CHECK(sim->pid_resets == old_pid_resets + 1U);
    CHECK(sim->pwm_left > 0.0f && sim->pwm_right > 0.0f);
    CHECK(sim->feedback_fault == 0U);
}

static void test_straight_and_manual(simulation_t *sim, uint8_t *image)
{
    draw_lane(image, sim->rows, sim->cols, 0);
    prepare_running(sim, image);
    advance(sim, image, 1U, 30U);
    CHECK(sim->output.running && sim->output.ready);
    CHECK(sim->control.config.speed_straight == TRACK_SPEED_STRAIGHT);
    CHECK(sim->control.config.speed_curve_min == TRACK_SPEED_CURVE_MIN);
    CHECK(sim->output.target_speed > TRACK_SPEED_STRAIGHT * 0.98f);
    CHECK(sim->output.target_speed <= TRACK_SPEED_STRAIGHT);
    CHECK(fabsf(sim->output.servo_angle - sim->control.config.servo_center) < 0.2f);
    CHECK(sim->pwm_left == 5000.0f && sim->pwm_right == 5000.0f);
    CHECK(isfinite(sim->pid_left.integrator) && isfinite(sim->pid_right.integrator));
    CHECK(fabsf(sim->pid_left.integrator) <= sim->pid_left.imax);
    CHECK(fabsf(sim->pid_right.integrator) <= sim->pid_right.imax);
    CHECK(sim->button_events == 1U); /* Holding C13 does not toggle repeatedly. */
    advance(sim, image, 0U, 4U);
    press(sim, image, 0U);
    CHECK(sim->output.stop_reason == TRACK_STOP_MANUAL);
    const uint32_t stopped_pid_calls = sim->pid_calls;
    CHECK(sim->measured_left == 4.0f && sim->measured_right == 9.0f);
    advance(sim, image, 1U, 30U);
    CHECK(!sim->output.running && sim->output.ready);
    CHECK(sim->button_events == 2U);
    CHECK(sim->pid_calls == stopped_pid_calls);
    advance(sim, image, 0U, 4U);
    press(sim, image, 1U);
    CHECK(sim->pid_calls == stopped_pid_calls + 2U);
}

static void test_curve(simulation_t *sim, uint8_t *image, int direction)
{
    draw_lane(image, sim->rows, sim->cols, direction * (int32_t)(sim->cols / 8U));
    prepare_running(sim, image);
    advance(sim, image, 1U, 30U);
    CHECK(sim->vision.valid && sim->output.running);
    CHECK((int32_t)sim->vision.heading * direction > 90);
    CHECK((int32_t)sim->vision.error * direction > 0);
    const float center = sim->control.config.servo_center;
    const float endpoint = direction > 0 ? sim->control.config.servo_right
                                         : sim->control.config.servo_left;
    CHECK((sim->output.servo_angle - center) * (endpoint - center) > 0.0f);
    CHECK(fabsf(sim->output.servo_angle - center) > 0.5f);
    CHECK(sim->output.target_speed >= sim->control.config.speed_curve_min);
    CHECK(sim->output.target_speed < sim->control.config.speed_straight - 0.5f);
}

static void test_loss(simulation_t *sim, uint8_t *image)
{
    draw_lane(image, sim->rows, sim->cols, 0);
    prepare_running(sim, image);
    memset(image, 0, (size_t)sim->rows * sim->cols);
    step(sim, image, 1U); /* 110 ms has no new image yet. */
    CHECK(sim->output.running);
    step(sim, image, 1U); /* First invalid image at 120 ms enters low-speed grace. */
    CHECK(!sim->vision.valid && sim->output.running && !sim->output.ready);
    CHECK(sim->control.lost_frames == 1U);
    CHECK(sim->output.target_speed == sim->control.config.speed_curve_min);
    step(sim, image, 1U);
    step(sim, image, 1U); /* Second invalid image at 140 ms remains in grace. */
    CHECK(sim->output.running && sim->control.lost_frames == 2U);
    CHECK(sim->output.target_speed == sim->control.config.speed_curve_min);
    step(sim, image, 1U);
    step(sim, image, 1U); /* Third consecutive invalid image at 160 ms stops. */
    CHECK(!sim->vision.valid && !sim->output.running && !sim->output.ready);
    CHECK(sim->output.stop_reason == TRACK_STOP_LOST);
    CHECK(sim->output.target_speed == 0.0f && sim->button.enabled == 0U);
    CHECK(sim->measured_left == 4.0f && sim->measured_right == 9.0f);
    CHECK(sim->control.ready_frames == 0U);
    draw_lane(image, sim->rows, sim->cols, 0);
    check_three_new_frames_while_stopped(sim, image);
}

static void test_stale(simulation_t *sim, uint8_t *image)
{
    draw_lane(image, sim->rows, sim->cols, 0);
    prepare_running(sim, image);
    CHECK(sim->control.last_frame_ms == 100U);
    for (unsigned int tick = 1U; tick <= 15U; ++tick)
    {
        step(sim, NULL, 1U);
        CHECK(sim->output.running); /* Exactly 150 ms remains within freshness limit. */
    }
    CHECK(sim->now_ms - sim->control.last_frame_ms == 150U);
    step(sim, NULL, 1U);
    CHECK(sim->now_ms - sim->control.last_frame_ms == 160U);
    CHECK(!sim->output.running && !sim->output.ready);
    CHECK(sim->output.stop_reason == TRACK_STOP_STALE);
    CHECK(sim->output.target_speed == 0.0f && sim->button.enabled == 0U);
    CHECK(sim->measured_left == 4.0f && sim->measured_right == 9.0f);
    CHECK(sim->control.ready_frames == 0U);
    check_three_new_frames_while_stopped(sim, image);
}

static void test_loss_stop_override(simulation_t *sim, uint8_t *image)
{
    draw_lane(image, sim->rows, sim->cols, 0);
    prepare_running(sim, image);
    const uint32_t old_pid_resets = sim->pid_resets;
    track_control_set_loss_stop(&sim->control, 0U);
    CHECK(sim->control.loss_stop_enabled == 0U);

    memset(image, 0, (size_t)sim->rows * sim->cols);
    step(sim, image, 1U);
    step(sim, image, 1U);
    CHECK(!sim->vision.valid && sim->output.running && !sim->output.ready);
    CHECK(sim->output.stop_reason == TRACK_STOP_NONE);
    CHECK(sim->output.target_speed == sim->control.config.speed_curve_min);
    CHECK(sim->pwm_left > 0.0f && sim->pwm_right > 0.0f);
    CHECK(sim->pid_resets == old_pid_resets);

    /* With no further frames, stale protection is also bypassed, while the
     * existing target, PID, encoder polarity and no-feedback guards continue. */
    advance(sim, NULL, 1U, 20U);
    CHECK(sim->output.running && !sim->output.ready);
    CHECK(sim->pwm_left > 0.0f && sim->pwm_right > 0.0f);

    /* The last invalid publication has also exceeded the freshness limit, so
     * re-enabling C14 must retain the more precise STALE cause. */
    track_control_set_loss_stop(&sim->control, 1U);
    step(sim, NULL, 1U);
    CHECK(!sim->output.running && sim->output.stop_reason == TRACK_STOP_STALE);
    CHECK(sim->pid_resets == old_pid_resets + 1U);
    CHECK(sim->pwm_left == 0.0f && sim->pwm_right == 0.0f);
    check_pid_history_zero(&sim->pid_left);
    check_pid_history_zero(&sim->pid_right);

    /* With C14 protection disabled, C13 can deliberately start even while the
     * current camera observation is invalid; C13 still stops it normally. */
    track_control_set_loss_stop(&sim->control, 0U);
    advance(sim, NULL, 0U, 4U);
    press(sim, NULL, 1U);
    CHECK(sim->output.running && !sim->output.ready);
    CHECK(sim->output.target_speed == sim->control.config.speed_curve_min);
    CHECK(sim->pwm_left > 0.0f && sim->pwm_right > 0.0f);
    advance(sim, NULL, 0U, 4U);
    press(sim, NULL, 0U);
    CHECK(!sim->output.running && sim->output.stop_reason == TRACK_STOP_MANUAL);
    track_control_set_loss_stop(&sim->control, 1U);
}

static void test_press_before_ready(simulation_t *sim, const uint8_t *image)
{
    advance(sim, NULL, 0U, 4U);
    press(sim, NULL, 0U);
    CHECK(sim->output.stop_reason == TRACK_STOP_NOT_READY);
    CHECK(sim->now_ms == 80U && sim->control.ready_frames == 0U);
    check_three_new_frames_while_stopped(sim, image);
}

static void test_encoder_stop(simulation_t *sim, uint8_t *image, uint8_t fault)
{
    draw_lane(image, sim->rows, sim->cols, 0);
    prepare_running(sim, image);
    step(sim, image, 1U); /* Arrange the fault at the next 20-ms publication. */
    const uint32_t old_pid_calls = sim->pid_calls;
    sim->feedback_left = (fault & 1U) ? -4 : 4;
    sim->feedback_right = (fault & 2U) ? -9 : 9;
    step(sim, image, 1U);
    CHECK(sim->now_ms == 120U);
    CHECK(!sim->output.running && sim->output.stop_reason == TRACK_STOP_ENCODER);
    CHECK(sim->feedback_fault == fault && sim->control.ready_frames == 0U);
    CHECK(sim->pid_calls == old_pid_calls); /* Negative feedback never enters PID. */
    CHECK(sim->pwm_left == 0.0f && sim->pwm_right == 0.0f);
    CHECK(sim->measured_left != 0.0f && sim->measured_right != 0.0f);
    sim->feedback_left = 4;
    sim->feedback_right = 9;
    check_three_new_frames_while_stopped(sim, image);
    CHECK(sim->feedback_fault == 0U);
}

static void test_no_feedback_stop(simulation_t *sim, uint8_t *image, uint8_t fault)
{
    draw_lane(image, sim->rows, sim->cols, 0);
    prepare_running(sim, image);
    sim->feedback_left = (fault & 1U) ? 0 : 4;
    sim->feedback_right = (fault & 2U) ? 0 : 9;
    const uint32_t old_pid_resets = sim->pid_resets;
    /* Startup itself is not a feedback sample: previous PWM was zero then.
     * Five complete 100-ms driven windows, not 49 ticks, are required. */
    for (unsigned int tick = 1U; tick < 50U; ++tick)
    {
        step(sim, image, 1U);
        CHECK(sim->output.running && sim->feedback_fault == 0U);
        CHECK(sim->pwm_left >= 500.0f && sim->pwm_right >= 500.0f);
        CHECK(sim->pid_resets == old_pid_resets);
    }
    const uint32_t old_pid_calls = sim->pid_calls;
    step(sim, image, 1U);
    CHECK(sim->now_ms == 600U);
    CHECK(!sim->output.running && !sim->output.ready);
    CHECK(sim->output.stop_reason == TRACK_STOP_NO_FEEDBACK);
    CHECK(sim->feedback_fault == fault && sim->control.ready_frames == 0U);
    CHECK(sim->pid_calls == old_pid_calls);
    CHECK(sim->pid_resets == old_pid_resets + 1U);
    CHECK(sim->pwm_left == 0.0f && sim->pwm_right == 0.0f);
    check_pid_history_zero(&sim->pid_left);
    check_pid_history_zero(&sim->pid_right);

    sim->feedback_left = 4;
    sim->feedback_right = 9;
    check_three_new_frames_while_stopped(sim, image);
    CHECK(sim->feedback_fault == 0U);
    advance(sim, image, 1U, 60U);
    CHECK(sim->output.running && sim->output.stop_reason == TRACK_STOP_NONE);
    CHECK(sim->feedback_fault == 0U);
}

static void test_power_on_held(simulation_t *sim, uint8_t *image)
{
    draw_lane(image, sim->rows, sim->cols, 0);
    advance(sim, image, 1U, 60U);
    CHECK(sim->output.ready && !sim->output.running);
    CHECK(sim->button_events == 0U && sim->pid_calls == 0U);
    CHECK(sim->feedback_fault == 0U);
    advance(sim, image, 0U, 4U);
    press(sim, image, 1U);
    CHECK(sim->button_events == 1U && sim->pid_calls == 2U);
}

static void test_no_feedback_priority(simulation_t *sim, uint8_t *image,
                                      uint8_t manual_stop)
{
    draw_lane(image, sim->rows, sim->cols, 0);
    prepare_running(sim, image);
    sim->feedback_left = sim->feedback_right = 0;
    /* Arrange the fifth bad window and either manual stop or reverse fault in
     * the same tick. Earlier safety actions must prevent a NO_FEEDBACK rewrite. */
    advance(sim, image, 0U, 46U);
    if (manual_stop)
    {
        press(sim, image, 0U);
        CHECK(sim->output.stop_reason == TRACK_STOP_MANUAL);
        CHECK(sim->feedback_fault == 0U);
    }
    else
    {
        advance(sim, image, 0U, 3U);
        sim->feedback_right = -1;
        step(sim, image, 0U);
        CHECK(sim->output.stop_reason == TRACK_STOP_ENCODER);
        CHECK(sim->feedback_fault == 2U);
    }
    CHECK(sim->now_ms == 600U && !sim->output.running);
    CHECK(sim->pwm_left == 0.0f && sim->pwm_right == 0.0f);
    check_pid_history_zero(&sim->pid_left);
    check_pid_history_zero(&sim->pid_right);
}

static void test_pid_feedback_and_limits(simulation_t *sim, uint8_t *image)
{
    draw_lane(image, sim->rows, sim->cols, 0);
    prepare_running(sim, image);
    const float left_error = sim->output.target_speed - 4.0f;
    const float right_error = sim->output.target_speed - 9.0f;
    /* First post-reset sample uses independent gains and t=1 (one 10-ms tick). */
    CHECK(fabsf(sim->pwm_left - constrain_float(
        (37.0f + 0.5f + 0.25f) * left_error, 0.0f, 5000.0f)) < 0.001f);
    CHECK(fabsf(sim->pwm_right - constrain_float(
        (61.0f + 0.75f + 0.5f) * right_error, 0.0f, 5000.0f)) < 0.001f);
    CHECK(sim->pid_left.last_error == left_error);
    CHECK(sim->pid_right.last_error == right_error);
    CHECK(fabsf(sim->pwm_left - sim->pwm_right) > 1.0f);

    sim->feedback_left = 8;
    step(sim, image, 1U);
    CHECK(sim->pid_left.last_error == sim->output.target_speed - 8.0f);
    CHECK(sim->pid_right.last_error == sim->output.target_speed - 9.0f);

    sim->feedback_left = sim->feedback_right = 1000;
    step(sim, image, 1U);
    CHECK(sim->output.running && sim->feedback_fault == 0U);
    CHECK(sim->pwm_left == 0.0f && sim->pwm_right == 0.0f); /* No reverse drive. */

    sim->feedback_left = sim->feedback_right = 0;
    sim->control.config.speed_straight = 500.0f;
    sim->control.config.speed_curve_min = 500.0f;
    step(sim, image, 1U);
    CHECK(sim->output.target_speed == 500.0f);
    CHECK(sim->pwm_left == 5000.0f && sim->pwm_right == 5000.0f);
    CHECK(sim->pid_left.out == sim->pwm_left && sim->pid_right.out == sim->pwm_right);
}

static void test_resolution(uint16_t rows, uint16_t cols)
{
    uint8_t *image = malloc((size_t)rows * cols);
    CHECK(image != NULL);
    simulation_t sim;
    simulation_init(&sim, rows, cols);
    test_straight_and_manual(&sim, image);
    simulation_init(&sim, rows, cols);
    test_curve(&sim, image, 1);
    simulation_init(&sim, rows, cols);
    test_curve(&sim, image, -1);
    simulation_init(&sim, rows, cols);
    test_loss(&sim, image);
    simulation_init(&sim, rows, cols);
    test_stale(&sim, image);
    simulation_init(&sim, rows, cols);
    test_loss_stop_override(&sim, image);
    simulation_init(&sim, rows, cols);
    draw_lane(image, rows, cols, 0);
    test_press_before_ready(&sim, image);
    for (uint8_t fault = 1U; fault <= 3U; ++fault)
    {
        simulation_init(&sim, rows, cols);
        test_encoder_stop(&sim, image, fault);
        simulation_init(&sim, rows, cols);
        test_no_feedback_stop(&sim, image, fault);
    }
    simulation_init(&sim, rows, cols);
    test_power_on_held(&sim, image);
    simulation_init(&sim, rows, cols);
    test_no_feedback_priority(&sim, image, 1U);
    simulation_init(&sim, rows, cols);
    test_no_feedback_priority(&sim, image, 0U);
    simulation_init(&sim, rows, cols);
    test_pid_feedback_and_limits(&sim, image);
    free(image);
    printf("PASS %u x %u: vision/control, dual PID/PWM, C13, manual/loss/stale/reverse/no-feedback stop\n",
           (unsigned int)cols, (unsigned int)rows);
}

static void test_production_pi_tuning(void)
{
    My_Pid_Init();
    CHECK(speed_KP == 50.0f && speed_KI == 0.2f && speed_KD == 0.0f);
    CHECK(speed_IMAX == 5000.0f && speed_OUTMAX == 2000.0f);
    CHECK(speed_pid_l.kp == speed_KP && speed_pid_r.kp == speed_KP);
    CHECK(speed_pid_l.ki == speed_KI && speed_pid_r.ki == speed_KI);
    CHECK(speed_pid_l.outmin == 0.0f && speed_pid_r.outmin == 0.0f);
    CHECK(speed_pid_l.outmax == 2000.0f && speed_pid_r.outmax == 2000.0f);
    CHECK(speed_target == 0.0f && speed_pwm_l == 0.0f && speed_pwm_r == 0.0f);
    float previous = 0.0f;
    for (unsigned int tick = 1U; tick <= 50U; ++tick)
    {
        const float output = PidLocCtrl(&speed_pid_l, TRACK_SPEED_STRAIGHT, 1.0f);
        CHECK(output == 2000.0f);
        CHECK(speed_pid_l.integrator == 0.0f);
        CHECK(speed_pid_l.out_i <= 1000.0f);
        previous = output;
        check_pid_history_zero(&speed_pid_r);
    }
    CHECK(fabsf(previous - 2000.0f) < 0.01f); /* Hard 20% PWM ceiling. */
    Pid_Reset(&speed_pid_l);
    check_pid_history_zero(&speed_pid_l);
    CHECK(speed_pid_l.ki == 0.2f && speed_pid_l.outmax == 2000.0f);
    puts("PASS actual firmware PI defaults, bounded load compensation, independent state/reset");
}

int main(void)
{
    test_production_pi_tuning();
    test_resolution(120, 188);
    test_resolution(240, 376);
    test_resolution(480, 752);
    puts("All line-following integration tests passed.");
    return EXIT_SUCCESS;
}
