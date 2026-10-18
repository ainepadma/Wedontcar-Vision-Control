#include "track_control.h"

#include <math.h>
#include <string.h>

#define TRACK_READY_FRAMES 3U
#define TRACK_MIN_CONFIDENCE 60U
#define TRACK_STALE_MS 150U
#define TRACK_MAX_FORWARD_TIME UINT32_C(0x7fffffff)

static double track_clamp(double value, double low, double high)
{
    return value < low ? low : (value > high ? high : value);
}

static float track_finite_or(float value, float fallback)
{
    return isfinite(value) ? value : fallback;
}

static uint8_t track_frame_fresh(const track_control_state_t *state,
                                 uint32_t now_ms)
{
    return (uint8_t)(state->has_frame && state->frame_valid &&
                    (uint32_t)(now_ms - state->last_frame_ms) <= TRACK_STALE_MS);
}

static uint8_t track_ready(const track_control_state_t *state, uint32_t now_ms)
{
    return (uint8_t)(state->ready_frames >= TRACK_READY_FRAMES &&
                    track_frame_fresh(state, now_ms));
}

void track_control_init(track_control_state_t *state,
                        const track_control_config_t *config)
{
    static const track_control_config_t defaults = {
        92.5f, 100.0f, 85.0f, TRACK_SPEED_STRAIGHT, TRACK_SPEED_CURVE_MIN,
        0.02f, 0.0005f, 60.0f
    };
    track_control_config_t safe;
    double low;
    double high;
    if (state == NULL)
        return;
    safe = config != NULL ? *config : defaults;
    safe.servo_left = track_finite_or(safe.servo_left, defaults.servo_left);
    safe.servo_right = track_finite_or(safe.servo_right, defaults.servo_right);
    low = fmin((double)safe.servo_left, (double)safe.servo_right);
    high = fmax((double)safe.servo_left, (double)safe.servo_right);
    safe.servo_center = (float)track_clamp(
        track_finite_or(safe.servo_center, defaults.servo_center), low, high);
    safe.speed_straight = fmaxf(0.0f, track_finite_or(safe.speed_straight, 0.0f));
    safe.speed_curve_min = (float)track_clamp(
        track_finite_or(safe.speed_curve_min, 0.0f), 0.0, safe.speed_straight);
    safe.kp = fmaxf(0.0f, track_finite_or(safe.kp, 0.0f));
    safe.kd = fmaxf(0.0f, track_finite_or(safe.kd, 0.0f));
    safe.max_servo_rate_deg_s = fmaxf(
        0.0f, track_finite_or(safe.max_servo_rate_deg_s, 0.0f));
    memset(state, 0, sizeof(*state));
    state->config = safe;
    state->servo_angle = safe.servo_center;
    state->stop_reason = TRACK_STOP_MANUAL;
}

void track_control_stop(track_control_state_t *state, uint8_t reason)
{
    if (state == NULL)
        return;
    state->running = 0U;
    state->stop_reason = reason >= TRACK_STOP_MANUAL && reason <= TRACK_STOP_NO_FEEDBACK
                             ? reason : TRACK_STOP_MANUAL;
    state->derivative = 0.0f;
    state->derivative_ready = 0U;
    if (state->stop_reason == TRACK_STOP_LOST ||
        state->stop_reason == TRACK_STOP_STALE ||
        state->stop_reason == TRACK_STOP_ENCODER ||
        state->stop_reason == TRACK_STOP_NO_FEEDBACK)
        state->ready_frames = 0U;
    /* Keep the last angle: the hardware owner can disable PWM on a stop. */
}

void track_control_observe(track_control_state_t *state, int16_t error,
                           int16_t heading, uint8_t confidence, uint8_t valid,
                           uint32_t now_ms)
{
    uint32_t elapsed;
    double old_error;
    double alpha;
    double dt;
    uint8_t previous_valid;
    if (state == NULL)
        return;
    elapsed = state->has_frame ? (uint32_t)(now_ms - state->last_frame_ms) : 0U;
    /* Reject old valid publications; an explicitly invalid frame always fails
     * closed, even if it shares a timer timestamp with another observation. */
    if (valid && confidence >= TRACK_MIN_CONFIDENCE && state->has_frame &&
        (elapsed == 0U || elapsed > TRACK_MAX_FORWARD_TIME))
        return;
    if (state->has_frame && elapsed > TRACK_STALE_MS)
    {
        if (state->running)
            track_control_stop(state, TRACK_STOP_STALE);
        state->ready_frames = 0U;
        state->frame_valid = 0U;
    }
    previous_valid = state->frame_valid;
    state->has_frame = 1U;
    state->last_frame_ms = now_ms;
    if (!valid || confidence < TRACK_MIN_CONFIDENCE)
    {
        if (state->running)
            track_control_stop(state, TRACK_STOP_LOST);
        state->frame_valid = 0U;
        state->ready_frames = 0U;
        state->derivative = 0.0f;
        state->derivative_ready = 0U;
        return;
    }
    state->frame_valid = 1U;
    if (state->ready_frames < TRACK_READY_FRAMES)
        ++state->ready_frames;
    old_error = state->filtered_error;
    dt = (double)elapsed / 1000.0;
    alpha = previous_valid ? dt / (0.040 + dt) : 1.0;
    state->filtered_error = (float)(old_error + alpha *
        (track_clamp(error, -1000.0, 1000.0) - old_error));
    state->filtered_heading = (float)((double)state->filtered_heading + alpha *
        (track_clamp(heading, -1000.0, 1000.0) - state->filtered_heading));
    if (state->running && state->derivative_ready && previous_valid && elapsed > 0U)
    {
        double raw_derivative = ((double)state->filtered_error - old_error) / dt;
        double derivative_alpha = dt / (0.060 + dt);
        state->derivative = (float)((double)state->derivative + derivative_alpha *
                                   (raw_derivative - state->derivative));
    }
    else
        state->derivative = 0.0f;
    state->derivative_ready = state->running;
}

void track_control_toggle(track_control_state_t *state, uint32_t now_ms)
{
    if (state == NULL)
        return;
    if (state->running)
    {
        track_control_stop(state, TRACK_STOP_MANUAL);
        return;
    }
    if (!track_ready(state, now_ms))
    {
        track_control_stop(state, state->has_frame &&
            (uint32_t)(now_ms - state->last_frame_ms) > TRACK_STALE_MS
                ? TRACK_STOP_STALE : TRACK_STOP_NOT_READY);
        return;
    }
    state->running = 1U;
    state->stop_reason = TRACK_STOP_NONE;
    state->derivative = 0.0f;
    state->derivative_ready = 0U;
    state->last_tick_ms = now_ms;
    state->has_tick = 1U;
}

track_control_output_t track_control_tick(track_control_state_t *state,
                                         uint32_t now_ms)
{
    track_control_output_t output = {0U, 0U, TRACK_STOP_NOT_READY, 0.0f, 0.0f};
    uint32_t elapsed;
    if (state == NULL)
        return output;
    elapsed = state->has_tick ? (uint32_t)(now_ms - state->last_tick_ms) : 0U;
    if (elapsed > TRACK_MAX_FORWARD_TIME)
        elapsed = 0U;
    state->last_tick_ms = now_ms;
    state->has_tick = 1U;
    if (state->has_frame && !track_frame_fresh(state, now_ms))
    {
        if (state->running)
            track_control_stop(state, state->frame_valid
                ? TRACK_STOP_STALE : TRACK_STOP_LOST);
        state->ready_frames = 0U;
        state->frame_valid = 0U;
    }
    if (state->running)
    {
        const track_control_config_t *cfg = &state->config;
        double mixed_error = track_clamp((double)state->filtered_error +
                                        0.35 * state->filtered_heading, -1000.0, 1000.0);
        double correction = (double)cfg->kp * mixed_error +
                            (double)cfg->kd * state->derivative;
        double endpoint = correction >= 0.0 ? cfg->servo_right : cfg->servo_left;
        double span = fabs(endpoint - cfg->servo_center);
        double offset = fmin(fabs(correction), span);
        double desired = cfg->servo_center +
                         (endpoint >= cfg->servo_center ? offset : -offset);
        double max_step = (double)cfg->max_servo_rate_deg_s * elapsed / 1000.0;
        double severity = fmax(fabs((double)state->filtered_error) / 600.0,
                               fabs((double)state->filtered_heading) / 500.0);
        severity = fmax(severity, span > 0.0 ? offset / span : 0.0);
        severity = track_clamp(severity, 0.0, 1.0);
        state->servo_angle = (float)track_clamp(desired,
            (double)state->servo_angle - max_step, (double)state->servo_angle + max_step);
        state->servo_angle = (float)track_clamp(state->servo_angle,
            fmin((double)cfg->servo_left, (double)cfg->servo_right),
            fmax((double)cfg->servo_left, (double)cfg->servo_right));
        output.target_speed = (float)((double)cfg->speed_straight * (1.0 - severity) +
                                     (double)cfg->speed_curve_min * severity);
    }
    output.running = state->running;
    output.ready = track_ready(state, now_ms);
    output.stop_reason = state->stop_reason;
    output.servo_angle = state->servo_angle;
    return output;
}
