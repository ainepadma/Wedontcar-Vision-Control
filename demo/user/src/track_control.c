#include "track_control.h"

#include <float.h>
#include <math.h>
#include <string.h>

#define TRACK_READY_FRAMES 3U
#define TRACK_MIN_CONFIDENCE 45U
#define TRACK_MAX_FORWARD_TIME UINT32_C(0x7fffffff)
#define TRACK_HEADING_WEIGHT 3.00
/* 出弯偏离中线较远时，前瞻方向和微分项都不能反向压过回中指令。 */
#define TRACK_RECENTER_BLEND_START 80.0
#define TRACK_RECENTER_BLEND_END 160.0
#define TRACK_OPPOSING_HEADING_FRACTION 0.25
#define TRACK_OPPOSING_DERIVATIVE_FRACTION 0.25
#define TRACK_RECENTER_SPEED_ERROR 350.0

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
        87.0f, 98.5f, 75.5f, TRACK_SPEED_STRAIGHT, TRACK_SPEED_CURVE_MIN,
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
    state->loss_stop_enabled = 1U;
    state->servo_angle = safe.servo_center;
    state->stop_reason = TRACK_STOP_MANUAL;
}

void track_control_stop(track_control_state_t *state, uint8_t reason)
{
    if (state == NULL)
        return;
    state->running = 0U;
    state->stop_reason = reason >= TRACK_STOP_MANUAL && reason <= TRACK_STOP_OVERSPEED
                             ? reason : TRACK_STOP_MANUAL;
    state->derivative = 0.0f;
    state->derivative_ready = 0U;
    state->turn_override = 0;
    if (state->stop_reason == TRACK_STOP_LOST ||
        state->stop_reason == TRACK_STOP_STALE ||
        state->stop_reason == TRACK_STOP_ENCODER ||
        state->stop_reason == TRACK_STOP_NO_FEEDBACK ||
        state->stop_reason == TRACK_STOP_OVERSPEED)
        state->ready_frames = 0U;
    /* Keep the last angle: the hardware owner can disable PWM on a stop. */
}

void track_control_set_turn_override(track_control_state_t *state,
                                     int8_t direction)
{
    if (state == NULL)
        return;
    state->turn_override = direction > 0 ? 1 : (direction < 0 ? -1 : 0);
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
        if (state->running && state->loss_stop_enabled)
            track_control_stop(state, TRACK_STOP_STALE);
        state->ready_frames = 0U;
        state->frame_valid = 0U;
        state->frame_stale = 1U;
        state->derivative = 0.0f;
        state->derivative_ready = 0U;
    }
    previous_valid = state->frame_valid;
    state->has_frame = 1U;
    state->last_frame_ms = now_ms;
    state->frame_stale = 0U;
    if (!valid || confidence < TRACK_MIN_CONFIDENCE)
    {
        if (state->lost_frames < TRACK_LOST_STOP_FRAMES)
            ++state->lost_frames;
        if (state->running && state->loss_stop_enabled &&
            state->lost_frames >= TRACK_LOST_STOP_FRAMES)
            track_control_stop(state, TRACK_STOP_LOST);
        state->frame_valid = 0U;
        state->ready_frames = 0U;
        state->derivative = 0.0f;
        state->derivative_ready = 0U;
        return;
    }
    state->lost_frames = 0U;
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
    if (state->loss_stop_enabled && !track_ready(state, now_ms))
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

void track_control_set_loss_stop(track_control_state_t *state, uint8_t enabled)
{
    if (state == NULL)
        return;
    enabled = enabled != 0U ? 1U : 0U;
    state->loss_stop_enabled = enabled;
    if (enabled && state->running)
    {
        if (!state->has_frame)
            track_control_stop(state, TRACK_STOP_NOT_READY);
        else if (state->frame_stale)
            track_control_stop(state, TRACK_STOP_STALE);
        else if (!state->frame_valid)
            track_control_stop(state, TRACK_STOP_LOST);
    }
}

track_control_output_t track_control_tick(track_control_state_t *state,
                                         uint32_t now_ms)
{
    track_control_output_t output = {
        0U, 0U, TRACK_STOP_NOT_READY, 0.0f, 0.0f, 0.0f, 0.0f
    };
    uint32_t elapsed;
    if (state == NULL)
        return output;
    elapsed = state->has_tick ? (uint32_t)(now_ms - state->last_tick_ms) : 0U;
    if (elapsed > TRACK_MAX_FORWARD_TIME)
        elapsed = 0U;
    state->last_tick_ms = now_ms;
    state->has_tick = 1U;
    if (state->has_frame &&
        (uint32_t)(now_ms - state->last_frame_ms) > TRACK_STALE_MS)
    {
        if (state->running && state->loss_stop_enabled)
            track_control_stop(state, TRACK_STOP_STALE);
        state->ready_frames = 0U;
        state->frame_valid = 0U;
        state->frame_stale = 1U;
        state->derivative = 0.0f;
        state->derivative_ready = 0U;
    }
    if (state->running)
    {
        const track_control_config_t *cfg = &state->config;
        if (state->turn_override != 0)
        {
            const double desired = state->turn_override > 0 ?
                                   cfg->servo_left : cfg->servo_right;
            const double max_step =
                (double)cfg->max_servo_rate_deg_s * elapsed / 1000.0;
            state->servo_angle = (float)track_clamp(desired,
                (double)state->servo_angle - max_step,
                (double)state->servo_angle + max_step);
            state->servo_angle = (float)track_clamp(state->servo_angle,
                fmin((double)cfg->servo_left, (double)cfg->servo_right),
                fmax((double)cfg->servo_left, (double)cfg->servo_right));
            output.target_speed = cfg->speed_curve_min;
        }
        else if (state->frame_valid)
        {
            /* 近端偏差表示车身横向位置，不能被远端道路方向抵消。
             * 小偏差仍允许前瞻项主导入弯；明显偏离时先向中线回正。 */
            double lateral = (double)state->filtered_error;
            double heading = TRACK_HEADING_WEIGHT * (double)state->filtered_heading;
            double recenter = track_clamp((fabs(lateral) - TRACK_RECENTER_BLEND_START) /
                (TRACK_RECENTER_BLEND_END - TRACK_RECENTER_BLEND_START), 0.0, 1.0);
            if (recenter > 0.0 && lateral * heading < 0.0)
            {
                double limited_heading = track_clamp(heading,
                    -TRACK_OPPOSING_HEADING_FRACTION * fabs(lateral),
                     TRACK_OPPOSING_HEADING_FRACTION * fabs(lateral));
                heading += recenter * (limited_heading - heading);
            }
            double mixed_error = track_clamp(lateral + heading, -1000.0, 1000.0);
            double proportional = (double)cfg->kp * mixed_error;
            double derivative = (double)cfg->kd * state->derivative;
            if (recenter > 0.0 && proportional * derivative < 0.0)
            {
                double limited_derivative = track_clamp(derivative,
                    -TRACK_OPPOSING_DERIVATIVE_FRACTION * fabs(proportional),
                     TRACK_OPPOSING_DERIVATIVE_FRACTION * fabs(proportional));
                derivative += recenter * (limited_derivative - derivative);
            }
            double correction = proportional + derivative;
            double endpoint = correction >= 0.0 ? cfg->servo_right : cfg->servo_left;
            double span = fabs(endpoint - cfg->servo_center);
            double offset = fmin(fabs(correction), span);
            double desired = cfg->servo_center +
                             (endpoint >= cfg->servo_center ? offset : -offset);
            double max_step = (double)cfg->max_servo_rate_deg_s * elapsed / 1000.0;
            /* 车仍偏在边线附近时维持弯道低速，给横向回中留出距离。 */
            double severity = fmax(fabs((double)state->filtered_error) /
                                   TRACK_RECENTER_SPEED_ERROR,
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
        else
        {
            state->derivative = 0.0f;
            state->derivative_ready = 0U;
            output.target_speed = cfg->speed_curve_min;
        }
    }
    output.running = state->running;
    output.ready = track_ready(state, now_ms);
    output.stop_reason = state->stop_reason;
    output.servo_angle = state->servo_angle;
    output.target_speed_left = output.target_speed;
    output.target_speed_right = output.target_speed;
    if (output.running && output.target_speed > 0.0f)
    {
        const double center = state->config.servo_center;
        const double angle = state->servo_angle;
        double turn = 0.0;
        if (angle < center)
        {
            const double right_span = center - state->config.servo_right;
            if (right_span > 0.0)
                turn = (center - angle) / right_span;
        }
        else if (angle > center)
        {
            const double left_span = state->config.servo_left - center;
            if (left_span > 0.0)
                turn = -(angle - center) / left_span;
        }
        turn = track_clamp(turn, -1.0, 1.0);
        const double delta = (double)output.target_speed * TRACK_DIFF_GAIN * turn;
        output.target_speed_left = (float)track_clamp(
            (double)output.target_speed + delta, 0.0, FLT_MAX);
        output.target_speed_right = (float)track_clamp(
            (double)output.target_speed - delta, 0.0, FLT_MAX);
    }
    return output;
}
