#include "track_control.h"

#include <math.h>
#include <string.h>

#define TRACK_READY_FRAMES 3U
#define TRACK_MIN_CONFIDENCE 45U
#define TRACK_LOST_STOP_FRAMES 3U
#define TRACK_STALE_MS 150U
#define TRACK_MAX_FORWARD_TIME UINT32_C(0x7fffffff)
/* 远处走向的前瞻权重：降低航向影响，几何识别范围保持不变。 */
#define TRACK_HEADING_WEIGHT 0.40
/* 只影响“弯还没到近处”这个阶段：近处一旦出现弯（aligned=1），
 * heading_weight 会升到 TRACK_HEADING_WEIGHT，弯中转向强度不受这里影响。
 * 0.55 时入弯偏早，降到 0.35 把前瞻压下来；仍偏早就再降，弯中拐不动请调 KP 而不是这里。 */
#define TRACK_APPROACH_HEADING_WEIGHT 0.05
#define TRACK_NEAR_BEND_START 30.0
#define TRACK_NEAR_BEND_FULL 120.0
#define TRACK_BEND_P_BOOST 0.40
#define TRACK_NEAR_HEADING_WEIGHT 0.80
#define TRACK_BOTTOM_TRIGGER 80.0
#define TRACK_BOTTOM_RELEASE 40.0
#define TRACK_BOTTOM_PULSE_MS 160U
#define TRACK_BOTTOM_MAX_DEG 2.0
#define TRACK_BOTTOM_GAIN 0.01

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
    state->bottom_armed = 1U;
    state->servo_angle = safe.servo_center;
    state->stop_reason = TRACK_STOP_MANUAL;
}

void track_control_stop(track_control_state_t *state, uint8_t reason)
{
    if (state == NULL)
        return;
    state->running = 0U;
    state->bottom_active = state->bottom_votes = state->bottom_center_votes = 0U;
    state->bottom_armed = 1U;
    state->stop_reason = reason >= TRACK_STOP_MANUAL && reason <= TRACK_STOP_RING
                             ? reason : TRACK_STOP_MANUAL;
    state->derivative = 0.0f;
    state->derivative_ready = 0U;
    if (state->stop_reason == TRACK_STOP_LOST ||
        state->stop_reason == TRACK_STOP_STALE ||
        state->stop_reason == TRACK_STOP_ENCODER ||
        state->stop_reason == TRACK_STOP_NO_FEEDBACK ||
        state->stop_reason == TRACK_STOP_OVERSPEED)
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
        if (state->running && state->loss_stop_enabled)
            track_control_stop(state, TRACK_STOP_STALE);
        state->ready_frames = 0U;
        state->frame_valid = 0U;
        state->frame_stale = 1U;
        state->derivative = 0.0f;
        state->derivative_ready = 0U;
    }
    state->near_geometry_valid = 0U;
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

void track_control_observe_near(track_control_state_t *state, int16_t error,
    int16_t heading, int16_t near_error, int16_t near_heading, uint8_t near_valid,
    uint8_t confidence, uint8_t valid, uint32_t now_ms)
{
    if (state == NULL) return;
    uint32_t elapsed = state->has_frame ? now_ms - state->last_frame_ms : 0U;
    if (valid && confidence >= TRACK_MIN_CONFIDENCE && state->has_frame &&
        (elapsed == 0U || elapsed > TRACK_MAX_FORWARD_TIME)) return;
    uint8_t was_near = state->near_geometry_valid && state->frame_valid &&
                       elapsed <= TRACK_STALE_MS;
    double old_near = state->filtered_near_heading;
    /* Changing paths resets D: global and near lateral errors differ. */
    if (state->near_geometry_valid != (near_valid != 0U)) {
        state->frame_valid = 0U; state->derivative_ready = 0U;
    }
    track_control_observe(state, near_valid ? near_error : error, heading,
                          confidence, valid, now_ms);
    state->near_geometry_valid = (uint8_t)(near_valid && state->frame_valid);
    if (state->near_geometry_valid) {
        double alpha = was_near ? (double)elapsed / (40.0 + elapsed) : 1.0;
        state->filtered_near_heading = (float)(old_near + alpha *
            (track_clamp(near_heading, -1000.0, 1000.0) - old_near));
    } else state->filtered_near_heading = 0.0f;
}

void track_control_observe_bottom(track_control_state_t *state, int16_t error,
                                  uint8_t valid, uint32_t now_ms)
{
    if (state == NULL || !state->has_frame || state->last_frame_ms != now_ms) return;
    if (state->bottom_seen && state->bottom_frame_ms == now_ms) return;
    state->bottom_seen = 1U; state->bottom_frame_ms = now_ms;
    if (!state->running || !state->frame_valid || !valid) {
        state->bottom_active = state->bottom_votes = state->bottom_center_votes = 0U;
        return;
    }
    double e = track_clamp(error, -1000.0, 1000.0);
    int8_t sign = e > 0.0 ? 1 : -1;
    if (fabs(e) <= TRACK_BOTTOM_RELEASE) {
        state->bottom_active = state->bottom_votes = 0U;
        if (state->bottom_center_votes < 2U) ++state->bottom_center_votes;
        if (state->bottom_center_votes >= 2U) state->bottom_armed = 1U;
        return;
    }
    state->bottom_center_votes = 0U;
    if (state->bottom_active && sign != state->bottom_sign) state->bottom_active = 0U;
    if (fabs(e) < TRACK_BOTTOM_TRIGGER) {state->bottom_votes = 0U;return;}
    if (!state->bottom_armed) return;
    if (sign != state->bottom_sign) state->bottom_votes = 0U;
    state->bottom_sign = sign;
    if (++state->bottom_votes < 2U) return;
    state->bottom_votes = 0U; state->bottom_armed = 0U; state->bottom_active = 1U;
    state->bottom_start_ms = now_ms;
    state->bottom_pulse_deg = (float)(sign * fmin(TRACK_BOTTOM_MAX_DEG,
                                                TRACK_BOTTOM_GAIN * fabs(e)));
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

void track_control_force_straight(track_control_state_t *state,track_control_output_t *output)
{
    if(!state || !output || !state->running || !output->running) return;
    state->servo_angle=state->config.servo_center;
    output->servo_angle=state->servo_angle;
    state->filtered_error=0.0f;state->filtered_heading=0.0f;state->filtered_near_heading=0.0f;
    state->near_geometry_valid=0U;state->derivative=0.0f;state->derivative_ready=0U;
    state->bottom_active=0U;state->bottom_votes=0U;state->bottom_center_votes=0U;
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
    if (!state->frame_valid || !state->running ||
        (uint32_t)(now_ms - state->bottom_start_ms) >= TRACK_BOTTOM_PULSE_MS)
        state->bottom_active = 0U;
    if (state->running)
    {
        const track_control_config_t *cfg = &state->config;
        if (state->frame_valid)
        {
            double heading_weight = TRACK_HEADING_WEIGHT;
            double boost = 1.0, local_heading = 0.0;
            if (state->near_geometry_valid) {
                local_heading = state->filtered_near_heading;
                double bend = track_clamp((fabs(local_heading) - TRACK_NEAR_BEND_START) /
                    (TRACK_NEAR_BEND_FULL - TRACK_NEAR_BEND_START), 0.0, 1.0);
                /* A reversing/S-bend far tangent must not boost opposite steering. */
                double aligned = local_heading * state->filtered_heading >= 0.0 ? bend : 0.0;
                heading_weight = TRACK_APPROACH_HEADING_WEIGHT + aligned *
                    (TRACK_HEADING_WEIGHT - TRACK_APPROACH_HEADING_WEIGHT);
                boost += TRACK_BEND_P_BOOST * bend;
            }
            double mixed_error = track_clamp((double)state->filtered_error +
                heading_weight * state->filtered_heading +
                TRACK_NEAR_HEADING_WEIGHT * local_heading, -1000.0, 1000.0);
            /* Never amplify a command pointing away from the measured near bend. */
            if (local_heading * mixed_error <= 0.0) boost = 1.0;
            double correction = (double)cfg->kp * boost * mixed_error +
                                (double)cfg->kd * state->derivative;
            if (state->bottom_active) {
                double fade = 1.0 - (double)(uint32_t)(now_ms - state->bottom_start_ms) /
                                    TRACK_BOTTOM_PULSE_MS;
                correction += state->bottom_pulse_deg * fade;
            }
            double endpoint = correction >= 0.0 ? cfg->servo_right : cfg->servo_left;
            double span = fabs(endpoint - cfg->servo_center);
            double offset = fmin(fabs(correction), span);
            double desired = cfg->servo_center +
                             (endpoint >= cfg->servo_center ? offset : -offset);
            double max_step = (double)cfg->max_servo_rate_deg_s * elapsed / 1000.0;
            /* 弯道降速的提前量：分母越小，同样的航向越早把速度压到弯道目标。
             * 原来 500 要 heading 到 500 才降满，实车到弯口已经来不及；380 提前约 25%。
             * 注意减速没有主动制动，只靠滑行，所以这个提前量只能补偿一部分。 */
            double severity = fmax(fabs((double)state->filtered_error) / 600.0,
                                   fabs((double)state->filtered_heading) / 380.0);
            if (state->near_geometry_valid)
                severity = fmax(severity, fabs(local_heading) / 120.0);
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
    return output;
}
