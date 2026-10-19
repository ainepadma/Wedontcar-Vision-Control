#ifndef MOTOR_SWITCH_H
#define MOTOR_SWITCH_H

#include <stdint.h>

/* Zero initialization starts stopped and requires a stable release before use. */
typedef struct
{
    uint8_t enabled;
    uint8_t sample_pressed;
    uint8_t sample_count;
    uint8_t armed;
    uint8_t observed;
} motor_switch_t;

/* Forward-only drive: do not feed a negative speed into the positive target PID. */
static inline uint8_t motor_feedback_fault_mask(int16_t left, int16_t right)
{
    return (uint8_t)((left < 0 ? 1U : 0U) | (right < 0 ? 2U : 0U));
}

/* Two independent 100 ms feedback windows, sampled at a fixed 10 ms period. */
typedef struct
{
    uint8_t window_ticks[2];
    uint8_t positive_counts[2];
    uint8_t bad_windows[2];
} motor_feedback_watchdog_t;

/* A wheel held off the ground can accelerate far beyond its closed-loop
 * target. Ten consecutive 10-ms samples above 2*target+20 report that wheel.
 * The persistence and margin reject short encoder spikes and speed changes. */
typedef struct
{
    uint8_t high_ticks[2];
} motor_overspeed_watchdog_t;

static inline void motor_overspeed_watchdog_reset(motor_overspeed_watchdog_t *state)
{
    if (state == 0)
        return;
    state->high_ticks[0] = 0U;
    state->high_ticks[1] = 0U;
}

static inline uint8_t motor_overspeed_watchdog_sample(
    motor_overspeed_watchdog_t *state, uint8_t enabled, int16_t left, int16_t right,
    float target)
{
    uint8_t fault_mask = 0U;
    const float limit = target * 2.0f + 20.0f;

    if (state == 0)
        return 0U;
    for (uint8_t wheel = 0U; wheel < 2U; ++wheel)
    {
        const int16_t feedback = wheel == 0U ? left : right;
        if (enabled == 0U || target <= 0.0f || (float)feedback <= limit)
            state->high_ticks[wheel] = 0U;
        else if (state->high_ticks[wheel] < 10U)
            ++state->high_ticks[wheel];

        if (state->high_ticks[wheel] >= 10U)
            fault_mask = (uint8_t)(fault_mask | (uint8_t)(1U << wheel));
    }
    return fault_mask;
}

static inline void motor_feedback_watchdog_reset(motor_feedback_watchdog_t *state)
{
    if (state == 0)
        return;

    for (uint8_t wheel = 0U; wheel < 2U; ++wheel)
    {
        state->window_ticks[wheel] = 0U;
        state->positive_counts[wheel] = 0U;
        state->bad_windows[wheel] = 0U;
    }
}

/*
 * Call every 10 ms with encoder counts for the preceding interval and its
 * actual PWM, not the next command. enabled must mean continuously running
 * across that interval. PWM below 500, or disabled, clears that wheel's state.
 * Ten monitored samples form a 100 ms window. Fewer than two positive encoder
 * counts in five consecutive windows reports bit 0 (left) / bit 1 (right),
 * first possible at 500 ms. This already includes the startup allowance.
 * Counts saturate at two, bad windows at five; negative feedback is NOT made
 * positive and remains the responsibility of motor_feedback_fault_mask().
 * The caller must latch the returned fault into its manual-restart stop state.
 */
static inline uint8_t motor_feedback_watchdog_sample(
    motor_feedback_watchdog_t *state, uint8_t enabled, int16_t left, int16_t right,
    uint32_t previous_pwm_left, uint32_t previous_pwm_right)
{
    uint8_t fault_mask = 0U;

    if (state == 0)
        return 0U;

    for (uint8_t wheel = 0U; wheel < 2U; ++wheel)
    {
        const int16_t feedback = wheel == 0U ? left : right;
        const uint32_t previous_pwm = wheel == 0U ? previous_pwm_left : previous_pwm_right;

        if (enabled == 0U || previous_pwm < 500U)
        {
            state->window_ticks[wheel] = 0U;
            state->positive_counts[wheel] = 0U;
            state->bad_windows[wheel] = 0U;
            continue;
        }

        if (feedback > 0 && state->positive_counts[wheel] < 2U)
        {
            const uint32_t total = (uint32_t)state->positive_counts[wheel] +
                                   (uint32_t)feedback;
            state->positive_counts[wheel] = total >= 2U ? 2U : (uint8_t)total;
        }

        if (state->window_ticks[wheel] < 10U)
            ++state->window_ticks[wheel];
        if (state->window_ticks[wheel] >= 10U)
        {
            if (state->positive_counts[wheel] < 2U)
            {
                if (state->bad_windows[wheel] < 5U)
                    ++state->bad_windows[wheel];
            }
            else
            {
                state->bad_windows[wheel] = 0U;
            }
            state->window_ticks[wheel] = 0U;
            state->positive_counts[wheel] = 0U;
        }

        if (state->bad_windows[wheel] >= 5U)
            fault_mask = (uint8_t)(fault_mask | (uint8_t)(1U << wheel));
    }
    return fault_mask;
}

/*
 * Call every 10 ms with pressed != 0 for an active button.
 * First observation/change starts at zero; three further equal observations
 * establish a stable level after at least 30 ms (four observations in total).
 * Return 1 only when a newly accepted press toggles enabled; otherwise return 0.
 */
static inline uint8_t motor_switch_sample(motor_switch_t *state, uint8_t pressed)
{
    pressed = pressed != 0U ? 1U : 0U;
    if (state->observed == 0U || pressed != state->sample_pressed)
    {
        state->observed = 1U;
        state->sample_pressed = pressed;
        state->sample_count = 0U;
        return 0U;
    }
    else if (state->sample_count < 3U)
    {
        ++state->sample_count;
    }

    if (state->sample_count < 3U)
        return 0U;

    if (pressed == 0U)
    {
        state->armed = 1U;
    }
    else if (state->armed != 0U)
    {
        state->armed = 0U;
        state->enabled = state->enabled == 0U ? 1U : 0U;
        return 1U;
    }
    return 0U;
}

#endif
