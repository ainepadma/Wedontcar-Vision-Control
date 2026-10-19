#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "motor_switch.h"

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); \
        exit(EXIT_FAILURE); \
    } \
} while (0)

static void no_toggle(motor_switch_t *state, uint8_t pressed,
                      unsigned int count, uint8_t expected_enabled)
{
    for (unsigned int i = 0; i < count; ++i)
    {
        CHECK(motor_switch_sample(state, pressed) == 0U);
        CHECK(state->enabled == expected_enabled);
        CHECK(state->sample_count <= 3U);
    }
}

static void accepted_press(motor_switch_t *state, uint8_t expected_enabled)
{
    uint8_t old_enabled = state->enabled;
    no_toggle(state, 1U, 3U, old_enabled);
    CHECK(motor_switch_sample(state, 1U) == 1U);
    CHECK(state->enabled == expected_enabled);
    CHECK(state->armed == 0U);
}

static void test_default_and_startup_hold(void)
{
    motor_switch_t state = {0};
    CHECK(state.enabled == 0U && state.armed == 0U);
    no_toggle(&state, 1U, 1000U, 0U);
    CHECK(state.armed == 0U);

    /* A brief release during the power-on hold must not arm the button. */
    no_toggle(&state, 0U, 3U, 0U);
    no_toggle(&state, 1U, 20U, 0U);
    CHECK(state.armed == 0U);
    no_toggle(&state, 0U, 4U, 0U);
    CHECK(state.armed == 1U);
    accepted_press(&state, 1U);
    puts("PASS default stop and startup-held button");
}

static void test_thirty_ms_and_repeat(void)
{
    motor_switch_t state = {0};
    /* At t=0, 10, 20 ms the input is not yet stable for the required 30 ms. */
    no_toggle(&state, 0U, 3U, 0U);
    CHECK(state.armed == 0U);
    no_toggle(&state, 0U, 1U, 0U);
    CHECK(state.armed == 1U);

    for (unsigned int cycle = 0; cycle < 20U; ++cycle)
    {
        uint8_t expected = state.enabled == 0U ? 1U : 0U;
        accepted_press(&state, expected);
        no_toggle(&state, 1U, 1000U, expected); /* Long press toggles only once. */
        no_toggle(&state, 0U, 3U, expected);
        CHECK(state.armed == 0U);
        no_toggle(&state, 0U, 1U, expected);
        CHECK(state.armed == 1U);
    }
    CHECK(state.enabled == 0U);
    puts("PASS at least 30 ms debounce, long hold, repeated start/stop");
}

static void test_bounce(void)
{
    motor_switch_t state = {0};
    no_toggle(&state, 0U, 4U, 0U);
    const uint8_t bounce[] = {1U, 0U, 1U, 1U, 0U, 1U, 0U, 0U};
    for (unsigned int i = 0; i < sizeof(bounce) / sizeof(bounce[0]); ++i)
        no_toggle(&state, bounce[i], 1U, 0U);
    accepted_press(&state, 1U);

    /* Fewer than four released observations cannot re-arm during a held press. */
    no_toggle(&state, 0U, 1U, 1U);
    no_toggle(&state, 1U, 3U, 1U);
    no_toggle(&state, 0U, 2U, 1U);
    no_toggle(&state, 1U, 3U, 1U);
    no_toggle(&state, 0U, 3U, 1U);
    no_toggle(&state, 1U, 4U, 1U);
    CHECK(state.armed == 0U);
    no_toggle(&state, 0U, 4U, 1U);
    accepted_press(&state, 0U);
    puts("PASS press bounce and release bounce");
}

static void test_nonzero_pressed(void)
{
    motor_switch_t state = {0};
    no_toggle(&state, 0U, 4U, 0U);
    no_toggle(&state, 255U, 1U, 0U);
    no_toggle(&state, 2U, 1U, 0U);
    no_toggle(&state, 1U, 1U, 0U);
    CHECK(motor_switch_sample(&state, 128U) == 1U);
    CHECK(state.enabled == 1U);
    no_toggle(&state, 255U, 100U, 1U);
    puts("PASS nonzero pressed values are normalized");
}

static void test_feedback_fault_mask(void)
{
    static const struct
    {
        int16_t left;
        int16_t right;
        uint8_t expected;
    } cases[] = {
        {0, 0, 0U},
        {1, 0, 0U},
        {0, 1, 0U},
        {1, 1, 0U},
        {INT16_MAX, INT16_MAX, 0U},
        {-1, 0, 1U},
        {-1, 1, 1U},
        {INT16_MIN, INT16_MAX, 1U},
        {0, -1, 2U},
        {1, -1, 2U},
        {INT16_MAX, INT16_MIN, 2U},
        {-1, -1, 3U},
        {INT16_MIN, -1, 3U},
        {-1, INT16_MIN, 3U},
        {INT16_MIN, INT16_MIN, 3U}
    };

    for (unsigned int i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i)
        CHECK(motor_feedback_fault_mask(cases[i].left, cases[i].right) == cases[i].expected);
    puts("PASS feedback fault masks, zero/positive speeds, and int16 limits");
}

static void test_fault_stop_requires_release(void)
{
    static const int16_t feedback[][2] = {
        {-1, 1}, {1, -1}, {INT16_MIN, INT16_MIN}
    };

    for (unsigned int i = 0; i < sizeof(feedback) / sizeof(feedback[0]); ++i)
    {
        motor_switch_t state = {0};
        no_toggle(&state, 0U, 4U, 0U);
        accepted_press(&state, 1U);
        no_toggle(&state, 1U, 20U, 1U);

        CHECK(motor_feedback_fault_mask(feedback[i][0], feedback[i][1]) != 0U);
        state.enabled = 0U; /* Mirror the ISR's latched stop after negative feedback. */
        no_toggle(&state, 1U, 1000U, 0U);
        CHECK(state.armed == 0U);

        /* A release shorter than 30 ms still cannot permit a restart. */
        no_toggle(&state, 0U, 3U, 0U);
        no_toggle(&state, 1U, 1000U, 0U);
        CHECK(state.armed == 0U);

        no_toggle(&state, 0U, 4U, 0U);
        CHECK(state.armed == 1U);
        accepted_press(&state, 1U);
        no_toggle(&state, 1U, 1000U, 1U);
    }
    puts("PASS fault stop remains stopped on hold until stable release and new press");
}

static void watchdog_bounds(const motor_feedback_watchdog_t *state)
{
    for (unsigned int wheel = 0U; wheel < 2U; ++wheel)
    {
        CHECK(state->window_ticks[wheel] < 10U);
        CHECK(state->positive_counts[wheel] <= 2U);
        CHECK(state->bad_windows[wheel] <= 5U);
    }
}

static void watchdog_cleared(const motor_feedback_watchdog_t *state, unsigned int wheel)
{
    CHECK(state->window_ticks[wheel] == 0U);
    CHECK(state->positive_counts[wheel] == 0U);
    CHECK(state->bad_windows[wheel] == 0U);
}

static void test_watchdog_500ms_masks(void)
{
    static const struct
    {
        int16_t left;
        int16_t right;
        uint8_t expected;
    } cases[] = {{0, 2, 1U}, {2, 0, 2U}, {0, 0, 3U}};

    for (unsigned int test = 0U; test < sizeof(cases) / sizeof(cases[0]); ++test)
    {
        motor_feedback_watchdog_t state = {0};
        for (unsigned int tick = 1U; tick <= 50U; ++tick)
        {
            const uint8_t fault = motor_feedback_watchdog_sample(
                &state, 1U, cases[test].left, cases[test].right, 500U, 500U);
            CHECK(fault == (tick < 50U ? 0U : cases[test].expected));
            watchdog_bounds(&state);
        }
    }
    puts("PASS watchdog left/right/both masks and exact 490/500 ms boundary");
}

static void test_watchdog_disable_and_low_pwm(void)
{
    motor_feedback_watchdog_t state = {0};

    /* The PWM threshold is inclusive and evaluated independently per wheel. */
    for (unsigned int tick = 1U; tick <= 50U; ++tick)
    {
        CHECK(motor_feedback_watchdog_sample(&state, 1U, 0, 0, 500U, 499U) ==
              (tick < 50U ? 0U : 1U));
        watchdog_cleared(&state, 1U);
    }
    CHECK(motor_feedback_watchdog_sample(&state, 0U, INT16_MAX, INT16_MAX,
                                         UINT32_MAX, UINT32_MAX) == 0U);
    watchdog_cleared(&state, 0U);
    watchdog_cleared(&state, 1U);

    /* Stop and low PWM both clear a partial window and the four bad windows. */
    for (unsigned int mode = 0U; mode < 2U; ++mode)
    {
        for (unsigned int tick = 0U; tick < 49U; ++tick)
            CHECK(motor_feedback_watchdog_sample(&state, 1U, 0, 0, 500U, 500U) == 0U);
        CHECK(state.bad_windows[0] == 4U && state.window_ticks[0] == 9U);
        CHECK(motor_feedback_watchdog_sample(&state, (uint8_t)mode, 0, 0,
                                             mode == 0U ? 500U : 499U,
                                             mode == 0U ? 500U : 499U) == 0U);
        watchdog_cleared(&state, 0U);
        watchdog_cleared(&state, 1U);
    }

    /* Resetting only the left wheel does not delay the right wheel's fault. */
    for (unsigned int tick = 0U; tick < 45U; ++tick)
        CHECK(motor_feedback_watchdog_sample(&state, 1U, 0, 0, 500U, 500U) == 0U);
    CHECK(motor_feedback_watchdog_sample(&state, 1U, 0, 0, 499U, 500U) == 0U);
    watchdog_cleared(&state, 0U);
    for (unsigned int tick = 1U; tick <= 4U; ++tick)
        CHECK(motor_feedback_watchdog_sample(&state, 1U, 0, 0, 500U, 500U) ==
              (tick < 4U ? 0U : 2U));
    CHECK(state.window_ticks[0] == 4U && state.bad_windows[0] == 0U);

    /* Explicit reset also clears latched window results and partial counts. */
    motor_feedback_watchdog_reset(&state);
    watchdog_cleared(&state, 0U);
    watchdog_cleared(&state, 1U);
    for (unsigned int tick = 1U; tick <= 50U; ++tick)
        CHECK(motor_feedback_watchdog_sample(&state, 1U, 0, 0, 500U, 500U) ==
              (tick < 50U ? 0U : 3U));
    puts("PASS watchdog disabled/low PWM/reset clearing and independent wheels");
}

static void test_watchdog_window_feedback(void)
{
    motor_feedback_watchdog_t state = {0};

    /* Intermittent zero samples are valid when the whole window totals two. */
    for (unsigned int window = 0U; window < 1000U; ++window)
    {
        for (unsigned int tick = 0U; tick < 10U; ++tick)
        {
            const int16_t left = tick == 9U ? 2 : 0;
            const int16_t right = tick == 2U || tick == 7U ? 1 : 0;
            CHECK(motor_feedback_watchdog_sample(&state, 255U, left, right,
                                                 500U, UINT32_MAX) == 0U);
            watchdog_bounds(&state);
        }
        CHECK(state.bad_windows[0] == 0U && state.bad_windows[1] == 0U);
    }

    /* One positive count per 100 ms is insufficient, even with no zero window. */
    for (unsigned int tick = 1U; tick <= 50U; ++tick)
    {
        const int16_t feedback = tick % 10U == 0U ? 1 : 0;
        CHECK(motor_feedback_watchdog_sample(&state, 1U, feedback, feedback,
                                             500U, 500U) == (tick < 50U ? 0U : 3U));
    }

    motor_feedback_watchdog_reset(&state);
    for (unsigned int tick = 0U; tick < 40U; ++tick)
        CHECK(motor_feedback_watchdog_sample(&state, 1U, 0, 0, 500U, 500U) == 0U);
    CHECK(state.bad_windows[0] == 4U && state.bad_windows[1] == 4U);
    for (unsigned int tick = 0U; tick < 10U; ++tick)
        CHECK(motor_feedback_watchdog_sample(&state, 1U, tick == 9U ? 2 : 0,
                                             tick == 9U ? 2 : 0, 500U, 500U) == 0U);
    CHECK(state.bad_windows[0] == 0U && state.bad_windows[1] == 0U);
    for (unsigned int tick = 1U; tick <= 50U; ++tick)
        CHECK(motor_feedback_watchdog_sample(&state, 1U, 0, 0, 500U, 500U) ==
              (tick < 50U ? 0U : 3U));
    puts("PASS watchdog window accumulation, insufficient one-count windows, recovery reset");
}

static void test_watchdog_saturation_and_negative(void)
{
    motor_feedback_watchdog_t state = {0};

    /* Negative counts are ignored, never abs()-converted to positive movement. */
    for (unsigned int tick = 1U; tick <= 50U; ++tick)
    {
        CHECK(motor_feedback_watchdog_sample(&state, 1U, -1, INT16_MIN,
                                             UINT32_MAX, UINT32_MAX) ==
              (tick < 50U ? 0U : 3U));
        CHECK(state.positive_counts[0] == 0U && state.positive_counts[1] == 0U);
    }
    for (unsigned int tick = 0U; tick < 100000U; ++tick)
    {
        CHECK(motor_feedback_watchdog_sample(&state, 1U, 0, 0,
                                             UINT32_MAX, UINT32_MAX) == 3U);
        CHECK(state.bad_windows[0] == 5U && state.bad_windows[1] == 5U);
        watchdog_bounds(&state);
    }

    /* The helper reports windows; the controller, not this helper, latches stop. */
    for (unsigned int tick = 1U; tick <= 10U; ++tick)
    {
        CHECK(motor_feedback_watchdog_sample(&state, 1U, INT16_MAX, INT16_MAX,
                                             UINT32_MAX, UINT32_MAX) ==
              (tick < 10U ? 3U : 0U));
        CHECK(state.positive_counts[0] == (tick < 10U ? 2U : 0U));
        watchdog_bounds(&state);
    }
    for (unsigned int tick = 0U; tick < 100000U; ++tick)
    {
        CHECK(motor_feedback_watchdog_sample(&state, 1U, INT16_MAX, INT16_MAX,
                                             UINT32_MAX, UINT32_MAX) == 0U);
        watchdog_bounds(&state);
    }
    motor_feedback_watchdog_reset(0);
    CHECK(motor_feedback_watchdog_sample(0, 1U, 0, 0, 500U, 500U) == 0U);
    puts("PASS watchdog saturation, long runtime, signed extremes, and null guards");
}

static void test_overspeed_watchdog(void)
{
    motor_overspeed_watchdog_t state = {0};

    /* At target 120 the strict fault boundary is ten samples above 260. */
    for (unsigned int tick = 1U; tick <= 100U; ++tick)
        CHECK(motor_overspeed_watchdog_sample(&state, 1U, 260, 240, 120.0f) == 0U);
    for (unsigned int tick = 1U; tick <= 10U; ++tick)
        CHECK(motor_overspeed_watchdog_sample(&state, 1U, 261, 240, 120.0f) ==
              (tick < 10U ? 0U : 1U));

    motor_overspeed_watchdog_reset(&state);
    for (unsigned int tick = 1U; tick <= 9U; ++tick)
        CHECK(motor_overspeed_watchdog_sample(&state, 1U, 300, 300, 120.0f) == 0U);
    CHECK(motor_overspeed_watchdog_sample(&state, 0U, INT16_MAX, INT16_MAX,
                                          120.0f) == 0U);
    CHECK(state.high_ticks[0] == 0U && state.high_ticks[1] == 0U);

    for (unsigned int tick = 1U; tick <= 10U; ++tick)
        CHECK(motor_overspeed_watchdog_sample(&state, 1U, 0, 501, 240.0f) ==
              (tick < 10U ? 0U : 2U));
    motor_overspeed_watchdog_reset(0);
    CHECK(motor_overspeed_watchdog_sample(0, 1U, INT16_MAX, INT16_MAX,
                                          1.0f) == 0U);
    puts("PASS overspeed persistence, low/full thresholds, disable/reset, and masks");
}

int main(void)
{
    test_default_and_startup_hold();
    test_thirty_ms_and_repeat();
    test_bounce();
    test_nonzero_pressed();
    test_feedback_fault_mask();
    test_fault_stop_requires_release();
    test_watchdog_500ms_masks();
    test_watchdog_disable_and_low_pwm();
    test_watchdog_window_feedback();
    test_watchdog_saturation_and_negative();
    test_overspeed_watchdog();
    puts("All motor-switch host tests passed.");
    return EXIT_SUCCESS;
}
