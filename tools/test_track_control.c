#include <float.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "track_control.h"

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); \
        exit(EXIT_FAILURE); \
    } \
} while (0)

static const track_control_config_t config = {
    87.0f, 98.5f, 75.5f, 20.0f, 10.0f, 0.02f, 0.0005f, 60.0f
};

static void qualify(track_control_state_t *state, int16_t error, int16_t heading,
                    uint32_t first_ms)
{
    for (uint32_t i = 0U; i < 3U; ++i)
        track_control_observe(state, error, heading, 100U, 1U, first_ms + i * 20U);
    CHECK(state->ready_frames == 3U);
}

static void check_output(track_control_output_t output)
{
    CHECK(isfinite(output.servo_angle));
    CHECK(isfinite(output.target_speed));
    CHECK(output.target_speed >= 0.0f);
    if (!output.running)
        CHECK(output.target_speed == 0.0f);
}

static void test_readiness(void)
{
    track_control_state_t state;
    track_control_init(&state, &config);
    track_control_output_t output = track_control_tick(&state, 0U);
    CHECK(!output.running && !output.ready && output.target_speed == 0.0f);
    CHECK(output.servo_angle == config.servo_center);
    track_control_toggle(&state, 0U);
    CHECK(!state.running && state.stop_reason == TRACK_STOP_NOT_READY);
    track_control_observe(&state, 0, 0, 100U, 1U, 0U);
    for (unsigned int i = 0U; i < 20U; ++i)
        track_control_observe(&state, 0, 0, 100U, 1U, 0U);
    CHECK(state.ready_frames == 1U);
    track_control_observe(&state, 0, 0, 44U, 1U, 20U);
    CHECK(state.ready_frames == 0U);
    qualify(&state, 0, 0, 40U);
    CHECK(!state.running);
    track_control_toggle(&state, 80U);
    output = track_control_tick(&state, 90U);
    CHECK(output.running && output.ready && output.stop_reason == TRACK_STOP_NONE);
    CHECK(output.target_speed == config.speed_straight);
    track_control_toggle(&state, 90U);
    output = track_control_tick(&state, 100U);
    CHECK(!output.running && output.stop_reason == TRACK_STOP_MANUAL);
    check_output(output);
    puts("PASS default stop, confidence, distinct frames, manual start/stop");
}

static void test_steering_and_speed(void)
{
    for (int sign = -1; sign <= 1; sign += 2)
    {
        track_control_state_t state;
        track_control_init(&state, &config);
        qualify(&state, (int16_t)(sign * 1000), 0, 0U);
        track_control_toggle(&state, 40U);
        float previous = config.servo_center;
        for (uint32_t now = 50U; now <= 350U; now += 10U)
        {
            if (now % 20U == 0U)
                track_control_observe(&state, (int16_t)(sign * 1000), 0, 100U, 1U, now);
            track_control_output_t output = track_control_tick(&state, now);
            CHECK(output.running);
            CHECK(fabsf(output.servo_angle - previous) <= 0.60001f);
            CHECK(output.servo_angle >= config.servo_right &&
                  output.servo_angle <= config.servo_left);
            CHECK(sign > 0 ? output.servo_angle <= previous : output.servo_angle >= previous);
            CHECK(output.target_speed == config.speed_curve_min);
            previous = output.servo_angle;
        }
        CHECK(previous == (sign > 0 ? config.servo_right : config.servo_left));
    }
    track_control_config_t reverse = config;
    reverse.servo_left = 85.0f;
    reverse.servo_right = 100.0f;
    track_control_state_t state;
    track_control_init(&state, &reverse);
    qualify(&state, 500, 0, 0U);
    track_control_toggle(&state, 40U);
    CHECK(track_control_tick(&state, 50U).servo_angle > config.servo_center);

    /* On a bend, look-ahead heading must override a small opposite lateral
     * offset; this is the measured curve-centre condition on the real car. */
    track_control_init(&state, &config);
    qualify(&state, -100, 400, 0U);
    track_control_toggle(&state, 40U);
    CHECK(track_control_tick(&state, 50U).servo_angle < config.servo_center);
    track_control_init(&state, &config);
    qualify(&state, 100, -400, 0U);
    track_control_toggle(&state, 40U);
    CHECK(track_control_tick(&state, 50U).servo_angle > config.servo_center);
    puts("PASS semantic steering direction, heading-dominant bends, endpoints, slew rate and slowdown");
}

static void test_independent_servo_center(void)
{
    track_control_state_t defaults;
    track_control_init(&defaults, NULL);
    CHECK(defaults.config.servo_center == 87.0f);
    CHECK(config.servo_center == 87.0f);

    track_control_config_t asymmetric = config;
    asymmetric.servo_center = 90.25f;
    CHECK(asymmetric.servo_center !=
          (asymmetric.servo_left + asymmetric.servo_right) * 0.5f);
    for (int sign = -1; sign <= 1; sign += 2)
    {
        track_control_state_t state;
        track_control_init(&state, &asymmetric);
        CHECK(state.servo_angle == asymmetric.servo_center);
        qualify(&state, (int16_t)(sign * 1000), 0, 0U);
        track_control_toggle(&state, 40U);
        float previous = asymmetric.servo_center;
        for (uint32_t now = 50U; now <= 440U; now += 10U)
        {
            if (now % 20U == 0U)
                track_control_observe(&state, (int16_t)(sign * 1000), 0,
                                      100U, 1U, now);
            track_control_output_t output = track_control_tick(&state, now);
            check_output(output);
            CHECK(output.running);
            CHECK(output.servo_angle >= asymmetric.servo_right);
            CHECK(output.servo_angle <= asymmetric.servo_left);
            CHECK(fabsf(output.servo_angle - previous) <= 0.60001f);
            CHECK(sign > 0 ? output.servo_angle <= previous : output.servo_angle >= previous);
            if (now == 50U)
            {
                CHECK(sign > 0 ? output.servo_angle < asymmetric.servo_center
                               : output.servo_angle > asymmetric.servo_center);
                CHECK(fabsf(fabsf(output.servo_angle - previous) - 0.6f) < 0.00001f);
            }
            previous = output.servo_angle;
        }
        CHECK(previous == (sign > 0 ? asymmetric.servo_right : asymmetric.servo_left));

        /* Keep the default PD/filter settings and allow their transient to settle.
         * Zero error must return to the calibrated center, not endpoint average. */
        for (uint32_t now = 450U; now <= 2440U; now += 10U)
        {
            if (now % 20U == 0U)
                track_control_observe(&state, 0, 0, 100U, 1U, now);
            track_control_output_t output = track_control_tick(&state, now);
            check_output(output);
            CHECK(output.running);
            CHECK(output.servo_angle >= asymmetric.servo_right);
            CHECK(output.servo_angle <= asymmetric.servo_left);
            CHECK(fabsf(output.servo_angle - previous) <= 0.60001f);
            previous = output.servo_angle;
        }
        CHECK(fabsf(previous - asymmetric.servo_center) < 0.0001f);
        CHECK(fabsf(previous - 87.0f) > 3.0f);
    }
    puts("PASS independent calibrated center, both directions, zero return, bounds/rate, default 87.0");
}

static void test_loss_stale_and_encoder(void)
{
    const uint8_t reasons[] = {
        TRACK_STOP_LOST, TRACK_STOP_STALE, TRACK_STOP_ENCODER, TRACK_STOP_NO_FEEDBACK,
        TRACK_STOP_OVERSPEED
    };
    for (unsigned int i = 0U; i < sizeof(reasons) / sizeof(reasons[0]); ++i)
    {
        track_control_state_t state;
        track_control_init(&state, &config);
        qualify(&state, 0, 0, 0U);
        track_control_toggle(&state, 40U);
        if (reasons[i] == TRACK_STOP_LOST)
        {
            track_control_observe(&state, 0, 0, 100U, 0U, 60U);
            CHECK(state.running && state.lost_frames == 1U);
            CHECK(track_control_tick(&state, 60U).target_speed == config.speed_curve_min);
            track_control_observe(&state, 0, 0, 100U, 0U, 80U);
            CHECK(state.running && state.lost_frames == 2U);
            track_control_observe(&state, 0, 0, 100U, 0U, 100U);
        }
        else if (reasons[i] == TRACK_STOP_STALE)
        {
            CHECK(track_control_tick(&state, 190U).running);
            (void)track_control_tick(&state, 191U);
        }
        else
            track_control_stop(&state, reasons[i]);
        track_control_output_t output = track_control_tick(&state, 200U);
        CHECK(!output.running && !output.ready && output.stop_reason == reasons[i]);
        check_output(output);
        qualify(&state, 0, 0, 220U);
        output = track_control_tick(&state, 270U);
        CHECK(!output.running && output.ready && output.stop_reason == reasons[i]);
        check_output(output);
        track_control_toggle(&state, 270U);
        CHECK(track_control_tick(&state, 280U).running);
    }
    track_control_state_t state;
    track_control_init(&state, &config);
    qualify(&state, 0, 0, 0U);
    track_control_toggle(&state, 40U);
    track_control_observe(&state, 0, 0, 100U, 1U, 200U);
    CHECK(!state.running && state.stop_reason == TRACK_STOP_STALE && state.ready_frames == 1U);
    puts("PASS 3-frame lost grace, stale/reverse/no-feedback latches, recovery needs toggle");
}

static void test_default_speed_profile(void)
{
    track_control_state_t state;
    track_control_init(&state, NULL);
    CHECK(TRACK_SPEED_STRAIGHT == 120.0f && TRACK_SPEED_CURVE_MIN == 90.0f);
    CHECK(TRACK_SPEED_FULL_STRAIGHT == 253.0f && TRACK_SPEED_FULL_CURVE_MIN == 145.0f);
    CHECK(state.config.speed_straight == TRACK_SPEED_STRAIGHT);
    CHECK(state.config.speed_curve_min == TRACK_SPEED_CURVE_MIN);
    qualify(&state, 0, 0, 0U);
    track_control_toggle(&state, 40U);
    track_control_output_t output = track_control_tick(&state, 50U);
    CHECK(output.running && output.target_speed == 120.0f);
    track_control_init(&state, NULL);
    qualify(&state, 1000, 0, 0U);
    track_control_toggle(&state, 40U);
    output = track_control_tick(&state, 50U);
    CHECK(output.running && output.target_speed == 90.0f);
    for (uint32_t now = 60U; now <= 400U; now += 10U)
    {
        track_control_observe(&state, 0, 0, 100U, 1U, now);
        output = track_control_tick(&state, now);
        CHECK(output.running && output.target_speed >= 90.0f && output.target_speed <= 120.0f);
    }
    track_control_toggle(&state, 400U);
    output = track_control_tick(&state, 410U);
    CHECK(!output.running && output.target_speed == 0.0f);
    puts("PASS production speed defaults LOW 120/90, FULL 253/145, bounds and stopped zero target");
}

static void test_loss_stop_switch(void)
{
    track_control_state_t state;
    track_control_output_t output;
    track_control_init(&state, &config);
    CHECK(state.loss_stop_enabled == 1U);

    /* C14 override permits an explicit C13 start without a valid track. */
    track_control_set_loss_stop(&state, 0U);
    track_control_toggle(&state, 0U);
    CHECK(state.running && state.stop_reason == TRACK_STOP_NONE);
    output = track_control_tick(&state, 10U);
    CHECK(output.running && !output.ready);
    CHECK(output.target_speed == state.config.speed_curve_min);
    track_control_toggle(&state, 10U);
    CHECK(!state.running && state.stop_reason == TRACK_STOP_MANUAL);

    qualify(&state, 300, 100, 20U);
    track_control_toggle(&state, 60U);
    CHECK(state.running);
    track_control_observe(&state, 0, 0, 0U, 0U, 80U);
    output = track_control_tick(&state, 80U);
    CHECK(output.running && !output.ready);
    CHECK(output.stop_reason == TRACK_STOP_NONE && output.target_speed > 0.0f);
    CHECK(state.loss_stop_enabled == 0U);

    /* Camera loss and staleness no longer stop, but observations still run. */
    output = track_control_tick(&state, 400U);
    CHECK(output.running && !output.ready && output.target_speed > 0.0f);
    qualify(&state, -200, -100, 420U);
    output = track_control_tick(&state, 460U);
    CHECK(output.running && output.ready);

    /* Re-enabling the protection while the current image is invalid stops on
     * the next control tick; it never auto-restarts. */
    track_control_observe(&state, 0, 0, 0U, 0U, 480U);
    CHECK(state.running);
    track_control_set_loss_stop(&state, 1U);
    output = track_control_tick(&state, 480U);
    CHECK(!output.running && !output.ready);
    CHECK(output.stop_reason == TRACK_STOP_LOST);
    CHECK(state.loss_stop_enabled == 1U);

    track_control_set_loss_stop(NULL, 0U);
    puts("PASS C14 free-run override bypasses vision gates but retains explicit manual control");
}

static void test_loss_stop_reenable_reasons(void)
{
    track_control_state_t state;

    /* No camera publication at all: C14 must close the deliberate free-run
     * session as NOT_READY, and it must not auto-restart later. */
    track_control_init(&state, &config);
    track_control_set_loss_stop(&state, 0U);
    track_control_toggle(&state, 0U);
    CHECK(state.running);
    track_control_set_loss_stop(&state, 1U);
    CHECK(!state.running && state.stop_reason == TRACK_STOP_NOT_READY);
    track_control_observe(&state, 0, 0, 100U, 1U, 20U);
    CHECK(!state.running);

    /* A current invalid observation is LOST, not NOT_READY. */
    track_control_init(&state, &config);
    track_control_set_loss_stop(&state, 0U);
    qualify(&state, 0, 0, 0U);
    track_control_toggle(&state, 40U);
    track_control_observe(&state, 0, 0, 0U, 0U, 60U);
    CHECK(state.running);
    track_control_set_loss_stop(&state, 1U);
    CHECK(!state.running && state.stop_reason == TRACK_STOP_LOST);

    /* If the last publication has already aged out while C14 is disabled,
     * preserve STALE as the cause when protection is re-enabled. */
    track_control_init(&state, &config);
    track_control_set_loss_stop(&state, 0U);
    qualify(&state, 0, 0, 0U);
    track_control_toggle(&state, 40U);
    CHECK(track_control_tick(&state, 191U).running);
    track_control_set_loss_stop(&state, 1U);
    CHECK(!state.running && state.stop_reason == TRACK_STOP_STALE);

    /* A fresh valid frame must remain running when C14 is re-enabled. */
    track_control_init(&state, &config);
    track_control_set_loss_stop(&state, 0U);
    qualify(&state, 0, 0, 0U);
    track_control_toggle(&state, 40U);
    track_control_set_loss_stop(&state, 1U);
    CHECK(state.running && state.stop_reason == TRACK_STOP_NONE);
    puts("PASS C14 re-enable distinguishes no-frame, lost, stale and fresh states");
}

static void test_invalid_frame_holds_steering(void)
{
    track_control_state_t state;
    track_control_output_t output;
    float held_angle;

    track_control_init(&state, &config);
    qualify(&state, 1000, 0, 0U);
    track_control_toggle(&state, 40U);
    output = track_control_tick(&state, 50U);
    held_angle = output.servo_angle;

    /* The first two bad frames use the grace period but must not continue
     * steering toward a target derived from stale geometry. */
    track_control_observe(&state, 0, 0, 0U, 0U, 60U);
    output = track_control_tick(&state, 60U);
    CHECK(output.running && output.target_speed == config.speed_curve_min);
    CHECK(output.servo_angle == held_angle);
    CHECK(state.derivative == 0.0f && !state.derivative_ready);
    track_control_observe(&state, 0, 0, 0U, 0U, 80U);
    output = track_control_tick(&state, 80U);
    CHECK(output.running && output.servo_angle == held_angle);

    /* With C14 disabled, stale data may not stop the motors, but steering and
     * derivative state must still be frozen/reset until a new valid frame. */
    track_control_init(&state, &config);
    track_control_set_loss_stop(&state, 0U);
    qualify(&state, -1000, 0, 0U);
    track_control_toggle(&state, 40U);
    held_angle = track_control_tick(&state, 50U).servo_angle;
    state.derivative = 123.0f;
    state.derivative_ready = 1U;
    output = track_control_tick(&state, 191U);
    CHECK(output.running && output.target_speed == config.speed_curve_min);
    CHECK(output.servo_angle == held_angle);
    CHECK(state.derivative == 0.0f && !state.derivative_ready);

    /* The first recovered frame starts a new derivative sequence and remains
     * subject to the configured servo slew limit. */
    track_control_observe(&state, 1000, 0, 100U, 1U, 200U);
    CHECK(state.derivative == 0.0f && state.derivative_ready);
    output = track_control_tick(&state, 201U);
    CHECK(fabsf(output.servo_angle - held_angle) <=
          config.max_servo_rate_deg_s * 0.01001f);
    puts("PASS invalid/stale frames hold steering and recovery resets derivative");
}

static void test_timing_and_pd(void)
{
    track_control_state_t state;
    track_control_init(&state, &config);
    qualify(&state, 1000, 0, UINT32_MAX - 49U);
    track_control_toggle(&state, UINT32_MAX - 9U);
    track_control_output_t output = track_control_tick(&state, 0U);
    CHECK(output.running && fabsf(output.servo_angle - (config.servo_center - 0.6f)) < 0.0001f);
    track_control_observe(&state, 1000, 0, 100U, 1U, 10U);
    CHECK(state.derivative == 0.0f);
    track_control_observe(&state, 0, 0, 100U, 1U, 30U);
    CHECK(fabsf(state.filtered_error - (2000.0f / 3.0f)) < 0.001f);
    CHECK(fabsf(state.derivative - (-12500.0f / 3.0f)) < 0.01f);
    CHECK(track_control_tick(&state, 180U).running);
    CHECK(!track_control_tick(&state, 181U).running);
    CHECK(state.stop_reason == TRACK_STOP_STALE);
    track_control_init(&state, &config);
    track_control_observe(&state, 0, 0, 100U, 1U, 100U);
    track_control_observe(&state, 0, 0, 100U, 1U, 99U);
    CHECK(state.ready_frames == 1U && state.last_frame_ms == 100U);
    track_control_observe(&state, 0, 0, 100U, 0U, 100U);
    CHECK(state.ready_frames == 0U);
    track_control_init(&state, &config);
    qualify(&state, 0, 0, 0U);
    track_control_toggle(&state, 40U);
    track_control_observe(&state, 0, 0, 60U, 1U, 60U);
    track_control_observe(&state, 600, 0, 60U, 1U, 100U);
    CHECK(fabsf(state.filtered_error - 300.0f) < 0.001f);
    CHECK(fabsf(state.derivative - 3000.0f) < 0.01f);
    track_control_toggle(&state, 100U);
    track_control_toggle(&state, 100U);
    CHECK(state.running && state.derivative == 0.0f);
    track_control_observe(&state, -600, 0, 60U, 1U, 120U);
    CHECK(state.derivative == 0.0f);
    track_control_observe(&state, -600, 0, 44U, 1U, 140U);
    CHECK(state.running && state.lost_frames == 1U);
    track_control_observe(&state, -600, 0, 44U, 1U, 160U);
    CHECK(state.running && state.lost_frames == 2U);
    track_control_observe(&state, -600, 0, 44U, 1U, 180U);
    CHECK(!state.running && state.stop_reason == TRACK_STOP_LOST);
    puts("PASS uint32 wrap, actual-frame-dt filtered derivative, old/duplicate frames");
}

static void test_randomized_invariants(void)
{
    track_control_state_t state;
    uint32_t random = UINT32_C(123456789);
    uint32_t now = UINT32_MAX - 1000U;
    track_control_init(&state, &config);
    for (unsigned int i = 0U; i < 10000U; ++i)
    {
        random = random * UINT32_C(1664525) + UINT32_C(1013904223);
        now += 10U;
        uint8_t previously_running = state.running;
        uint8_t toggled = 0U;
        float previous_angle = state.servo_angle;
        if (i % 2U == 0U)
        {
            int16_t error = (int16_t)((int32_t)(random % 2001U) - 1000);
            int16_t heading = (int16_t)((int32_t)((random >> 8) % 2001U) - 1000);
            track_control_observe(&state, error, heading, 100U,
                                  (uint8_t)(random % 23U != 0U), now);
        }
        if (random % 17U == 0U)
        {
            track_control_toggle(&state, now);
            toggled = 1U;
        }
        if (random % 101U == 0U)
            track_control_stop(&state, TRACK_STOP_ENCODER);
        track_control_output_t output = track_control_tick(&state, now);
        check_output(output);
        CHECK(output.servo_angle >= fminf(state.config.servo_left,
                                          state.config.servo_right));
        CHECK(output.servo_angle <= fmaxf(state.config.servo_left,
                                          state.config.servo_right));
        CHECK(fabsf(output.servo_angle - previous_angle) <= 0.60001f);
        CHECK(state.ready_frames <= 3U);
        CHECK(!output.running || state.frame_valid ||
              !state.loss_stop_enabled ||
              (state.lost_frames > 0U && state.lost_frames < 3U));
        CHECK(!output.running || (output.target_speed >= 10.0f && output.target_speed <= 20.0f));
        CHECK(previously_running || toggled || !output.running);
    }
    puts("PASS 10000 mixed frames/toggles/faults retain bounds and never auto-start");
}

static void test_abnormal_inputs(void)
{
    track_control_state_t state;
    track_control_config_t bad = {NAN, INFINITY, -INFINITY, NAN, INFINITY,
                                  INFINITY, NAN, -1.0f};
    track_control_init(&state, &bad);
    qualify(&state, INT16_MAX, INT16_MIN, 0U);
    track_control_toggle(&state, 40U);
    check_output(track_control_tick(&state, 50U));
    bad = config;
    bad.kp = FLT_MAX;
    bad.kd = FLT_MAX;
    bad.max_servo_rate_deg_s = FLT_MAX;
    bad.speed_straight = FLT_MAX;
    bad.speed_curve_min = -FLT_MAX;
    track_control_init(&state, &bad);
    qualify(&state, INT16_MAX, INT16_MIN, 0U);
    track_control_toggle(&state, 40U);
    for (uint32_t now = 50U; now < 2000U; now += 10U)
    {
        track_control_observe(&state, now % 20U ? INT16_MIN : INT16_MAX,
                              INT16_MAX, 255U, 1U, now);
        check_output(track_control_tick(&state, now));
    }
    track_control_stop(&state, 255U);
    check_output(track_control_tick(&state, 2000U));
    track_control_init(NULL, NULL);
    track_control_observe(NULL, 0, 0, 0U, 0U, 0U);
    track_control_toggle(NULL, 0U);
    track_control_stop(NULL, TRACK_STOP_ENCODER);
    check_output(track_control_tick(NULL, 0U));
    track_control_init(&state, NULL);
    check_output(track_control_tick(&state, 0U));
    puts("PASS bounded malformed samples/configuration, finite outputs and null guards");
}

static float near_case(int sign, int entered, int legacy)
{
    track_control_state_t state;
    track_control_init(&state, &config);
    track_control_set_loss_stop(&state, 0U);
    track_control_toggle(&state, 0U);
    float previous = config.servo_center;
    for (uint32_t t=20U;t<=600U;t+=20U) {
        if (legacy) track_control_observe(&state, (int16_t)(sign*200),
            (int16_t)(sign*200),100U,1U,t);
        else track_control_observe_near(&state,(int16_t)(sign*200),
            (int16_t)(sign*200),(int16_t)(entered?sign*80:0),
            (int16_t)(entered?sign*120:0),1U,100U,1U,t);
        track_control_output_t o=track_control_tick(&state,t);
        CHECK(fabsf(o.servo_angle-previous)<=1.2001f);
        CHECK(o.servo_angle>=75.5f && o.servo_angle<=98.5f);
        previous=o.servo_angle;
    }
    float offset=fabsf(previous-config.servo_center);
    if (!legacy) {
        float saved=state.filtered_near_heading;
        track_control_observe_near(&state,0,0,0,0,1U,100U,1U,600U);
        CHECK(state.filtered_near_heading==saved); /* duplicate cannot change mode */
        track_control_observe_near(&state,0,0,0,0,0U,0U,0U,620U);
        CHECK(!state.near_geometry_valid);
        CHECK(track_control_tick(&state,620U).servo_angle==previous);
        track_control_observe_near(&state,0,0,0,0,1U,100U,1U,640U);
        CHECK(state.derivative==0.0f);
    }
    return offset;
}
static void test_near_adaptive_steering(void)
{
    float mirrored[2];
    for(int sign=-1;sign<=1;sign+=2) {
        float approach=near_case(sign,0,0);
        float baseline=near_case(sign,0,1);
        float bend=near_case(sign,1,0);
        CHECK(approach<baseline*0.3f);
        CHECK(bend>baseline && bend<=11.5f);
        mirrored[sign>0]=bend;
    }
    CHECK(fabsf(mirrored[0]-mirrored[1])<0.001f);
    /* S bend: distant opposite tangent cannot reverse a strong near turn. */
    for(int sign=-1;sign<=1;sign+=2) {
        track_control_state_t state;
        track_control_init(&state,&config);
        track_control_set_loss_stop(&state,0U);
        track_control_toggle(&state,0U);
        for(uint32_t t=20U;t<=400U;t+=20U) {
            track_control_observe_near(&state,0,(int16_t)(-sign*200),
                (int16_t)(sign*80),(int16_t)(sign*120),1U,100U,1U,t);
            track_control_output_t o=track_control_tick(&state,t);
            CHECK(sign>0 ? o.servo_angle<87.0f : o.servo_angle>87.0f);
        }
        /* Leaving the bend removes gain progressively, no angle jump. */
        float previous=state.servo_angle;
        for(uint32_t t=420U;t<=1000U;t+=20U) {
            track_control_observe_near(&state,0,0,0,0,1U,100U,1U,t);
            track_control_output_t o=track_control_tick(&state,t);
            CHECK(fabsf(o.servo_angle-previous)<=1.2001f);
            previous=o.servo_angle;
        }
        CHECK(fabsf(previous-87.0f)<0.1f);
        track_control_observe_near(&state,100,0,0,0,0U,100U,1U,1020U);
        CHECK(!state.near_geometry_valid && state.derivative==0.0f);
    }

    puts("PASS near-adaptive steering: approach suppression, bend boost, mirrors, limits, duplicate and invalid frames");
}

static void bottom_frame(track_control_state_t *s, int16_t e, uint8_t valid, uint32_t t)
{
    track_control_observe_near(s,0,0,0,0,1U,100U,1U,t);
    track_control_observe_bottom(s,e,valid,t);
}
static void test_bottom_pulse(void)
{
    for(int sign=-1;sign<=1;sign+=2) {
        track_control_state_t s;
        track_control_init(&s,&config);
        track_control_set_loss_stop(&s,0U); track_control_toggle(&s,0U);
        bottom_frame(&s,(int16_t)(sign*200),1U,20U);
        CHECK(!s.bottom_active);
        track_control_observe_bottom(&s,(int16_t)(sign*200),1U,20U);
        CHECK(!s.bottom_active); /* duplicate cannot cast a second vote */
        bottom_frame(&s,(int16_t)(sign*200),1U,40U);
        CHECK(s.bottom_active && fabsf(s.bottom_pulse_deg)<=2.0f);
        track_control_output_t o=track_control_tick(&s,40U);
        CHECK(sign>0 ? o.servo_angle<87.0f : o.servo_angle>87.0f);
        float previous=o.servo_angle;
        for(uint32_t t=60;t<=400;t+=20) {
            bottom_frame(&s,(int16_t)(sign*200),1U,t);
            o=track_control_tick(&s,t);
            CHECK(fabsf(o.servo_angle-previous)<=1.2001f);
            CHECK(o.servo_angle>=75.5f && o.servo_angle<=98.5f);
            previous=o.servo_angle;
        }
        CHECK(!s.bottom_active && !s.bottom_armed && fabsf(o.servo_angle-87.0f)<0.01f);
        bottom_frame(&s,0,1U,420U);bottom_frame(&s,0,1U,440U);
        CHECK(s.bottom_armed);
        bottom_frame(&s,(int16_t)(sign*200),1U,460U);
        bottom_frame(&s,(int16_t)(sign*200),1U,480U);CHECK(s.bottom_active);
        bottom_frame(&s,0,0U,500U);CHECK(!s.bottom_active); /* missing bottom */
        bottom_frame(&s,0,1U,520U);bottom_frame(&s,0,1U,540U);
        bottom_frame(&s,(int16_t)(sign*200),1U,560U);
        bottom_frame(&s,(int16_t)(sign*200),1U,580U);CHECK(s.bottom_active);
        bottom_frame(&s,(int16_t)(-sign*200),1U,600U);CHECK(!s.bottom_active);
        track_control_stop(&s,TRACK_STOP_MANUAL);CHECK(!s.bottom_active);
    }
    track_control_state_t s;
    track_control_init(&s,&config);track_control_set_loss_stop(&s,0U);
    track_control_toggle(&s,UINT32_MAX-60U);
    bottom_frame(&s,100,1U,UINT32_MAX-40U);
    bottom_frame(&s,100,1U,UINT32_MAX-20U);CHECK(s.bottom_active);
    bottom_frame(&s,100,1U,20U);check_output(track_control_tick(&s,20U));CHECK(s.bottom_active);
    bottom_frame(&s,100,1U,140U);check_output(track_control_tick(&s,140U));CHECK(!s.bottom_active);
    puts("PASS bottom pulse: two-frame direction, cap, expiry, no repeat, recenter rearm, missing/reversal/stop, wrap");
}

int main(void)
{
    test_bottom_pulse();
    test_near_adaptive_steering();
    test_readiness();
    test_steering_and_speed();
    test_independent_servo_center();
    test_loss_stale_and_encoder();
    test_default_speed_profile();
    test_loss_stop_switch();
    test_loss_stop_reenable_reasons();
    test_invalid_frame_holds_steering();
    test_timing_and_pd();
    test_abnormal_inputs();
    test_randomized_invariants();
    puts("All track-control host tests passed.");
    return EXIT_SUCCESS;
}
