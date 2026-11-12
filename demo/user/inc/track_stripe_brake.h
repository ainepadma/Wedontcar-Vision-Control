#ifndef TRACK_STRIPE_BRAKE_H
#define TRACK_STRIPE_BRAKE_H
#include <stdint.h>

/* Brake-only settings. Existing running PI, steering and speeds are untouched. */
#define STRIPE_BRAKE_ARM_FRAMES 2U
#define STRIPE_BRAKE_MAX_AGE_MS 150U
#define STRIPE_BRAKE_BOTTOM_PERCENT 80U
#define STRIPE_BRAKE_HEIGHT_PERCENT 10U
/* From the near-image trigger to the rear-wheel stop reference. The previous
 * 2800 counts still stopped early. Add 1100 counts: about 0.2 s at the
 * 55-count/10-ms approach speed, or 76 mm using 2904.25 counts/rev and a
 * 64-mm wheel. Tune by rear-wheel position. */
#define STRIPE_BRAKE_PASS_COUNTS 3900
#define STRIPE_BRAKE_APPROACH_SPEED 55.0f
/* First launch only: after the starting zebra disappears, both wheels must
 * advance this far and the camera must confirm three clear fresh frames. */
#define STRIPE_BRAKE_START_CLEAR_COUNTS 1500
#define STRIPE_BRAKE_START_CLEAR_FRAMES 3U
#define STRIPE_BRAKE_PWM_LIMIT 2500
#define STRIPE_BRAKE_POSITION_KP 12
#define STRIPE_BRAKE_SPEED_KD 35

typedef struct {
    uint8_t reliable, visible, complete, near, clipped, large, partial, approaching;
    uint16_t top, bottom, center, span, height;
} stripe_brake_observation_t;
typedef struct {
    uint8_t armed, crossing, stopped, arm_votes, has_time, size_seen, lost_votes, approach_seen;
    uint16_t center, span;
    uint32_t last_ms;
    int32_t position_error[2];
    int32_t passed_counts[2];
} stripe_brake_state_t;
typedef struct {
    uint8_t started, enabled, clear_frames;
    int32_t clear_counts[2];
} stripe_brake_start_gate_t;

void stripe_brake_init(stripe_brake_state_t *state);
void stripe_brake_start_gate_init(stripe_brake_start_gate_t *gate);
void stripe_brake_start_gate_begin(stripe_brake_start_gate_t *gate);
uint8_t stripe_brake_start_gate_tick(stripe_brake_start_gate_t *gate,
    uint8_t running, uint8_t fresh, const stripe_brake_observation_t *observation,
    int16_t left_delta, int16_t right_delta);
/* Nonzero is white. Does not write pixels or depend on ordinary track validity. */
void stripe_brake_detect(const uint8_t *binary, uint16_t rows, uint16_t cols,
                         stripe_brake_observation_t *out);
/* Build a road-only copy: remove confirmed repeated bars, preserving the
 * original camera image for display and stripe detection. */
void stripe_brake_road_view(const uint8_t *binary, uint8_t *road,
    uint16_t rows, uint16_t cols, const stripe_brake_observation_t *observation);
/* New reliable observations only. A miss/C13/C14 never clears either latch. */
uint8_t stripe_brake_observe(stripe_brake_state_t *state,
    const stripe_brake_observation_t *observation, uint32_t observed_ms,
    uint32_t now_ms);
/* Call before observe on every 10-ms control tick, using signed forward counts. */
uint8_t stripe_brake_advance(stripe_brake_state_t *state,
                             int16_t left_delta, int16_t right_delta);
/* Signed PWM resisting displacement from the latched wheel positions. */
int16_t stripe_brake_hold(stripe_brake_state_t *state, uint8_t wheel,
                         int16_t encoder_delta, uint8_t capture_reference);
#endif
