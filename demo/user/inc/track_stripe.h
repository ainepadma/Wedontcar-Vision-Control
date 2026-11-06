#ifndef TRACK_STRIPE_H
#define TRACK_STRIPE_H
#include <stdint.h>

/* Percent of the upright binary camera frame, not the LCD/text area.
 * Initial P4 estimates: calibrate against actual camera frames on the car. */
#define TRACK_STRIPE_BOTTOM_PERCENT 92U
#define TRACK_STRIPE_HEIGHT_PERCENT 28U
#define TRACK_STRIPE_ARM_FRAMES 3U
#define TRACK_STRIPE_NEAR_FRAMES 2U
#define TRACK_STRIPE_CLEAR_FRAMES 12U
#define TRACK_STRIPE_MAX_AGE_MS 150U

typedef struct {
    uint8_t reliable, visible, complete, near;
    uint16_t top, bottom, center, span;
} track_stripe_observation_t;

typedef struct {
    uint8_t armed, stopped, bypass, arm_votes, near_votes, clear_votes;
    uint8_t has_time;
    uint16_t center, span;
    uint32_t last_ms;
} track_stripe_state_t;

/* Nonzero=white. Never modifies pixels; independent of ordinary road validity. */
void track_stripe_detect(const uint8_t *binary, uint16_t rows, uint16_t cols,
                         track_stripe_observation_t *out);
void track_stripe_init(track_stripe_state_t *state);
/* Called on C13 restart. A stopped/near marker is bypassed until 12 reliable
 * clear frames WHILE MOVING. Camera failure never counts as leaving a marker. */
void track_stripe_restart(track_stripe_state_t *state, uint8_t marker_near);
/* ISR owner only, once per NEW camera generation. Returns latched stop request. */
uint8_t track_stripe_observe(track_stripe_state_t *state,
    const track_stripe_observation_t *observation, uint32_t observed_ms,
    uint32_t now_ms, uint8_t running);
#endif
