#ifndef TRACK_STRIPE_BRAKE_H
#define TRACK_STRIPE_BRAKE_H
#include <stdint.h>

/* Brake-only settings. Existing running PI, steering and speeds are untouched. */
#define STRIPE_BRAKE_ARM_FRAMES 3U
#define STRIPE_BRAKE_MAX_AGE_MS 150U
#define STRIPE_BRAKE_BOTTOM_PERCENT 92U
#define STRIPE_BRAKE_HEIGHT_PERCENT 23U
#define STRIPE_BRAKE_PWM_LIMIT 2500
#define STRIPE_BRAKE_POSITION_KP 12
#define STRIPE_BRAKE_SPEED_KD 35

typedef struct {
    uint8_t reliable, visible, complete, near, clipped, large;
    uint16_t top, bottom, center, span, height;
} stripe_brake_observation_t;
typedef struct {
    uint8_t armed, stopped, arm_votes, has_time, size_seen;
    uint16_t center, span;
    uint32_t last_ms;
    int32_t position_error[2];
} stripe_brake_state_t;

void stripe_brake_init(stripe_brake_state_t *state);
/* Nonzero is white. Does not write pixels or depend on ordinary track validity. */
void stripe_brake_detect(const uint8_t *binary, uint16_t rows, uint16_t cols,
                         stripe_brake_observation_t *out);
/* New reliable observations only. A miss/C13/C14 never clears either latch. */
uint8_t stripe_brake_observe(stripe_brake_state_t *state,
    const stripe_brake_observation_t *observation, uint32_t observed_ms,
    uint32_t now_ms);
/* Signed PWM resisting displacement from the latched wheel positions. */
int16_t stripe_brake_hold(stripe_brake_state_t *state, uint8_t wheel,
                         int16_t encoder_delta, uint8_t capture_reference);
#endif
