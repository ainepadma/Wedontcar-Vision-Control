#ifndef RING_CONTROL_H
#define RING_CONTROL_H

#include <stdint.h>

#define RING_CONFIRM_FRAMES 3U
#define RING_CLEAR_FRAMES 3U
#define RING_ENTER_CLEAR_FRAMES 5U
#define RING_YAW_START_DPS 12.0f
#define RING_MIN_EXIT_DEG 200.0f
#define RING_APPROACH_TIMEOUT_MS 1500U
#define RING_ACTIVE_TIMEOUT_MS 12000U
#define RING_COOLDOWN_MS 3000U

typedef enum
{
    RING_STATE_IDLE = 0,
    RING_STATE_CANDIDATE,
    RING_STATE_ENTERING,
    RING_STATE_INSIDE,
    RING_STATE_EXITING,
    RING_STATE_COOLDOWN
} ring_control_phase_t;

typedef struct
{
    uint8_t phase;
    int8_t direction; /* +1 left, -1 right, 0 not resolved yet. */
    uint8_t candidate_frames;
    uint8_t clear_frames;
    uint8_t yaw_motion_ticks;
    uint8_t vision_candidate;
    uint8_t track_valid;
    uint32_t phase_started_ms;
    uint32_t last_tick_ms;
    uint32_t cooldown_until_ms;
    float yaw_progress_deg;
    float yaw_rate_dps;
} ring_control_state_t;

void ring_control_init(ring_control_state_t *state);
void ring_control_observe(ring_control_state_t *state, uint8_t candidate,
                          int8_t direction_hint, uint8_t track_valid,
                          uint8_t confidence, uint32_t now_ms);
void ring_control_tick(ring_control_state_t *state, uint8_t vehicle_running,
                       uint8_t yaw_ready, float yaw_rate_dps,
                       uint32_t now_ms);
/* Keep normal centerline steering through CANDIDATE/ENTERING.  The ring may
 * override steering only after sustained IMU yaw confirms physical entry. */
int8_t ring_control_turn_override(const ring_control_state_t *state);

#endif
