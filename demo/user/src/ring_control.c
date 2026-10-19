#include "ring_control.h"

#include <math.h>
#include <string.h>

#define RING_TRACK_CLEAR_CONFIDENCE 60U
#define RING_YAW_CONFIRM_TICKS 5U
#define RING_MAX_FORWARD_TIME UINT32_C(0x7fffffff)

static void ring_reset_detection(ring_control_state_t *state)
{
    state->phase = RING_STATE_IDLE;
    state->direction = 0;
    state->candidate_frames = 0U;
    state->clear_frames = 0U;
    state->yaw_motion_ticks = 0U;
    state->vision_candidate = 0U;
    state->track_valid = 0U;
    state->yaw_progress_deg = 0.0f;
    state->yaw_rate_dps = 0.0f;
}

void ring_control_init(ring_control_state_t *state)
{
    if (state == NULL)
        return;
    memset(state, 0, sizeof(*state));
    state->phase = RING_STATE_IDLE;
}

void ring_control_observe(ring_control_state_t *state, uint8_t candidate,
                          int8_t direction_hint, uint8_t track_valid,
                          uint8_t confidence, uint32_t now_ms)
{
    if (state == NULL)
        return;
    candidate = candidate != 0U;
    track_valid = track_valid != 0U;
    direction_hint = direction_hint > 0 ? 1 : (direction_hint < 0 ? -1 : 0);
    state->vision_candidate = candidate;
    state->track_valid = track_valid;

    if (state->phase == RING_STATE_IDLE || state->phase == RING_STATE_CANDIDATE)
    {
        if (!candidate)
        {
            ring_reset_detection(state);
            return;
        }
        if (state->direction != 0 && direction_hint != 0 &&
            direction_hint != state->direction)
        {
            state->candidate_frames = 0U;
            state->direction = direction_hint;
        }
        else if (state->direction == 0 && direction_hint != 0)
            state->direction = direction_hint;
        if (state->candidate_frames < RING_CONFIRM_FRAMES)
            ++state->candidate_frames;
        state->phase = RING_STATE_CANDIDATE;
        if (state->candidate_frames >= RING_CONFIRM_FRAMES)
        {
            state->phase = RING_STATE_ENTERING;
            state->phase_started_ms = now_ms;
            state->yaw_progress_deg = 0.0f;
            state->yaw_motion_ticks = 0U;
        }
        return;
    }

    if (!candidate && track_valid && confidence >= RING_TRACK_CLEAR_CONFIDENCE)
    {
        if (state->clear_frames < UINT8_MAX)
            ++state->clear_frames;
    }
    else
        state->clear_frames = 0U;
}

void ring_control_tick(ring_control_state_t *state, uint8_t vehicle_running,
                       uint8_t yaw_ready, float yaw_rate_dps,
                       uint32_t now_ms)
{
    if (state == NULL)
        return;
    state->yaw_rate_dps = isfinite(yaw_rate_dps) ? yaw_rate_dps : 0.0f;
    if (!vehicle_running)
    {
        ring_reset_detection(state);
        state->last_tick_ms = now_ms;
        return;
    }
    if (state->phase == RING_STATE_COOLDOWN)
    {
        /* A persistent false visual shape must not re-arm every few frames.
         * Re-arm only after both the cooldown elapsed and that shape left the
         * camera view. */
        if ((int32_t)(now_ms - state->cooldown_until_ms) >= 0 &&
            !state->vision_candidate)
            ring_reset_detection(state);
        state->last_tick_ms = now_ms;
        return;
    }

    uint32_t elapsed = (uint32_t)(now_ms - state->last_tick_ms);
    if (elapsed > RING_MAX_FORWARD_TIME)
        elapsed = 0U;
    state->last_tick_ms = now_ms;

    if (state->phase == RING_STATE_ENTERING)
    {
        if ((uint32_t)(now_ms - state->phase_started_ms) >
            RING_APPROACH_TIMEOUT_MS)
        {
            state->phase = RING_STATE_COOLDOWN;
            state->cooldown_until_ms = now_ms + RING_COOLDOWN_MS;
            state->candidate_frames = 0U;
            state->clear_frames = 0U;
            state->yaw_motion_ticks = 0U;
            return;
        }
        /* A visual candidate may be a transient widening or crossing.  Until
         * the IMU has seen a sustained turn, a few clear track frames are
         * enough to revoke it instead of leaving the red indicator latched
         * for the full approach timeout. */
        if (state->clear_frames >= RING_ENTER_CLEAR_FRAMES &&
            state->yaw_motion_ticks == 0U)
        {
            ring_reset_detection(state);
            return;
        }
        if (!yaw_ready)
            return;
        if (state->direction == 0 && fabsf(state->yaw_rate_dps) >= RING_YAW_START_DPS)
            state->direction = state->yaw_rate_dps > 0.0f ? 1 : -1;
        if (state->direction != 0 &&
            (float)state->direction * state->yaw_rate_dps >= RING_YAW_START_DPS)
        {
            if (state->yaw_motion_ticks < RING_YAW_CONFIRM_TICKS)
                ++state->yaw_motion_ticks;
        }
        else
            state->yaw_motion_ticks = 0U;
        if (state->yaw_motion_ticks >= RING_YAW_CONFIRM_TICKS)
        {
            state->phase = RING_STATE_INSIDE;
            state->phase_started_ms = now_ms;
            state->yaw_progress_deg = 0.0f;
        }
        return;
    }

    if (state->phase == RING_STATE_INSIDE || state->phase == RING_STATE_EXITING)
    {
        if ((uint32_t)(now_ms - state->phase_started_ms) > RING_ACTIVE_TIMEOUT_MS)
        {
            state->phase = RING_STATE_COOLDOWN;
            state->cooldown_until_ms = now_ms + RING_COOLDOWN_MS;
            return;
        }
        if (yaw_ready && state->direction != 0 && elapsed > 0U)
        {
            float signed_rate = (float)state->direction * state->yaw_rate_dps;
            if (signed_rate > 0.0f)
                state->yaw_progress_deg += signed_rate * (float)elapsed / 1000.0f;
        }
        if (state->phase == RING_STATE_INSIDE &&
            state->yaw_progress_deg >= RING_MIN_EXIT_DEG &&
            state->clear_frames >= RING_CLEAR_FRAMES)
        {
            state->phase = RING_STATE_EXITING;
            state->phase_started_ms = now_ms;
        }
        else if (state->phase == RING_STATE_EXITING &&
                 state->clear_frames >= RING_CLEAR_FRAMES &&
                 fabsf(state->yaw_rate_dps) < RING_YAW_START_DPS)
        {
            state->phase = RING_STATE_COOLDOWN;
            state->cooldown_until_ms = now_ms + RING_COOLDOWN_MS;
        }
    }
}

int8_t ring_control_turn_override(const ring_control_state_t *state)
{
    if (state == NULL || state->phase != RING_STATE_INSIDE)
        return 0;
    return state->direction > 0 ? 1 : (state->direction < 0 ? -1 : 0);
}
