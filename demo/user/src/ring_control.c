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
        if (fabsf(state->yaw_rate_dps) >= RING_YAW_START_DPS)
        {
            const int8_t observed_direction = state->yaw_rate_dps > 0.0f ? 1 : -1;
            if (observed_direction != state->direction)
            {
                /* 远近中线的偏移只表示画面分叉，不能可靠确定实际转向。 */
                state->direction = observed_direction;
                state->yaw_motion_ticks = 0U;
            }
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
            state->yaw_motion_ticks = 0U;
        }
        return;
    }

    if (state->phase == RING_STATE_INSIDE || state->phase == RING_STATE_EXITING)
    {
        uint32_t phase_timeout = state->phase == RING_STATE_INSIDE ?
            RING_ACTIVE_TIMEOUT_MS : RING_EXIT_TIMEOUT_MS;
        if (!yaw_ready)
        {
            /* 陀螺仪失效时退出环岛特殊控制，继续由视觉循线与丢帧保护接管。 */
            state->phase = RING_STATE_COOLDOWN;
            state->cooldown_until_ms = now_ms + RING_COOLDOWN_MS;
            return;
        }
        if ((uint32_t)(now_ms - state->phase_started_ms) > phase_timeout)
        {
            /* 环岛阶段超时仅撤销特殊转向，不因角度未达标直接停车。 */
            state->phase = RING_STATE_COOLDOWN;
            state->cooldown_until_ms = now_ms + RING_COOLDOWN_MS;
            return;
        }
        if (state->phase == RING_STATE_INSIDE &&
            state->yaw_progress_deg < RING_EARLY_REVERSE_DEG &&
            (float)state->direction * state->yaw_rate_dps <= -RING_YAW_START_DPS)
        {
            /* 入环首弯可能与环内转向相反；持续反向转动后重新锁定方向。 */
            if (state->yaw_motion_ticks < RING_YAW_CONFIRM_TICKS)
                ++state->yaw_motion_ticks;
            if (state->yaw_motion_ticks >= RING_YAW_CONFIRM_TICKS)
            {
                state->direction = (int8_t)-state->direction;
                state->yaw_progress_deg = 0.0f;
                state->yaw_motion_ticks = 0U;
                state->phase_started_ms = now_ms;
            }
        }
        else
            state->yaw_motion_ticks = 0U;
        if (yaw_ready && state->direction != 0 && elapsed > 0U)
        {
            float signed_rate = (float)state->direction * state->yaw_rate_dps;
            /* 积分净转角，避免左右摆动被累计成完整的一圈。 */
            state->yaw_progress_deg += signed_rate * (float)elapsed / 1000.0f;
            if (state->yaw_progress_deg < 0.0f)
                state->yaw_progress_deg = 0.0f;
        }
        if (state->phase == RING_STATE_INSIDE &&
            state->yaw_progress_deg >= RING_MIN_EXIT_DEG)
        {
            /* 到达出环角度就尝试向外转，不等待环岛画面完全消失。 */
            state->phase = RING_STATE_EXITING;
            state->phase_started_ms = now_ms;
        }
        if (state->phase == RING_STATE_EXITING &&
            (uint32_t)(now_ms - state->phase_started_ms) >= RING_EXIT_MIN_MS &&
            state->clear_frames >= RING_CLEAR_FRAMES)
        {
            state->phase = RING_STATE_COOLDOWN;
            state->cooldown_until_ms = now_ms + RING_COOLDOWN_MS;
        }
        else if (state->phase == RING_STATE_EXITING &&
                 state->yaw_progress_deg >= RING_MAX_EXIT_DEG)
        {
            state->phase = RING_STATE_COOLDOWN;
            state->cooldown_until_ms = now_ms + RING_COOLDOWN_MS;
        }
    }
}

int8_t ring_control_turn_override(const ring_control_state_t *state)
{
    if (state == NULL)
        return 0;
    if (state->phase == RING_STATE_EXITING &&
        (uint32_t)(state->last_tick_ms - state->phase_started_ms) <
            RING_EXIT_OVERRIDE_MS)
        return state->direction > 0 ? -1 : (state->direction < 0 ? 1 : 0);
    return 0;
}
