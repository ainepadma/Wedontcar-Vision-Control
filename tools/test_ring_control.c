#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "ring_control.h"

#define CHECK(c) do { if (!(c)) { fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #c); exit(EXIT_FAILURE); } } while (0)

int main(void)
{
    ring_control_state_t state;
    ring_control_init(&state);
    ring_control_observe(&state, 1U, 0, 1U, 90U, 10U);
    CHECK(state.phase == RING_STATE_CANDIDATE);
    ring_control_observe(&state, 0U, 0, 1U, 90U, 20U);
    CHECK(state.phase == RING_STATE_IDLE);

    // A confirmed visual candidate must clear quickly if the track becomes
    // unambiguous before any sustained yaw confirms entry.
    for (uint32_t i = 0; i < RING_CONFIRM_FRAMES; ++i)
        ring_control_observe(&state, 1U, 0, 1U, 90U, 30U + i * 20U);
    CHECK(state.phase == RING_STATE_ENTERING);
    CHECK(ring_control_turn_override(&state) == 0);
    for (uint32_t i = 0; i < RING_ENTER_CLEAR_FRAMES; ++i)
        ring_control_observe(&state, 0U, 0, 1U, 90U, 100U + i * 20U);
    ring_control_tick(&state, 1U, 1U, 0.0f, 200U);
    CHECK(state.phase == RING_STATE_IDLE);

    for (uint32_t i = 0; i < RING_CONFIRM_FRAMES; ++i)
        ring_control_observe(&state, 1U, 0, 1U, 90U, 100U + i * 20U);
    CHECK(state.phase == RING_STATE_ENTERING && state.direction == 0);
    CHECK(ring_control_turn_override(&state) == 0);
    for (uint32_t i = 0; i < 5U; ++i)
        ring_control_tick(&state, 1U, 1U, 100.0f, 160U + i * 10U);
    CHECK(state.phase == RING_STATE_INSIDE && state.direction == 1);
    CHECK(ring_control_turn_override(&state) == 1);

    uint32_t now = 210U;
    for (uint32_t i = 0; i < 210U; ++i)
    {
        now += 10U;
        ring_control_tick(&state, 1U, 1U, 100.0f, now);
    }
    CHECK(state.yaw_progress_deg >= 200.0f);
    for (uint32_t i = 0; i < RING_CLEAR_FRAMES; ++i)
        ring_control_observe(&state, 0U, 0, 1U, 90U, now);
    ring_control_tick(&state, 1U, 1U, 100.0f, now + 10U);
    CHECK(state.phase == RING_STATE_EXITING);
    CHECK(ring_control_turn_override(&state) == 0);
    ring_control_tick(&state, 1U, 1U, 0.0f, now + 20U);
    CHECK(state.phase == RING_STATE_COOLDOWN);
    ring_control_tick(&state, 0U, 1U, 0.0f, now + 30U);
    CHECK(state.phase == RING_STATE_IDLE);
    CHECK(ring_control_turn_override(&state) == 0);
    CHECK(ring_control_turn_override(NULL) == 0);

    ring_control_observe(&state, 1U, -1, 1U, 90U, 10000U);
    ring_control_observe(&state, 1U, -1, 1U, 90U, 10010U);
    ring_control_observe(&state, 1U, -1, 1U, 90U, 10020U);
    ring_control_tick(&state, 1U, 1U, 0.0f, 10020U + RING_APPROACH_TIMEOUT_MS + 1U);
    CHECK(state.phase == RING_STATE_COOLDOWN);
    // A persistent false shape cannot immediately arm another red frame.
    ring_control_tick(&state, 1U, 1U, 0.0f,
                      10020U + RING_APPROACH_TIMEOUT_MS + RING_COOLDOWN_MS + 2U);
    CHECK(state.phase == RING_STATE_COOLDOWN);
    ring_control_observe(&state, 0U, 0, 1U, 90U,
                         10020U + RING_APPROACH_TIMEOUT_MS + RING_COOLDOWN_MS + 3U);
    ring_control_tick(&state, 1U, 1U, 0.0f,
                      10020U + RING_APPROACH_TIMEOUT_MS + RING_COOLDOWN_MS + 4U);
    CHECK(state.phase == RING_STATE_IDLE);
    puts("All ring-control host tests passed.");
    return EXIT_SUCCESS;
}
