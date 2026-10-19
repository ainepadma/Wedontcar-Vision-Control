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
    CHECK(ring_control_turn_override(&state) == 0);

    ring_control_tick(&state, 1U, 1U, 100.0f, 210U);
    ring_control_tick(&state, 1U, 1U, -100.0f, 220U);
    CHECK(state.yaw_progress_deg < 0.01f);

    uint32_t now = 220U;
    for (uint32_t i = 0; i < 300U && state.phase == RING_STATE_INSIDE; ++i)
    {
        now += 10U;
        ring_control_tick(&state, 1U, 1U, 100.0f, now);
    }
    CHECK(state.yaw_progress_deg >= RING_MIN_EXIT_DEG);
    CHECK(state.phase == RING_STATE_EXITING);
    CHECK(ring_control_turn_override(&state) == -1);
    // Ring-looking frames may persist; the measured angle must still trigger exit.
    CHECK(state.clear_frames == 0U);
    for (uint32_t i = 0; i < RING_CLEAR_FRAMES; ++i)
        ring_control_observe(&state, 0U, 0, 1U, 90U, now);
    ring_control_tick(&state, 1U, 1U, 100.0f, now + 10U);
    CHECK(state.phase == RING_STATE_EXITING);
    ring_control_tick(&state, 1U, 1U, 100.0f, now + RING_EXIT_MIN_MS);
    CHECK(state.phase == RING_STATE_COOLDOWN);
    ring_control_tick(&state, 0U, 1U, 0.0f, now + RING_EXIT_MIN_MS + 10U);
    CHECK(state.phase == RING_STATE_IDLE);
    CHECK(ring_control_turn_override(&state) == 0);
    CHECK(ring_control_turn_override(NULL) == 0);

    // A persistent ring image eventually releases the forced exit turn and
    // returns control to vision instead of hard-stopping at an angle limit.
    for (uint32_t i = 0; i < RING_CONFIRM_FRAMES; ++i)
        ring_control_observe(&state, 1U, -1, 1U, 90U, 3000U + i * 10U);
    for (uint32_t i = 0; i < 5U; ++i)
        ring_control_tick(&state, 1U, 1U, -100.0f, 3030U + i * 10U);
    CHECK(state.phase == RING_STATE_INSIDE);
    now = 3070U;
    for (uint32_t i = 0; i < 500U && state.phase != RING_STATE_COOLDOWN; ++i)
    {
        now += 10U;
        ring_control_tick(&state, 1U, 1U, -100.0f, now);
    }
    CHECK(state.yaw_progress_deg >= RING_MAX_EXIT_DEG);
    CHECK(state.phase == RING_STATE_COOLDOWN);
    CHECK(ring_control_turn_override(&state) == 0);
    ring_control_tick(&state, 0U, 1U, 0.0f, now + 10U);
    CHECK(state.phase == RING_STATE_IDLE);

    // A gyro dropout cancels only the special ring steering; vision remains.
    for (uint32_t i = 0; i < RING_CONFIRM_FRAMES; ++i)
        ring_control_observe(&state, 1U, 1, 1U, 90U, 8000U + i * 10U);
    for (uint32_t i = 0; i < 5U; ++i)
        ring_control_tick(&state, 1U, 1U, 100.0f, 8030U + i * 10U);
    CHECK(state.phase == RING_STATE_INSIDE);
    ring_control_tick(&state, 1U, 0U, 0.0f, 8080U);
    CHECK(state.phase == RING_STATE_COOLDOWN);
    ring_control_tick(&state, 0U, 0U, 0.0f, 8090U);
    CHECK(state.phase == RING_STATE_IDLE);

    // An opposite visual branch hint must not set the circle direction.
    // The first bend may later reverse; neither stage may force a turn.
    for (uint32_t i = 0; i < RING_CONFIRM_FRAMES; ++i)
        ring_control_observe(&state, 1U, -1, 1U, 90U, 9000U + i * 10U);
    for (uint32_t i = 0; i < 5U; ++i)
        ring_control_tick(&state, 1U, 1U, 100.0f, 9030U + i * 10U);
    CHECK(state.phase == RING_STATE_INSIDE && state.direction == 1);
    CHECK(ring_control_turn_override(&state) == 0);
    for (uint32_t i = 0; i < 5U; ++i)
        ring_control_tick(&state, 1U, 1U, -100.0f, 9080U + i * 10U);
    CHECK(state.phase == RING_STATE_INSIDE && state.direction == -1);
    CHECK(state.yaw_progress_deg < 5.0f);
    CHECK(ring_control_turn_override(&state) == 0);
    ring_control_tick(&state, 0U, 1U, 0.0f, 9140U);

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
