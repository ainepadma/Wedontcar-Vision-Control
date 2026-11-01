#include "ring_control.h"
#include "track_control.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define H 240U
#define W 376U
#define CHECK(c) do { if (!(c)) { fprintf(stderr, "FAIL %d: %s\n", __LINE__, #c); exit(1); } } while (0)

static uint8_t pixels[H * W], saved[H * W];
static track_vision_result_t result;
static track_vision_result_t ordinary;
static track_ring_vision_state_t model;
static ring_control_state_t ring;
static uint32_t frame_id, clock_ms;

static void road(void)
{
    memset(pixels, 0, sizeof(pixels));
    for (unsigned y = 0U; y < H; ++y)
        for (unsigned x = W * 29U / 100U; x <= W * 70U / 100U; ++x)
            pixels[y * W + x] = 255U;
}

static void mirror(void)
{
    for (unsigned y = 0U; y < H; ++y) for (unsigned x = 0U; x < W / 2U; ++x) {
        uint8_t p = pixels[y * W + x];
        pixels[y * W + x] = pixels[y * W + W - 1U - x];
        pixels[y * W + W - 1U - x] = p;
    }
}

static void junction(uint8_t kind, unsigned tip, uint8_t reflected)
{
    road();
    int edge = (int)(W * 29U / 100U);
    for (unsigned y = 0U; y < H; ++y)
    for (int x = kind == RING_CORNER_A ? 20 : 0; x < edge; ++x) {
        float diagonal = (float)tip + (kind == RING_CORNER_A ? .5f : -.5f) * (float)(edge - x);
        if ((kind == RING_CORNER_A && (float)y < diagonal) ||
            (kind == RING_CORNER_B && (float)y > diagonal)) pixels[y * W + (unsigned)x] = 255U;
    }
    if (reflected) mirror();
}

static track_path_observation_t frame(void)
{
    memcpy(saved, pixels, sizeof(pixels));
    track_vision_process(pixels, H, W, &result);
    ordinary = result;
    result.frame_number = ++frame_id;
    track_ring_vision_process(&model, pixels, H, W, &result);
    CHECK(!memcmp(saved, pixels, sizeof(pixels)));
    CHECK(result.error == ordinary.error && result.heading == ordinary.heading);
    CHECK(result.valid == ordinary.valid && result.confidence == ordinary.confidence);
    CHECK(!memcmp(result.left, ordinary.left, sizeof(result.left)));
    CHECK(!memcmp(result.right, ordinary.right, sizeof(result.right)));
    CHECK(!memcmp(result.center, ordinary.center, sizeof(result.center)));
    CHECK(!memcmp(result.row_valid, ordinary.row_valid, sizeof(result.row_valid)));
    clock_ms += 20U;
    ring_control_tick(&ring, 1U, 0U, 0.f, 25.f, clock_ms);
    ring_control_observe(&ring, &result.ring, clock_ms, 1U);
    return ring_control_select_path(&ring, &result.ring);
}

static void reset(void)
{
    track_ring_vision_init(&model);
    ring_control_init(&ring);
    frame_id = clock_ms = 0U;
}

static void setup(void)
{
    reset();
    road();
    for (unsigned i = 0U; i < 8U; ++i) (void)frame();
    CHECK(model.ready && !ring.action_locked);
}

static void same_path(track_path_observation_t a, track_path_observation_t b)
{
    CHECK(a.error == b.error && a.heading == b.heading &&
          a.valid == b.valid && a.confidence == b.confidence);
}

static void check_approach(track_path_observation_t chosen, unsigned side)
{
    CHECK(!ring.action_locked && !ring.line_active && !ring.corner_stage && !ring.fault);
    CHECK(result.ring.approach_path.valid && result.ring.approach_source == 2U - side);
    CHECK(ring.path_mode == 2U - side);
    same_path(chosen, result.ring.approach_path);
    CHECK(chosen.valid && chosen.confidence >= 65U);
    CHECK(abs(chosen.error) <= 10 && abs(chosen.heading) <= 3);
    unsigned count = 0U;
    for (unsigned y = 0U; y < H; ++y) if (result.approach_valid[y]) {
        CHECK(y >= H * 40U / 100U && y < H * 68U / 100U);
        CHECK(abs((int)result.approach_center[y] - (side ? 189 : 186)) <= 1);
        CHECK(pixels[y * W + result.approach_center[y]]);
        ++count;
    }
    CHECK(count >= 60U);
}

static void test_far_approach(unsigned side)
{
    setup();
    track_control_state_t direct, corrected;
    const track_control_config_t config = {87.f, 98.5f, 75.5f, TRACK_SPEED_STRAIGHT,
        TRACK_SPEED_CURVE_MIN, .02f, .0005f, 120.f};
    track_control_init(&direct, &config);
    track_control_init(&corrected, &config);
    for (unsigned i = 1U; i <= 8U; ++i) {
        track_control_observe(&direct, 0, 0, 100U, 1U, i * 20U);
        track_control_observe(&corrected, 0, 0, 100U, 1U, i * 20U);
    }
    track_control_toggle(&direct, clock_ms);
    track_control_toggle(&corrected, clock_ms);
    CHECK(direct.running && corrected.running);
    track_control_output_t a = {0}, b = {0};
    junction(RING_CORNER_A, 110U, (uint8_t)side);
    for (unsigned i = 0U; i < 6U; ++i) {
        track_path_observation_t chosen = frame();
        CHECK(result.valid && abs(result.error) >= 20 && abs(result.heading) >= 120);
        CHECK(side ? result.error > 0 && result.heading > 0 : result.error < 0 && result.heading < 0);
        CHECK(result.ring.corners.count[side] == 1U && result.ring.corners.count[1U - side] == 0U);
        CHECK(result.ring.corners.point[side][0].kind == RING_CORNER_A);
        CHECK(result.ring.corners.point[side][0].y < (float)(H * RING_CORNER_ACTION_PERCENT / 100U));
        check_approach(chosen, side);
        track_control_observe(&direct, result.error, result.heading, result.confidence, result.valid, clock_ms);
        track_control_observe(&corrected, chosen.error, chosen.heading, chosen.confidence, chosen.valid, clock_ms);
        a = track_control_tick(&direct, clock_ms);
        b = track_control_tick(&corrected, clock_ms);
    }
    CHECK(a.running && b.running && isfinite(b.servo_angle));
    CHECK(fabsf(b.servo_angle - 87.f) < .5f);
    CHECK(fabsf(a.servo_angle - 87.f) > fabsf(b.servo_angle - 87.f) + .8f);
    CHECK(b.servo_angle >= 75.5f && b.servo_angle <= 98.5f);
    printf("PASS approach mirror=%u: ordinary=%d/%d corrected=%d/%d steering=%.2f->%.2f\n",
           side, result.error, result.heading, result.ring.approach_path.error,
           result.ring.approach_path.heading, (double)a.servo_angle, (double)b.servo_angle);
}

static void test_locked_priority(unsigned side)
{
    setup();
    junction(RING_CORNER_A, 125U, (uint8_t)side);
    for (unsigned i = 0U; i < 2U; ++i) check_approach(frame(), side);
    track_path_observation_t chosen = frame();
    CHECK(ring.action_locked && ring.corner_stage == 1U && ring.line_active);
    CHECK(ring.corner_guide_id == RING_GUIDE_A_LEFT + side);
    CHECK(chosen.error == result.ring.corners.guide[RING_GUIDE_A_LEFT + side].error);
    road();
    for (unsigned i = 0U; i < 4U; ++i) (void)frame();
    junction(RING_CORNER_B, 105U, (uint8_t)side);
    for (unsigned i = 0U; i < 3U; ++i) chosen = frame();
    CHECK(ring.action_locked && ring.corner_stage == 2U && ring.line_active);
    CHECK(ring.corner_guide_id == RING_GUIDE_B_BRANCH_LEFT + side);
    CHECK(chosen.error == result.ring.corners.guide[RING_GUIDE_B_BRANCH_LEFT + side].error);
    CHECK(side ? chosen.error > 100 : chosen.error < -100);
}

static void check_rejected(track_path_observation_t chosen)
{
    CHECK(!result.ring.approach_path.valid && !result.ring.approach_source);
    CHECK(!ring.action_locked && !ring.line_active && !ring.fault);
    same_path(chosen, result.ring.path[TRACK_PATH_NORMAL]);
}

static void test_unreliable_evidence(unsigned side)
{
    /* Starting on a mouth cannot borrow a default width as measured history. */
    reset();
    junction(RING_CORNER_A, 110U, (uint8_t)side);
    check_rejected(frame());

    /* Both mouths are unresolved topology, even with valid old lane width. */
    setup();
    road();
    for (unsigned y = 0U; y < 110U; ++y) memset(pixels + y * W, 255, W);
    check_rejected(frame());
    CHECK(result.ring.left_open && result.ring.right_open);

    /* A gap through the opposite edge breaks its per-row continuity. Enough
     * white main road remains to stop a simple aggregate row count hiding it. */
    setup();
    junction(RING_CORNER_A, 110U, (uint8_t)side);
    for (unsigned y = 119U; y < 128U; ++y)
        for (unsigned x = 227U; x <= 263U; ++x)
            pixels[y * W + (side ? W - 1U - x : x)] = 0U;
    check_rejected(frame());

    /* Full black rows disconnect the far mouth from this near road. */
    setup();
    junction(RING_CORNER_A, 110U, (uint8_t)side);
    for (unsigned y = 122U; y < 125U; ++y) memset(pixels + y * W, 0, W);
    check_rejected(frame());
}

static void test_curve_preserved(unsigned side)
{
    for (unsigned strength = 1U; strength <= 5U; ++strength) {
        setup();
        memset(pixels, 0, sizeof(pixels));
        for (unsigned y = 0U; y < H; ++y) {
            float dy = (float)y - 162.f;
            int offset = (int)(.002f * (float)strength * dy * dy);
            for (unsigned x = 0U; x < W; ++x)
                if ((int)x >= 109 + offset && (int)x <= 263 + offset) pixels[y * W + x] = 255U;
        }
        if (side) mirror();
        for (unsigned i = 0U; i < 4U; ++i) {
            check_rejected(frame());
            CHECK(result.valid && abs(result.heading) > 10);
            CHECK(!result.ring.corners.count[0] && !result.ring.corners.count[1]);
        }
    }
}

static void test_mild_width_freeze(unsigned side)
{
    setup();
    float width = model.width_near, width_slope = model.width_slope;
    float center = model.center_near, center_slope = model.center_slope;
    /* 189/154 = 1.227: below the former 1.25 model-learning tolerance, yet
     * only one edge has expanded. Do not learn its midpoint as the lane. */
    road();
    for (unsigned y = 0U; y < H; ++y)
        for (unsigned x = 74U; x < 109U; ++x) pixels[y * W + x] = 255U;
    if (side) mirror();
    for (unsigned i = 0U; i < 6U; ++i) {
        (void)frame();
        CHECK(fabsf(model.width_near - width) < .001f);
        CHECK(fabsf(model.width_slope - width_slope) < .001f);
        /* The stored incoming center must not follow the mouth midpoint. */
        CHECK(fabsf(model.center_near - center) < .001f);
        CHECK(fabsf(model.center_slope - center_slope) < .001f);
    }
}

static void test_freshness_and_recovery(unsigned side)
{
    setup();
    junction(RING_CORNER_A, 110U, (uint8_t)side);
    check_approach(frame(), side);
    /* A candidate calculated for another captured image is not the current
     * transaction, even when its old geometry is otherwise usable. */
    --result.ring.corners.frame_number;
    same_path(ring_control_select_path(&ring, &result.ring), result.ring.path[TRACK_PATH_NORMAL]);
    CHECK(ring.path_mode == TRACK_PATH_NORMAL && !ring.line_active);
    ++result.ring.corners.frame_number;
    check_approach(ring_control_select_path(&ring, &result.ring), side);

    clock_ms += RING_VISION_STALE_MS + 1U;
    ring_control_tick(&ring, 1U, 0U, 0.f, 25.f, clock_ms);
    same_path(ring_control_select_path(&ring, &result.ring), result.ring.path[TRACK_PATH_NORMAL]);
    CHECK(ring.path_mode == TRACK_PATH_NORMAL && !ring.line_active && !ring.fault);

    setup();
    model.age_frames = RING_APPROACH_MODEL_MAX_AGE;
    junction(RING_CORNER_A, 110U, (uint8_t)side);
    check_rejected(frame());
    CHECK(result.ring.approach_reject == TRACK_APPROACH_STALE);
    CHECK(model.age_frames >= RING_APPROACH_MODEL_MAX_AGE);

    /* Clean paired road renews the width model and removes compensation.
     * A later real mouth may then use that newly measured history again. */
    unsigned learned = model.learned_frames;
    road();
    for (unsigned i = 0U; i < 8U; ++i) {
        check_rejected(frame());
        CHECK(ring.path_mode == TRACK_PATH_NORMAL);
    }
    CHECK(model.ready && model.age_frames == 0U && model.learned_frames > learned);
    CHECK(result.ring.approach_reject == TRACK_APPROACH_NO_OPEN);
    junction(RING_CORNER_A, 110U, (uint8_t)side);
    check_approach(frame(), side);
    road();
    check_rejected(frame());
    CHECK(ring.path_mode == TRACK_PATH_NORMAL);
}

static void test_starting_on_junction_recovers(unsigned side)
{
    /* A first-frame mouth may initialize geometric coordinates, but it is
     * never measured clean-road history. Returning to real paired road must
     * still let learning recover rather than retain an x=0 search anchor. */
    reset();
    junction(RING_CORNER_A, 110U, (uint8_t)side);
    check_rejected(frame());
    CHECK(model.learned_frames == 0U);
    road();
    for (unsigned i = 0U; i < 8U; ++i) check_rejected(frame());
    CHECK(model.ready && model.learned_frames >= RING_APPROACH_LEARN_FRAMES);
    CHECK(model.age_frames == 0U && fabsf(model.center_near - 186.f) <= 1.f);
    junction(RING_CORNER_A, 110U, (uint8_t)side);
    check_approach(frame(), side);
}

int main(void)
{
    for (unsigned side = 0U; side < 2U; ++side) {
        test_far_approach(side);
        test_locked_priority(side);
        test_unreliable_evidence(side);
        test_curve_preserved(side);
        test_mild_width_freeze(side);
        test_freshness_and_recovery(side);
        test_starting_on_junction_recovers(side);
    }
    puts("PASS same-frame junction approach: measured-width mirrors, ordinary bias, PD correction, A1/B2 priority, missing evidence rejection, curve isolation, frozen mild widening, stale/frame rejection and model recovery, immutable input");
    return 0;
}
