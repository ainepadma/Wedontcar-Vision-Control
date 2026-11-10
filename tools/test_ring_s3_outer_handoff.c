#include "ring_control.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define H 154U
#define W 304U
#define CHECK(expr) do { if (!(expr)) { \
    fprintf(stderr, "S3 handoff failed at line %d: %s\n", __LINE__, #expr); \
    exit(EXIT_FAILURE); \
} } while (0)

static uint8_t image[H][W];
static track_ring_vision_state_t tracker;
static track_vision_result_t road;
static ring_control_state_t state;
static uint32_t now, frame;

static void load(const char *dir, const char *name, unsigned mirror)
{
    char path[1024];
    snprintf(path, sizeof(path), "%s/%s.bin", dir, name);
    FILE *file = fopen(path, "rb");
    CHECK(file && fread(image, 1, sizeof(image), file) == sizeof(image));
    fclose(file);
    if (mirror) {
        for (unsigned y = 0; y < H; ++y) {
            for (unsigned x = 0; x < W / 2; ++x) {
                uint8_t tmp = image[y][x];
                image[y][x] = image[y][W - 1 - x];
                image[y][W - 1 - x] = tmp;
            }
        }
    }
}

static void prepare(void)
{
    track_vision_process(&image[0][0], H, W, &road);
    road.frame_number = ++frame;
    track_ring_vision_process(&tracker, &image[0][0], H, W, &road);
}

static void publish(void)
{
    now += 20;
    ring_control_observe(&state, &road.ring, now, 1);
    ring_control_select_path(&state, &road.ring);
}

static void enter_s3(const char *dir, unsigned inner)
{
    const char *const names[] = {"entry", "island", "first_v"};
    ring_control_init(&state);
    track_ring_vision_init(&tracker);
    for (unsigned s = 0; s < 3; ++s) {
        load(dir, names[s], inner);
        for (unsigned i = 0; i < 3; ++i) {
            prepare();
            publish();
        }
        CHECK(state.kind == TRACK_RING_360 && state.corner_stage == s + 1);
        CHECK(state.corner_guide_id == ring_pdf_guide(s + 1, inner));
    }
}

static void check_side(const char *dir, unsigned inner)
{
    unsigned outer = 1U - inner;
    enter_s3(dir, inner);
    load(dir, "cross", inner);
    for (unsigned i = 0; i < 4; ++i) {
        prepare();
        CHECK(road.ring.corners.cross_entry);
        publish();
        CHECK(state.corner_stage == 3 && state.transition_frames == 0);
    }
    load(dir, "p2", inner);
    prepare();
    ring_corner_frame_t *f = &road.ring.corners;
    CHECK(f->v[inner].quality && f->v[outer].quality && !f->v_reacquired);
    CHECK(inner ? f->v[inner].x < 0.45f * (W-1) : f->v[inner].x > 0.55f * (W-1));
    CHECK(f->guide[ring_pdf_guide(4, inner)].valid);
    publish();
    CHECK(state.corner_stage == 3 && state.transition_frames == 1);
    ring_control_observe(&state, &road.ring, now + 1, 1);
    CHECK(state.transition_frames == 1);
    prepare();
    publish();
    CHECK(state.corner_stage == 4 && state.corner_guide_id == ring_pdf_guide(4, inner));
    CHECK(state.selected_path.valid && !state.corner_guide_held);

    /* Based on the same camera observation, isolate the gates independently.
     * These edits model detector outputs; the clean path above uses pixels. */
    enter_s3(dir, inner);
    load(dir, "p2", inner);
    for (unsigned i = 0; i < 6; ++i) {
        prepare();
        if (i == 1) f->v_reacquired |= (uint8_t)(1U << outer);
        if (i == 3) f->guide[ring_pdf_guide(4, inner)].valid = 0;
        if (i == 5) f->v[inner].x = inner ? 200.f : 103.f;
        publish();
        CHECK(state.corner_stage == 3 && state.transition_frames == (i % 2 == 0));
    }
    prepare();
    publish();
    prepare();
    f->v[outer].y -= 30.f; /* Another corner outside the previous tip window. */
    publish();
    CHECK(state.corner_stage == 3 && state.transition_frames == 1);
    prepare();
    publish();
    CHECK(state.corner_stage == 3 && state.transition_frames == 1);
    now += RING_VISION_STALE_MS;
    prepare();
    publish();
    CHECK(state.corner_stage == 3 && state.transition_frames == 1);
    prepare();
    publish();
    CHECK(state.corner_stage == 4 && state.corner_guide_id == ring_pdf_guide(4, inner));
}

int main(int argc, char **argv)
{
    CHECK(argc == 2);
    check_side(argv[1], 0);
    check_side(argv[1], 1);
    puts("PASS S1-S4 real images and mirrors, dual tracer overlap, true inner and cross rejection, raw guide, same tip, duplicate and stale gates");
    return 0;
}
