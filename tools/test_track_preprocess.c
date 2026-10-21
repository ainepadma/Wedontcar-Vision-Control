#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "img_process.h"
#include "track_vision.h"
#include "track_control.h"

#define CHECK(c) do { if (!(c)) { \
    fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #c); exit(EXIT_FAILURE); \
} } while (0)

static uint8_t pixels[TRACK_VISION_MAX_ROWS * TRACK_VISION_MAX_COLS];
static uint8_t scratch[5U * TRACK_VISION_MAX_COLS];
static track_vision_result_t result;

static uint8_t process(uint16_t rows, uint16_t cols)
{
    uint32_t histogram[256];
    uint8_t threshold;
    CHECK(image_track_preprocess(pixels, rows, cols,
        (uint16_t)((uint32_t)rows * 30U / 100U),
        (uint16_t)((uint32_t)rows * TRACK_VISION_ROI_END_PERCENT / 100U),
        scratch, sizeof(scratch), histogram, &threshold));
    track_vision_process(pixels, rows, cols, &result);
    return threshold;
}

static void draw_gray_lane(uint16_t rows, uint16_t cols, int mode)
{
    uint32_t rng = 1327U;
    const uint16_t roi_start = (uint16_t)((uint32_t)rows *
        TRACK_VISION_ROI_START_PERCENT / 100U);
    const uint16_t roi_end = (uint16_t)((uint32_t)rows *
        TRACK_VISION_ROI_END_PERCENT / 100U);
    const uint16_t zone_rows = (uint16_t)((roi_end - roi_start) / 3U);
    for (uint16_t y = 0; y < rows; ++y) {
        for (uint16_t x = 0; x < cols; ++x) {
            int white = x >= cols / 3U && x < cols * 2U / 3U;
            if (mode == 1 && y >= roi_end - zone_rows && y < roi_end) white = 0;
            if (mode == 2 && y >= roi_start && y < roi_start + zone_rows) white = 0;
            if (mode == 3) white = x < cols / 2U;
            rng = rng * 1664525U + 1013904223U;
            pixels[(uint32_t)y * cols + x] =
                (uint8_t)((white ? 19U : 5U) + ((rng >> 16) % 5U));
        }
    }
}

static void synthetic(uint16_t rows, uint16_t cols)
{
    const size_t size = (size_t)rows * cols;
    const uint8_t uniform[] = {0, 1, 8, 17, 127, 254, 255};
    for (size_t i = 0; i < sizeof(uniform); ++i) {
        memset(pixels, uniform[i], size);
        process(rows, cols);
        CHECK(!result.valid);
    }
    draw_gray_lane(rows, cols, 0);
    process(rows, cols);
    CHECK(result.valid && result.confidence >= 95U && abs(result.error) < 20);

    // The user confirmed that the fixed body starts around 70%H. The calibrated
    // 68%H lower ROI bound must exclude it without inventing hidden track pixels.
    draw_gray_lane(rows, cols, 0);
    for (uint16_t y = (uint16_t)((uint32_t)rows * 70U / 100U); y < rows; ++y)
        memset(pixels + (uint32_t)y * cols + cols / 3U, 0, cols / 3U);
    process(rows, cols);
    CHECK(result.valid && abs(result.error) < 20);
    CHECK(result.confidence == 100U);
    CHECK(result.reject_flags == TRACK_VISION_REJECT_NONE);

    // A three-frame observation only arms readiness; it must not start motors.
    track_control_state_t control;
    track_control_init(&control, NULL);
    for (uint32_t now = 20; now <= 60; now += 20) {
        track_control_observe(&control, result.error, result.heading,
                              result.confidence, result.valid, now);
    }
    track_control_output_t output = track_control_tick(&control, 60U);
    CHECK(output.ready && !output.running && output.target_speed == 0.0f);
    track_control_toggle(&control, 60U); // Host simulation only, no hardware.
    output = track_control_tick(&control, 60U);
    CHECK(output.running);
    memset(pixels, 0, size);
    process(rows, cols);
    track_control_observe(&control, result.error, result.heading,
                          result.confidence, result.valid, 80U);
    output = track_control_tick(&control, 80U);
    CHECK(output.running && output.target_speed == control.config.speed_curve_min);
    track_control_observe(&control, result.error, result.heading,
                          result.confidence, result.valid, 100U);
    output = track_control_tick(&control, 100U);
    CHECK(output.running && output.target_speed == control.config.speed_curve_min);
    track_control_observe(&control, result.error, result.heading,
                          result.confidence, result.valid, 120U);
    output = track_control_tick(&control, 120U);
    CHECK(!output.running && output.target_speed == 0.0f);
    CHECK(output.stop_reason == TRACK_STOP_LOST);
    draw_gray_lane(rows, cols, 0);
    process(rows, cols);
    for (uint32_t now = 140; now <= 180; now += 20)
        track_control_observe(&control, result.error, result.heading,
                              result.confidence, result.valid, now);
    output = track_control_tick(&control, 180U);
    CHECK(output.ready && !output.running && output.target_speed == 0.0f);

    for (int mode = 1; mode <= 3; ++mode) {
        draw_gray_lane(rows, cols, mode);
        process(rows, cols);
        CHECK(!result.valid);
    }
    printf("PASS gray pipeline %ux%u: noise, body mask, missing near/far, clipped edge, manual/LOST gate\n",
           (unsigned int)cols, (unsigned int)rows);
}

static void captured_frame(const char *path)
{
    FILE *file = fopen(path, "rb");
    if (file == NULL) { perror(path); exit(EXIT_FAILURE); }
    const size_t read_count = fread(pixels, 1, 376U * 240U, file);
    const int extra = fgetc(file);
    const int failed = ferror(file);
    fclose(file);
    CHECK(read_count == 376U * 240U && extra == EOF && !failed);
    const uint8_t threshold = process(240, 376);
    printf("CAPTURE %s threshold=%u valid=%u Q=%u rows=%u error=%d heading=%d\n",
           path, (unsigned int)threshold, (unsigned int)result.valid,
           (unsigned int)result.confidence, (unsigned int)result.valid_rows,
           (int)result.error, (int)result.heading);
    CHECK(result.valid);
}

int main(int argc, char **argv)
{
    synthetic(120, 188);
    synthetic(240, 376);
    synthetic(480, 752);
    for (int i = 1; i < argc; ++i) captured_frame(argv[i]);
    puts("All grayscale-to-control pipeline tests passed; no hardware accessed.");
    return EXIT_SUCCESS;
}
