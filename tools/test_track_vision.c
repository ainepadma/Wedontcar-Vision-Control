#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "track_vision.h"

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); \
        exit(EXIT_FAILURE); \
    } \
} while (0)

typedef struct
{
    uint64_t before;
    track_vision_result_t result;
    uint64_t after;
} guarded_result_t;

static uint32_t image_checksum(const uint8_t *image, uint32_t size)
{
    uint32_t hash = 2166136261U;
    for (uint32_t i = 0; i < size; ++i)
        hash = (hash ^ image[i]) * 16777619U;
    return hash;
}

static void verify(const uint8_t *image, uint16_t rows, uint16_t cols,
                   guarded_result_t *guarded)
{
    const uint32_t count = (uint32_t)rows * cols;
    const uint32_t checksum = image_checksum(image, count);
    memset(&guarded->result, 0xFF, sizeof(guarded->result));
    guarded->before = UINT64_C(0x1122334455667788);
    guarded->after = UINT64_C(0x8877665544332211);
    track_vision_process(image, rows, cols, &guarded->result);
    CHECK(guarded->before == UINT64_C(0x1122334455667788));
    CHECK(guarded->after == UINT64_C(0x8877665544332211));
    CHECK(image_checksum(image, count) == checksum);
    CHECK(guarded->result.confidence <= 100U);
    CHECK(guarded->result.roi_rows ==
          (uint32_t)rows * TRACK_VISION_ROI_END_PERCENT / 100U -
          (uint32_t)rows * TRACK_VISION_ROI_START_PERCENT / 100U);
    const uint16_t expected_zone_rows = guarded->result.roi_rows / 3U > 0U
        ? guarded->result.roi_rows / 3U : 1U;
    CHECK(guarded->result.near_valid_rows <= expected_zone_rows);
    CHECK(guarded->result.far_valid_rows <= expected_zone_rows);
    CHECK(guarded->result.error >= -1000 && guarded->result.error <= 1000);
    CHECK(guarded->result.heading >= -1000 && guarded->result.heading <= 1000);
    CHECK(guarded->result.near_width <= cols);
    CHECK(guarded->result.far_width <= cols);
    CHECK(guarded->result.ring_candidate <= 1U);
    CHECK(guarded->result.cross_candidate <= 1U);
    CHECK(guarded->result.ring_direction >= -1 &&
          guarded->result.ring_direction <= 1);
    uint16_t valid_rows = 0;
    for (uint16_t y = 0; y < TRACK_VISION_MAX_ROWS; ++y)
    {
        CHECK(guarded->result.left[y] < cols);
        CHECK(guarded->result.center[y] < cols);
        CHECK(guarded->result.right[y] < cols);
        if (guarded->result.row_valid[y])
        {
            ++valid_rows;
            CHECK(y >= (uint32_t)rows * TRACK_VISION_ROI_START_PERCENT / 100U);
            CHECK(y < (uint32_t)rows * TRACK_VISION_ROI_END_PERCENT / 100U);
            CHECK(guarded->result.left[y] > 0U);
            CHECK(guarded->result.right[y] < cols - 1U);
            CHECK(guarded->result.left[y] <= guarded->result.center[y]);
            CHECK(guarded->result.center[y] <= guarded->result.right[y]);
        }
        else
        {
            CHECK(guarded->result.left[y] == 0U);
            CHECK(guarded->result.center[y] == 0U);
            CHECK(guarded->result.right[y] == 0U);
        }
    }
    CHECK(guarded->result.valid_rows == valid_rows);
    if (!guarded->result.valid)
        CHECK(guarded->result.error == 0 && guarded->result.heading == 0);
    else
        CHECK(guarded->result.reject_flags == TRACK_VISION_REJECT_NONE);
}

static void draw_lane(uint8_t *image, uint16_t rows, uint16_t cols,
                      int32_t offset, int32_t far_bend)
{
    memset(image, 0, (size_t)rows * cols);
    const int32_t start_row = (int32_t)((uint32_t)rows * TRACK_VISION_ROI_START_PERCENT / 100U);
    const int32_t end_row = (int32_t)((uint32_t)rows * TRACK_VISION_ROI_END_PERCENT / 100U);
    const int32_t width = cols / 3;
    for (int32_t y = start_row; y < end_row; ++y)
    {
        const int32_t center = cols / 2 + offset +
            far_bend * (end_row - 1 - y) / (end_row - 1 - start_row);
        const int32_t left = center - width / 2;
        CHECK(left > 0 && left + width < cols);
        memset(image + (uint32_t)y * cols + (uint32_t)left, 255, (size_t)width);
    }
}

static void draw_opening_lane(uint8_t *image, uint16_t rows, uint16_t cols,
                              int32_t far_offset)
{
    memset(image, 0, (size_t)rows * cols);
    const int32_t start_row = (int32_t)((uint32_t)rows * TRACK_VISION_ROI_START_PERCENT / 100U);
    const int32_t end_row = (int32_t)((uint32_t)rows * TRACK_VISION_ROI_END_PERCENT / 100U);
    const int32_t span = end_row - 1 - start_row;
    for (int32_t y = start_row; y < end_row; ++y)
    {
        const int32_t distance = end_row - 1 - y;
        const int32_t width = cols * 3 / 10 + (cols * 3 / 10) * distance / span;
        const int32_t center = cols / 2 + far_offset * distance / span;
        const int32_t left = center - width / 2;
        CHECK(left > 0 && left + width < cols);
        memset(image + (uint32_t)y * cols + (uint32_t)left, 255, (size_t)width);
    }
}

static void test_resolution(uint16_t rows, uint16_t cols)
{
    const uint32_t count = (uint32_t)rows * cols;
    uint8_t *storage = malloc((size_t)count + 2U);
    CHECK(storage != NULL);
    uint8_t *image = storage + 1;
    storage[0] = 0xA5;
    storage[count + 1U] = 0x5A;
    guarded_result_t guarded;
    track_vision_result_t *result = &guarded.result;
    const uint16_t roi_start = (uint16_t)((uint32_t)rows * TRACK_VISION_ROI_START_PERCENT / 100U);
    const uint16_t roi_end = (uint16_t)((uint32_t)rows * TRACK_VISION_ROI_END_PERCENT / 100U);
    const uint16_t roi_rows = roi_end - roi_start;
    const uint16_t zone_rows = roi_rows / 3U;

    memset(image, 0, count);
    verify(image, rows, cols, &guarded);
    CHECK(!result->valid && result->valid_rows == 0U);
    CHECK((result->reject_flags & TRACK_VISION_REJECT_NO_SEED) != 0U);
    CHECK((result->reject_flags & TRACK_VISION_REJECT_TOTAL_COVERAGE) != 0U);
    CHECK((result->reject_flags & TRACK_VISION_REJECT_NEAR_COVERAGE) != 0U);
    CHECK((result->reject_flags & TRACK_VISION_REJECT_FAR_COVERAGE) != 0U);
    memset(image, 255, count);
    verify(image, rows, cols, &guarded);
    CHECK(!result->valid && result->valid_rows == 0U);

    // Isolated white noise must not become a track after merging short gaps.
    memset(image, 0, count);
    for (uint16_t y = roi_start; y < roi_end; ++y)
        for (uint16_t x = 2U; x < cols - 2U; x += 3U)
            image[(uint32_t)y * cols + x] = 255;
    verify(image, rows, cols, &guarded);
    CHECK(!result->valid && result->valid_rows == 0U);

    draw_lane(image, rows, cols, 0, 0);
    verify(image, rows, cols, &guarded);
    CHECK(result->valid && result->confidence == 100U);
    CHECK(result->valid_rows == roi_rows);
    CHECK(result->near_valid_rows == zone_rows);
    CHECK(result->far_valid_rows == zone_rows);
    CHECK(abs(result->error) <= 10 && result->heading == 0);

    draw_lane(image, rows, cols, cols / 8, 0);
    verify(image, rows, cols, &guarded);
    CHECK(result->valid && result->error > 200 && result->error < 300);
    CHECK(result->heading == 0);
    draw_lane(image, rows, cols, -(int32_t)(cols / 8), 0);
    verify(image, rows, cols, &guarded);
    CHECK(result->valid && result->error < -200 && result->error > -300);
    CHECK(result->heading == 0);

    // 近端最靠车的一段丢失，但远处仍有完整白轨：不得从远处重新起线。
    draw_lane(image, rows, cols, 0, 0);
    const uint16_t blank_bottom = (uint16_t)(roi_rows / 6U + 1U);
    memset(image + (uint32_t)(roi_end - blank_bottom) * cols, 0,
           (size_t)blank_bottom * cols);
    verify(image, rows, cols, &guarded);
    CHECK(!result->valid);
    CHECK((result->reject_flags & TRACK_VISION_REJECT_NEAR_ANCHOR) != 0U);
    CHECK(result->near_valid_rows * 100U >= zone_rows * 45U);

    // 仅远处突然转向时，远端方向不能像相连近端弯道一样拉动舵机。
    draw_lane(image, rows, cols, 0, 0);
    for (uint16_t y = roi_start; y < roi_start + zone_rows; ++y)
    {
        const int32_t shift = (int32_t)(cols / 6U) *
            (int32_t)(roi_start + zone_rows - 1U - y) / (int32_t)zone_rows;
        const int32_t left = (int32_t)(cols / 2U) + shift - (int32_t)(cols / 6U);
        memset(image + (uint32_t)y * cols, 0, cols);
        memset(image + (uint32_t)y * cols + left, 255, cols / 3U);
    }
    verify(image, rows, cols, &guarded);
    CHECK(result->valid && result->heading > 0 && result->heading < 140);

    draw_lane(image, rows, cols, 0, cols / 8);
    verify(image, rows, cols, &guarded);
    CHECK(result->valid && result->heading > 90 && result->error > 0);
    draw_lane(image, rows, cols, 0, -(int32_t)(cols / 8));
    verify(image, rows, cols, &guarded);
    CHECK(result->valid && result->heading < -90 && result->error < 0);

    // 近端在车身右侧、远端却弯向左侧：横向偏差仍必须反映近端位置。
    draw_lane(image, rows, cols, cols / 8, -(int32_t)(cols / 4));
    verify(image, rows, cols, &guarded);
    CHECK(result->valid && result->error > 100 && result->heading < -300);
    draw_lane(image, rows, cols, -(int32_t)(cols / 8), cols / 4);
    verify(image, rows, cols, &guarded);
    CHECK(result->valid && result->error < -100 && result->heading > 300);

    // Constant-width curves are ordinary track, not ring entrances.
    CHECK(!result->ring_candidate);
    draw_opening_lane(image, rows, cols, -(int32_t)(cols / 10));
    verify(image, rows, cols, &guarded);
    CHECK(result->valid);
    CHECK(result->ring_candidate);
    CHECK(result->ring_direction == 1);
    CHECK(result->far_width > result->near_width);
    CHECK(result->width_growth_percent >= 135U);
    // A symmetric opening (e.g. a cross) must not arm ring steering.
    draw_opening_lane(image, rows, cols, 0);
    verify(image, rows, cols, &guarded);
    CHECK(result->valid && !result->ring_candidate);
    draw_opening_lane(image, rows, cols, cols / 10);
    verify(image, rows, cols, &guarded);
    CHECK(result->valid && result->ring_candidate);
    CHECK(result->ring_direction == -1);

    draw_lane(image, rows, cols, 0, 0);
    for (uint16_t y = roi_start; y < roi_end; ++y)
    {
        image[(uint32_t)y * cols + cols / 2U] = 0;
        image[(uint32_t)y * cols + cols / 2U + 1U] = 0;
        image[(uint32_t)y * cols + 2U] = 255;
        image[(uint32_t)y * cols + cols / 8U] = 255;
        image[(uint32_t)y * cols + cols - 3U] = 255;
    }
    verify(image, rows, cols, &guarded);
    CHECK(result->valid && result->confidence == 100U);
    CHECK(abs(result->error) <= 10 && result->heading == 0);

    // Two substantial, separated white blocks cannot seed a unique track.
    memset(image, 0, count);
    for (uint16_t y = roi_start; y < roi_end; ++y)
    {
        memset(image + (uint32_t)y * cols + cols / 5U, 255, cols / 6U);
        memset(image + (uint32_t)y * cols + cols * 3U / 5U, 255, cols / 6U);
    }
    verify(image, rows, cols, &guarded);
    CHECK(!result->valid && result->valid_rows == 0U);
    CHECK(result->ambiguous_rows == roi_rows);
    CHECK((result->reject_flags & TRACK_VISION_REJECT_NO_SEED) != 0U);

    // A distant unrelated bright block must not steal the established track.
    draw_lane(image, rows, cols, 0, 0);
    for (uint16_t y = roi_start; y < roi_end - zone_rows; ++y)
        memset(image + (uint32_t)y * cols + cols / 16U, 255, cols / 10U);
    verify(image, rows, cols, &guarded);
    CHECK(result->valid && abs(result->error) <= 10);

    // An asymmetric fork must stop the centerline instead of selecting the
    // slightly nearer branch. Both arms still overlap the established lane.
    draw_lane(image, rows, cols, 0, 0);
    const uint16_t fork_row = roi_start + roi_rows / 2U;
    for (uint16_t y = roi_start; y < fork_row; ++y)
    {
        memset(image + (uint32_t)y * cols, 0, cols);
        memset(image + (uint32_t)y * cols + cols / 4U, 255, cols / 5U);
        memset(image + (uint32_t)y * cols + cols * 11U / 20U,
               255, cols * 11U / 50U);
    }
    verify(image, rows, cols, &guarded);
    CHECK(result->ambiguous_rows > 0U);
    for (uint16_t y = roi_start; y < fork_row; ++y)
        CHECK(!result->row_valid[y]);

    // 十字入口远端变成宽白区时，仍应凭近端赛道回中，而非保持旧转向。
    for (int sign = -1; sign <= 1; sign += 2)
    {
        draw_lane(image, rows, cols, sign * (int32_t)(cols / 8U), 0);
        for (uint16_t y = roi_start; y < roi_start + zone_rows; ++y)
            memset(image + (uint32_t)y * cols, 255, cols);
        verify(image, rows, cols, &guarded);
        CHECK(result->valid && result->cross_candidate && !result->ring_candidate);
        CHECK(result->heading == 0);
        CHECK(sign > 0 ? result->error > 200 : result->error < -200);
        CHECK(result->near_valid_rows == zone_rows);
        for (uint16_t y = roi_start; y < roi_start + zone_rows; ++y)
            CHECK(!result->row_valid[y]);
    }
    // 单行强反光及整幅全白不能被认成十字入口。
    draw_lane(image, rows, cols, 0, 0);
    memset(image + (uint32_t)roi_start * cols, 255, cols);
    verify(image, rows, cols, &guarded);
    CHECK(result->valid && !result->cross_candidate);
    memset(image, 255, count);
    verify(image, rows, cols, &guarded);
    CHECK(!result->valid && !result->cross_candidate);

    // Missing far/near thirds invalidate output even if 2/3 of ROI remains.
    draw_lane(image, rows, cols, 0, 0);
    memset(image + (uint32_t)roi_start * cols, 0, (size_t)zone_rows * cols);
    verify(image, rows, cols, &guarded);
    CHECK(!result->valid);
    CHECK((result->reject_flags & TRACK_VISION_REJECT_FAR_COVERAGE) != 0U);
    draw_lane(image, rows, cols, 0, 0);
    memset(image + (uint32_t)(roi_end - zone_rows) * cols, 0, (size_t)zone_rows * cols);
    verify(image, rows, cols, &guarded);
    CHECK(!result->valid);
    CHECK((result->reject_flags & TRACK_VISION_REJECT_NEAR_COVERAGE) != 0U);

    // Fixed car-body occlusion begins below the calibrated 68% lower bound.
    // It must not split the near-field lane or appear as a false inner edge.
    draw_lane(image, rows, cols, 0, 0);
    for (uint16_t y = roi_end; y < rows; ++y)
        memset(image + (uint32_t)y * cols + cols / 3U, 0, cols / 3U);
    verify(image, rows, cols, &guarded);
    CHECK(result->valid && result->confidence == 100U);
    CHECK(abs(result->error) <= 10 && result->heading == 0);
    for (uint16_t y = roi_end; y < rows; ++y)
        CHECK(result->row_valid[y] == 0U);

    // A full-width interruption cannot be replaced with an invented center.
    draw_lane(image, rows, cols, 0, 0);
    const uint16_t missing_row = roi_start + roi_rows / 2U;
    memset(image + (uint32_t)missing_row * cols, 0, cols);
    verify(image, rows, cols, &guarded);
    CHECK(result->valid && result->row_valid[missing_row] == 0U);
    CHECK(result->valid_rows == roi_rows - 1U);

    // Abruptly displaced far section must not be reacquired as the same track.
    draw_lane(image, rows, cols, 0, 0);
    for (uint16_t y = roi_start; y < roi_start + zone_rows; ++y)
    {
        memset(image + (uint32_t)y * cols, 0, cols);
        memset(image + (uint32_t)y * cols + cols * 3U / 4U, 255, cols / 6U);
    }
    verify(image, rows, cols, &guarded);
    CHECK(!result->valid);
    for (uint16_t y = roi_start; y < roi_start + zone_rows; ++y)
        CHECK(result->row_valid[y] == 0U);

    // Partial boundary and overly wide white region are both rejected.
    memset(image, 0, count);
    for (uint16_t y = roi_start; y < roi_end; ++y)
        memset(image + (uint32_t)y * cols, 255, cols / 2U);
    verify(image, rows, cols, &guarded);
    CHECK(!result->valid && result->valid_rows == 0U);
    memset(image, 0, count);
    for (uint16_t y = roi_start; y < roi_end; ++y)
        memset(image + (uint32_t)y * cols + cols / 20U, 255, cols * 9U / 10U);
    verify(image, rows, cols, &guarded);
    CHECK(!result->valid && result->valid_rows == 0U);

    CHECK(storage[0] == 0xA5 && storage[count + 1U] == 0x5A);
    free(storage);
    printf("PASS %u x %u: straight/offset/curve, noise/gaps, ambiguity, loss, bounds\n",
           (unsigned int)cols, (unsigned int)rows);
}

static void test_temporal_lock(void)
{
    const uint16_t rows = 240U, cols = 376U;
    uint8_t *image = malloc((size_t)rows * cols);
    CHECK(image != NULL);
    guarded_result_t guarded;
    track_vision_lock_t lock;
    track_vision_lock_init(&lock);
    draw_lane(image, rows, cols, 0, 0);
    for (unsigned int frame = 0U; frame < 3U; ++frame)
    {
        verify(image, rows, cols, &guarded);
        track_vision_lock_update(&lock, &guarded.result);
        CHECK(guarded.result.valid == (frame == 2U));
    }
    CHECK(lock.phase == TRACK_VISION_LOCKED);
    draw_lane(image, rows, cols, cols / 4U, 0);
    verify(image, rows, cols, &guarded);
    track_vision_lock_update(&lock, &guarded.result);
    CHECK(!guarded.result.valid && guarded.result.error == 0);
    CHECK((guarded.result.reject_flags & TRACK_VISION_REJECT_TRACK_JUMP) != 0U);
    CHECK(!guarded.result.ring_candidate);
    // 单帧远处白轨消失后，应立即重新锁回原近端赛道。
    draw_lane(image, rows, cols, 0, 0);
    verify(image, rows, cols, &guarded);
    track_vision_lock_update(&lock, &guarded.result);
    CHECK(guarded.result.valid && lock.phase == TRACK_VISION_LOCKED);
    // 真实换线须稳定三帧才可接受，不会因一个误识别帧转向。
    draw_lane(image, rows, cols, cols / 4U, 0);
    for (unsigned int frame = 0U; frame < 3U; ++frame)
    {
        verify(image, rows, cols, &guarded);
        track_vision_lock_update(&lock, &guarded.result);
        CHECK(guarded.result.valid == (frame == 2U));
    }
    CHECK(lock.phase == TRACK_VISION_LOCKED && guarded.result.error > 400);
    free(image);
    puts("PASS near-anchor and temporal-lock state machine");
}

static void test_invalid_dimensions(void)
{
    guarded_result_t guarded;
    const uint8_t one_pixel = 255;
    static const uint16_t invalid[][2] = {
        {0, 376}, {240, 0}, {9, 376}, {240, 15},
        {481, 376}, {240, 753}, {UINT16_MAX, UINT16_MAX}
    };
    for (unsigned int i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i)
    {
        memset(&guarded, 0xA5, sizeof(guarded));
        guarded.before = 123U;
        guarded.after = 456U;
        track_vision_process(&one_pixel, invalid[i][0], invalid[i][1], &guarded.result);
        CHECK(guarded.before == 123U && guarded.after == 456U);
        CHECK(!guarded.result.valid && guarded.result.valid_rows == 0U);
        CHECK(guarded.result.reject_flags == TRACK_VISION_REJECT_INPUT);
        for (uint16_t row = 0; row < TRACK_VISION_MAX_ROWS; ++row)
            CHECK(guarded.result.row_valid[row] == 0U && guarded.result.center[row] == 0U);
    }
    track_vision_process(NULL, 240, 376, &guarded.result);
    CHECK(!guarded.result.valid && guarded.result.valid_rows == 0U);
    CHECK(guarded.result.reject_flags == TRACK_VISION_REJECT_INPUT);
    track_vision_process(NULL, 0, 0, NULL);
    puts("PASS empty/null inputs, invalid dimensions, result-array guards");
}

int main(void)
{
    test_invalid_dimensions();
    test_temporal_lock();
    test_resolution(120, 188);
    test_resolution(240, 376);
    test_resolution(480, 752);
    puts("All track-vision host tests passed.");
    return EXIT_SUCCESS;
}
