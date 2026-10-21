#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "track_vision.h"

#define H 240U
#define W 376U
#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); exit(1); \
} } while (0)

static uint8_t image[H * W];
static track_vision_result_t result;

/* 近处右边界出画，远处双边界完整；镜像后测试左弯。 */
static void make_bend(uint8_t mirror)
{
    memset(image, 0, sizeof(image));
    for (unsigned y = H * 40U / 100U; y < H * 68U / 100U; ++y)
    {
        unsigned distance = H * 68U / 100U - 1U - y;
        int center = 250 + (int)(distance * 16U / 66U);
        int width = 286 - (int)(distance * 126U / 66U);
        int left = center - width / 2;
        int right = left + width - 1;
        for (int x = left; x <= right; ++x)
        {
            int col = mirror ? (int)W - 1 - x : x;
            if (col >= 0 && col < (int)W) image[y * W + (unsigned)col] = 255U;
        }
    }
}

static void test_bend(uint8_t mirror)
{
    make_bend(mirror);
    track_vision_process(image, H, W, &result);
    CHECK(result.valid);
    CHECK(result.near_valid_rows >= 12U);
    CHECK(result.row_valid[155U]);
    CHECK(result.row_valid[110U]);
    CHECK(result.center[155U] != result.center[110U]);
    if (!mirror)
    {
        CHECK(result.center[155U] >= 245U && result.center[155U] <= 260U);
        CHECK(result.center[110U] > result.center[155U]);
        CHECK(result.heading > 0);
    }
    else
    {
        CHECK(result.center[155U] >= 115U && result.center[155U] <= 130U);
        CHECK(result.center[110U] < result.center[155U]);
        CHECK(result.heading < 0);
    }
}

static void test_display_noise(void)
{
    make_bend(0U);
    /* 赛道外的左侧亮块与顶部反光即使存在，也不能作为弯道。 */
    for (unsigned y = 102U; y < 116U; ++y)
        memset(image + y * W + 8U, 255, 12U);
    memset(image + 10U * W + 25U, 255, 25U);
    track_vision_process(image, H, W, &result);
    CHECK(result.valid);
    int16_t error = result.error, heading = result.heading;
    uint16_t center = result.center[110U];
    CHECK(image[108U * W + 12U] == 255U);
    CHECK(image[10U * W + 30U] == 255U);
    track_vision_filter_display(image, H, W);
    CHECK(image[108U * W + 12U] == 0U);
    CHECK(image[10U * W + 30U] == 0U);
    CHECK(image[110U * W + center] == 255U);
    CHECK(result.error == error && result.heading == heading);
}

int main(void)
{
    test_bend(0U);
    test_bend(1U);
    test_display_noise();
    make_bend(0U);
    /* 完整的近端黑带不能靠远处的白路“脑补”出近端中线。 */
    memset(image + 135U * W, 0, 4U * W);
    track_vision_process(image, H, W, &result);
    CHECK(!result.row_valid[155U]);
    puts("PASS curved edge: right/left bend centre follows road; black gap blocks extrapolation");
    return 0;
}
