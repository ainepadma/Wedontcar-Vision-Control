#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "img_process.h"
#include "track_vision.h"

#define CHECK(c) do { if (!(c)) { \
    fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #c); exit(EXIT_FAILURE); \
} } while (0)

static uint8_t image[TRACK_VISION_MAX_ROWS * TRACK_VISION_MAX_COLS];
static uint8_t saved[sizeof(image)];
static uint8_t scratch[5U * TRACK_VISION_MAX_COLS];
static uint32_t histogram[256];
static track_vision_result_t result;

static uint8_t process(uint16_t rows, uint16_t cols)
{
    uint8_t threshold = 0;
    CHECK(image_track_preprocess(image, rows, cols,
        (uint16_t)((uint32_t)rows * 30U / 100U),
        (uint16_t)((uint32_t)rows * TRACK_VISION_ROI_END_PERCENT / 100U),
        scratch, sizeof(scratch), histogram, &threshold));
    for (uint32_t i = 0; i < (uint32_t)rows * cols; ++i)
        CHECK(image[i] == 0U || image[i] == 255U);
    if (threshold != 255U)
    track_vision_process(image, rows, cols, &result);
    else memset(&result, 0, sizeof(result));
    return threshold;
}

static void lane(uint16_t rows, uint16_t cols, uint8_t dark, uint8_t light, uint16_t width)
{
    uint16_t left = (uint16_t)((cols - width) / 2U);
    for (uint16_t y = 0; y < rows; ++y)
        for (uint16_t x = 0; x < cols; ++x)
            image[(uint32_t)y * cols + x] = x >= left && x < left + width ? light : dark;
}

static void roi_isolation(uint16_t rows, uint16_t cols, uint8_t dark, uint8_t light)
{
    const uint16_t first = (uint16_t)((uint32_t)rows * 30U / 100U);
    const uint16_t end = (uint16_t)((uint32_t)rows * TRACK_VISION_ROI_END_PERCENT / 100U);
    lane(rows, cols, dark, light, cols / 3U);
    uint8_t expected = process(rows, cols);
    CHECK(result.valid && abs(result.error) < 20);
    memcpy(saved, image, (size_t)rows * cols);
    for (unsigned int pattern = 0; pattern < 3U; ++pattern)
    {
        lane(rows, cols, dark, light, cols / 3U);
        for (uint16_t y = 0; y < rows; ++y)
            if (y < first || y >= end)
                for (uint16_t x = 0; x < cols; ++x)
                    image[(uint32_t)y * cols + x] = pattern == 0U ? 0U :
                        pattern == 1U ? 255U : (uint8_t)((x * 31U + y * 17U) & 255U);
        CHECK(process(rows, cols) == expected);
        CHECK(memcmp(saved + (uint32_t)first * cols, image + (uint32_t)first * cols,
                     (size_t)(end - first) * cols) == 0);
        CHECK(result.valid && abs(result.error) < 20);
    }
    printf("PASS %ux%u ground isolation (including adjacent bright rows), grey %u/%u\n",
           (unsigned int)cols, (unsigned int)rows, dark, light);
}

static void resolution(uint16_t rows, uint16_t cols)
{
    const uint8_t uniform[] = {0, 1, 8, 127, 254, 255};
    for (unsigned int i = 0; i < sizeof(uniform); ++i)
    {
        memset(image, uniform[i], (size_t)rows * cols);
        process(rows, cols);
        CHECK(!result.valid);
    }
    const uint16_t widths[] = { (uint16_t)(cols / 16U + 1U), (uint16_t)(cols / 3U),
                               (uint16_t)(cols * 3U / 4U) };
    for (unsigned int i = 0; i < sizeof(widths) / sizeof(widths[0]); ++i)
    {
        lane(rows, cols, 20U, 60U, widths[i]);
        process(rows, cols);
        CHECK(result.valid && abs(result.error) < 20);
    }
    roi_isolation(rows, cols, 20U, 60U);
    roi_isolation(rows, cols, 0U, 1U); /* Fractional dark means must survive. */
    puts("PASS constants rejected, thin/ordinary/wide road, dark precision");
}

static void islands_and_stains(void)
{
    const uint16_t rows = 240, cols = 376;
    memset(image, 60, (size_t)rows * cols);
    for (uint16_t y = 110; y < 114; ++y)
        memset(image + (uint32_t)y * cols + 160U, 20, 50);
    process(rows, cols);
    CHECK(image[100U * cols + 180U] == 255U);
    CHECK(image[111U * cols + 180U] == 0U);
    /* Small white class is the symmetric tail-trimming regression. */
    memset(image, 20, (size_t)rows * cols);
    for (uint16_t y = 110; y < 114; ++y)
        memset(image + (uint32_t)y * cols + 160U, 60, 50);
    process(rows, cols);
    CHECK(image[100U * cols + 180U] == 0U);
    CHECK(image[111U * cols + 180U] == 255U);

    lane(rows, cols, 20, 60, cols / 3U);
    for (uint16_t y = 115; y < 122; ++y)
        memset(image + (uint32_t)y * cols + 180U, 45, 7);
    process(rows, cols);
    CHECK(result.valid && image[118U * cols + 183U] == 255U);
    lane(rows, cols, 20, 60, cols / 3U);
    for (uint16_t y = 112; y < 126; ++y)
        memset(image + (uint32_t)y * cols + 176U, 25, 14);
    process(rows, cols);
    CHECK(image[118U * cols + 183U] == 0U); /* A large deep stain is not a small hole. */
    puts("PASS minority black island / white branch retained; shallow stain white, deep stain retained");
}

static void cleanup(void)
{
    const uint16_t rows = 30, cols = 80;
    memset(image, 255, (size_t)rows * cols);
    image[4U * cols + 4U] = 0;
    image[4U * cols + 10U] = image[4U * cols + 11U] = 0;
    image[4U * cols + 20U] = image[4U * cols + 21U] = 0;
    image[5U * cols + 20U] = image[5U * cols + 21U] = 0;
    for (uint16_t y = 10; y < 14; ++y) memset(image + (uint32_t)y * cols + 25, 0, 30);
    for (uint16_t y = 1; y < 29; ++y) image[(uint32_t)y * cols + 65] = 0;
    memset(image + 20U * cols + 5U, 0, 50);
    image[24U * cols] = image[24U * cols + 1U] = 0;
    memcpy(saved, image, (size_t)rows * cols);
    CHECK(image_clean_binary(image, rows, cols, scratch, sizeof(scratch)));
    CHECK(image[4U * cols + 4U] == 255U);
    CHECK(image[4U * cols + 10U] == 255U && image[4U * cols + 11U] == 255U);
    CHECK(image[4U * cols + 20U] == 0U && image[5U * cols + 21U] == 0U);
    for (uint16_t y = 10; y < 14; ++y)
        for (uint16_t x = 25; x < 55; ++x) CHECK(image[(uint32_t)y * cols + x] == 0U);
    for (uint16_t y = 1; y < 29; ++y) CHECK(image[(uint32_t)y * cols + 65] == 0U);
    CHECK(image[20U * cols + 20U] == 0U && image[24U * cols + 1U] == 0U);
    uint8_t expected[30U * 80U];
    memcpy(expected, image, sizeof(expected));
    for (uint16_t y = 0; y < rows; ++y)
        for (uint16_t x = 0; x < cols; ++x)
            image[(uint32_t)y * cols + x] = saved[(uint32_t)y * cols + cols - 1U - x];
    CHECK(image_clean_binary(image, rows, cols, scratch, sizeof(scratch)));
    for (uint16_t y = 0; y < rows; ++y)
        for (uint16_t x = 0; x < cols; ++x)
            CHECK(image[(uint32_t)y * cols + x] == expected[(uint32_t)y * cols + cols - 1U - x]);
    memset(image, 0, (size_t)rows * cols);
    for (uint16_t y = 4U; y < 6U; ++y)
        for (uint16_t x = 4U; x < 6U; ++x) image[(uint32_t)y * cols + x] = 255U;
    for (uint16_t y = 10U; y < 25U; ++y)
        memset(image + (uint32_t)y * cols + 30U, 255, 20U);
    CHECK(image_clean_binary(image, rows, cols, scratch, sizeof(scratch)));
    CHECK(image[4U * cols + 4U] == 0U && image[5U * cols + 5U] == 0U);
    CHECK(image[15U * cols + 35U] == 255U);
    puts("PASS isolated cleanup, white 2x2 spots, black holes, long lines, road and mirror symmetry");
}

static void invalid_input(void)
{
    uint8_t threshold = 77;
    memset(image, 123, 100);
    memcpy(saved, image, 100);
    CHECK(!image_track_preprocess(NULL, 10, 10, 3, 9, scratch, sizeof(scratch), histogram, &threshold));
    CHECK(!image_track_preprocess(image, 10, 10, 3, 9, scratch, 49, histogram, &threshold));
    CHECK(!image_track_preprocess(image, 10, 10, 8, 5, scratch, sizeof(scratch), histogram, &threshold));
    CHECK(!image_track_preprocess(image, 10, 10, 3, 11, scratch, sizeof(scratch), histogram, &threshold));
    CHECK(!image_track_preprocess(image, 10, 10, 3, 7, scratch, sizeof(scratch), histogram, &threshold));
    CHECK(!image_track_preprocess(image, 10, 10, 3, 9, image, sizeof(scratch), histogram, &threshold));
    CHECK(!image_track_preprocess(image, 10, 10, 3, 9, scratch, sizeof(scratch), histogram, image));
    CHECK(!image_track_preprocess(image, 10, 10, 3, 9, scratch, sizeof(scratch), NULL, &threshold));
    CHECK(!image_clean_binary(image, 10, 10, image, sizeof(scratch)));
    CHECK(threshold == 77U && memcmp(image, saved, 100) == 0);
    puts("PASS invalid/overlap/short-buffer rejection without image mutation");
}

static void captured(const char *path)
{
    FILE *file = fopen(path, "rb");
    CHECK(file != NULL);
    CHECK(fread(image, 1, 376U * 240U, file) == 376U * 240U);
    CHECK(fgetc(file) == EOF && !ferror(file));
    fclose(file);
    memcpy(saved, image, 376U * 240U);
    CHECK(image_mean3x3(image, 240, 376, scratch, sizeof(scratch)));
    get_hist_gram(image, 240, 376, histogram);
    uint8_t old_threshold = get_threshold_otsu(histogram);
    if (old_threshold < 1U) old_threshold = 1U;
    if (old_threshold > 254U) old_threshold = 254U;
    binaryzation_process(image, 240, 376, old_threshold);
    track_vision_process(image, 240, 376, &result);
    printf("OLD    %s T=%u valid=%u Q=%u rows=%u\n", path,
        old_threshold, result.valid, result.confidence, result.valid_rows);
    memcpy(image, saved, 376U * 240U);
    uint8_t threshold = process(240, 376);
    printf("REPLAY %s T=%u valid=%u Q=%u rows=%u error=%d heading=%d\n", path,
        threshold, result.valid, result.confidence, result.valid_rows, result.error, result.heading);
}

int main(int argc, char **argv)
{
    /* Weak ground must not erase visible pixels outside the recognition ROI. */
    memset(image, 0, sizeof(image));
    memset(image, 255, 60U * 376U);
    CHECK(process(240, 376) == 255U);
    CHECK(!result.valid);
    CHECK(image[20U * 376U + 100U] == 255U);
    CHECK(image[120U * 376U + 100U] == 0U);
    invalid_input();
    cleanup();
    resolution(120, 188);
    resolution(240, 376);
    resolution(480, 752);
    islands_and_stains();
    for (int i = 1; i < argc; ++i) captured(argv[i]);
    puts("All ground preprocessing tests passed; no hardware accessed.");
    return EXIT_SUCCESS;
}
