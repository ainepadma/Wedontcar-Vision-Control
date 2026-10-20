#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "img_process.h"

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); \
        exit(EXIT_FAILURE); \
    } \
} while (0)

static void check_line_bounds(const uint16_t *left, const uint16_t *mid,
                              const uint16_t *right, uint16_t rows, uint16_t cols)
{
    for (uint16_t y = 0; y < rows; ++y) {
        CHECK(left[y] < cols);
        CHECK(mid[y] < cols);
        CHECK(right[y] < cols);
        CHECK(mid[y] == ((uint32_t)left[y] + right[y]) / 2U);
    }
}

static void test_resolution(uint16_t rows, uint16_t cols)
{
    uint32_t count = (uint32_t)rows * cols;
    uint8_t *storage = malloc((size_t)count + 2U);
    uint16_t *lines = malloc(((size_t)rows * 3U + 2U) * sizeof(*lines));
    uint32_t histogram[256];
    CHECK(storage != NULL && lines != NULL);
    uint8_t *pixels = storage + 1;
    uint16_t *left = lines + 1;
    uint16_t *mid = left + rows;
    uint16_t *right = mid + rows;
    storage[0] = 0xA5;
    storage[count + 1U] = 0x5A;
    lines[0] = 0x1234;
    lines[(size_t)rows * 3U + 1U] = 0x5678;

    // A full single-bin histogram must retain all 360960 pixels at 752 x 480.
    memset(pixels, 0, count);
    memset(histogram, 0xFF, sizeof(histogram));
    get_hist_gram(pixels, rows, cols, histogram);
    CHECK(histogram[0] == count);
    for (unsigned int i = 1; i < 256; ++i) CHECK(histogram[i] == 0);
    CHECK(get_threshold_otsu(histogram) == 0);
    binaryzation_process(pixels, rows, cols, 127);
    for (uint32_t i = 0; i < count; ++i) CHECK(pixels[i] == 0);
    memset(left, 0xFF, (size_t)rows * 3U * sizeof(*left));
    auxiliary_process(pixels, rows, cols, 127, left, mid, right);
    check_line_bounds(left, mid, right, rows, cols);

    memset(pixels, 255, count);
    get_hist_gram(pixels, rows, cols, histogram);
    CHECK(histogram[255] == count);
    CHECK(histogram[0] == 0); // No stale histogram counts from the preceding frame.
    CHECK(get_threshold_otsu(histogram) == 255); // Preserve constant-image semantics.
    binaryzation_process(pixels, rows, cols, 127);
    for (uint32_t i = 0; i < count; ++i) CHECK(pixels[i] == 255);
    memset(left, 0xFF, (size_t)rows * 3U * sizeof(*left));
    auxiliary_process(pixels, rows, cols, 127, left, mid, right);
    for (uint16_t y = 0; y < rows; ++y) {
        CHECK(left[y] == 0);
        CHECK(right[y] == cols - 1U);
        CHECK(mid[y] == (cols - 1U) / 2U);
    }

    // Bright track between two dark background regions; expected edges are exact.
    uint16_t stripe_start = cols / 4U;
    uint16_t stripe_end = (uint16_t)((uint32_t)cols * 3U / 4U);
    for (uint16_t y = 0; y < rows; ++y) {
        for (uint16_t x = 0; x < cols; ++x) {
            pixels[(uint32_t)y * cols + x] =
                x >= stripe_start && x < stripe_end ? 210 : 40;
        }
    }
    get_hist_gram(pixels, rows, cols, histogram);
    CHECK(histogram[210] == (uint32_t)rows * (stripe_end - stripe_start));
    CHECK(histogram[40] + histogram[210] == count);
    CHECK(get_threshold_otsu(histogram) == 40);
    binaryzation_process(pixels, rows, cols, get_threshold_otsu(histogram));
    for (uint16_t y = 0; y < rows; ++y) {
        for (uint16_t x = 0; x < cols; ++x) {
            CHECK(pixels[(uint32_t)y * cols + x] ==
                  (x >= stripe_start && x < stripe_end ? 255 : 0));
        }
    }
    auxiliary_process(pixels, rows, cols, 127, left, mid, right);
    for (uint16_t y = 0; y < rows; ++y) {
        CHECK(left[y] == stripe_start - 1U);
        CHECK(right[y] == stripe_end);
        CHECK(mid[y] == (stripe_start - 1U + stripe_end) / 2U);
    }

    // Track touches the left image edge: missing left boundary defaults to zero.
    for (uint16_t y = 0; y < rows; ++y) {
        memset(pixels + (uint32_t)y * cols, 255, stripe_end);
        memset(pixels + (uint32_t)y * cols + stripe_end, 0, cols - stripe_end);
    }
    memset(left, 0xFF, (size_t)rows * 3U * sizeof(*left));
    auxiliary_process(pixels, rows, cols, 127, left, mid, right);
    for (uint16_t y = 0; y < rows; ++y) {
        CHECK(left[y] == 0);
        CHECK(right[y] == stripe_end);
    }
    check_line_bounds(left, mid, right, rows, cols);

    CHECK(storage[0] == 0xA5 && storage[count + 1U] == 0x5A);
    CHECK(lines[0] == 0x1234 && lines[(size_t)rows * 3U + 1U] == 0x5678);
    free(lines);
    free(storage);
    printf("PASS %u x %u: histogram, black/white, stripe, missing edge, bounds\n",
           (unsigned int)cols, (unsigned int)rows);
}

static void test_empty_and_small(void)
{
    uint32_t histogram[256];
    memset(histogram, 0, sizeof(histogram));
    CHECK(get_threshold_otsu(histogram) == 0);
    memset(histogram, 0xFF, sizeof(histogram));
    get_hist_gram(NULL, 0, 752, histogram);
    for (unsigned int i = 0; i < 256; ++i) CHECK(histogram[i] == 0);
    binaryzation_process(NULL, 0, 752, 127);
    auxiliary_process(NULL, 0, 752, 127, NULL, NULL, NULL);
    auxiliary_process(NULL, 480, 0, 127, NULL, NULL, NULL);

    for (uint16_t cols = 1; cols <= 4; ++cols) {
        uint8_t pixels[4] = {0, 255, 0, 255};
        uint16_t left = UINT16_MAX, mid = UINT16_MAX, right = UINT16_MAX;
        auxiliary_process(pixels, 1, cols, 127, &left, &mid, &right);
        check_line_bounds(&left, &mid, &right, 1, cols);
    }
    puts("PASS empty histogram, zero dimensions, narrow images");
}

static void reference_mean3x3(const uint8_t *source, uint8_t *destination,
                              uint16_t rows, uint16_t cols)
{
    memcpy(destination, source, (size_t)rows * cols);
    for (uint16_t row = 1U; row + 1U < rows; ++row) {
        for (uint16_t col = 1U; col + 1U < cols; ++col) {
            unsigned int sum = 0U;
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    uint32_t y = (uint32_t)((int)row + dy);
                    uint32_t x = (uint32_t)((int)col + dx);
                    sum += source[y * cols + x];
                }
            }
            destination[(uint32_t)row * cols + col] = (uint8_t)(sum / 9U);
        }
    }
}

static void check_mean_border(const uint8_t *image, const uint8_t *original,
                              uint16_t rows, uint16_t cols)
{
    CHECK(memcmp(image, original, cols) == 0);
    CHECK(memcmp(image + (uint32_t)(rows - 1U) * cols,
                 original + (uint32_t)(rows - 1U) * cols, cols) == 0);
    for (uint16_t row = 0U; row < rows; ++row) {
        CHECK(image[(uint32_t)row * cols] == original[(uint32_t)row * cols]);
        CHECK(image[(uint32_t)row * cols + cols - 1U] ==
              original[(uint32_t)row * cols + cols - 1U]);
    }
}

static void test_mean_resolution(uint16_t rows, uint16_t cols)
{
    uint32_t count = (uint32_t)rows * cols;
    uint32_t scratch_size = (uint32_t)cols * 2U;
    uint8_t *storage = malloc((size_t)count + 2U);
    uint8_t *original = malloc(count);
    uint8_t *reference = malloc(count);
    uint8_t *scratch_storage = malloc((size_t)scratch_size + 2U);
    CHECK(storage != NULL && original != NULL && reference != NULL && scratch_storage != NULL);
    uint8_t *image = storage + 1;
    uint8_t *scratch = scratch_storage + 1;
    uint32_t random = UINT32_C(123456789);
    storage[0] = 0xA5U;
    storage[count + 1U] = 0x5AU;
    scratch_storage[0] = 0x69U;
    scratch_storage[scratch_size + 1U] = 0x96U;

    for (unsigned int pattern = 0U; pattern < 6U; ++pattern) {
        for (uint16_t row = 0U; row < rows; ++row) {
            for (uint16_t col = 0U; col < cols; ++col) {
                uint8_t pixel;
                switch (pattern) {
                    case 0U: pixel = 0U; break;
                    case 1U: pixel = 255U; break;
                    case 2U: pixel = col % 2U ? 255U : 0U; break;
                    case 3U: pixel = row % 2U ? 255U : 0U; break;
                    case 4U: pixel = (uint8_t)(((uint32_t)row + col) % 256U); break;
                    default:
                        random = random * UINT32_C(1664525) + UINT32_C(1013904223);
                        pixel = (uint8_t)(random >> 24);
                        break;
                }
                image[(uint32_t)row * cols + col] = pixel;
            }
        }
        /* Reuse the exact two-row buffer across patterns and repeated passes.
         * A separate source/destination reference never sees filtered neighbours. */
        for (unsigned int pass = 0U; pass < 3U; ++pass) {
            memcpy(original, image, count);
            reference_mean3x3(original, reference, rows, cols);
            memset(scratch, (int)(0x31U + pass), scratch_size);
            CHECK(image_mean3x3(image, rows, cols, scratch, scratch_size) == 1U);
            CHECK(memcmp(image, reference, count) == 0);
            check_mean_border(image, original, rows, cols);
            CHECK(storage[0] == 0xA5U && storage[count + 1U] == 0x5AU);
            CHECK(scratch_storage[0] == 0x69U && scratch_storage[scratch_size + 1U] == 0x96U);
        }
    }
    free(scratch_storage);
    free(reference);
    free(original);
    free(storage);
    printf("PASS mean3x3 %u x %u: black/white, stripes, gradient/random, exact reference, borders/canaries, reuse\n",
           (unsigned int)cols, (unsigned int)rows);
}

static void test_mean_invalid_and_rounding(void)
{
    uint8_t image[64];
    uint8_t saved[sizeof(image)];
    uint8_t scratch[64];
    uint8_t saved_scratch[sizeof(scratch)];
    for (unsigned int i = 0U; i < sizeof(image); ++i) image[i] = (uint8_t)(i * 3U);
    memset(scratch, 0xC3, sizeof(scratch));
    memcpy(saved, image, sizeof(image));
    memcpy(saved_scratch, scratch, sizeof(scratch));
    CHECK(image_mean3x3(NULL, 3U, 3U, scratch, sizeof(scratch)) == 0U);
    CHECK(image_mean3x3(image, 3U, 3U, NULL, sizeof(scratch)) == 0U);
    for (uint16_t rows = 0U; rows < 3U; ++rows)
        CHECK(image_mean3x3(image, rows, 8U, scratch, sizeof(scratch)) == 0U);
    for (uint16_t cols = 0U; cols < 3U; ++cols)
        CHECK(image_mean3x3(image, 8U, cols, scratch, sizeof(scratch)) == 0U);
    CHECK(image_mean3x3(image, 8U, 8U, scratch, 15U) == 0U);
    CHECK(image_mean3x3(image, 8U, 8U, scratch, 0U) == 0U);
    CHECK(image_mean3x3(image, 8U, 8U, image, 16U) == 0U);
    CHECK(image_mean3x3(image, 8U, 8U, image + 63U, 16U) == 0U);
    CHECK(image_mean3x3(image + 8U, 3U, 8U, image, 16U) == 0U);
    CHECK(memcmp(image, saved, sizeof(image)) == 0);
    CHECK(memcmp(scratch, saved_scratch, sizeof(scratch)) == 0);

    /* 8 / 9 must truncate to zero, not round up. Only the center may change. */
    uint8_t small[9] = {1U, 1U, 1U, 1U, 0U, 1U, 1U, 1U, 1U};
    uint8_t small_scratch[6];
    uint8_t small_copy[9];
    memcpy(small_copy, small, sizeof(small));
    CHECK(image_mean3x3(small, 3U, 3U, small_scratch, sizeof(small_scratch)) == 1U);
    CHECK(small[4] == 0U);
    check_mean_border(small, small_copy, 3U, 3U);
    memset(small, 255, sizeof(small));
    CHECK(image_mean3x3(small, 3U, 3U, small_scratch, sizeof(small_scratch)) == 1U);
    CHECK(small[4] == 255U); /* Nine white pixels sum to 2295, not uint8 overflow. */
    puts("PASS mean3x3 null/small/short/overlap rejection, no mutation, integer truncation");
}

static void test_low_dynamic_adaptive_threshold(void)
{
    uint8_t image[32];
    for (uint16_t row = 0U; row < 4U; ++row) {
        const uint8_t high = row < 2U ? 10U : 20U;
        memset(image + (uint32_t)row * 8U, 0, 4U);
        memset(image + (uint32_t)row * 8U + 4U, high, 4U);
    }
    uint32_t histogram[256];
    CHECK(image_banded_percentile_stretch(image, 4U, 8U, 2U,
                                           25U, 75U, histogram) == 1U);
    for (uint16_t row = 0U; row < 4U; ++row) {
        for (uint16_t col = 0U; col < 8U; ++col)
            CHECK(image[(uint32_t)row * 8U + col] == (col < 4U ? 0U : 255U));
    }
    CHECK(image_banded_otsu_binary_offset(image, 4U, 8U, 2U,
                                           12, histogram) == 12U);
    for (uint16_t row = 0U; row < 4U; ++row) {
        for (uint16_t col = 0U; col < 8U; ++col)
            CHECK(image[(uint32_t)row * 8U + col] == (col < 4U ? 0U : 255U));
    }
    CHECK(image_banded_percentile_stretch(NULL, 4U, 8U, 2U,
                                           1U, 99U, histogram) == 0U);
    CHECK(image_banded_percentile_stretch(image, 4U, 8U, 2U,
                                           99U, 1U, histogram) == 0U);
    CHECK(image_banded_otsu_binary_offset(NULL, 4U, 8U, 2U,
                                           0, histogram) == 0U);
    puts("PASS low-dynamic band stretch and offset Otsu");
}

static void test_banded_stretch_has_no_horizontal_seams(void)
{
    enum { ROWS = 32, COLS = 64 };
    uint8_t image[ROWS * COLS];
    uint32_t histogram[256];
    for (uint16_t row = 0U; row < ROWS; ++row)
        for (uint16_t col = 0U; col < COLS; ++col)
            image[(uint32_t)row * COLS + col] =
                (uint8_t)(8U + row * 2U + col / 2U);

    CHECK(image_banded_percentile_stretch(image, ROWS, COLS, 8U,
                                           5U, 95U, histogram) == 1U);
    for (uint16_t row = 8U; row < ROWS; row += 8U) {
        uint32_t previous = 0U;
        uint32_t current = 0U;
        for (uint16_t col = 0U; col < COLS; ++col) {
            previous += image[(uint32_t)(row - 1U) * COLS + col];
            current += image[(uint32_t)row * COLS + col];
        }
        const uint32_t difference = previous > current ?
            previous - current : current - previous;
        CHECK(difference <= 6U * COLS);
    }
    puts("PASS band parameters interpolate across rows without horizontal seams");
}

static void test_vertical_median_rejects_row_impulses(void)
{
    enum { ROWS = 7, COLS = 8 };
    uint8_t image[ROWS * COLS];
    uint8_t scratch[2U * COLS];

    for (uint16_t row = 0U; row < ROWS; ++row)
        for (uint16_t col = 0U; col < COLS; ++col)
            image[(uint32_t)row * COLS + col] = col < 3U ? 6U : 18U;
    memset(image + 3U * COLS, 220, COLS);
    CHECK(image_vertical_median3(image, ROWS, COLS, scratch,
                                 sizeof(scratch)) == 1U);
    for (uint16_t row = 0U; row < ROWS; ++row)
        for (uint16_t col = 0U; col < COLS; ++col)
            CHECK(image[(uint32_t)row * COLS + col] ==
                  (col < 3U ? 6U : 18U));

    memset(image, 180, sizeof(image));
    memset(image + 3U * COLS, 0, COLS);
    CHECK(image_vertical_median3(image, ROWS, COLS, scratch,
                                 sizeof(scratch)) == 1U);
    for (uint32_t index = 0U; index < sizeof(image); ++index)
        CHECK(image[index] == 180U);

    CHECK(image_vertical_median3(NULL, ROWS, COLS, scratch,
                                 sizeof(scratch)) == 0U);
    CHECK(image_vertical_median3(image, 2U, COLS, scratch,
                                 sizeof(scratch)) == 0U);
    CHECK(image_vertical_median3(image, ROWS, COLS, scratch,
                                 sizeof(scratch) - 1U) == 0U);
    CHECK(image_vertical_median3(image, ROWS, COLS, image,
                                 sizeof(image)) == 0U);
    puts("PASS vertical median removes bright/dark row impulses and preserves columns");
}

static void test_near_color_denoise_uses_two_dimensional_neighborhood(void)
{
    enum { ROWS = 7, COLS = 9 };
    uint8_t image[ROWS * COLS];
    uint8_t original[ROWS * COLS];
    uint8_t scratch[2U * COLS];
    for (uint16_t row = 0U; row < ROWS; ++row) {
        for (uint16_t col = 0U; col < COLS; ++col) {
            const uint8_t base = col < 4U ? 6U : 20U;
            image[(uint32_t)row * COLS + col] =
                (uint8_t)(base + ((row + col) % 3U) - 1U);
        }
    }
    image[3U * COLS + 2U] = 220U;
    image[3U * COLS + 6U] = 0U;
    memcpy(original, image, sizeof(image));

    CHECK(image_near_color_denoise3x3(image, ROWS, COLS, 3U,
                                      scratch, sizeof(scratch)) == 1U);
    CHECK(image[3U * COLS + 2U] >= 5U && image[3U * COLS + 2U] <= 7U);
    CHECK(image[3U * COLS + 6U] >= 19U && image[3U * COLS + 6U] <= 21U);
    for (uint16_t row = 1U; row + 1U < ROWS; ++row) {
        CHECK(image[(uint32_t)row * COLS + 3U] < 12U);
        CHECK(image[(uint32_t)row * COLS + 4U] > 14U);
    }
    for (uint16_t col = 0U; col < COLS; ++col) {
        CHECK(image[col] == original[col]);
        CHECK(image[(ROWS - 1U) * COLS + col] ==
              original[(ROWS - 1U) * COLS + col]);
    }
    for (uint16_t row = 0U; row < ROWS; ++row) {
        CHECK(image[(uint32_t)row * COLS] == original[(uint32_t)row * COLS]);
        CHECK(image[(uint32_t)row * COLS + COLS - 1U] ==
              original[(uint32_t)row * COLS + COLS - 1U]);
    }
    CHECK(image_near_color_denoise3x3(NULL, ROWS, COLS, 3U,
                                      scratch, sizeof(scratch)) == 0U);
    CHECK(image_near_color_denoise3x3(image, 2U, COLS, 3U,
                                      scratch, sizeof(scratch)) == 0U);
    CHECK(image_near_color_denoise3x3(image, ROWS, COLS, 3U,
                                      scratch, sizeof(scratch) - 1U) == 0U);
    CHECK(image_near_color_denoise3x3(image, ROWS, COLS, 3U,
                                      image, sizeof(image)) == 0U);
    puts("PASS 3x3 near-color denoise removes pixel impulses and preserves edges");
}

static void test_binary_despeckle(void)
{
    uint8_t image[49] = {0};
    uint8_t scratch[14];
    image[3U * 7U + 3U] = 255U;
    CHECK(image_binary_despeckle3x3(image, 7U, 7U, scratch,
                                    sizeof(scratch)) == 1U);
    CHECK(image[3U * 7U + 3U] == 0U);

    memset(image, 255, sizeof(image));
    image[3U * 7U + 3U] = 0U;
    CHECK(image_binary_despeckle3x3(image, 7U, 7U, scratch,
                                    sizeof(scratch)) == 1U);
    CHECK(image[3U * 7U + 3U] == 255U);

    memset(image, 0, sizeof(image));
    for (uint16_t row = 1U; row < 6U; ++row)
        for (uint16_t col = 2U; col < 5U; ++col)
            image[(uint32_t)row * 7U + col] = 255U;
    uint8_t expected[49];
    memcpy(expected, image, sizeof(image));
    CHECK(image_binary_despeckle3x3(image, 7U, 7U, scratch,
                                    sizeof(scratch)) == 1U);
    CHECK(memcmp(image, expected, sizeof(image)) == 0);
    CHECK(image_binary_despeckle3x3(NULL, 7U, 7U, scratch,
                                    sizeof(scratch)) == 0U);
    CHECK(image_binary_despeckle3x3(image, 7U, 7U, scratch, 13U) == 0U);
    CHECK(image_binary_despeckle3x3(image, 7U, 7U, image,
                                    sizeof(image)) == 0U);
    puts("PASS binary isolated-speck and pinhole cleanup without edge motion");
}


int main(void)
{
    test_empty_and_small();
    test_mean_invalid_and_rounding();
    test_low_dynamic_adaptive_threshold();
    test_banded_stretch_has_no_horizontal_seams();
    test_vertical_median_rejects_row_impulses();
    test_near_color_denoise_uses_two_dimensional_neighborhood();
    test_binary_despeckle();
    test_mean_resolution(3, 3);
    test_mean_resolution(4, 5);
    test_mean_resolution(5, 4);
    test_resolution(120, 188);
    test_mean_resolution(120, 188);
    test_resolution(240, 376);
    test_mean_resolution(240, 376);
    test_resolution(480, 752);
    test_mean_resolution(480, 752);
    puts("All image-processing host tests passed.");
    return EXIT_SUCCESS;
}
