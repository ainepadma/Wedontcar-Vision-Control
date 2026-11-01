/*
 *@第十九届东南大学智能车竞赛基础四轮组部分开源学习代码
 */


#include "img_process.h"
#include "string.h"

uint8_t image_mean3x3(uint8_t *image, uint16_t rows, uint16_t cols,
                      uint8_t *scratch, uint32_t scratch_size)
{
    uint32_t required = (uint32_t)cols * 2U;
    uint32_t image_size = (uint32_t)rows * cols;
    uintptr_t image_address = (uintptr_t)image;
    uintptr_t scratch_address = (uintptr_t)scratch;
    uint8_t *previous;
    uint8_t *current;

    if (image == NULL || scratch == NULL || rows < 3U || cols < 3U ||
        scratch_size < required)
        return 0U;
    /* Compare integer address distances, not unrelated C pointers, and avoid
     * endpoint-addition overflow. Only the two actually used rows are checked. */
    if ((scratch_address >= image_address &&
         scratch_address - image_address < image_size) ||
        (image_address > scratch_address &&
         image_address - scratch_address < required))
        return 0U;

    previous = scratch;
    current = scratch + cols;
    memcpy(previous, image, cols);
    memcpy(current, image + cols, cols);
    for (uint16_t row = 1U; row < rows - 1U; ++row)
    {
        uint8_t *destination = image + (uint32_t)row * cols;
        const uint8_t *next = destination + cols;
        for (uint16_t col = 1U; col < cols - 1U; ++col)
        {
            uint32_t sum = (uint32_t)previous[col - 1U] + previous[col] + previous[col + 1U] +
                           current[col - 1U] + current[col] + current[col + 1U] +
                           next[col - 1U] + next[col] + next[col + 1U];
            destination[col] = (uint8_t)(sum / 9U);
        }
        /* Save the next unfiltered row before it becomes the destination;
         * the other scratch row already holds this row's original pixels. */
        uint8_t *recycled = previous;
        previous = current;
        current = recycled;
        if (row + 1U < rows - 1U)
            memcpy(current, next, cols);
    }
    return 1U;
}

static uint8_t image_regions_overlap(const void *a, uint32_t a_size,
                                     const void *b, uint32_t b_size)
{
    uintptr_t x = (uintptr_t)a, y = (uintptr_t)b;
    return (uint8_t)(x >= y ? x - y < b_size : y - x < a_size);
}

/* Sliding box filter; all rows in the window are original, enhanced samples.
 * Clamp at the ROI border rather than importing sky/lights or the car body.
 * Only five byte rows are needed even for the dark-frame 5x5 window.
 */
static void image_ground_mean(uint8_t *image, uint16_t rows, uint16_t cols,
                              uint8_t *scratch, uint16_t window, uint16_t gain_q8)
{
    uint16_t radius = window / 2U, oldest = 0U;
    for (uint16_t i = 0U; i < window; ++i)
    {
        uint16_t source = i > radius ? (uint16_t)(i - radius) : 0U;
        if (source >= rows) source = rows - 1U;
        memcpy(scratch + (uint32_t)i * cols, image + (uint32_t)source * cols, cols);
    }
    for (uint16_t row = 0U; row < rows; ++row)
    {
        uint32_t sum = 0U;
        for (uint16_t i = 0U; i < window; ++i)
            for (uint16_t x = 0U; x < window; ++x)
            {
                uint16_t col = x > radius ? (uint16_t)(x - radius) : 0U;
                if (col >= cols) col = cols - 1U;
                sum += scratch[(uint32_t)i * cols + col];
            }
        for (uint16_t col = 0U; col < cols; ++col)
        {
            uint32_t value = sum * gain_q8 / ((uint32_t)window * window * 256U);
            image[(uint32_t)row * cols + col] = (uint8_t)(value > 255U ? 255U : value);
            if (col + 1U < cols)
            {
                uint16_t leaving = col > radius ? (uint16_t)(col - radius) : 0U;
                uint32_t entering = (uint32_t)col + radius + 1U;
                if (entering >= cols) entering = cols - 1U;
                for (uint16_t i = 0U; i < window; ++i)
                {
                    sum -= scratch[(uint32_t)i * cols + leaving];
                    sum += scratch[(uint32_t)i * cols + entering];
                }
            }
        }
        if (row + 1U < rows)
        {
            uint32_t source = (uint32_t)row + radius + 1U;
            if (source >= rows) source = rows - 1U;
            memcpy(scratch + (uint32_t)oldest * cols,
                   image + (uint32_t)source * cols, cols);
            oldest = (uint16_t)((oldest + 1U) % window);
        }
    }
}

uint8_t image_clean_binary(uint8_t *image, uint16_t rows, uint16_t cols,
                           uint8_t *scratch, uint32_t scratch_size)
{
    uint8_t *previous, *current;
    if (image == NULL || scratch == NULL || rows < 3U || cols < 3U ||
        scratch_size < (uint32_t)cols * 2U ||
        image_regions_overlap(image, (uint32_t)rows * cols, scratch, (uint32_t)cols * 2U))
        return 0U;
    previous = scratch;
    current = scratch + cols;
    memcpy(previous, image, cols);
    for (uint16_t row = 1U; row + 1U < rows; ++row)
    {
        uint8_t *destination = image + (uint32_t)row * cols;
        const uint8_t *next = destination + cols;
        memcpy(current, destination, cols);
        uint16_t col = 1U;
        while (col + 1U < cols)
        {
            uint16_t end = col;
            uint8_t value = current[col], surround = (uint8_t)(255U - value);
            if (current[col - 1U] == value) { ++col; continue; }
            while (end + 1U < cols && current[end + 1U] == value) ++end;
            if (end + 1U < cols && (uint16_t)(end - col) < 2U)
            {
                uint8_t isolated = 1U;
                for (uint16_t x = col - 1U; x <= end + 1U; ++x)
                    if (previous[x] != surround || next[x] != surround) isolated = 0U;
                if (isolated)
                    for (uint16_t x = col; x <= end; ++x) destination[x] = surround;
            }
            col = end + 1U;
        }
        uint8_t *recycled = previous;
        previous = current;
        current = recycled;
    }
    /* 小块反光常形成 2x2 白岛；只有四周一圈全黑时才删除，
     * 因而不会削掉连到赛道的白色边缘或环岛白区。 */
    for (uint16_t row = 1U; row + 2U < rows; ++row)
        for (uint16_t col = 1U; col + 2U < cols; ++col)
        {
            uint32_t top = (uint32_t)row * cols + col;
            if (!image[top] || !image[top + 1U] ||
                !image[top + cols] || !image[top + cols + 1U]) continue;
            uint8_t surrounded = 1U;
            for (uint16_t x = col - 1U; x <= col + 2U; ++x)
                if (image[(uint32_t)(row - 1U) * cols + x] ||
                    image[(uint32_t)(row + 2U) * cols + x]) surrounded = 0U;
            if (image[top - 1U] || image[top + 2U] ||
                image[top + cols - 1U] || image[top + cols + 2U]) surrounded = 0U;
            if (surrounded)
                image[top] = image[top + 1U] =
                    image[top + cols] = image[top + cols + 1U] = 0U;
        }
    return 1U;
}

uint32_t image_fill_small_black_holes(uint8_t *image, uint16_t rows, uint16_t cols)
{
    uint16_t qx[IMAGE_TRACK_HOLE_MAX_AREA], qy[IMAGE_TRACK_HOLE_MAX_AREA];
    uint32_t filled = 0U;
    if (image == NULL || rows < 3U || cols < 3U) return 0U;
    for (uint16_t y = 1U; y + 1U < rows; ++y)
    {
        uint16_t x = 1U;
        while (x + 1U < cols)
        {
            if (image[(uint32_t)y * cols + x] != 0U) { ++x; continue; }
            uint16_t end = x;
            while (end + 1U < cols && image[(uint32_t)y * cols + end + 1U] == 0U) ++end;
            uint8_t possible = (uint8_t)(image[(uint32_t)y * cols + x - 1U] != 0U &&
                end + 1U < cols && (uint16_t)(end - x) < IMAGE_TRACK_HOLE_MAX_SIDE);
            /* Only the topmost row of a component may start a search. This
             * avoids rescanning the large background and diagonal snakes.
             */
            if (possible)
                for (uint16_t c = x - 1U; c <= end + 1U; ++c)
                    if (image[(uint32_t)(y - 1U) * cols + c] == 0U) possible = 0U;
            if (possible)
            {
                uint16_t count = 1U, left = x, right = x, top = y, bottom = y;
                qx[0] = x; qy[0] = y;
                for (uint16_t head = 0U; head < count && possible; ++head)
                    for (int dy = -1; dy <= 1 && possible; ++dy)
                        for (int dx = -1; dx <= 1 && possible; ++dx)
                        {
                            if (dx == 0 && dy == 0) continue;
                            int nx = (int)qx[head] + dx, ny = (int)qy[head] + dy;
                            if (nx < 0 || ny < 0 || nx >= cols || ny >= rows) { possible = 0U; break; }
                            if (image[(uint32_t)ny * cols + (uint32_t)nx] != 0U) continue;
                            if (nx == 0 || ny == 0 || nx == cols - 1 || ny == rows - 1) { possible = 0U; break; }
                            uint16_t seen = 0U;
                            while (seen < count && (qx[seen] != nx || qy[seen] != ny)) ++seen;
                            if (seen != count) continue;
                            if (count == IMAGE_TRACK_HOLE_MAX_AREA) { possible = 0U; break; }
                            if (nx < left) left = (uint16_t)nx;
                            if (nx > right) right = (uint16_t)nx;
                            if (ny < top) top = (uint16_t)ny;
                            if (ny > bottom) bottom = (uint16_t)ny;
                            if ((uint16_t)(right - left) >= IMAGE_TRACK_HOLE_MAX_SIDE ||
                                (uint16_t)(bottom - top) >= IMAGE_TRACK_HOLE_MAX_SIDE)
                            { possible = 0U; break; }
                            qx[count] = (uint16_t)nx; qy[count] = (uint16_t)ny; ++count;
                        }
                if (possible)
                    for (uint16_t i = 0U; i < count; ++i)
                    { image[(uint32_t)qy[i] * cols + qx[i]] = 255U; ++filled; }
            }
            x = end + 1U;
        }
    }
    return filled;
}

typedef struct {
    uint32_t low_q8, high_q8, cut_q8, low_count, high_count;
    uint16_t quality;
    uint8_t valid;
} image_grey_model_t;

static image_grey_model_t image_grey_model(const uint32_t *histogram)
{
    image_grey_model_t model = {0};
    uint8_t split = get_threshold_otsu(histogram);
    uint64_t low = 0U, high = 0U, square = 0U;
    for (uint16_t i = 0U; i < 256U; ++i)
    {
        uint64_t value = (uint64_t)i * histogram[i];
        square += value * i;
        if (i <= split) { model.low_count += histogram[i]; low += value; }
        else { model.high_count += histogram[i]; high += value; }
    }
    if (model.low_count == 0U || model.high_count == 0U) return model;
    model.low_q8 = (uint32_t)(low * 256U / model.low_count);
    model.high_q8 = (uint32_t)(high * 256U / model.high_count);
    model.cut_q8 = (model.low_q8 * (100U - IMAGE_TRACK_WHITE_WEIGHT_PERCENT) +
                    model.high_q8 * IMAGE_TRACK_WHITE_WEIGHT_PERCENT) / 100U;
    double count = (double)model.low_count + model.high_count;
    double mean = (double)(low + high) / count;
    double variance = (double)square / count - mean * mean;
    double gap = (double)high / model.high_count - (double)low / model.low_count;
    double between = (double)model.low_count * model.high_count * gap * gap / (count * count);
    if (variance > 0.0)
    {
        double quality = between * 1000.0 / variance;
        model.quality = (uint16_t)(quality > 1000.0 ? 1000.0 : quality);
    }
    model.valid = 1U;
    return model;
}

static uint16_t image_parameter_step(uint16_t previous, uint16_t target, uint32_t dt)
{
    /* Low-pass remains active at 19FPS and at slower processing rates too. */
    return (uint16_t)(((uint64_t)previous * IMAGE_TRACK_PARAMETER_TAU_MS +
        (uint64_t)target * dt + (IMAGE_TRACK_PARAMETER_TAU_MS + dt) / 2U) /
        (IMAGE_TRACK_PARAMETER_TAU_MS + dt));
}

static void image_region_histogram(const uint8_t *image, uint16_t cols,
                                    uint16_t y0, uint16_t y1, uint16_t x0, uint16_t x1,
                                    uint32_t *histogram)
{
    memset(histogram, 0, 256U * sizeof(*histogram));
    for (uint16_t y = y0; y < y1; ++y)
        for (uint16_t x = x0; x < x1; ++x) ++histogram[image[(uint32_t)y * cols + x]];
}

static uint16_t image_tile_cut(const uint16_t *tiles, uint16_t rows, uint16_t cols,
                               uint16_t y, uint16_t x)
{
    int32_t fx = (int32_t)(((uint32_t)x * 2U + 1U) * IMAGE_TRACK_TILE_COLS * 128U / cols) - 128;
    int32_t fy = (int32_t)(((uint32_t)y * 2U + 1U) * IMAGE_TRACK_TILE_ROWS * 128U / rows) - 128;
    if (fx < 0) fx = 0;
    if (fy < 0) fy = 0;
    if (fx > (int32_t)(IMAGE_TRACK_TILE_COLS - 1U) * 256) fx = (IMAGE_TRACK_TILE_COLS - 1U) * 256;
    if (fy > (int32_t)(IMAGE_TRACK_TILE_ROWS - 1U) * 256) fy = (IMAGE_TRACK_TILE_ROWS - 1U) * 256;
    uint16_t x0 = (uint16_t)(fx / 256), y0 = (uint16_t)(fy / 256);
    uint16_t x1 = x0 + 1U < IMAGE_TRACK_TILE_COLS ? x0 + 1U : x0;
    uint16_t y1 = y0 + 1U < IMAGE_TRACK_TILE_ROWS ? y0 + 1U : y0;
    uint32_t wx = (uint32_t)fx & 255U, wy = (uint32_t)fy & 255U;
    uint32_t a = ((uint32_t)tiles[y0 * IMAGE_TRACK_TILE_COLS + x0] * (256U - wx) +
                  (uint32_t)tiles[y0 * IMAGE_TRACK_TILE_COLS + x1] * wx) / 256U;
    uint32_t b = ((uint32_t)tiles[y1 * IMAGE_TRACK_TILE_COLS + x0] * (256U - wx) +
                  (uint32_t)tiles[y1 * IMAGE_TRACK_TILE_COLS + x1] * wx) / 256U;
    return (uint16_t)((a * (256U - wy) + b * wy) / 256U);
}

uint8_t image_track_preprocess_stable(uint8_t *image, uint16_t rows, uint16_t cols,
                                      uint16_t roi_first, uint16_t roi_end,
                                      uint8_t *scratch, uint32_t scratch_size,
                                      uint32_t *histogram, uint8_t *threshold,
                                      image_track_preprocess_state_t *state, uint32_t now_ms)
{
    uint32_t size = (uint32_t)rows * cols, required = (uint32_t)cols * 5U;
    if (image == NULL || scratch == NULL || histogram == NULL || threshold == NULL ||
        rows > 480U || cols > 752U || cols < 5U || roi_end > rows || roi_first >= roi_end ||
        (uint16_t)(roi_end - roi_first) < 5U || scratch_size < required ||
        image_regions_overlap(image, size, scratch, required) ||
        image_regions_overlap(image, size, histogram, 256U * sizeof(*histogram)) ||
        image_regions_overlap(scratch, required, histogram, 256U * sizeof(*histogram)) ||
        image_regions_overlap(image, size, threshold, 1U) ||
        image_regions_overlap(scratch, required, threshold, 1U) ||
        image_regions_overlap(histogram, 256U * sizeof(*histogram), threshold, 1U) ||
        (state != NULL && (image_regions_overlap(image, size, state, sizeof(*state)) ||
         image_regions_overlap(scratch, required, state, sizeof(*state)) ||
         image_regions_overlap(histogram, 256U * sizeof(*histogram), state, sizeof(*state)) ||
         image_regions_overlap(threshold, 1U, state, sizeof(*state))))) return 0U;

    uint8_t *ground = image + (uint32_t)roi_first * cols;
    uint16_t ground_rows = roi_end - roi_first;
    uint32_t count = (uint32_t)ground_rows * cols, cumulative = 0U;
    get_hist_gram(ground, ground_rows, cols, histogram);
    uint16_t p99 = 0U;
    for (; p99 < 255U; ++p99)
    { cumulative += histogram[p99]; if (cumulative >= count - count / 100U) break; }
    uint32_t desired = p99 == 0U ? IMAGE_TRACK_MAX_GAIN * 256U : IMAGE_TRACK_TARGET_LEVEL * 256U / p99;
    if (desired > IMAGE_TRACK_MAX_GAIN * 256U) desired = IMAGE_TRACK_MAX_GAIN * 256U;
    if (desired < 256U) desired = 256U;
    uint32_t dt = state != NULL ? now_ms - state->last_ms : 0U;
    uint8_t reset = (uint8_t)(state == NULL || !state->ready || dt > IMAGE_TRACK_RESET_GAP_MS ||
        state->gain_q8 < 256U || state->gain_q8 > IMAGE_TRACK_MAX_GAIN * 256U ||
        state->rows != rows || state->cols != cols || state->roi_first != roi_first || state->roi_end != roi_end);
    uint16_t gain = reset ? (uint16_t)desired : image_parameter_step(state->gain_q8, (uint16_t)desired, dt);
    /* On a sudden brighter frame, prevent a slowly falling gain from clipping
     * both ground classes to white. This safety cap may react immediately.
     */
    uint32_t gain_ceiling = p99 == 0U ? IMAGE_TRACK_MAX_GAIN * 256U : 240U * 256U / p99;
    if (gain_ceiling < 256U) gain_ceiling = 256U;
    if (gain > gain_ceiling) gain = (uint16_t)gain_ceiling;
    /* Always the same 5x5 kernel. Scale the sum before division, never clip
     * each noisy dark input pixel before averaging. All three ROIs stay isolated.
     */
    if (roi_first != 0U) image_ground_mean(image, roi_first, cols, scratch, 5U, gain);
    image_ground_mean(ground, ground_rows, cols, scratch, 5U, gain);
    if (roi_end < rows) image_ground_mean(image + (uint32_t)roi_end * cols,
        (uint16_t)(rows - roi_end), cols, scratch, 5U, gain);

    get_hist_gram(ground, ground_rows, cols, histogram);
    image_grey_model_t original = image_grey_model(histogram), model = original;
    /* Reject only a small bright tail. Never discard the dark island class.
     * If a rare white branch disappears or quality gets worse, keep the full model.
     */
    uint32_t remaining = count / 100U;
    for (uint16_t i = 256U; i > 0U && remaining != 0U; --i)
    {
        uint32_t removed = histogram[i - 1U] < remaining ? histogram[i - 1U] : remaining;
        histogram[i - 1U] -= removed; remaining -= removed;
    }
    image_grey_model_t trimmed = image_grey_model(histogram);
    if (trimmed.valid && (!original.valid || trimmed.quality >= original.quality)) model = trimmed;
    uint32_t gap = model.high_q8 - model.low_q8;
    uint8_t credible = (uint8_t)(model.valid && model.quality >= IMAGE_TRACK_GLOBAL_QUALITY_MIN &&
        gap * 100U >= model.high_q8 * 12U && gap * 256U / gain >= 64U);
    uint8_t brief_weak = (uint8_t)(!credible && !reset && model.valid &&
        now_ms - state->last_good_ms <= IMAGE_TRACK_WEAK_HOLD_MS && model.quality >= IMAGE_TRACK_WEAK_QUALITY_MIN &&
        gap * 100U >= model.high_q8 * 8U && gap * 256U / gain >= 64U);
    if (!credible && !brief_weak)
    {
        /* Keep a current-frame binary preview even when contrast is unreliable.
         * 255 remains the invalid-observation sentinel for control consumers. */
        uint32_t preview_cut = !reset ?
            (uint32_t)state->threshold_raw_q8 * gain / 65536U :
            (model.valid ? model.cut_q8 / 256U : 128U);
        if (preview_cut > 254U) preview_cut = 254U;
        for (uint32_t i = 0U; i < size; ++i)
            image[i] = image[i] > preview_cut ? 255U : 0U;
        *threshold = 255U;
        if (state != NULL) memset(state, 0, sizeof(*state));
        return 1U;
    }
    /* A real lighting step can put the old threshold outside both current
     * classes. Adopt fresh thresholds immediately instead of blacking a newly
     * dark road for several frames while the old bright threshold decays.
     */
    if (!reset && credible)
    {
        uint32_t old_cut = (uint32_t)state->threshold_raw_q8 * gain / 256U;
        if (old_cut <= model.low_q8 || old_cut >= model.high_q8) reset = 1U;
    }
    uint16_t raw_cut = (uint16_t)(model.cut_q8 * 256U / gain);
    uint16_t global_cut = reset ? raw_cut : brief_weak ? state->threshold_raw_q8 :
        image_parameter_step(state->threshold_raw_q8, raw_cut, dt);
    uint16_t tiles[IMAGE_TRACK_TILE_COUNT];
    for (uint16_t ty = 0U; ty < IMAGE_TRACK_TILE_ROWS; ++ty)
        for (uint16_t tx = 0U; tx < IMAGE_TRACK_TILE_COLS; ++tx)
        {
            uint16_t y0 = (uint16_t)((uint32_t)ground_rows * ty / IMAGE_TRACK_TILE_ROWS);
            uint16_t y1 = (uint16_t)((uint32_t)ground_rows * (ty + 1U) / IMAGE_TRACK_TILE_ROWS);
            uint16_t x0 = (uint16_t)((uint32_t)cols * tx / IMAGE_TRACK_TILE_COLS);
            uint16_t x1 = (uint16_t)((uint32_t)cols * (tx + 1U) / IMAGE_TRACK_TILE_COLS);
            image_region_histogram(ground, cols, y0, y1, x0, x1, histogram);
            image_grey_model_t local = image_grey_model(histogram);
            uint32_t tile_count = (uint32_t)(y1 - y0) * (x1 - x0);
            uint32_t candidate = model.cut_q8;
            /* A uniform white/blue tile must not invent its own black/white
             * split. Require both classes and support from the global contrast.
             */
            if (local.valid && local.quality >= IMAGE_TRACK_LOCAL_QUALITY_MIN &&
                local.low_count * 20U >= tile_count && local.high_count * 20U >= tile_count &&
                local.low_q8 <= model.low_q8 + gap / 3U &&
                local.high_q8 >= model.low_q8 + gap * 3U / 5U &&
                local.high_q8 - local.low_q8 >= gap / 2U)
            {
                candidate = local.cut_q8;
                uint32_t minimum = model.low_q8 + gap / 5U, maximum = model.low_q8 + gap * 7U / 10U;
                if (candidate < minimum) candidate = minimum;
                if (candidate > maximum) candidate = maximum;
            }
            candidate = (candidate * IMAGE_TRACK_LOCAL_STRENGTH_PERCENT +
                model.cut_q8 * (100U - IMAGE_TRACK_LOCAL_STRENGTH_PERCENT)) / 100U;
            uint16_t index = ty * IMAGE_TRACK_TILE_COLS + tx;
            uint16_t target = (uint16_t)(candidate * 256U / gain);
            tiles[index] = reset ? target : brief_weak ? state->tile_raw_q8[index] :
                image_parameter_step(state->tile_raw_q8[index], target, dt);
        }
    uint32_t enhanced_cut = (uint32_t)global_cut * gain / 65536U;
    *threshold = (uint8_t)(enhanced_cut > 255U ? 255U : enhanced_cut);
    for (uint16_t y = 0U; y < rows; ++y)
        for (uint16_t x = 0U; x < cols; ++x)
        {
            uint16_t cut = y >= roi_first && y < roi_end ?
                image_tile_cut(tiles, ground_rows, cols, (uint16_t)(y - roi_first), x) : global_cut;
            uint32_t current = (uint32_t)image[(uint32_t)y * cols + x] * 65536U;
            image[(uint32_t)y * cols + x] = current > (uint32_t)cut * gain ? 255U : 0U;
        }
    const uint16_t limits[4] = {0U, roi_first, roi_end, rows};
    for (uint16_t i = 0U; i < 3U; ++i)
        if ((uint16_t)(limits[i + 1U] - limits[i]) >= 3U)
        {
            uint8_t *region = image + (uint32_t)limits[i] * cols;
            uint16_t height = limits[i + 1U] - limits[i];
            (void)image_clean_binary(region, height, cols, scratch, scratch_size);
            (void)image_fill_small_black_holes(region, height, cols);
        }
    if (state != NULL)
    {
        state->rows = rows; state->cols = cols; state->roi_first = roi_first; state->roi_end = roi_end;
        state->gain_q8 = gain; state->threshold_raw_q8 = global_cut;
        memcpy(state->tile_raw_q8, tiles, sizeof(tiles)); state->last_ms = now_ms; state->ready = 1U;
        if (credible) state->last_good_ms = now_ms;
    }
    return 1U;
}

uint8_t image_track_preprocess(uint8_t *image, uint16_t rows, uint16_t cols,
                               uint16_t roi_first, uint16_t roi_end,
                               uint8_t *scratch, uint32_t scratch_size,
                               uint32_t *histogram, uint8_t *threshold)
{
    return image_track_preprocess_stable(image, rows, cols, roi_first, roi_end,
                                         scratch, scratch_size, histogram, threshold, NULL, 0U);
}

/*
 *@func: 二值化函数
 */
void binaryzation_process(uint8_t* _img,
                          uint16_t _rows,
                          uint16_t _cols,
                          uint32_t _threshold_value)
{
	uint32_t _size = (uint32_t)_rows * _cols;
	for (uint32_t _i = 0; _i < _size; ++_i)
	{
		if (*(_img + _i) > _threshold_value)
			*(_img + _i) = 255;
		else
			*(_img + _i) = 0;
	}
}

/*
 *@func: 获取直方图
 */
void get_hist_gram(const uint8_t* _img,
                   uint16_t _rows,
                   uint16_t _cols,
                   uint32_t* _hist_gram)
{
	memset(_hist_gram, 0, 256 * sizeof(*_hist_gram));
    uint32_t _size = (uint32_t)_rows * _cols;
    for (uint32_t _i = 0; _i < _size; ++_i)
    {
        _hist_gram[_img[_i]]++;
    }
}

/*
 *@func: 大津法求阈值
 */
uint8_t get_threshold_otsu(const uint32_t* _hist_gram)
{
    int Y;
    uint32_t amount = 0, pixel_back = 0, pixel_fore = 0;
    uint64_t pixel_integral_back = 0, pixel_integral_fore = 0, pixel_integral = 0;
    double OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // 类间方差;
    int pixel_min_value, pixel_max_value;
    int threshold = 0;

    for (pixel_min_value = 0; pixel_min_value < 256 && _hist_gram[pixel_min_value] == 0; pixel_min_value++)
        ;
    if (pixel_min_value == 256)
        return 0; // 空直方图，不进行除法或访问越界灰度项
    for (pixel_max_value = 255; pixel_max_value > pixel_min_value && _hist_gram[pixel_max_value] == 0; pixel_max_value--)
        ;
    if (pixel_max_value == pixel_min_value)
        return (uint8_t)pixel_max_value; // 图像中只有一个颜色
    if (pixel_min_value + 1 == pixel_max_value)
        return (uint8_t)pixel_min_value; // 图像中只有二个颜色

    for (Y = pixel_min_value; Y <= pixel_max_value; Y++)
        amount += _hist_gram[Y]; //  像素总数

    pixel_integral = 0;
    for (Y = pixel_min_value; Y <= pixel_max_value; Y++)
        pixel_integral += (uint64_t)_hist_gram[Y] * (uint32_t)Y;
    SigmaB = -1;
    for (Y = pixel_min_value; Y < pixel_max_value; Y++)
    {
        pixel_back = pixel_back + _hist_gram[Y];
        pixel_fore = amount - pixel_back;
        pixel_integral_back += (uint64_t)_hist_gram[Y] * (uint32_t)Y;
        if (pixel_back == 0 || pixel_fore == 0)
            continue;
        OmegaBack = (double)pixel_back / amount;
        OmegaFore = (double)pixel_fore / amount;
        pixel_integral_fore = pixel_integral - pixel_integral_back;
        MicroBack = (double)pixel_integral_back / pixel_back;
        MicroFore = (double)pixel_integral_fore / pixel_fore;
        Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);
        if (Sigma > SigmaB)
        {
            SigmaB = Sigma;
            threshold = Y;
        }
    }
    return (uint8_t)threshold;
}

void auxiliary_process(const uint8_t* _src_pixel_mat,
                       uint16_t _src_rows, uint16_t _src_cols,
                       uint8_t _threshold_val,
                       uint16_t* _left_line, uint16_t* _mid_line, uint16_t* _right_line)
{
    if (_src_rows == 0 || _src_cols == 0)
        return;
    uint16_t mid_point = _src_cols >> 1;
    uint32_t bottom_row = (uint32_t)(_src_rows - 1U) * _src_cols;
    if (_src_pixel_mat[bottom_row + mid_point] < _threshold_val)
    {
        // 重新寻找扫线中点
        uint16_t _tmp_left_mid = 0;
        uint16_t _tmp_right_mid = _src_cols;
        for (uint16_t j = (_src_cols >> 1); (uint32_t)j + 2U < _src_cols; ++j)
        {
            if (_src_pixel_mat[bottom_row + j] > _threshold_val &&
                _src_pixel_mat[bottom_row + j + 1U] > _threshold_val &&
                _src_pixel_mat[bottom_row + j + 2U] > _threshold_val)
            {
                _tmp_right_mid = j + 2;
            }
        }

        for (uint16_t j = (_src_cols >> 1); j > 2U; --j)
        {
            if (_src_pixel_mat[bottom_row + j] > _threshold_val &&
                _src_pixel_mat[bottom_row + j - 1U] > _threshold_val &&
                _src_pixel_mat[bottom_row + j - 2U] > _threshold_val)
            {
                _tmp_left_mid = j - 2;
            }
        }
        if (_tmp_right_mid - (_src_cols >> 1) < (_src_cols >> 1) - _tmp_left_mid)
        {
            mid_point = _tmp_right_mid;
        }
        else
        {
            mid_point = _tmp_left_mid;
        }
    }
    // 未找到白色起点时仍使用合法列，避免将列数哨兵当作像素坐标。
    if (mid_point >= _src_cols)
        mid_point = _src_cols >> 1;
    for (uint16_t row = _src_rows; row > 0; --row)
    {
        uint16_t i = row - 1U;
        uint32_t row_offset = (uint32_t)i * _src_cols;
        uint16_t cur_point = mid_point;
        _left_line[i] = 0;
        _right_line[i] = _src_cols - 1U;
       // 扫描左线
        while (cur_point > 2U)
        {
            if (_src_pixel_mat[row_offset + cur_point] < _threshold_val &&
                _src_pixel_mat[row_offset + cur_point - 1U] < _threshold_val &&
                _src_pixel_mat[row_offset + cur_point - 2U] < _threshold_val)
            {
                _left_line[i] = cur_point;
                break;
            }
            --cur_point;
        }
        // 扫描右线
        cur_point = mid_point;
        while ((uint32_t)cur_point + 2U < _src_cols)
        {
            if (_src_pixel_mat[row_offset + cur_point] < _threshold_val &&
                _src_pixel_mat[row_offset + cur_point + 1U] < _threshold_val &&
                _src_pixel_mat[row_offset + cur_point + 2U] < _threshold_val)
            {
                _right_line[i] = cur_point;
                break;
            }
            ++cur_point;
        }

        _mid_line[i] = (uint16_t)(((uint32_t)_right_line[i] + _left_line[i]) >> 1);
        if (_left_line[i] != 0 || 
			_right_line[i] != _src_cols - 1)
        {
            mid_point = _mid_line[i];
        }
        else
        {
            mid_point = _src_cols >> 1;
        }
    }
}

