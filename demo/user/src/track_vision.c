#include "track_vision.h"

#include <string.h>

static uint16_t track_max(uint16_t a, uint16_t b)
{
    return a > b ? a : b;
}

static uint16_t track_min(uint16_t a, uint16_t b)
{
    return a < b ? a : b;
}

static uint16_t track_distance(uint16_t a, uint16_t b)
{
    return a > b ? (uint16_t)(a - b) : (uint16_t)(b - a);
}

static int16_t track_clamp_error(int64_t value)
{
    if (value > 1000)
        return 1000;
    if (value < -1000)
        return -1000;
    return (int16_t)value;
}

void track_vision_process(const uint8_t *binary, uint16_t rows, uint16_t cols,
                          track_vision_result_t *out)
{
    if (out == NULL)
        return;
    memset(out, 0, sizeof(*out));
    if (binary == NULL || rows < 10U || rows > TRACK_VISION_MAX_ROWS ||
        cols < 16U || cols > TRACK_VISION_MAX_COLS)
        return;

    const uint16_t roi_start = (uint16_t)((uint32_t)rows * TRACK_VISION_ROI_START_PERCENT / 100U);
    const uint16_t roi_end = (uint16_t)((uint32_t)rows * TRACK_VISION_ROI_END_PERCENT / 100U);
    const uint16_t roi_rows = roi_end - roi_start;
    const uint16_t zone_rows = roi_rows / 3U;
    const uint16_t near_start = roi_end - zone_rows;
    const uint16_t far_end = roi_start + zone_rows;
    const uint16_t minimum_width = track_max(6U, cols / 16U);
    const uint16_t maximum_width = (uint16_t)((uint32_t)cols * 4U / 5U);
    // 弯道允许每行中线移动约 2.1%W，并跨过约 4.2%H 的短暂缺口；
    // 宽度、密度、重叠和双边界约束仍负责拒绝灯光与零散白点。
    const uint16_t jump_per_row = track_max(2U, cols / 48U);
    const uint16_t maximum_missing_rows = track_max(2U, rows / 24U);

    uint16_t previous_row = 0;
    uint16_t previous_left = 0;
    uint16_t previous_right = 0;
    uint16_t previous_center = 0;
    uint8_t have_previous = 0;
    uint16_t near_count = 0;
    uint16_t far_count = 0;
    uint32_t near_sum = 0;
    uint32_t far_sum = 0;
    uint32_t weighted_sum = 0;
    uint32_t weight_total = 0;

    for (uint16_t next_row = roi_end; next_row > roi_start; --next_row)
    {
        const uint16_t row = next_row - 1U;
        const uint8_t *pixels = binary + (uint32_t)row * cols;
        const uint16_t row_gap = have_previous ? (uint16_t)(previous_row - row) : 1;
        if (have_previous && row_gap > maximum_missing_rows + 1U)
            break; // 连续缺失过长：停止向远端追踪，禁止跳到另一物体。

        uint16_t best_left = 0;
        uint16_t best_right = 0;
        uint16_t best_center = 0;
        uint16_t best_distance = UINT16_MAX;
        uint16_t second_distance = UINT16_MAX;
        uint16_t candidates = 0;
        uint16_t x = 0;
        while (x < cols)
        {
            while (x < cols && pixels[x] == 0U)
                ++x;
            if (x == cols)
                break;

            const uint16_t start = x;
            uint16_t end = x;
            uint16_t white_pixels = 0;
            uint8_t black_gap = 0;
            while (x < cols)
            {
                if (pixels[x] != 0U)
                {
                    ++white_pixels;
                    end = x;
                    black_gap = 0;
                }
                else if (++black_gap > 2U)
                {
                    ++x;
                    break;
                }
                ++x;
            }

            const uint16_t width = end - start + 1U;
            if (start == 0U || end == cols - 1U ||
                width < minimum_width || width > maximum_width ||
                (uint32_t)white_pixels * 4U < (uint32_t)width * 3U)
                continue;

            const uint16_t center = (start + end) / 2U;
            uint16_t distance = track_distance(center, cols / 2U);
            if (have_previous)
            {
                const uint16_t overlap_left = track_max(start, previous_left);
                const uint16_t overlap_right = track_min(end, previous_right);
                const uint16_t minimum_overlap = track_max(3U,
                    track_min(width, previous_right - previous_left + 1U) / 4U);
                distance = track_distance(center, previous_center);
                if (distance > (uint32_t)jump_per_row * row_gap ||
                    overlap_right < overlap_left ||
                    overlap_right - overlap_left + 1U < minimum_overlap)
                    continue;
            }

            ++candidates;
            if (distance < best_distance)
            {
                second_distance = best_distance;
                best_distance = distance;
                best_left = start;
                best_right = end;
                best_center = center;
            }
            else if (distance < second_distance)
            {
                second_distance = distance;
            }
        }

        // 起始行必须唯一；追踪时近似等距的多个候选视为岔路歧义。
        if (candidates == 0U || (!have_previous && candidates != 1U) ||
            (candidates > 1U && second_distance - best_distance <= jump_per_row))
            continue;

        out->left[row] = best_left;
        out->center[row] = best_center;
        out->right[row] = best_right;
        out->row_valid[row] = 1U;
        ++out->valid_rows;
        previous_row = row;
        previous_left = best_left;
        previous_right = best_right;
        previous_center = best_center;
        have_previous = 1U;

        uint16_t weight = 2U;
        if (row >= near_start)
        {
            ++near_count;
            near_sum += best_center;
            weight = 3U;
        }
        else if (row < far_end)
        {
            ++far_count;
            far_sum += best_center;
            weight = 1U;
        }
        weighted_sum += (uint32_t)best_center * weight;
        weight_total += weight;
    }

    out->confidence = (uint8_t)((uint32_t)out->valid_rows * 100U / roi_rows);
    if ((uint32_t)out->valid_rows * 100U < (uint32_t)roi_rows * 45U ||
        (uint32_t)near_count * 100U < (uint32_t)zone_rows * 45U ||
        (uint32_t)far_count * 100U < (uint32_t)zone_rows * 45U)
        return;

    out->valid = 1U;
    out->error = track_clamp_error(
        ((int64_t)2 * weighted_sum - (int64_t)(cols - 1U) * weight_total) * 1000 /
        ((int64_t)(cols - 1U) * weight_total));
    out->heading = track_clamp_error(
        ((int64_t)far_sum * near_count - (int64_t)near_sum * far_count) * 2000 /
        ((int64_t)near_count * far_count * (cols - 1U)));
}
