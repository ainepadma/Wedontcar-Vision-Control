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
    {
        out->reject_flags = TRACK_VISION_REJECT_INPUT;
        return;
    }

    const uint16_t roi_start = (uint16_t)((uint32_t)rows * TRACK_VISION_ROI_START_PERCENT / 100U);
    const uint16_t roi_end = (uint16_t)((uint32_t)rows * TRACK_VISION_ROI_END_PERCENT / 100U);
    const uint16_t roi_rows = roi_end - roi_start;
    const uint16_t zone_rows = track_max(1U, roi_rows / 3U);
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
    uint32_t near_width_sum = 0;
    uint32_t far_width_sum = 0;
    uint32_t weighted_sum = 0;
    uint32_t weight_total = 0;
    out->roi_rows = roi_rows;

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
        uint16_t candidates = 0;
        uint16_t connected_shapes = 0;
        uint16_t shape_candidates = 0;
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

            ++shape_candidates;

            const uint16_t center = (start + end) / 2U;
            uint16_t distance = track_distance(center, cols / 2U);
            if (have_previous)
            {
                const uint16_t overlap_left = track_max(start, previous_left);
                const uint16_t overlap_right = track_min(end, previous_right);
                const uint16_t minimum_overlap = track_max(3U,
                    track_min(width, previous_right - previous_left + 1U) / 4U);
                distance = track_distance(center, previous_center);
                if (overlap_right < overlap_left ||
                    overlap_right - overlap_left + 1U < minimum_overlap)
                {
                    continue;
                }
                ++connected_shapes;
                if (distance > (uint32_t)jump_per_row * row_gap)
                    continue;
            }

            ++candidates;
            if (distance < best_distance)
            {
                best_distance = distance;
                best_left = start;
                best_right = end;
                best_center = center;
            }
        }

        // 两个白色区域只要都与上一行赛道充分重叠，就已经构成分叉；
        // 即使两支中心跳变超过普通弯道上限，也禁止误选其中一支继续追踪。
        if (have_previous && connected_shapes > 1U)
        {
            ++out->ambiguous_rows;
            continue;
        }

        // 起始行必须唯一；通过普通弯道连续性检查后仍有多个候选也视为分叉。
        if (candidates == 0U)
        {
            ++out->no_candidate_rows;
            if (have_previous && shape_candidates != 0U)
                ++out->disconnected_rows;
            continue;
        }
        if ((!have_previous && candidates != 1U) || candidates > 1U)
        {
            ++out->ambiguous_rows;
            continue;
        }

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
            near_width_sum += best_right - best_left + 1U;
            weight = 3U;
        }
        else if (row < far_end)
        {
            ++far_count;
            far_sum += best_center;
            far_width_sum += best_right - best_left + 1U;
            weight = 1U;
        }
        weighted_sum += (uint32_t)best_center * weight;
        weight_total += weight;
    }

    out->near_valid_rows = near_count;
    out->far_valid_rows = far_count;
    if (near_count != 0U)
        out->near_width = (uint16_t)(near_width_sum / near_count);
    if (far_count != 0U)
        out->far_width = (uint16_t)(far_width_sum / far_count);
    if (out->near_width != 0U)
        out->width_growth_percent = (uint16_t)((uint32_t)out->far_width * 100U /
                                                out->near_width);

    /* A ring entrance opens into a much wider far-field white region. Keep
     * this deliberately conservative: it is only a candidate and is later
     * confirmed by distinct frames plus signed yaw from the IMU. */
    if ((uint32_t)near_count * 100U >= (uint32_t)zone_rows * 35U &&
        (uint32_t)far_count * 100U >= (uint32_t)zone_rows * 35U &&
        out->width_growth_percent >= 135U &&
        (uint32_t)out->far_width * 100U >= (uint32_t)cols * 40U)
    {
        const int32_t near_center = (int32_t)(near_sum / near_count);
        const int32_t far_center = (int32_t)(far_sum / far_count);
        const int32_t center_delta = far_center - near_center;
        const uint16_t branch_rows = out->ambiguous_rows + out->disconnected_rows;
        const uint8_t strong_shape =
            (uint8_t)(track_distance((uint16_t)near_center, (uint16_t)far_center) >= cols / 20U ||
                      branch_rows >= track_max(2U, zone_rows / 3U) ||
                      (uint32_t)out->far_width * 100U >= (uint32_t)cols * 55U);
        if (strong_shape)
        {
            out->ring_candidate = 1U;
            if (center_delta <= -(int32_t)(cols / 16U))
                out->ring_direction = 1;
            else if (center_delta >= (int32_t)(cols / 16U))
                out->ring_direction = -1;
        }
    }
    out->confidence = (uint8_t)((uint32_t)out->valid_rows * 100U / roi_rows);
    if (!have_previous)
        out->reject_flags |= TRACK_VISION_REJECT_NO_SEED;
    if ((uint32_t)out->valid_rows * 100U < (uint32_t)roi_rows * 45U)
        out->reject_flags |= TRACK_VISION_REJECT_TOTAL_COVERAGE;
    if ((uint32_t)near_count * 100U < (uint32_t)zone_rows * 45U)
        out->reject_flags |= TRACK_VISION_REJECT_NEAR_COVERAGE;
    if ((uint32_t)far_count * 100U < (uint32_t)zone_rows * 45U)
        out->reject_flags |= TRACK_VISION_REJECT_FAR_COVERAGE;
    if (out->reject_flags != TRACK_VISION_REJECT_NONE)
        return;

    out->valid = 1U;
    out->error = track_clamp_error(
        ((int64_t)2 * weighted_sum - (int64_t)(cols - 1U) * weight_total) * 1000 /
        ((int64_t)(cols - 1U) * weight_total));
    out->heading = track_clamp_error(
        ((int64_t)far_sum * near_count - (int64_t)near_sum * far_count) * 2000 /
        ((int64_t)near_count * far_count * (cols - 1U)));
}
