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
    const uint16_t anchor_start = roi_end - track_max(3U, roi_rows / 6U);
    const uint16_t minimum_width = track_max(6U, cols / 16U);
    const uint16_t maximum_width = (uint16_t)((uint32_t)cols * 4U / 5U);
    // 弯道允许每行中线移动约 2.1%W，并跨过约 4.2%H 的短暂缺口；
    // 宽度、密度、重叠和双边界约束仍负责拒绝灯光与零散白点。
    const uint16_t jump_per_row = track_max(2U, cols / 48U);
    const uint16_t maximum_missing_rows = track_max(2U, rows / 24U);

    uint16_t previous_row = 0;
    uint16_t seed_row = 0;
    uint16_t previous_left = 0;
    uint16_t previous_right = 0;
    uint16_t previous_center = 0;
    uint8_t have_previous = 0;
    uint16_t near_count = 0;
    uint16_t mid_count = 0;
    uint16_t far_count = 0;
    uint32_t near_sum = 0;
    uint32_t mid_sum = 0;
    uint32_t far_sum = 0;
    uint32_t near_width_sum = 0;
    uint32_t far_width_sum = 0;
    uint16_t broad_rows = 0;
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
            /* 十字横向白区会跨过图像两侧；先记录，再按普通双边界规则
             * 拒绝该行，避免把宽白区本身误当作赛道中线。 */
            if (start <= cols / 8U && end >= cols - 1U - cols / 8U &&
                (uint32_t)width * 5U >= (uint32_t)cols * 4U &&
                (uint32_t)white_pixels * 4U >= (uint32_t)width * 3U)
                ++broad_rows;
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
        if (!have_previous)
            seed_row = row;
        have_previous = 1U;

        if (row >= near_start)
        {
            ++near_count;
            near_sum += best_center;
            near_width_sum += best_right - best_left + 1U;
        }
        else if (row < far_end)
        {
            ++far_count;
            far_sum += best_center;
            far_width_sum += best_right - best_left + 1U;
        }
        else
        {
            ++mid_count;
            mid_sum += best_center;
        }
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
        /* Symmetric widening alone is a cross, not a directional ring entry.
         * Require a visible left/right displacement before arming the ring. */
        if (track_distance((uint16_t)near_center, (uint16_t)far_center) >=
            track_max(2U, cols / 16U))
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
    else if (seed_row < anchor_start)
        out->reject_flags |= TRACK_VISION_REJECT_NEAR_ANCHOR;
    if ((uint32_t)out->valid_rows * 100U < (uint32_t)roi_rows * 45U)
        out->reject_flags |= TRACK_VISION_REJECT_TOTAL_COVERAGE;
    if ((uint32_t)near_count * 100U < (uint32_t)zone_rows * 45U)
        out->reject_flags |= TRACK_VISION_REJECT_NEAR_COVERAGE;
    if ((uint32_t)far_count * 100U < (uint32_t)zone_rows * 45U)
        out->reject_flags |= TRACK_VISION_REJECT_FAR_COVERAGE;
    /* 仅在连续宽白区且近端单赛道仍完整时允许十字入口回中。
     * 普通丢线、单行白光、全白图以及无近端种子仍保持无效。 */
    if (broad_rows >= track_max(3U, roi_rows / 12U) &&
        (uint32_t)near_count * 100U >= (uint32_t)zone_rows * 70U &&
        (out->reject_flags & TRACK_VISION_REJECT_NEAR_ANCHOR) == 0U &&
        (out->reject_flags & TRACK_VISION_REJECT_FAR_COVERAGE) != 0U)
    {
        out->cross_candidate = 1U;
        out->ring_candidate = 0U;
        out->ring_direction = 0;
        out->reject_flags = TRACK_VISION_REJECT_NONE;
        out->valid = 1U;
        out->confidence = (uint8_t)((uint32_t)near_count * 100U / zone_rows);
        out->error = track_clamp_error(
            ((int64_t)2 * near_sum - (int64_t)(cols - 1U) * near_count) * 1000 /
            ((int64_t)(cols - 1U) * near_count));
        out->heading = 0;
        return;
    }
    if (out->reject_flags != TRACK_VISION_REJECT_NONE)
        return;

    out->valid = 1U;
    /* 横向偏差只取车前最近的有效区域；整段平均可能在出弯时抵消
     * 车身旁的明显偏移，使车沿边线平行行驶而不主动回中。 */
    out->error = track_clamp_error(
        ((int64_t)2 * near_sum - (int64_t)(cols - 1U) * near_count) * 1000 /
        ((int64_t)(cols - 1U) * near_count));
    /* 远端可能是环岛另一段白轨。远端偏移不得突然超过近—中段
     * 已验证的走势；真正的急弯仍可由连续近端帧逐步跟踪。 */
    const int32_t near_center = (int32_t)(near_sum / near_count);
    const int32_t mid_center = mid_count != 0U ?
        (int32_t)(mid_sum / mid_count) : near_center;
    const int32_t far_center = (int32_t)(far_sum / far_count);
    const int32_t local_step = mid_center - near_center;
    const int32_t far_limit = 2 * (local_step < 0 ? -local_step : local_step) +
                              (int32_t)track_max(2U, cols / 16U);
    int32_t far_step = far_center - mid_center;
    if (far_step > far_limit)
        far_step = far_limit;
    else if (far_step < -far_limit)
        far_step = -far_limit;
    out->heading = track_clamp_error(
        (int64_t)(mid_center + far_step - near_center) * 2000 / (cols - 1U));
}

void track_vision_lock_init(track_vision_lock_t *lock)
{
    if (lock != NULL)
        memset(lock, 0, sizeof(*lock));
}

static void track_vision_reject_jump(track_vision_result_t *result)
{
    result->valid = 0U;
    result->reject_flags |= TRACK_VISION_REJECT_TRACK_JUMP;
    result->error = 0;
    result->heading = 0;
    result->ring_candidate = 0U;
    result->cross_candidate = 0U;
    result->ring_direction = 0;
}

void track_vision_lock_update(track_vision_lock_t *lock,
                              track_vision_result_t *result)
{
    if (lock == NULL || result == NULL)
        return;
    if (!result->valid)
    {
        if (lock->missing_frames < UINT8_MAX)
            ++lock->missing_frames;
        lock->candidate_frames = 0U;
        if (lock->missing_frames >= 5U)
        {
            lock->phase = TRACK_VISION_SEARCHING;
            lock->has_lock = 0U;
        }
        return;
    }
    lock->missing_frames = 0U;
    const int32_t error = result->error;
    const int32_t locked_delta = error - lock->locked_error;
    const int32_t locked_distance = locked_delta < 0 ? -locked_delta : locked_delta;
    if (lock->phase == TRACK_VISION_LOCKED && locked_distance <= 400)
    {
        lock->locked_error = result->error;
        return;
    }
    if (lock->phase == TRACK_VISION_SUSPECT && lock->has_lock &&
        locked_distance <= 400)
    {
        lock->phase = TRACK_VISION_LOCKED;
        lock->candidate_frames = 0U;
        lock->locked_error = result->error;
        return;
    }
    if (lock->phase != TRACK_VISION_SUSPECT)
    {
        lock->phase = TRACK_VISION_SUSPECT;
        lock->candidate_error = result->error;
        lock->candidate_frames = 1U;
    }
    else
    {
        const int32_t candidate_delta = error - lock->candidate_error;
        if (candidate_delta < -150 || candidate_delta > 150)
            lock->candidate_frames = 1U;
        else if (lock->candidate_frames < 3U)
            ++lock->candidate_frames;
        lock->candidate_error = result->error;
    }
    if (lock->candidate_frames >= 3U)
    {
        lock->phase = TRACK_VISION_LOCKED;
        lock->locked_error = result->error;
        lock->has_lock = 1U;
        lock->candidate_frames = 0U;
        return;
    }
    track_vision_reject_jump(result);
}
