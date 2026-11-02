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

/* When a sharp turn leaves only one near road boundary in frame, that edge
 * still indicates which way the car must steer. Require a long, smooth run
 * attached to the image edge in the lower third; opposite-side white bands
 * make the view ambiguous and suppress this fallback. */
static uint8_t track_near_edge_rescue(const uint8_t *binary, uint16_t cols,
    uint16_t roi_start, uint16_t near_start, uint16_t roi_end,
    track_vision_result_t *out)
{
    uint16_t count[2] = {0U, 0U}, longest[2] = {0U, 0U}, streak[2] = {0U, 0U};
    uint16_t recent[2] = {0U, 0U}, extended_recent[2] = {0U, 0U};
    uint16_t previous[2] = {0U, 0U};
    uint32_t edge_sum[2] = {0U, 0U};
    uint8_t have_previous[2] = {0U, 0U};
    const uint16_t zone_rows = roi_end - near_start;
    const uint16_t min_width = track_max(6U, cols / 6U);
    const uint16_t max_width = (uint16_t)((uint32_t)cols * 3U / 4U);
    const uint16_t max_jump = track_max(3U, cols / 16U);
    for (uint16_t y = near_start; y < roi_end; ++y) {
        const uint8_t *pixels = binary + (uint32_t)y * cols;
        uint16_t width[2] = {0U, 0U};
        while (width[0] < cols && pixels[width[0]]) ++width[0];
        while (width[1] < cols && pixels[cols - 1U - width[1]]) ++width[1];
        for (uint8_t side = 0U; side < 2U; ++side) {
            uint16_t w = width[side];
            if (w < min_width || w > max_width) {
                streak[side] = 0U;
                have_previous[side] = 0U;
                continue;
            }
            uint16_t edge = side == 0U ? w - 1U : cols - w;
            ++count[side];
            edge_sum[side] += edge;
            if (roi_end - y <= 8U) ++recent[side];
            if (roi_end - y <= 16U) ++extended_recent[side];
            if (have_previous[side] && track_distance(edge, previous[side]) <= max_jump)
                ++streak[side];
            else
                streak[side] = 1U;
            if (streak[side] > longest[side]) longest[side] = streak[side];
            previous[side] = edge;
            have_previous[side] = 1U;
        }
    }
    for (uint8_t side = 0U; side < 2U; ++side) {
        uint8_t other = (uint8_t)(1U - side);
        uint16_t far_same_edge = 0U;
        for (uint16_t y = roi_start; y < roi_start + zone_rows; ++y) {
            const uint8_t *pixels = binary + (uint32_t)y * cols;
            uint16_t w = 0U;
            while (w < cols && pixels[side == 0U ? w : cols - 1U - w]) ++w;
            if (w >= min_width && w <= max_width) ++far_same_edge;
        }
        /* 弯道内只剩外沿一条边时，这条边会从底部一直延伸到远端，
         * 原来的 20% 门槛把它全部拒掉，导致弯道内整帧判无效、舵机冻住后直线冲出。
         * 现在只在“该边铺满整个远端”时才拒绝；单边、连续、触及底部仍由其余四项保证。 */
        /* A few damaged bottom scanlines must not erase a long one-sided bend.
         * Require the same edge in part of the far third before relaxing the
         * bottom contact, so a cropped near-only white patch still rejects. */
        uint8_t bottom_supported = (uint8_t)(recent[side] >= 3U ||
            (extended_recent[side] >= 3U &&
             (uint32_t)far_same_edge * 100U >= (uint32_t)zone_rows * 15U));
        if ((uint32_t)count[side] * 100U < (uint32_t)zone_rows * 75U ||
            (uint32_t)count[other] * 100U > (uint32_t)zone_rows * 8U ||
            (uint32_t)far_same_edge * 100U >= (uint32_t)zone_rows * 100U ||
            (uint32_t)longest[side] * 100U < (uint32_t)zone_rows * 50U ||
            !bottom_supported) continue;
        uint16_t edge = (uint16_t)(edge_sum[side] / count[side]);
        int32_t inward = side == 0U ? (int32_t)(cols - 1U - edge) : (int32_t)edge;
        /* Start with a moderate correction. The old 400+600 slope saturated
         * the servo at the edge and the car cut across the inside of a bend. */
        int32_t magnitude = 180 + inward * 300 / cols;
        if (magnitude > 1000) magnitude = 1000;
        out->edge_rescue = 1U;
        out->valid = 1U;
        out->confidence = (uint8_t)((uint32_t)count[side] * 100U / zone_rows);
        out->reject_flags = TRACK_VISION_REJECT_NONE;
        out->near_geometry_valid = 1U;
        out->near_error = out->error = (int16_t)(side == 0U ? -magnitude : magnitude);
        out->near_heading = out->heading = 0;
        out->bottom_valid = 0U;
        out->ring_candidate = 0U;
        out->ring_direction = 0;
        return 1U;
    }
    return 0U;
}

#include "track_near_route.inc"

/* Test the complete pixel connection, including both cells at diagonal steps.
 * Never bridge a black separator merely because two white runs overlap. */
static uint8_t track_white_connection(const uint8_t *binary, uint16_t cols,
                                      int x0, int y0, int x1, int y1)
{
    int dx = x1 > x0 ? x1 - x0 : x0 - x1;
    int dy = y1 > y0 ? y1 - y0 : y0 - y1;
    int steps = dx > dy ? dx : dy;
    int px = x0, py = y0;
    if (!binary[(uint32_t)y0 * cols + x0]) return 0U;
    for (int i = 1; i <= steps; ++i) {
        int x = x0 + (x1 - x0) * i / steps;
        int y = y0 + (y1 - y0) * i / steps;
        if (!binary[(uint32_t)y * cols + x] ||
            (x != px && y != py &&
             (!binary[(uint32_t)py * cols + x] ||
              !binary[(uint32_t)y * cols + px]))) return 0U;
        px = x; py = y;
    }
    return 1U;
}

/* 仅补回紧贴已确认赛道、在近处单侧出画的连续行。宽度来自同帧
 * 双边界行，不拿屏幕边框当赛道边界，也不跨黑带寻找远处白块。 */
static void track_extend_clipped_near(const uint8_t *binary, uint16_t cols,
                                      uint16_t roi_end, uint16_t roi_rows,
                                      uint16_t maximum_width, uint16_t jump_per_row,
                                      track_vision_result_t *out)
{
    uint16_t base = roi_end;
    for (uint16_t y = roi_end; y > roi_end - roi_rows; --y)
        if (out->row_valid[y - 1U]) { base = y - 1U; break; }
    if (base == roi_end || out->valid_rows < track_max(4U, roi_rows / 5U)) return;

    /* 取离出画位置最近的一段完整赛道估计透视路宽斜率。 */
    uint16_t far = base;
    for (uint16_t y = base; y > roi_end - roi_rows &&
         (uint16_t)(base - far) < roi_rows / 3U; --y)
        if (out->row_valid[y - 1U]) far = y - 1U;
    if (far == base || base - far < track_max(3U, roi_rows / 10U)) return;
    const uint16_t base_width = out->right[base] - out->left[base] + 1U;
    const uint16_t far_width = out->right[far] - out->left[far] + 1U;
    int32_t width_slope_q8 = ((int32_t)base_width - far_width) * 256 / (base - far);
    if (width_slope_q8 < 0) width_slope_q8 = 0;
    if (width_slope_q8 > (int32_t)cols * 512 / roi_rows)
        width_slope_q8 = (int32_t)cols * 512 / roi_rows;

    uint16_t previous_left = out->left[base], previous_right = out->right[base];
    uint16_t previous_center = out->center[base];
    for (uint16_t y = base + 1U; y < roi_end; ++y)
    {
        const uint8_t *pixels = binary + (uint32_t)y * cols;
        uint16_t x = 0U, chosen_left = 0U, chosen_right = 0U;
        uint8_t matches = 0U;
        while (x < cols)
        {
            while (x < cols && pixels[x] == 0U) ++x;
            if (x == cols) break;
            uint16_t start = x, end = x, white = 0U;
            uint8_t gap = 0U;
            while (x < cols)
            {
                if (pixels[x]) { ++white; end = x; gap = 0U; }
                else if (++gap > 2U) { ++x; break; }
                ++x;
            }
            uint16_t visible_width = end - start + 1U;
            if ((start == 0U) == (end == cols - 1U) ||
                (uint32_t)white * 4U < (uint32_t)visible_width * 3U ||
                !route_member(y, (start + end) / 2U)) continue;
            if (end == cols - 1U && track_distance(start, previous_left) <= jump_per_row)
            { chosen_left = start; chosen_right = end; ++matches; }
            if (start == 0U && track_distance(end, previous_right) <= jump_per_row)
            { chosen_left = start; chosen_right = end; ++matches; }
        }
        if (matches != 1U) break;
        int32_t projected_width = (int32_t)base_width +
            width_slope_q8 * (int32_t)(y - base) / 256;
        /* 行宽与斜率的整数舍入可相差 1~2 像素；贴边首行也允许进入。 */
        if (projected_width > maximum_width ||
            projected_width + 2 < (int32_t)(chosen_right - chosen_left + 1U)) break;
        int32_t center = chosen_right == cols - 1U ?
            (int32_t)chosen_left + projected_width / 2 :
            (int32_t)chosen_right - projected_width / 2;
        if (center <= 0 || center >= (int32_t)cols - 1 ||
            track_distance((uint16_t)center, previous_center) > jump_per_row ||
            !track_white_connection(binary, cols, previous_center, y - 1U, center, y)) break;
        out->left[y] = chosen_left;
        out->right[y] = chosen_right;
        out->center[y] = (uint16_t)center;
        out->row_valid[y] = 1U;
        ++out->valid_rows;
        if (out->no_candidate_rows != 0U) --out->no_candidate_rows;
        previous_left = chosen_left;
        previous_right = chosen_right;
        previous_center = (uint16_t)center;
    }
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
    route_build(binary,rows,cols);
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
    uint16_t middle_count = 0;
    uint32_t near_sum = 0;
    uint32_t far_sum = 0;
    uint32_t middle_sum = 0;
    uint32_t near_width_sum = 0;
    uint32_t far_width_sum = 0;
    uint32_t weighted_sum = 0;
    uint32_t weight_total = 0;
    out->roi_rows = roi_rows;

    for (uint16_t next_row = roi_end; next_row > roi_start; --next_row)
    {
        const uint16_t row = next_row - 1U;
        const uint8_t *pixels = binary + (uint32_t)row * cols;
        if(route_seed_ambiguous) {++out->ambiguous_rows;continue;}
        const uint16_t row_gap = have_previous ? (uint16_t)(previous_row - row) : 1;
        if (have_previous && row_gap > maximum_missing_rows + 1U)
            break; // 连续缺失过长：停止向远端追踪，禁止跳到另一物体。

        uint16_t best_left = 0;
        uint16_t best_right = 0;
        uint16_t best_center = 0;
        uint16_t best_distance = UINT16_MAX;
        uint16_t second_distance = UINT16_MAX;
        uint16_t candidates = 0;
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

            uint16_t center = (start + end) / 2U;
            /* A one- or two-pixel dark seam can cross the road midpoint.
             * Keep the same white run, then use its nearest white center for
             * the pixel-continuity check. Never jump to another component. */
            if (!pixels[center]) {
                if (center > start && pixels[center - 1U]) --center;
                else if (center + 1U <= end && pixels[center + 1U]) ++center;
                else if (center > start + 1U && pixels[center - 2U]) center -= 2U;
                else if (center + 2U <= end && pixels[center + 2U]) center += 2U;
                else continue;
            }
            if (!route_member(row, center)) continue;
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
                    overlap_right - overlap_left + 1U < minimum_overlap ||
                    !track_white_connection(binary, cols, previous_center, previous_row, center, row))
                {
                    continue;
                }
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
        if (candidates == 0U)
        {
            ++out->no_candidate_rows;
            if (have_previous && shape_candidates != 0U)
                ++out->disconnected_rows;
            continue;
        }
        if ((!have_previous && candidates != 1U) ||
            (candidates > 1U && second_distance - best_distance <= jump_per_row))
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

    }

    track_extend_clipped_near(binary, cols, roi_end, roi_rows, maximum_width,
                              jump_per_row, out);
    /* 近端补线之后按最终行级结果重算统计，保证画出的中线与控制值同源。 */
    near_count = far_count = middle_count = 0U;
    near_sum = far_sum = middle_sum = near_width_sum = far_width_sum = weighted_sum = weight_total = 0U;
    for (uint16_t row = roi_start; row < roi_end; ++row)
    {
        if (!out->row_valid[row]) continue;
        uint16_t center = out->center[row];
        uint16_t width = out->right[row] - out->left[row] + 1U;
        uint16_t weight = 2U;
        if (row >= near_start) { ++near_count; near_sum += center; near_width_sum += width; weight = 3U; }
        else if (row < far_end) { ++far_count; far_sum += center; far_width_sum += width; weight = 1U; }
        else { ++middle_count; middle_sum += center; }
        weighted_sum += (uint32_t)center * weight;
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

    out->confidence = (uint8_t)((uint32_t)out->valid_rows * 100U / roi_rows);
    if (!have_previous)
        out->reject_flags |= TRACK_VISION_REJECT_NO_SEED;
    if ((uint32_t)out->valid_rows * 100U < (uint32_t)roi_rows * 45U)
        out->reject_flags |= TRACK_VISION_REJECT_TOTAL_COVERAGE;
    if ((uint32_t)near_count * 100U < (uint32_t)zone_rows * 45U)
        out->reject_flags |= TRACK_VISION_REJECT_NEAR_COVERAGE;
    if ((uint32_t)far_count * 100U < (uint32_t)zone_rows * 45U)
        out->reject_flags |= TRACK_VISION_REJECT_FAR_COVERAGE;
    /* Same-frame lower-third geometry: split into two populated bands. */
    uint32_t ns[2] = {0U, 0U};
    uint16_t nc[2] = {0U, 0U};
    const uint16_t split = near_start + zone_rows / 2U;
    for (uint16_t y = near_start; y < roi_end; ++y) {
        if (!out->row_valid[y]) continue;
        unsigned band = y < split ? 0U : 1U;
        ns[band] += out->center[y]; ++nc[band];
    }
    if (nc[0] >= track_max(2U, zone_rows / 6U) &&
        nc[1] >= track_max(2U, zone_rows / 6U)) {
        out->near_geometry_valid = 1U;
        out->near_error = track_clamp_error(
            ((int64_t)2 * near_sum - (int64_t)(cols - 1U) * near_count) * 1000 /
            ((int64_t)(cols - 1U) * near_count));
        out->near_heading = track_clamp_error(
            ((int64_t)ns[0] * nc[1] - (int64_t)ns[1] * nc[0]) * 2000 /
            ((int64_t)nc[0] * nc[1] * (cols - 1U)));
    }
    /* Require a continuous run reaching the actual bottom, not a floating
     * lowest-detected point higher in the image. Use up to the last five rows. */
    uint32_t bottom_sum = 0U;
    uint16_t bottom_count = 0U;
    for (uint16_t y = roi_end; y > roi_start && bottom_count < 5U; --y) {
        if (!out->row_valid[y - 1U]) break;
        bottom_sum += out->center[y - 1U]; ++bottom_count;
    }
    if (bottom_count >= 3U) {
        out->bottom_valid = 1U;
        out->bottom_error = track_clamp_error(
            ((int64_t)2 * bottom_sum - (int64_t)(cols - 1U) * bottom_count) * 1000 /
            ((int64_t)(cols - 1U) * bottom_count));
    }
    /* A tight bend can leave the far third of the image while the connected
     * road still fills the near third and reaches the bottom. Follow only that
     * confirmed near road; never promote a floating/distant white component. */
    /* In a hard bend, the near road can occupy only one third of the ROI.
     * Total coverage then falls below 45% even though both near boundaries
     * and the bottom rows remain continuous. Accept only these two missing
     * coverage flags; all seed/near failures still reject the frame. */
    uint8_t near_only = (uint8_t)(
        (out->reject_flags == TRACK_VISION_REJECT_FAR_COVERAGE ||
         out->reject_flags == (TRACK_VISION_REJECT_TOTAL_COVERAGE |
                               TRACK_VISION_REJECT_FAR_COVERAGE)) &&
        (uint32_t)near_count * 100U >= (uint32_t)zone_rows * 80U &&
        (uint32_t)far_count * 100U < (uint32_t)zone_rows * 20U &&
        out->near_geometry_valid && out->bottom_valid);
    /* Avoid a hard valid/invalid flip when a clean, bottom-connected bend
     * misses the ordinary 45% far coverage gate by only a few rows. */
    uint8_t partial_far = (uint8_t)(
        out->reject_flags == TRACK_VISION_REJECT_FAR_COVERAGE &&
        (uint32_t)far_count * 100U >= (uint32_t)zone_rows * 30U &&
        (uint32_t)near_count * 100U >= (uint32_t)zone_rows * 80U &&
        (uint32_t)out->valid_rows * 100U >= (uint32_t)roi_rows * 60U &&
        out->ambiguous_rows == 0U && out->disconnected_rows == 0U &&
        out->near_geometry_valid && out->bottom_valid);
    if (near_only) {
        out->near_only = 1U;
        out->reject_flags = TRACK_VISION_REJECT_NONE;
        /* Confidence is local support for this deliberately near-only path;
         * global coverage would remain below the controller's 45% gate. */
        out->confidence = (uint8_t)((uint32_t)near_count * 100U / zone_rows);
    }
    if (partial_far)
        out->reject_flags = TRACK_VISION_REJECT_NONE;
    if (out->reject_flags != TRACK_VISION_REJECT_NONE) {
        (void)track_near_edge_rescue(binary, cols, roi_start, near_start, roi_end, out);
        return;
    }
    out->valid = 1U;
    if (near_only) {
        out->ring_candidate = 0U;
        out->ring_direction = 0;
        out->error = out->near_error;
        /* The middle third is still connected to the near road. Its lateral
         * trend gives a modest early cue before a turn reaches the bottom;
         * the unrelated far third contributes nothing. */
        if (middle_count >= track_max(3U, zone_rows / 4U)) {
            int16_t middle_heading = track_clamp_error(
                ((int64_t)middle_sum * near_count - (int64_t)near_sum * middle_count) * 2000 /
                ((int64_t)middle_count * near_count * (cols - 1U)));
            if (middle_heading > 80) middle_heading = 80;
            if (middle_heading < -80) middle_heading = -80;
            out->heading = middle_heading;
            int32_t preview_extra = 0;
            if (out->near_error > -80 && out->near_error < 80) {
                int32_t strength = middle_heading < 0 ? -middle_heading : middle_heading;
                strength = strength > 30 ? strength - 30 : 0;
                if (strength > 30) strength = 30;
                preview_extra = (int32_t)middle_heading * strength * 2 / 30;
            }
            out->near_heading = track_clamp_error((int64_t)out->near_heading +
                                                  (int64_t)middle_heading * 3 + preview_extra);
        } else
            out->heading = out->near_heading;
        return;
    }
    out->error = track_clamp_error(
        ((int64_t)2 * weighted_sum - (int64_t)(cols - 1U) * weight_total) * 1000 /
        ((int64_t)(cols - 1U) * weight_total));
    out->heading = track_clamp_error(
        ((int64_t)far_sum * near_count - (int64_t)near_sum * far_count) * 2000 /
        ((int64_t)near_count * far_count * (cols - 1U)));
}

void track_vision_filter_display(uint8_t *binary, uint16_t rows, uint16_t cols)
{
    /* Compatibility entry point: preview masking has been removed. */
    (void)binary;
    (void)rows;
    (void)cols;
}


/* Ring vision uses native eight-neighbour boundaries. Ordinary output is immutable. */
void track_ring_vision_init(track_ring_vision_state_t *s)
{
    if(s) memset(s,0,sizeof(*s));
}
void track_ring_360_set_active(uint8_t stage,int8_t direction)
{ (void)stage;(void)direction; }
void track_ring_vision_process(track_ring_vision_state_t *s,const uint8_t *binary,
                               uint16_t rows,uint16_t cols,track_vision_result_t *out)
{
    if(!s || !out) return;
    memset(&out->ring,0,sizeof(out->ring));
    memset(out->raw_flags,0,sizeof(out->raw_flags));
    memset(out->guide_valid,0,sizeof(out->guide_valid));
    memset(out->approach_valid,0,sizeof(out->approach_valid));
    out->ring.path[0]=(track_path_observation_t){out->error,out->heading,out->valid,out->confidence};
    out->ring_candidate=0;out->ring_direction=0;
    if(!RING_FEATURE_ENABLED) return;
    ring_corner_process(binary,rows,cols,0,&out->ring.corners);
    ring_corner_frame_t *v=&out->ring.corners;
    ring_corner_stabilize(binary,0,&s->corner_tracker,v);
    v->frame_number=out->frame_number;
    for(unsigned y=0;y<rows && y<TRACK_VISION_MAX_ROWS;++y) {
        out->raw_left[y]=v->border[0][y];out->raw_right[y]=v->border[1][y];
        out->raw_flags[y]=v->border_flags[y];
    }
    /* Reject disconnected distant geometry using the ordinary near-seeded
     * connectivity map, without discarding strong A/P entrance evidence. */
    for(unsigned side=0;side<2;++side) {
        const ring_corner_point_t *point=&v->p[side];
        int px=(int)point->x+(side ? -3 : 3),py=(int)point->y;
        if(px<0 || px>=cols || py<0 || py>=rows || !route_member((unsigned)py,(unsigned)px)) v->entry[side]=0;
    }
    if(v->paired_mouth) {
        for(unsigned side=0;side<2;++side) {
            int x=(int)v->mouth_lower[side].x+(side ? -4 : 4);
            unsigned y=(unsigned)v->mouth_lower[side].y+2U;
            if(x<0 || x>=cols || y>=rows || !route_member(y,(unsigned)x)) v->paired_mouth=0;
            x=(int)v->mouth_upper[side].x+(side ? -4 : 4);
            y=(unsigned)v->mouth_upper[side].y+2U;
            if(x<0 || x>=cols || y>=rows || !route_member(y,(unsigned)x)) v->paired_mouth=0;
        }
        if(!v->paired_mouth) v->cross_entry=0;
    }
    if(!v->paired_mouth && (out->edge_rescue || out->near_only || out->valid_rows==0)) v->cross_entry=0;
    if(v->paired_mouth) {v->entry[0]=0;v->entry[1]=0;v->cross_entry=1;}
    out->ring.left_quality=v->quality[0];out->ring.right_quality=v->quality[1];
    out->ring.left_open=v->gap[0];out->ring.right_open=v->gap[1];
    out->ring.near_paired=v->paired;out->ring.road_clear=v->road_clear;
    out->ring.near_open=(uint8_t)(v->bottom_open[0] || v->bottom_open[1]);
    out->ring.junction=(uint8_t)(v->gap[0] || v->gap[1]);
    if(v->entry[0] || v->entry[1]) {
        out->ring.type_hint=TRACK_RING_360;
        out->ring.direction_hint=v->entry[0] ? 1 : -1;out->ring.entry_score=90;
    } else if(v->cross_entry) {out->ring.type_hint=TRACK_RING_270;out->ring.entry_score=85;}
    out->ring_candidate=out->ring.junction;out->ring_direction=out->ring.direction_hint;
}
