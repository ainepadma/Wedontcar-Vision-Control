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
            if (width < minimum_width || width > maximum_width ||
                (uint32_t)white_pixels * 4U < (uint32_t)width * 3U)
                continue;

            ++shape_candidates;

            const uint16_t center = (start + end) / 2U;
            if(!route_member(row,center) || !pixels[center]) continue;
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
    near_count = far_count = 0U;
    near_sum = far_sum = near_width_sum = far_width_sum = weighted_sum = weight_total = 0U;
    for (uint16_t row = roi_start; row < roi_end; ++row)
    {
        if (!out->row_valid[row]) continue;
        uint16_t center = out->center[row];
        uint16_t width = out->right[row] - out->left[row] + 1U;
        uint16_t weight = 2U;
        if (row >= near_start) { ++near_count; near_sum += center; near_width_sum += width; weight = 3U; }
        else if (row < far_end) { ++far_count; far_sum += center; far_width_sum += width; weight = 1U; }
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

    /* A ring entrance opens into a much wider far-field white region. Keep
     * this deliberately conservative: it is only a candidate and is later
     * confirmed by distinct frames plus signed yaw from the IMU. */
    if (RING_FEATURE_ENABLED && (uint32_t)near_count * 100U >= (uint32_t)zone_rows * 35U &&
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
    out->valid = 1U;
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


/* Ring-private native geometry. Never writes ordinary rows/scalars. */
#include "ring_control.h"
#include <math.h>
#include <stdlib.h>

void track_ring_vision_init(track_ring_vision_state_t *s)
{
    if(s!=NULL) memset(s,0,sizeof(*s));
}
static float ring_width(const track_ring_vision_state_t *s,unsigned row,unsigned near)
{
    float w=s->width_near+s->width_slope*((float)row-(float)near);
    return fmaxf((float)s->cols/16.f,fminf(w,(float)s->cols*0.65f));
}
/* Scan all runs, not a small radius around a possibly black island centre.
 * Select the corridor intersecting/nearest the predicted incoming reference. */
static uint8_t ring_run(const uint8_t *pix,unsigned cols,float anchor,unsigned row,uint16_t *left,uint16_t *right)
{
    float best=1e9f;uint8_t found=0U;
    for(unsigned x=0U;x<cols;) {
        while(x<cols && pix[x]==0U) ++x;
        if(x==cols) break;
        unsigned l=x,r=x,white=0U,gap=0U;
        while(x<cols) {
            if(pix[x]!=0U) {r=x;++white;gap=0U;}
            else if(++gap>2U) {++x;break;}
            ++x;
        }
        unsigned width=r-l+1U;
        if(width<cols/16U || white*4U<width*3U) continue;
        if(!route_member(row,(l+r)/2U)) continue;
        float distance=anchor<(float)l ? (float)l-anchor : (anchor>(float)r ? anchor-(float)r : 0.f);
        if(distance>(float)cols*0.20f) continue;
        float score=distance*4.f+fabsf(((float)l+(float)r)*0.5f-anchor)*0.10f;
        if(score<best) {best=score;*left=(uint16_t)l;*right=(uint16_t)r;found=1U;}
    }
    return found;
}
static void ring_path_finish(track_vision_result_t *out,unsigned mode,unsigned start,unsigned end,unsigned cols)
{
    uint32_t sum=0U,ws=0U,ns=0U,fs=0U,nc=0U,fc=0U,count=0U;
    unsigned zone=(end-start)/3U;
    track_path_observation_t *p=&out->ring.path[mode];
    for(unsigned y=start;y<end;++y) {
        if(!out->guide_valid[mode-1U][y]) continue;
        unsigned c=out->guide_center[mode-1U][y];
        unsigned w=y>=end-zone ? 3U : (y<start+zone ? 1U : 2U);
        sum+=c*w;ws+=w;++count;
        if(y>=end-zone) {ns+=c;++nc;}
        if(y<start+zone) {fs+=c;++fc;}
    }
    p->confidence=(uint8_t)(count*100U/(end-start));
    if(p->confidence<65U || nc<zone/2U || fc<zone/2U || ws==0U || nc==0U || fc==0U) return;
    p->valid=1U;
    p->error=track_clamp_error(((int64_t)2*sum-(int64_t)(cols-1U)*ws)*1000/((int64_t)(cols-1U)*ws));
    p->heading=track_clamp_error(((int64_t)fs*nc-(int64_t)ns*fc)*2000/((int64_t)nc*fc*(cols-1U)));
}

/* Use the PREVIOUS normal-road model. A gradually widening mouth must not
 * first move the reference and then be judged against its own biased fit. */
static uint8_t ring_approach_opening(const track_ring_vision_state_t *s,const uint8_t *image,
                                    unsigned start,unsigned end)
{
    if(!s->ready) return 0U;
    unsigned run[2]={0U,0U},best[2]={0U,0U};
    for(unsigned y=start;y<end;++y) {
        uint16_t l=0U,r=0U;float w=ring_width(s,y,end-1U);
        float c=s->center_near+s->center_slope*((float)y-(float)(end-1U));
        uint8_t found=ring_run(image+y*s->cols,s->cols,c,y,&l,&r);
        uint8_t wide=(uint8_t)(found && (float)(r-l)>w*RING_APPROACH_WIDTH_RATIO &&
                               (float)(r-l)-w>(float)s->cols*.025f);
        uint8_t open[2]={(uint8_t)(wide && (float)l<c-w*RING_APPROACH_SIDE_RATIO),
                         (uint8_t)(wide && (float)r>c+w*RING_APPROACH_SIDE_RATIO)};
        for(unsigned side=0U;side<2U;++side) {
            run[side]=open[side] ? run[side]+1U : 0U;
            if(run[side]>best[side]) best[side]=run[side];
        }
    }
    return (uint8_t)((best[0]>=3U ? 1U : 0U)|(best[1]>=3U ? 2U : 0U));
}

static void ring_approach_path(const track_ring_vision_state_t *history,const uint8_t *image,
                               track_vision_result_t *out,unsigned start,unsigned end,uint8_t opening)
{
    track_ring_observation_t *v=&out->ring;
    if(!history->ready || history->learned_frames<RING_APPROACH_LEARN_FRAMES) {
        v->approach_reject=TRACK_APPROACH_NO_MODEL;return;
    }
    if(history->age_frames>=RING_APPROACH_MODEL_MAX_AGE) {
        v->approach_reject=TRACK_APPROACH_STALE;return;
    }
    if(!opening) {v->approach_reject=TRACK_APPROACH_NO_OPEN;return;}
    if(opening==3U || (v->left_open && v->right_open)) {
        v->approach_reject=TRACK_APPROACH_BOTH_OPEN;return;
    }
    unsigned side=opening==1U ? 1U : 0U,mask=1U<<side,cols=history->cols;
    unsigned span=end-start,zone=span/3U,count=0U,nc=0U,fc=0U,paired=0U;
    unsigned gap=0U,max_gap=0U,changed=0U;
    float sum=0.f,weights=0.f,ns=0.f,fs=0.f,previous=0.f;
    for(unsigned next=end;next>start;--next) {
        unsigned y=next-1U;float width=ring_width(history,y,end-1U);
        if(!(out->raw_flags[y]&mask)) {if(++gap>max_gap) max_gap=gap;continue;}
        float edge=(float)(side ? out->raw_right[y] : out->raw_left[y]);
        float c=edge+(side ? -.5f : .5f)*width;
        float predicted=history->center_near+history->center_slope*((float)y-(float)(end-1U));
        /* Preserve a genuinely observed edge, including gentle curvature. A
         * displaced/fragmented opposite branch is not a replacement boundary. */
        if(fabsf(c-predicted)>width*.20f || (count && fabsf(c-previous)>(float)cols*.015f*(float)(gap+1U)+1.f)) {
            v->approach_reject=TRACK_APPROACH_EDGE;return;
        }
        unsigned raw_width=out->raw_right[y]-out->raw_left[y];
        uint8_t expanded=(uint8_t)((float)raw_width>width*1.08f || (out->raw_flags[y]&12U));
        if(!expanded && out->row_valid[y]) c=(float)out->center[y];
        else ++changed;
        int x=(int)lroundf(c),margin=(int)(width*.18f);
        if(x-margin<0 || x+margin>=(int)cols) {v->approach_reject=TRACK_APPROACH_SUPPORT;return;}
        unsigned black=0U;uint8_t supported=1U;
        for(int px=x-margin;px<=x+margin;++px) {
            if(!image[y*cols+(unsigned)px]) {if(++black>2U) {supported=0U;break;}}
            else black=0U;
        }
        if(!supported) {v->approach_reject=TRACK_APPROACH_SUPPORT;return;}
        previous=c;gap=0U;
        out->approach_center[y]=(uint16_t)x;out->approach_valid[y]=1U;
        float weight=y>=end-zone ? 3.f : (y<start+zone ? 1.f : 2.f);
        sum+=weight*c;weights+=weight;++count;
        if(y>=end-zone) {
            ns+=c;++nc;
            if((out->raw_flags[y]&3U)==3U && (float)raw_width>=width*.85f && (float)raw_width<=width*1.15f) ++paired;
        }
        if(y<start+zone) {fs+=c;++fc;}
    }
    if(count*100U<span*85U || nc*100U<zone*80U || fc*100U<zone*80U ||
       paired*100U<zone*60U || max_gap>2U || changed<3U || weights<1.f) {
        v->approach_reject=TRACK_APPROACH_EDGE;return;
    }
    v->approach_path=(track_path_observation_t){
        track_clamp_error((int64_t)lroundf((sum/weights*2.f/(float)(cols-1U)-1.f)*1000.f)),
        track_clamp_error((int64_t)lroundf((fs/(float)fc-ns/(float)nc)*2000.f/(float)(cols-1U))),
        1U,(uint8_t)(count*100U/span)};
    v->approach_source=(uint8_t)(side==0U ? TRACK_PATH_LEFT : TRACK_PATH_RIGHT);
}
/* ZORE T -> W -> T topology, with lengths normalized to this private ROI.
 * One isolated lost row does not become the missing middle segment. */
static uint8_t ring_topology(const track_vision_result_t *out,unsigned start,unsigned end,unsigned side,unsigned minimum)
{
    unsigned near=0U,lost=0U,far=0U,stage=0U;
    unsigned mask=side==0U ? 1U : 2U,opening=side==0U ? 4U : 8U;
    for(unsigned next=end;next>start;--next) {
        unsigned flags=out->raw_flags[next-1U];
        if(stage==0U) {
            if(flags&mask) ++near;
            else if((flags&opening) && near>=minimum) {stage=1U;lost=1U;}
        } else if(stage==1U) {
            if(flags&opening) ++lost;
            else if(flags&mask) {
                if(lost>=minimum) {stage=2U;far=1U;}
                else {stage=0U;near=1U;lost=0U;}
            }
        } else {
            if(flags&mask) {if(++far>=minimum) return 1U;}
            else far=0U;
        }
    }
    return 0U;
}
static void ring_island(const uint8_t *image,unsigned rows,unsigned cols,track_ring_observation_t *v)
{
    unsigned run[2]={0U,0U},best[2]={0U,0U};int previous[2]={-10000,-10000};
    unsigned flank=cols/16U,minimum=cols*RING_ISLAND_MIN_WIDTH_PERCENT/100U;
    for(unsigned y=rows*RING_SEARCH_START_PERCENT/100U;y<rows*RING_NEAR_START_PERCENT/100U;++y) {
        const uint8_t *pix=image+y*cols;int found[2]={-10000,-10000};
        for(unsigned x=flank;x+flank<cols;) {
            if(pix[x]!=0U) {++x;continue;}
            unsigned l=x;while(x<cols && pix[x]==0U) ++x;unsigned r=x;
            if(r-l<minimum || r-l>cols*3U/5U || r+flank>=cols) continue;
            unsigned wl=0U,wr=0U;
            for(unsigned d=1U;d<=flank;++d) {if(pix[l-d]) ++wl;if(pix[r+d-1U]) ++wr;}
            if(wl*5U<flank*4U || wr*5U<flank*4U) continue;
            unsigned mid=(l+r)/2U;found[mid<cols/2U ? 0U : 1U]=(int)mid;
        }
        for(unsigned side=0U;side<2U;++side) {
            int delta=found[side]-previous[side];
            if(found[side]!=-10000) {
                run[side]=(delta>-(int)cols/8 && delta<(int)cols/8) ? run[side]+1U : 1U;
                if(run[side]>best[side]) best[side]=run[side];
            } else run[side]=0U;
            previous[side]=found[side];
        }
    }
    v->island_left=(uint8_t)(best[0]>=RING_ISLAND_MIN_ROWS);
    v->island_right=(uint8_t)(best[1]>=RING_ISLAND_MIN_ROWS);
}
static void ring_edge_fit(const track_ring_vision_state_t *s,track_vision_result_t *out,
                          unsigned start,unsigned end,unsigned side)
{
    float sy=0.f,syy=0.f,sx=0.f,sxy=0.f,count=0.f;
    unsigned mask=1U<<side,near=end-1U;
    for(unsigned y=start;y<end;++y) {
        if(!(out->raw_flags[y]&mask)) continue;
        float edge=side==0U ? out->raw_left[y] : out->raw_right[y];
        float x=edge+(side==0U ? 0.5f : -0.5f)*ring_width(s,y,near),dy=(float)y-(float)near;
        sx+=x;sy+=dy;syy+=dy*dy;sxy+=x*dy;count+=1.f;
    }
    float den=count*syy-sy*sy;
    if(count<(float)(end-start)*0.60f || count<5.f || den<1.f) return;
    float k=(count*sxy-sy*sx)/den,x0=(sx-k*sy)/count,error=0.f;
    for(unsigned y=start;y<end;++y) {
        if(!(out->raw_flags[y]&mask)) continue;
        float edge=side==0U ? out->raw_left[y] : out->raw_right[y];
        float x=edge+(side==0U ? 0.5f : -0.5f)*ring_width(s,y,near);
        error+=fabsf(x-x0-k*((float)y-(float)near));
    }
    if(error/count>(float)s->cols*RING_EDGE_RESIDUAL_PERCENT/100.f) return;
    float far=(x0+k*((float)s->rows*RING_GUIDE_FAR_PERCENT/100.f-(float)near))*2000.f/(float)(s->cols-1U)-1000.f;
    float nx=(x0+k*((float)s->rows*RING_GUIDE_NEAR_PERCENT/100.f-(float)near))*2000.f/(float)(s->cols-1U)-1000.f;
    if(fabsf(nx)>750.f || fabsf(far)>850.f || fabsf(far-nx)>350.f) return;
    out->ring.edge_line_valid[side]=1U;out->ring.edge_near[side]=nx;out->ring.edge_far[side]=far;
}
/* Kept for source compatibility with old diagnostic tools. v0.6 computes all
 * corner guide candidates from this frame, independent of the ISR stage. */
void track_ring_360_set_active(uint8_t stage,int8_t direction)
{
    (void)stage;(void)direction;
}

void track_ring_vision_process(track_ring_vision_state_t *s,const uint8_t *binary,
                               uint16_t rows,uint16_t cols,track_vision_result_t *out)
{
    if(s==NULL || out==NULL) return;
    memset(&out->ring,0,sizeof(out->ring));memset(out->raw_flags,0,sizeof(out->raw_flags));
    memset(out->guide_valid,0,sizeof(out->guide_valid));
    memset(out->approach_valid,0,sizeof(out->approach_valid));
    out->ring.path[0]=(track_path_observation_t){out->error,out->heading,out->valid,out->confidence};
    if (!RING_FEATURE_ENABLED) {
        out->ring_candidate=0U;out->ring_direction=0;
        track_ring_vision_init(s);
        return;
    }
    if(binary==NULL || rows<60U || rows>TRACK_VISION_MAX_ROWS || cols<64U || cols>TRACK_VISION_MAX_COLS) return;
    route_build(binary,rows,cols);
    if(s->rows!=rows || s->cols!=cols) {track_ring_vision_init(s);s->rows=rows;s->cols=cols;}
    unsigned start=(unsigned)rows*RING_SEARCH_START_PERCENT/100U;
    unsigned end=(unsigned)rows*RING_SEARCH_END_PERCENT/100U,near=end-1U;
    unsigned close=(unsigned)rows*RING_NEAR_START_PERCENT/100U;
    unsigned guide=(unsigned)rows*RING_GUIDE_FAR_PERCENT/100U;
    unsigned minimum=track_max(3U,(uint16_t)((end-start)*RING_FEATURE_ROWS_PERCENT/100U));
    float incoming=s->ready ? s->center_near : (float)(cols-1U)*0.5f;
    float slope=s->ready ? s->center_slope : 0.f;
    if(!s->ready) {
        s->width_near=(float)cols*0.28f;
        s->width_slope=(float)cols*0.12f/(float)(end-guide);
    }
    const track_ring_vision_state_t history=*s;
    uint8_t approach_open=ring_approach_opening(&history,binary,guide,end);
    float sy=0.f,syy=0.f,sw=0.f,syw=0.f,sc=0.f,syc=0.f,count=0.f;
    for(unsigned y=close;y<end;++y) {
        uint16_t l=0U,r=0U;float expected=incoming+slope*((float)y-(float)near);
        if(!ring_run(binary+y*cols,cols,expected,y,&l,&r) || l<2U || r>=cols-2U) continue;
        float w=(float)(r-l),dy=(float)y-(float)near,c=((float)l+(float)r)*0.5f;
        if(w>(float)cols*0.65f || (s->ready && (w>ring_width(s,y,near)*1.25f || w<ring_width(s,y,near)*0.75f))) continue;
        sy+=dy;syy+=dy*dy;sw+=w;syw+=dy*w;sc+=c;syc+=dy*c;count+=1.f;
    }
    float den=count*syy-sy*sy,wn=s->width_near,ws=s->width_slope;
    uint8_t fresh=0U;
    if(count>=5.f && den>1.f) {
        float fit_ws=(count*syw-sy*sw)/den,fit_wn=(sw-fit_ws*sy)/count;
        float fit_cs=(count*syc-sy*sc)/den,fit_cn=(sc-fit_cs*sy)/count;
        if(fit_ws>=-0.05f && fit_ws<(float)cols/(float)rows*2.f && fit_wn>(float)cols/12.f &&
           fabsf(fit_cs)<(float)cols/(float)rows*1.5f) {
            wn=fit_wn;ws=fmaxf(0.f,fit_ws);fresh=1U;
            if(!approach_open) {incoming=fit_cn;slope=fit_cs;}
            if(!s->ready) {
                /* Geometry may initialize at a junction, but it is NOT learned
                 * road evidence until later clear frames. Keep its pose usable
                 * so returning to a normal road can acquire that evidence. */
                s->width_near=wn;s->width_slope=ws;
                s->center_near=fit_cn;s->center_slope=fit_cs;s->ready=1U;
            }
        }
    }
    if(s->age_frames<UINT8_MAX) ++s->age_frames;
    out->ring.model_ready=s->ready;
    out->ring.near_center=incoming*2000.f/(float)(cols-1U)-1000.f;
    ring_island(binary,rows,cols,&out->ring);
    unsigned lc=0U,rc=0U,lo=0U,ro=0U,near_l=0U,near_r=0U,near_open=0U;
    unsigned corridor=0U,central=0U,bar_run=0U,bar_best=0U;
    unsigned opened_nearest=start;
    uint32_t upper_white=0U,upper_black=0U;
    /* Structure outside a saturated white band prevents all-white-floor takeover. */
    for(unsigned y=start;y<guide;++y) for(unsigned x=0U;x<cols;x+=3U) {
        if(binary[y*cols+x]) ++upper_white;else ++upper_black;
    }
    for(unsigned next=end;next>start;--next) {
        unsigned y=next-1U;uint16_t l=0U,r=0U;
        float c=incoming+slope*((float)y-(float)near),w=ring_width(s,y,near);
        if(c<1.f || c>(float)cols-2.f || !ring_run(binary+y*cols,cols,c,y,&l,&r)) {bar_run=0U;continue;}
        const uint8_t *pix=binary+y*cols;
        out->raw_left[y]=l;out->raw_right[y]=r;
        uint8_t wide=(uint8_t)((float)(r-l)>w*RING_OPEN_WIDTH_RATIO);
        uint8_t ol=(uint8_t)(wide && (l<2U || (float)l<c-w*(0.5f+RING_OPEN_SIDE_RATIO)));
        uint8_t orr=(uint8_t)(wide && (r>=cols-2U || (float)r>c+w*(0.5f+RING_OPEN_SIDE_RATIO)));
        uint8_t lv=(uint8_t)(l>=2U && !ol),rv=(uint8_t)(r<cols-2U && !orr);
        out->raw_flags[y]=(uint8_t)((lv ? 1U : 0U)|(rv ? 2U : 0U)|(ol ? 4U : 0U)|(orr ? 8U : 0U));
        if(lv) {++lc;if(y>=close) ++near_l;}
        if(rv) {++rc;if(y>=close) ++near_r;}
        if(ol) {++lo;out->ring.left_lost_row=(uint16_t)y;}
        if(orr) {++ro;out->ring.right_lost_row=(uint16_t)y;}
        if(ol || orr) {opened_nearest=track_max((uint16_t)opened_nearest,(uint16_t)y);if(y>=close) ++near_open;}
        if((unsigned)(r-l+1U)*100U>=(unsigned)cols*RING_WHITE_BAND_PERCENT) {
            if(++bar_run>bar_best) bar_best=bar_run;
        } else bar_run=0U;
        int cx=(int)c,margin=(int)(w*0.18f);
        uint8_t support=(uint8_t)(cx-margin>=0 && cx+margin<(int)cols &&
            pix[cx] && pix[cx-margin] && pix[cx+margin]);
        if(y>=guide && support) ++central;
        for(unsigned mode=TRACK_PATH_LEFT;mode<TRACK_PATH_COUNT;++mode) {
            float pc=mode==TRACK_PATH_LEFT ? (float)l+w*0.5f : (mode==TRACK_PATH_RIGHT ? (float)r-w*0.5f : c);
            uint8_t ok=mode==TRACK_PATH_LEFT ? lv : (mode==TRACK_PATH_RIGHT ? rv : support);
            int x=(int)pc;
            if(!ok || x-margin<0 || x+margin>=(int)cols || !pix[x] || !pix[x-margin] || !pix[x+margin]) continue;
            out->guide_valid[mode-1U][y]=1U;out->guide_center[mode-1U][y]=(uint16_t)x;
            if(mode==TRACK_PATH_STRAIGHT && y>=guide) ++corridor;
        }
    }
    unsigned span=end-start,near_span=end-close;
    out->ring.left_quality=(uint8_t)(lc*100U/span);out->ring.right_quality=(uint8_t)(rc*100U/span);
    out->ring.left_open=(uint8_t)(lo>=minimum);out->ring.right_open=(uint8_t)(ro>=minimum);
    out->ring.near_open=(uint8_t)(near_open>=track_max(2U,(uint16_t)(near_span/8U)));
    out->ring.near_paired=(uint8_t)(near_l*100U>=near_span*65U && near_r*100U>=near_span*65U);
    out->ring.near_road=(uint8_t)(near_l*100U>=near_span*60U || near_r*100U>=near_span*60U);
    out->ring.topology[0]=ring_topology(out,start,end,0U,minimum);
    out->ring.topology[1]=ring_topology(out,start,end,1U,minimum);
    out->ring.straight_support=(uint8_t)(central*100U/(end-guide));
    out->ring.forward_visible=(uint8_t)(corridor*100U>=(end-guide)*65U);
    out->ring.junction=(uint8_t)(out->ring.left_open || out->ring.right_open);
    out->ring.junction_near=(uint8_t)((opened_nearest-start)*100U/(span-1U));
    ring_edge_fit(s,out,close,end,0U);ring_edge_fit(s,out,close,end,1U);
    out->ring.left_straight=out->ring.edge_line_valid[0];out->ring.right_straight=out->ring.edge_line_valid[1];
    /* Classic T/W/T and the visually structured broad white-band fallback are
     * separate evidence paths; distant recovery is not mandatory in a full mouth. */
    out->ring.white_band=(uint8_t)(bar_best>=minimum);
    uint8_t structured=(uint8_t)(upper_white>cols/4U && upper_black>cols/4U);
    out->ring.wide_white=(uint8_t)(out->ring.white_band && structured);
    const ring_corner_context_t corner_context={out->raw_left,out->raw_right,out->raw_flags,
        s->width_near,s->width_slope,incoming,slope,s->ready};
    ring_corner_process(binary,rows,cols,&corner_context,&out->ring.corners);
    ring_corner_stabilize(binary,&corner_context,&s->corner_tracker,&out->ring.corners);
    out->ring.corners.frame_number=out->frame_number;
    unsigned left_corners=out->ring.corners.count[0],right_corners=out->ring.corners.count[1];
    if(left_corners || right_corners || out->ring.corners.continued_mask) out->ring.junction=1U;
    /* Counts and paired topology select the element. Never classify by 90deg. */
    if(left_corners==right_corners && left_corners>0U && left_corners<=2U) {
        out->ring.type_hint=TRACK_RING_270;out->ring.entry_score=80U;
    } else if((left_corners>0U && left_corners<3U && right_corners==0U) ||
              (right_corners>0U && right_corners<3U && left_corners==0U)) {
        unsigned side=left_corners ? 0U : 1U;
        if(out->ring.corners.point[side][0].kind!=RING_CORNER_NONE) {
            out->ring.type_hint=TRACK_RING_360;out->ring.entry_score=80U;
            out->ring.direction_hint=side==0U ? 1 : -1;
        }
    }
    if(out->ring.junction && ((out->ring.white_band && s->ready) || out->ring.wide_white || out->ring.topology[0] || out->ring.topology[1] ||
       (out->ring.straight_support>=55U && (out->ring.edge_line_valid[0] || out->ring.edge_line_valid[1]))))
        out->ring.exit_score=80U;
    out->ring.road_clear=(uint8_t)(!out->ring.junction && out->valid && out->confidence>=45U && out->ring.near_paired);
    out->ring.corners.road_clear=out->ring.road_clear;
    out->ring.paired_road=out->ring.road_clear;
    for(unsigned mode=TRACK_PATH_LEFT;mode<TRACK_PATH_COUNT;++mode) ring_path_finish(out,mode,guide,end,cols);
    if(out->ring.path[0].valid && abs(out->heading)>=RING_VISUAL_TURN_MIN) out->ring.turn_hint=out->heading<0 ? 1 : -1;
    else {
        unsigned mode=out->ring.path[TRACK_PATH_RIGHT].valid ? TRACK_PATH_RIGHT : TRACK_PATH_LEFT;
        if(out->ring.path[mode].valid && abs(out->ring.path[mode].heading)>=RING_VISUAL_TURN_MIN)
            out->ring.turn_hint=out->ring.path[mode].heading<0 ? 1 : -1;
    }
    ring_approach_path(&history,binary,out,guide,end,approach_open);
    if(out->ring.road_clear && !approach_open && fresh) {
        /* A normal bend still updates pose. Only width calibration requires
         * a straight road; freezing pose on every bend would lose association. */
        s->center_near=incoming;s->center_slope=slope;s->age_frames=0U;
    }
    if(out->ring.road_clear && !approach_open && abs(out->heading)<120 && fresh) {
        float ry=0.f,ryy=0.f,rx=0.f,rxy=0.f,points=0.f;
        for(unsigned y=guide;y<end;++y) if(out->row_valid[y]) {
            float dy=(float)y-(float)near,x=out->center[y];
            ry+=dy;ryy+=dy*dy;rx+=x;rxy+=dy*x;points+=1.f;
        }
        float reference_den=points*ryy-ry*ry;
        if(points>=5.f && reference_den>1.f) {
            float k=(points*rxy-ry*rx)/reference_den,x0=(rx-k*ry)/points;
            out->ring.reference_near=(x0+k*((float)rows*RING_GUIDE_NEAR_PERCENT/100.f-(float)near))*2000.f/(float)(cols-1U)-1000.f;
            out->ring.reference_far=(x0+k*((float)rows*RING_GUIDE_FAR_PERCENT/100.f-(float)near))*2000.f/(float)(cols-1U)-1000.f;
            out->ring.reference_valid=1U;
        }
        s->width_near+=0.1f*(wn-s->width_near);s->width_slope+=0.1f*(ws-s->width_slope);
        if(s->learned_frames<UINT8_MAX) ++s->learned_frames;
    }
    /* Exit uses only current straight fits; no current circular midpoint is saved. */
    if(out->ring.edge_line_valid[0] || out->ring.edge_line_valid[1]) {
        unsigned side=out->ring.edge_line_valid[1] ? 1U : 0U;
        out->ring.exit_near=out->ring.edge_near[side];out->ring.exit_far=out->ring.edge_far[side];
        out->ring.exit_reference_valid=1U;
    }
    out->ring_candidate=out->ring.junction;out->ring_direction=out->ring.direction_hint;
}
