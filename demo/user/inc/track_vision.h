#ifndef TRACK_VISION_H
#define TRACK_VISION_H

#include <stdint.h>

#define TRACK_VISION_MAX_ROWS 480
#define TRACK_VISION_MAX_COLS 752
// 控制中线覆盖图像纵向 [20%H, 68%H)：避开过远端的不稳定区域，
// 同时保持此前确认的车体遮挡上边界。
#define TRACK_VISION_ROI_START_PERCENT 20U
#define TRACK_VISION_ROI_END_PERCENT 68U

typedef enum track_vision_reject_t
{
    TRACK_VISION_REJECT_NONE = 0U,
    TRACK_VISION_REJECT_INPUT = 1U << 0,
    TRACK_VISION_REJECT_NO_SEED = 1U << 1,
    TRACK_VISION_REJECT_TOTAL_COVERAGE = 1U << 2,
    TRACK_VISION_REJECT_NEAR_COVERAGE = 1U << 3,
    TRACK_VISION_REJECT_FAR_COVERAGE = 1U << 4
} track_vision_reject_t;

typedef struct track_vision_result_t
{
    uint16_t left[TRACK_VISION_MAX_ROWS];
    uint16_t center[TRACK_VISION_MAX_ROWS];
    uint16_t right[TRACK_VISION_MAX_ROWS];
    uint8_t row_valid[TRACK_VISION_MAX_ROWS];
    uint8_t valid;
    uint8_t confidence;
    uint8_t reject_flags;
    uint16_t valid_rows;
    uint16_t roi_rows;
    uint16_t near_valid_rows;
    uint16_t far_valid_rows;
    uint16_t no_candidate_rows;
    uint16_t ambiguous_rows;
    uint16_t disconnected_rows;
    uint16_t near_width;
    uint16_t far_width;
    uint16_t width_growth_percent;
    uint8_t ring_candidate;
    int8_t ring_direction; /* +1 left, -1 right, 0 unresolved. */
    uint32_t frame_number;
    int16_t error;
    int16_t heading;
} track_vision_result_t;

/*
 * 白轨黑底二值图，非零像素视为白色，输入图像不会被修改。
 * 仅跟踪 [20%H, 68%H) 行，排除过远端和下部固定车体区域，
 * 从近到远选择相连白色区域。
 * 保守限制：仅补偿不超过 2 像素的黑缝；白色宽度须在约 6.25%W
 * 至 80%W 之间，且左右边界都可见。单边出画、全白、窄亮点以及
 * 无法确定唯一连续分支的行不补线、不推测中线。不能代替岔路决策。
 * 有效行总比例、近端 1/3 和远端 1/3 各自的比例均须达到 45%。
 * confidence 为 ROI 有效行百分比；整体无效时 error/heading 均为零。
 * reject_flags、近/远有效行和行级拒绝计数均属于本次调用，便于把诊断
 * 与同一帧图像绑定；frame_number 由采集方在处理完成后填写。
 * near/far_width 为对应 1/3 区域的平均白轨宽度；ring_candidate 仅表示
 * 远端显著张开的环岛候选，必须再经连续帧与 IMU 转向确认，不能单独接管控制。
 * error 约 [-1000, 1000]，赛道偏右为正；heading 远端向右为正。
 * rows 范围 [10, 480]，cols 范围 [16, 752]；其他尺寸/空输入安全拒绝。
 */
void track_vision_process(const uint8_t *binary, uint16_t rows, uint16_t cols,
                          track_vision_result_t *out);

#endif
