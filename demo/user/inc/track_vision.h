#ifndef TRACK_VISION_H
#define TRACK_VISION_H

#include <stdint.h>

#define TRACK_VISION_MAX_ROWS 480
#define TRACK_VISION_MAX_COLS 752
// 当前相机安装已确认底部存在车体遮挡；这里只处理可见赛道，不做避障。
#define TRACK_VISION_ROI_START_PERCENT 40U
#define TRACK_VISION_ROI_END_PERCENT 80U

typedef struct track_vision_result_t
{
    uint16_t left[TRACK_VISION_MAX_ROWS];
    uint16_t center[TRACK_VISION_MAX_ROWS];
    uint16_t right[TRACK_VISION_MAX_ROWS];
    uint8_t row_valid[TRACK_VISION_MAX_ROWS];
    uint8_t valid;
    uint8_t confidence;
    uint16_t valid_rows;
    int16_t error;
    int16_t heading;
} track_vision_result_t;

/*
 * 白轨黑底二值图，非零像素视为白色，输入图像不会被修改。
 * 仅跟踪 [40%H, 80%H) 行，排除底部车体区域，从近到远选择相连白色区域。
 * 保守限制：仅补偿不超过 2 像素的黑缝；白色宽度须在约 6.25%W
 * 至 80%W 之间，且左右边界都可见。单边出画、全白、窄亮点以及
 * 无法确定唯一连续分支的行不补线、不推测中线。不能代替岔路决策。
 * 有效行总比例、近端 1/3 和远端 1/3 各自的比例均须达到 45%。
 * confidence 为 ROI 有效行百分比；整体无效时 error/heading 均为零。
 * error 约 [-1000, 1000]，赛道偏右为正；heading 远端向右为正。
 * rows 范围 [10, 480]，cols 范围 [16, 752]；其他尺寸/空输入安全拒绝。
 */
void track_vision_process(const uint8_t *binary, uint16_t rows, uint16_t cols,
                          track_vision_result_t *out);

#endif
