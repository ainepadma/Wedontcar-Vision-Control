#ifndef TRACK_VISION_H
#define TRACK_VISION_H

#include <stdint.h>
#include "ring_corner.h"

/* 1: recognize and drive 270/360 rings; 0: ordinary line following only.
 * Change this single setting and rebuild. Does not disable ordinary curves.
 */
#ifndef RING_FEATURE_ENABLED
#define RING_FEATURE_ENABLED 1
#endif
#if RING_FEATURE_ENABLED != 0 && RING_FEATURE_ENABLED != 1
#error "RING_FEATURE_ENABLED must be 0 or 1"
#endif

#define TRACK_VISION_MAX_ROWS 480
#define TRACK_VISION_MAX_COLS 752
#include "vision_geometry.h"
/* All rows are eligible. The lower-third seed anchors connectivity only;
 * association propagates to the top without erasing image pixels. */
#define TRACK_VISION_ROI_START_PERCENT VISION_TRACK_START_PERCENT
#define TRACK_VISION_ROI_END_PERCENT VISION_SEARCH_END_PERCENT
#define TRACK_ROUTE_SEED_START_PERCENT VISION_NEAR_START_PERCENT
#define TRACK_ROUTE_MAX_GAP_ROWS 1U

typedef enum track_vision_reject_t
{
    TRACK_VISION_REJECT_NONE = 0U,
    TRACK_VISION_REJECT_INPUT = 1U << 0,
    TRACK_VISION_REJECT_NO_SEED = 1U << 1,
    TRACK_VISION_REJECT_TOTAL_COVERAGE = 1U << 2,
    TRACK_VISION_REJECT_NEAR_COVERAGE = 1U << 3,
    TRACK_VISION_REJECT_FAR_COVERAGE = 1U << 4
} track_vision_reject_t;

/* Ring candidates use native eight-neighbour boundaries.
 * 270 uses eight-neighbour gap repair; 360 uses the PDF seven-stage sequence.
 * Hints alone never advance the motion state machine.
 */
typedef enum { TRACK_RING_UNKNOWN = 0, TRACK_RING_270, TRACK_RING_360 } track_ring_type_t;
typedef enum {
    TRACK_PATH_NORMAL = 0, TRACK_PATH_LEFT, TRACK_PATH_RIGHT,
    TRACK_PATH_STRAIGHT, TRACK_PATH_COUNT
} track_path_mode_t;

typedef struct {
    int16_t error, heading;
    uint8_t valid, confidence;
} track_path_observation_t;

enum {
    TRACK_APPROACH_NO_MODEL=1U, TRACK_APPROACH_NO_OPEN=2U,
    TRACK_APPROACH_BOTH_OPEN=4U, TRACK_APPROACH_EDGE=8U,
    TRACK_APPROACH_SUPPORT=16U, TRACK_APPROACH_STALE=32U
};

typedef struct {
    track_path_observation_t path[TRACK_PATH_COUNT];
    /* Pre-entry corridor only; source names the trusted edge, not turn direction. */
    track_path_observation_t approach_path;
    uint8_t approach_source, approach_reject;
    uint8_t model_ready, junction, type_hint;
    int8_t direction_hint; /* +1 left, -1 right; crossing may remain unresolved. */
    uint8_t left_open, right_open, forward_visible, road_clear;
    uint8_t left_quality, right_quality, left_straight, right_straight;
    uint8_t junction_near; /* 0=far private ROI, 100=near private ROI */
    uint8_t topology[2], island_left, island_right;
    uint8_t white_band, wide_white, straight_support, paired_road;
    uint8_t near_open, near_paired, near_road, entry_score, exit_score;
    int8_t turn_hint;
    uint8_t edge_line_valid[2], reference_valid, exit_reference_valid;
    float edge_near[2], edge_far[2];
    float reference_near, reference_far, exit_near, exit_far, near_center;
    uint16_t left_lost_row, right_lost_row, left_recover_row, right_recover_row;
    uint16_t left_corner_row, right_corner_row;
    ring_corner_frame_t corners; /* v0.6 corner counts, directed arms and candidate boundaries. */
} track_ring_observation_t;

typedef struct {
    uint16_t rows, cols;
    uint8_t ready, age_frames, learned_frames;
    float width_near, width_slope; /* width(row)=width_near+slope*(row-near_row) */
    float center_near, center_slope;
    ring_corner_tracker_t corner_tracker;
} track_ring_vision_state_t;

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
    track_ring_observation_t ring;
    uint16_t raw_left[TRACK_VISION_MAX_ROWS], raw_right[TRACK_VISION_MAX_ROWS];
    uint8_t raw_flags[TRACK_VISION_MAX_ROWS]; /* bit0 left, bit1 right */
    uint16_t guide_center[TRACK_PATH_COUNT - 1][TRACK_VISION_MAX_ROWS];
    uint8_t guide_valid[TRACK_PATH_COUNT - 1][TRACK_VISION_MAX_ROWS];
    uint16_t approach_center[TRACK_VISION_MAX_ROWS];
    uint8_t approach_valid[TRACK_VISION_MAX_ROWS];
    uint32_t frame_number;
    int16_t error;
    int16_t heading;
    int16_t near_error, near_heading;
    uint8_t near_geometry_valid;
    uint8_t edge_rescue; /* Near single-edge turn; ring geometry is suppressed. */
    uint8_t near_only; /* Connected near two-edge bend; no far ring evidence. */
    int16_t bottom_error;
    uint8_t bottom_valid;
} track_vision_result_t;

/*
 * 白轨黑底二值图，非零像素视为白色，输入图像不会被修改。
 * 使用 vision_geometry.h 定义的 ROI，从近到远选择相连白色区域。
 * 普通路径要求双边界及足够的整体/近端/远端覆盖；远端出画的急弯
 * 可以由连续且触及底部的近端双边界控制，即使整体覆盖不足45%。
 * 若近端只剩画面一侧的连续白轨，
 * 且另一侧与远端同侧白块均不足以造成歧义，则 edge_rescue 使用
 * 近端单边方向给出低速急弯指令，不用于环岛判定。
 * 普通路径 confidence 是 ROI 有效行比例；edge_rescue 是近端单边
 * 连续覆盖率。整体无效时 error/heading 均为零。
 * reject_flags、近/远有效行和行级拒绝计数均属于本次调用，便于把诊断
 * 与同一帧图像绑定；frame_number 由采集方在处理完成后填写。
 * near/far_width 为对应 1/3 区域的平均白轨宽度；ring_candidate 仅表示
 * 远端显著张开的历史诊断；由八邻域几何与顺序状态决定环岛，不能单独接管控制。
 * error 约 [-1000, 1000]，赛道偏右为正；heading 远端向右为正。
 * rows 范围 [10, 480]，cols 范围 [16, 752]；其他尺寸/空输入安全拒绝。
 */
void track_vision_process(const uint8_t *binary, uint16_t rows, uint16_t cols,
                          track_vision_result_t *out);

/* Compatibility no-op: display preserves all current binary pixels. */
void track_vision_filter_display(uint8_t *binary, uint16_t rows, uint16_t cols);

/* Call after ordinary processing, once per captured frame, before publishing.
 * Learns perspective width from narrow, paired near-field boundaries, freezes
 * the model through junctions. All image access stays outside the ISR.
 */
void track_ring_360_set_active(uint8_t stage,int8_t direction); /* Compatibility only; state selection never mutates vision. */
void track_ring_vision_init(track_ring_vision_state_t *state);
void track_ring_vision_process(track_ring_vision_state_t *state, const uint8_t *binary,
                               uint16_t rows, uint16_t cols, track_vision_result_t *out);

#endif
