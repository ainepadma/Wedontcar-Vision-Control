#ifndef TRACK_VISION_H
#define TRACK_VISION_H

#include <stdint.h>

/* 1: recognize and drive 270/360 rings; 0: ordinary line following only.
 * Change this single setting and rebuild. Does not disable ordinary curves.
 */
#ifndef RING_FEATURE_ENABLED
#define RING_FEATURE_ENABLED 0
#endif
#if RING_FEATURE_ENABLED != 0 && RING_FEATURE_ENABLED != 1
#error "RING_FEATURE_ENABLED must be 0 or 1"
#endif

#define TRACK_VISION_MAX_ROWS 480
#define TRACK_VISION_MAX_COLS 752
// 当前相机安装已确认车体从约 70%H 开始侵入画面；留出 2%H 标定余量，
// 只处理车体上方的可见赛道，不在固定遮挡区内猜测或补画赛道。
#define TRACK_VISION_ROI_START_PERCENT 40U
#define TRACK_VISION_ROI_END_PERCENT 68U
/* Route association only. Never seed an unrelated distant road above 55%H. */
#define TRACK_ROUTE_SEED_START_PERCENT 55U
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

/* Ring geometry is extracted independently of the ordinary connected-run filter.
 * The map's one-sided junction is the 360 route; a two-sided crossing is 270.
 * UNKNOWN is retained until several observations agree; no course-order table.
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

typedef struct {
    track_path_observation_t path[TRACK_PATH_COUNT];
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
    /* Pure per-frame ZORE outputs for each input stage and side. */
    uint8_t zore360_ready,zore360_next[2][10],zore360_done[2][10],zore360_det_valid[2][10];
    int16_t zore360_det[2][10],zore360_off[2][10];
    float zore360_error_scale[2][10];
    int16_t zore360_center[TRACK_VISION_MAX_ROWS]; /* Native camera coordinates. */
    uint8_t zore360_center_stage,zore360_width_ready;
    int8_t zore360_center_direction;
    uint16_t zore360_native_rows,zore360_native_cols,zore360_top,zore360_end;
    float zore360_center_x,zore360_width_near,zore360_scale_y;
    track_path_observation_t zore360_path[2][10];
    uint8_t zore360_match;
    uint16_t zore360_loss[2],zore360_both_lost,zore360_point1[2],zore360_point2[2];
    uint8_t zore360_help[2];
    uint32_t zore360_variance[2];
    uint16_t left_lost_row, right_lost_row, left_recover_row, right_recover_row;
    uint16_t left_corner_row, right_corner_row;
} track_ring_observation_t;

typedef struct {
    uint16_t rows, cols;
    uint8_t ready, age_frames;
    float width_near, width_slope; /* width(row)=width_near+slope*(row-near_row) */
    float center_near, center_slope;
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
    uint32_t frame_number;
    int16_t error;
    int16_t heading;
} track_vision_result_t;

/*
 * 白轨黑底二值图，非零像素视为白色，输入图像不会被修改。
 * 仅跟踪 [40%H, 68%H) 行，排除底部固定车体区域，从近到远选择相连白色区域。
 * 保守限制：仅补偿不超过 2 像素的黑缝；白色宽度须在约 6.25%W
 * 至 80%W 之间。弯道近端若仅有一侧边界出画，且同帧完整赛道足以
 * 估计路宽、可见边界逐行连续，则补算该侧的中线；无依据的单边出画、
 * 全白、窄亮点以及无法确定唯一连续分支的行仍拒绝。不能代替岔路决策。
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

/* 仅在本帧视觉计算及发布之后用于屏幕预览。把未连到近处赛道的白斑
 * 从预览图中隐藏；不会修改 result、控制输入或发送前冻结的原始帧。
 * 必须紧跟同一帧的 track_vision_process/track_ring_vision_process 调用。 */
void track_vision_filter_display(uint8_t *binary, uint16_t rows, uint16_t cols);

/* Call after ordinary processing, once per captured frame, before publishing.
 * Learns perspective width from narrow, paired near-field boundaries, freezes
 * the model through junctions. All image access stays outside the ISR.
 */
void track_ring_360_set_active(uint8_t stage,int8_t direction); /* 0=normal, 255=other special element */
void track_ring_vision_init(track_ring_vision_state_t *state);
void track_ring_vision_process(track_ring_vision_state_t *state, const uint8_t *binary,
                               uint16_t rows, uint16_t cols, track_vision_result_t *out);

#endif
