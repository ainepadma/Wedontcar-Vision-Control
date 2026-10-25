#ifndef RING_CONTROL_H
#define RING_CONTROL_H
#include <stdint.h>
#include "track_vision.h"

/* Ring-private image geometry. Ordinary TRACK_VISION_ROI_* stays unchanged. */
#define RING_SEARCH_START_PERCENT 30U
#define RING_SEARCH_END_PERCENT 68U
#define RING_NEAR_START_PERCENT 55U
#define RING_GUIDE_FAR_PERCENT 40U
#define RING_GUIDE_NEAR_PERCENT 68U
#define RING_OPEN_WIDTH_RATIO 1.40f
#define RING_OPEN_SIDE_RATIO 0.30f
#define RING_WHITE_BAND_PERCENT 70U
#define RING_FEATURE_ROWS_PERCENT 6U
#define RING_EDGE_RESIDUAL_PERCENT 2.0f
#define RING_ISLAND_MIN_WIDTH_PERCENT 8U
#define RING_ISLAND_MIN_ROWS 4U
#define RING_CONFIRM_FRAMES 3U
#define RING_NEAR_CONFIRM_FRAMES 2U
#define RING_CLEAR_FRAMES 3U
#define RING_REVISIT_CLEAR_FRAMES 3U
#define RING_TURN_CONFIRM_FRAMES 2U
#define RING_VISUAL_TURN_MIN 60
#define RING_YAW_TURN_ASSIST_DEG 5.0f
#define RING_EXIT_RESTORE_FRAMES 4U
#define RING_PREP_MISS_FRAMES 5U
#define RING_PREP_TIMEOUT_MS 2500U
#define RING_ACTIVE_TIMEOUT_MS 15000U
#define RING_VISION_STALE_MS 150U
#define RING_COOLDOWN_MS 150U
#define RING_LINE_MAX_MS 2500U
#define RING_LINE_MAX_COUNTS 12000.0f
#define RING_YAW_START_DPS 12.0f

/* Historical ZORE constants kept for old fixtures/source comparison only.
 * v0.6 corner geometry does NOT use the ZORE/lookahead/gain/speed constants.
 * RING_360_YAW_STALE_MS below remains the active IMU freshness limit.
 * Current corner thresholds live in ring_corner.h / ring_corner.c. */
#define RING_360_VIEW_START_PERCENT 30U
#define RING_360_VIEW_END_PERCENT 68U
#define RING_360_IGNORE_TOP 10
#define RING_360_STEER_CENTER_PERCENT 50.0f
#define RING_360_SAFE_WIDTH 40
#define RING_360_ZORE_LEFT_LOSS 19
#define RING_360_ZORE_RIGHT_LOSS 18
#define RING_360_LEFT_OUTER_LOSS_MAX 2
#define RING_360_RIGHT_OUTER_LOSS_MAX 1
#define RING_360_OFFLINE_MAX 23
#define RING_360_BOTH_LOSS_MAX 2
#define RING_360_ZORE_RIGHT_VARIANCE 200
#define RING_360_ZORE_LEFT_VARIANCE 90
#define RING_360_BORDER_STEP 2.0f
#define RING_360_CORNER_TOP 25
#define RING_360_CORNER_JUMP 4
#define RING_360_LEFT_CORNER_GAP 1
#define RING_360_RIGHT_CORNER_GAP 3
#define RING_360_LEFT_HELP_LOSS 20
#define RING_360_RIGHT_HELP_LOSS 19
#define RING_360_ENTRY_LOSS_ROWS 12
#define RING_360_LEFT_RING5TO6 10
#define RING_360_RIGHT_RING5TO6 9
#define RING_360_LEFT_5TO6_OUTER_LOSS 11
#define RING_360_RIGHT_5TO6_OUTER_LOSS 10
#define RING_360_LEFT_6TO7_OUTER_LOSS 5
#define RING_360_RIGHT_6TO7_OUTER_LOSS 6
#define RING_360_EXIT_CORNER_ROW 28
#define RING_360_LEFT_EXIT_VARIANCE 512
#define RING_360_RIGHT_EXIT_VARIANCE 768
#define RING_360_LEFT_EXIT_LOSS_MAX 9
#define RING_360_RIGHT_EXIT_LOSS_MAX 11
#define RING_360_LEFT_EXIT_OFF_MAX 21
#define RING_360_RIGHT_EXIT_OFF_MAX 22
#define RING_360_LEFT_FINISH_LOSS 8
#define RING_360_RIGHT_FINISH_LOSS 5
#define RING_360_TIMEOUT_FRAMES 120U
#define RING_360_STAGE7_TIMEOUT_FRAMES 400U
#define RING_360_YAW_STALE_MS 100U
#define RING_360_HALF_WIDTH_SCALE 1.0f
#define RING_360_LOOKAHEAD_ENTRY 21
#define RING_360_LOOKAHEAD_IN 21
#define RING_360_LOOKAHEAD_STAGE6 26
#define RING_360_LOOKAHEAD_OUT 20
#define RING_360_LOOKAHEAD_FINISH 24
#define RING_360_LEFT_ENTRY_TRIM 2
#define RING_360_LEFT_REPAIR_COLUMN 25
#define RING_360_RIGHT_REPAIR_COLUMN 50
#define RING_360_KP_BASE 120
#define RING_360_GYRO_NORMAL 1.4f
#define RING_360_GYRO_INSIDE 1.3f
#define RING_360_SPEED_BASE 200.0f
#define RING_360_SPEED_DROP 30.0f
#define RING_360_TURN_SLOW_DIV 20.0f
#define RING_360_SPEED_ACC_STEP 20.0f
#define RING_360_SPEED_DEC_STEP 40.0f
#if RING_360_VIEW_START_PERCENT < RING_SEARCH_START_PERCENT || RING_360_VIEW_END_PERCENT > RING_SEARCH_END_PERCENT || RING_360_VIEW_START_PERCENT >= RING_360_VIEW_END_PERCENT
#error "360 view must stay inside the existing raw-boundary ROI; do not change ordinary vision to expand it"
#endif

/* Historical 270 exit tuning, inactive in the v0.6 corner event sequence. */
#define RING_270_EXIT_CONFIRM_FRAMES 3U
#define RING_270_EXIT_MIN_MS 300U
#define RING_270_EXIT_RESTORE_FRAMES 4U
#define RING_270_EXIT_CENTER 0.0f
#define RING_DONE_270 1U
#define RING_DONE_360 2U

/* Normalized x: 1000 = half image width; positive moves RIGHT.
 * Same near/far trim translates the line; their difference changes heading.
 * Entry uses a remembered road/outer-edge fit. Exit captures a NEW reference. */
#define RING_270_ENTRY_NEAR_OFFSET 0.0f
#define RING_270_ENTRY_FAR_OFFSET 0.0f
#define RING_270_EXIT_NEAR_OFFSET 0.0f
#define RING_270_EXIT_FAR_OFFSET 0.0f
#define RING_LINE_MAX_TILT 80.0f
#define RING_LINE_UPDATE_ALPHA 0.15f
#define RING_LINE_STEP_LIMIT 12.0f
#define RING_STRAIGHT_SPEED_SCALE 1.0f

/* Legacy phase values retained for diagnostics; corner_stage is the current
 * event stage. A phase alone does not imply that a virtual boundary is valid. */
typedef enum {
    RING_STATE_IDLE=0, RING_STATE_CANDIDATE, RING_STATE_APPROACH,
    RING_STATE_ENTERING, RING_STATE_INSIDE, RING_STATE_EXITING,
    RING_STATE_COOLDOWN, RING_STATE_FAULT, RING_STATE_EXIT_APPROACH
} ring_control_phase_t;

typedef struct {
    float x,y,travel,yaw;
    uint8_t side,kind,missing,passed;
} ring_corner_consumed_t;

typedef struct {
    uint8_t phase, kind, type_vote, type_frames;
    int8_t direction;
    uint8_t clear_frames, near_frames, exit_frames, miss_frames, missing_path_frames;
    uint8_t vision_candidate, track_valid, entry_seen, entry_passed, exit_seen;
    uint8_t revisit_armed, turn_seen, turn_frames, yaw_motion_ticks, path_mode, fault, have_frame, have_tick;
    int8_t yaw_vote;
    uint8_t line_active, line_locked, line_source, remembered_line, reset_tracking_history;
    uint8_t protection_enabled, zore360_confirmed;
    uint8_t vote360_history, vote360_misses;
    uint8_t action_locked, control_refresh, completed_mask, completed_270, completed_360;
    uint8_t zore360_stage,zore360_input_stage,zore360_last_stage;
    uint16_t zore360_timeout;
    int16_t zore360_centers[TRACK_VISION_MAX_ROWS],zore360_det,zore360_previous_error,zore360_off;
    uint16_t zore360_top,zore360_end,zore360_native_rows,zore360_native_cols;
    float zore360_center_x,zore360_error_scale,zore360_width_near;
    float zore360_speed,zore360_servo_offset;
    uint8_t have_yaw, imu_wait;
    uint8_t fault_resume_phase, fault_line_locked;
    float line_near, line_far, remembered_near, remembered_far;
    track_path_observation_t selected_path;
    track_path_observation_t last_turn_path;
    uint32_t phase_started_ms, event_started_ms, last_tick_ms, last_frame_ms;
    uint32_t cooldown_until_ms, line_started_ms, event_id;
    uint32_t last_yaw_ms;
    float yaw_progress_deg, yaw_rate_dps, travel_counts, line_started_counts;
    /* v0.6: quantities/sequence own the transaction; no ZORE stage is executed.
     * 360 stage: 1=A1, 2=B2 branch, 3=A3, 4=B4 main.
     * 270 stage: 1=entry extend, 2=entry connect, 3=inside,
     *            4=exit extend, 5=exit connect. */
    uint8_t corner_stage,corner_guide_id,corner_counts[2];
    uint8_t corner_guide_held,corner_have_guide,corner_have_consumed;
    uint8_t corner_vote_guide,corner_vote_frames,corner_missing,corner_passed;
    uint8_t corner_consumed_side,corner_consumed_kind;
    uint32_t corner_frame_number,corner_guide_frame_number,corner_last_guide_ms;
    float corner_x,corner_y,corner_vote_x,corner_vote_y;
    float corner_event_travel,corner_event_yaw;
    ring_corner_guide_t corner_selected_guide;
    ring_corner_consumed_t corner_consumed[4]; /* Prevent A1/B2 being reused as A3/B4 while still visible. */
} ring_control_state_t;

void ring_control_init(ring_control_state_t *state);
void ring_control_set_protection(ring_control_state_t *state,uint8_t enabled);
/* Compatibility symbol only. v0.6 uses the ordinary bounded PD for both rings;
 * this function intentionally leaves angle and speed unchanged. */
void ring_control_360_motion(ring_control_state_t *state,float center,float left,float right,
                             float base_speed,float *angle,float *speed);
void ring_control_tick(ring_control_state_t *state,uint8_t running,uint8_t yaw_fresh,
                       float yaw_rate_dps,float forward_counts,uint32_t now_ms);
void ring_control_observe(ring_control_state_t *state,const track_ring_observation_t *vision,
                          uint32_t frame_ms,uint8_t running);
track_path_observation_t ring_control_select_path(ring_control_state_t *state,
                                                 const track_ring_observation_t *vision);
float ring_control_line_x(const ring_control_state_t *state,float row_percent);
uint8_t ring_control_guidance_active(const ring_control_state_t *state);
#endif
