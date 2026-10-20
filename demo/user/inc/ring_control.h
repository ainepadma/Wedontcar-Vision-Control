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

/* ZORE 360 admission: units are its private 80x60 view, not source pixels.
 * The search is confined to the existing ring ROI; 270 thresholds stay above. */
#define RING_360_ZORE_LEFT_LOSS 12
#define RING_360_ZORE_RIGHT_LOSS 12
#define RING_360_ZORE_RIGHT_VARIANCE 800
#define RING_360_ZORE_LEFT_VARIANCE 800
#define RING_360_ZORE_NARROW_RATIO 0.75f
#define RING_360_OUTER_LOSS_MAX 6
#define RING_360_BOTH_LOSS_MAX 4
#define RING_360_OFFLINE_MAX 30
#define RING_360_CORNER_JUMP 2
#define RING_360_CORNER_TOP 20
#define RING_360_CORNER_GAP 0
#define RING_360_CONFIRM_WINDOW 5U
#define RING_360_CONFIRM_HITS 2U
#define RING_360_CANDIDATE_MISS_MAX 2U
#if RING_360_CONFIRM_WINDOW<1 || RING_360_CONFIRM_WINDOW>8 || RING_360_CONFIRM_HITS<1 || RING_360_CONFIRM_HITS>RING_360_CONFIRM_WINDOW
#error "360 confirmation requires 1 <= hits <= window <= 8"
#endif

/* 360 transaction: straight -> same-direction turn -> net 280deg -> straight.
 * Time in ms; mixed steering is error + 2*heading, not servo degrees. */
#define RING_360_ENTRY_MIN_MS 150U
#define RING_360_ENTRY_POSITION_MIN 65U
#define RING_360_TURN_TRIGGER_MIXED 120.0f
#define RING_360_TURN_CONFIRM_FRAMES 2U
#define RING_360_TURN_MIN_MIXED 180.0f
#define RING_360_TURN_MAX_MIXED 500.0f
#define RING_360_TURN_DEG 280.0f
#define RING_360_EXIT_MIN_MS 300U
#define RING_360_EXIT_RESTORE_HEADING 100
#define RING_360_EXIT_CENTER 0.0f
#define RING_360_IMU_WARN_MS 100U
#define RING_360_IMU_RETRY_MS 500U
#define RING_DONE_270 1U
#define RING_DONE_360 2U

/* Normalized x: 1000 = half image width; positive moves RIGHT.
 * Same near/far trim translates the line; their difference changes heading.
 * Entry uses a remembered road/outer-edge fit. Exit captures a NEW reference. */
#define RING_270_ENTRY_NEAR_OFFSET 0.0f
#define RING_270_ENTRY_FAR_OFFSET 0.0f
#define RING_270_EXIT_NEAR_OFFSET 0.0f
#define RING_270_EXIT_FAR_OFFSET 0.0f
#define RING_360_ENTRY_NEAR_OFFSET 0.0f
#define RING_360_ENTRY_FAR_OFFSET 0.0f
#define RING_360_EXIT_NEAR_OFFSET 0.0f
#define RING_360_EXIT_FAR_OFFSET 0.0f
#define RING_LINE_MAX_TILT 80.0f
#define RING_LINE_UPDATE_ALPHA 0.15f
#define RING_LINE_STEP_LIMIT 12.0f
#define RING_STRAIGHT_SPEED_SCALE 1.0f

/* Values 0..7 remain compatible with the existing TFT codes.
 * 2/8 = mouth ahead, normal/arc following; 3/5 = mouth reached, straight locked. */
typedef enum {
    RING_STATE_IDLE=0, RING_STATE_CANDIDATE, RING_STATE_APPROACH,
    RING_STATE_ENTERING, RING_STATE_INSIDE, RING_STATE_EXITING,
    RING_STATE_COOLDOWN, RING_STATE_FAULT, RING_STATE_EXIT_APPROACH
} ring_control_phase_t;

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
    uint8_t have_yaw, imu_wait;
    uint8_t fault_resume_phase, fault_line_locked;
    float line_near, line_far, remembered_near, remembered_far;
    track_path_observation_t selected_path;
    track_path_observation_t last_turn_path;
    uint32_t phase_started_ms, event_started_ms, last_tick_ms, last_frame_ms;
    uint32_t cooldown_until_ms, line_started_ms, event_id;
    uint32_t last_yaw_ms;
    float yaw_progress_deg, yaw_rate_dps, travel_counts, line_started_counts;
} ring_control_state_t;

void ring_control_init(ring_control_state_t *state);
void ring_control_set_protection(ring_control_state_t *state,uint8_t enabled);
/* Signed yaw: left positive. For locked360, missing IMU holds the turn and
 * freezes net angle; valid calibrated samples must reach TURN_DEG to exit. */
void ring_control_tick(ring_control_state_t *state,uint8_t running,uint8_t yaw_fresh,
                       float yaw_rate_dps,float forward_counts,uint32_t now_ms);
void ring_control_observe(ring_control_state_t *state,const track_ring_observation_t *vision,
                          uint32_t frame_ms,uint8_t running);
track_path_observation_t ring_control_select_path(ring_control_state_t *state,
                                                 const track_ring_observation_t *vision);
float ring_control_line_x(const ring_control_state_t *state,float row_percent);
uint8_t ring_control_guidance_active(const ring_control_state_t *state);
#endif
