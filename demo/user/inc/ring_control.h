#ifndef RING_CONTROL_H
#define RING_CONTROL_H
#include <stdint.h>
#include "track_vision.h"

#define RING_SEARCH_START_PERCENT VISION_SEARCH_START_PERCENT
#define RING_SEARCH_END_PERCENT VISION_SEARCH_END_PERCENT
#define RING_NEAR_START_PERCENT VISION_NEAR_START_PERCENT
#define RING_GUIDE_FAR_PERCENT VISION_TRACK_START_PERCENT
#define RING_GUIDE_NEAR_PERCENT VISION_SEARCH_END_PERCENT
#define RING_CONFIRM_FRAMES 3U
#define RING_CLEAR_FRAMES 3U
/* 360 exit handoff / re-arm. Normalized image errors, not physical angles.
 * Require current aligned road repeatedly; elapsed time alone never unlocks. */
#define RING_EXIT_ROAD_MIN_CONFIDENCE 65U
#define RING_EXIT_ROAD_MAX_ERROR 180
#define RING_EXIT_ROAD_MAX_HEADING 120
#define RING_EXIT_NEAR_SIGN_DEADBAND 40
#define RING_EXIT_BOTTOM_MAX_ERROR 80
#define RING_EXIT_SETTLE_FRAMES 8U
#define RING_EXIT_SETTLE_MS 200U
#define RING_360_REARM_FRAMES 10U
#define RING_360_REARM_MS 600U
#define RING_ACTIVE_TIMEOUT_MS 20000U
#define RING_VISION_STALE_MS 150U
#define RING_STRAIGHT_SPEED_SCALE 1.0f
#define RING_DONE_270 1U
#define RING_DONE_360 2U
typedef enum {
    RING_STATE_IDLE=0, RING_STATE_CANDIDATE, RING_STATE_APPROACH,
    RING_STATE_ENTERING, RING_STATE_INSIDE, RING_STATE_EXITING,
    RING_STATE_COOLDOWN, RING_STATE_FAULT, RING_STATE_EXIT_APPROACH
} ring_control_phase_t;

/* Actual control path, independent of the screen's frame/preview selection. */
typedef enum {
    RING_SOURCE_NORMAL=0, RING_SOURCE_GUIDE, RING_SOURCE_HOLD, RING_SOURCE_INVALID
} ring_path_source_t;

typedef struct {
    uint8_t phase,kind; int8_t direction;
    uint8_t action_locked,line_active,fault,protection_enabled;
    uint8_t control_refresh,reset_tracking_history,path_mode;
    uint8_t entry_seen,entry_passed,exit_seen,completed_mask,completed_270,completed_360;
    uint8_t corner_stage,corner_guide_id,corner_guide_held,corner_have_guide;
    uint8_t vote_frames,vote_kind,vote_side,clear_frames,transition_frames,exit_gap_seen;
    uint8_t have_frame,have_tick,fault_resume_phase;
    uint8_t bridge_seen,bridge_passed,exit_armed,bridge_clear_frames;
    uint8_t exit_road_frames;
    uint32_t exit_road_started_ms;
    uint32_t corner_frame_number,corner_guide_frame_number,corner_last_guide_ms;
    uint32_t last_frame_ms,last_tick_ms,event_started_ms,phase_started_ms,event_id;
    float yaw_progress_deg,travel_counts;
    ring_corner_guide_t corner_selected_guide;
    track_path_observation_t selected_path;
} ring_control_state_t;

void ring_control_init(ring_control_state_t *state);
void ring_control_set_protection(ring_control_state_t *state,uint8_t enabled);
/* Compatibility symbol only. Ring vision uses the ordinary bounded PD for both rings;
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
uint8_t ring_control_path_source(const ring_control_state_t *state);
const char *ring_control_stage_name(uint8_t stage);
#endif
