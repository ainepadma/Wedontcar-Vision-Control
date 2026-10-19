#ifndef TRACK_CONTROL_H
#define TRACK_CONTROL_H

#include <stdint.h>

/* Shared by firmware and host defaults; encoder counts per 10-ms tick.
 * First loaded-running tune, not a calibrated physical speed specification. */
#define TRACK_SPEED_STRAIGHT 120.0f
#define TRACK_SPEED_CURVE_MIN 80.0f
#define TRACK_SPEED_FULL_STRAIGHT 240.0f
#define TRACK_SPEED_FULL_CURVE_MIN 160.0f
#define TRACK_SPEED_FULL_SCALE (TRACK_SPEED_FULL_STRAIGHT / TRACK_SPEED_STRAIGHT)
/* Full steering applies +/-10% wheel-speed differential around the base target. */
#define TRACK_DIFF_GAIN 0.10f

typedef enum
{
    TRACK_STOP_NONE = 0,
    TRACK_STOP_MANUAL,
    TRACK_STOP_NOT_READY,
    TRACK_STOP_LOST,
    TRACK_STOP_STALE,
    TRACK_STOP_ENCODER,
    TRACK_STOP_NO_FEEDBACK,
    TRACK_STOP_OVERSPEED
} track_control_stop_reason_t;

typedef struct
{
    float servo_center;
    float servo_left;
    float servo_right;
    float speed_straight;
    float speed_curve_min;
    float kp;
    float kd;
    float max_servo_rate_deg_s;
} track_control_config_t;

typedef struct
{
    uint8_t running;
    uint8_t loss_stop_enabled;
    uint8_t stop_reason;
    uint8_t ready_frames;
    uint8_t has_frame;
    uint8_t frame_valid;
    uint8_t frame_stale;
    uint8_t lost_frames;
    uint8_t derivative_ready;
    uint8_t has_tick;
    int8_t turn_override; /* +1 force left, -1 force right, 0 use vision. */
    uint32_t last_frame_ms;
    uint32_t last_tick_ms;
    track_control_config_t config;
    float filtered_error;
    float filtered_heading;
    float derivative;
    float servo_angle;
} track_control_state_t;

typedef struct
{
    uint8_t running;
    uint8_t ready;
    uint8_t stop_reason;
    float servo_angle;
    float target_speed;
    float target_speed_left;
    float target_speed_right;
} track_control_output_t;

/* A single ISR owns the state. No function calls hardware or allocates memory.
 * error/heading are normalized to [-1000,1000], positive toward servo_right.
 * Three distinct valid observations (confidence >= 45) qualify a start.
 * While running, one or two consecutive invalid observations retain the last
 * steering command at curve-minimum speed; the third invalid frame stops when
 * loss-stop protection is enabled. Any valid frame clears this streak.
 * Frame timestamps must increase modulo uint32_t; duplicate valid timestamps
 * do not qualify another frame. Call tick every 10 ms, even while stopped.
 */
void track_control_init(track_control_state_t *state,
                        const track_control_config_t *config);
void track_control_observe(track_control_state_t *state, int16_t error,
                           int16_t heading, uint8_t confidence, uint8_t valid,
                           uint32_t now_ms);
void track_control_toggle(track_control_state_t *state, uint32_t now_ms);
/* Image observations and steering always remain active. This is the track
 * portion of the C14 master protection switch; the ISR applies the same switch
 * to encoder direction, no-feedback and overspeed protection. */
void track_control_set_loss_stop(track_control_state_t *state, uint8_t enabled);
/* Ring control may temporarily choose a calibrated steering direction.  The
 * normal steering endpoint and slew-rate limits still apply. */
void track_control_set_turn_override(track_control_state_t *state,
                                     int8_t direction);
void track_control_stop(track_control_state_t *state, uint8_t reason);
track_control_output_t track_control_tick(track_control_state_t *state,
                                         uint32_t now_ms);

#endif
