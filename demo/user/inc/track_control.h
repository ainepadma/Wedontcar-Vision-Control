#ifndef TRACK_CONTROL_H
#define TRACK_CONTROL_H

#include <stdint.h>

/* Encoder counts per 10-ms tick. The nominal m/s estimates below use the
 * hand-measured 2904.25 counts/revolution and a 64-mm wheel diameter. */
#define TRACK_SPEED_STRAIGHT 120.0f
/* LOW档弯道最低目标与无有效图像时的兜底目标。 */
#define TRACK_SPEED_CURVE_MIN 90.0f
/* FULL档：直道约1.75 m/s，弯道最低约1.00 m/s（架空轮速估算）。 */
#define TRACK_SPEED_FULL_STRAIGHT 253.0f
#define TRACK_SPEED_FULL_CURVE_MIN 145.0f

typedef enum
{
    TRACK_STOP_NONE = 0,
    TRACK_STOP_MANUAL,
    TRACK_STOP_NOT_READY,
    TRACK_STOP_LOST,
    TRACK_STOP_STALE,
    TRACK_STOP_ENCODER,
    TRACK_STOP_NO_FEEDBACK,
    TRACK_STOP_OVERSPEED,
    TRACK_STOP_RING,
    TRACK_STOP_STRIPE
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
    uint32_t last_frame_ms;
    uint32_t last_tick_ms;
    track_control_config_t config;
    float filtered_error;
    float filtered_heading;
    float filtered_near_heading;
    uint8_t near_geometry_valid;
    uint8_t bottom_active, bottom_armed, bottom_votes, bottom_center_votes, bottom_seen;
    int8_t bottom_sign;
    uint32_t bottom_start_ms, bottom_frame_ms;
    float bottom_pulse_deg;
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
/* Ordinary path only. Missing near geometry uses the legacy observation. */
void track_control_observe_near(track_control_state_t *state, int16_t error,
    int16_t heading, int16_t near_error, int16_t near_heading, uint8_t near_valid,
    uint8_t confidence, uint8_t valid, uint32_t now_ms);
/* Call after the accepted same-frame ordinary-path observation. */
void track_control_observe_bottom(track_control_state_t *state, int16_t error,
                                  uint8_t valid, uint32_t now_ms);
void track_control_toggle(track_control_state_t *state, uint32_t now_ms);
/* Image observations and steering always remain active. This is the track
 * portion of C14; encoder direction, no-feedback and overspeed stops remain
 * active independently of this switch. */
void track_control_set_loss_stop(track_control_state_t *state, uint8_t enabled);
void track_control_stop(track_control_state_t *state, uint8_t reason);
track_control_output_t track_control_tick(track_control_state_t *state,
                                         uint32_t now_ms);

#endif
