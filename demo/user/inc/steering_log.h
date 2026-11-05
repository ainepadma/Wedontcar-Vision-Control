#ifndef STEERING_LOG_H
#define STEERING_LOG_H
#include <stdint.h>
#include <stddef.h>
#define STEERING_LOG_BYTES (1024U * 1024U)
#define STEERING_LOG_CAPACITY (STEERING_LOG_BYTES / 56U)
typedef struct {
    uint32_t ms, frame, frame_age_ms;
    int16_t error, heading, near_error, near_heading, bottom_error;
    int16_t filtered_error, filtered_heading, servo10;
    int16_t target_left10, target_right10, speed_left, speed_right;
    int16_t pwm_left, pwm_right;
    uint8_t running, ready, stop_reason, confidence, valid, ring_phase, ring_kind, protection;
    uint8_t ring_stage, ring_guide, ring_source;
    int8_t ring_direction;
    uint8_t bridge_seen, bridge_passed, exit_armed, transition_frames;
} steering_log_record_t;
typedef struct {
    uint32_t session, count, overwritten;
    uint8_t active;
} steering_log_info_t;
/* ISR owns start/tick; main must mask interrupts around info/read only.
 * No IO, allocation or formatting in the ISR. Start replaces the previous run. */
void steering_log_start(void);
void steering_log_tick(const steering_log_record_t *record);
steering_log_info_t steering_log_info(void);
int steering_log_read(uint32_t session, uint32_t index, steering_log_record_t *record);
const char *steering_log_csv_header(void);
int steering_log_csv(char *dst, size_t size, const steering_log_record_t *r);
#endif
