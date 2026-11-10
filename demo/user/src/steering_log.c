#include "steering_log.h"
#include <stdio.h>
#ifndef STEERING_LOG_HOST_TEST
#include "zf_common_typedef.h"
AT_SDRAM_SECTION_ALIGN(static uint8_t log_storage[STEERING_LOG_BYTES], 32);
#else
static union { steering_log_record_t align; uint8_t bytes[STEERING_LOG_BYTES]; } host_storage;
#define log_storage host_storage.bytes
#endif
#include <string.h>
typedef char record_must_be_72_bytes[(sizeof(steering_log_record_t) == 72U) ? 1 : -1];
static steering_log_info_t info;
static uint32_t next, last_ms;
void steering_log_start(void) {
    ++info.session;
    if (!info.session) ++info.session;
    info.count = info.overwritten = next = 0U;
    info.active = 1U;
}
void steering_log_tick(const steering_log_record_t *r) {
    if (!info.active) return;
    /* Include first sample and terminal stop, even off the 20ms grid. */
    if (!info.count || !r->running || (uint32_t)(r->ms-last_ms) >= 20U) {
        memcpy(log_storage + next * sizeof(*r), r, sizeof(*r));
        next = (next + 1U) % STEERING_LOG_CAPACITY;
        if (info.count < STEERING_LOG_CAPACITY) ++info.count;
        else ++info.overwritten;
        last_ms = r->ms;
    }
    if (!r->running) info.active = 0U;
}
steering_log_info_t steering_log_info(void) { return info; }
int steering_log_read(uint32_t session, uint32_t index, steering_log_record_t *r) {
    if (info.active || session != info.session || index >= info.count) return 0;
    uint32_t oldest = (next + STEERING_LOG_CAPACITY - info.count) % STEERING_LOG_CAPACITY;
    memcpy(r, log_storage + ((oldest + index) % STEERING_LOG_CAPACITY) * sizeof(*r), sizeof(*r));
    return 1;
}
const char *steering_log_csv_header(void) {
    return "ms,frame,frame_age_ms,error,heading,near_error,near_heading,bottom_error,filtered_error,filtered_heading,servo10,target_left10,target_right10,speed_left,speed_right,pwm_left,pwm_right,running,ready,stop_reason,confidence,valid,ring_phase,ring_kind,protection,ring_stage,ring_guide,ring_source,ring_direction,bridge_seen,bridge_passed,exit_armed,transition_frames,imu_acc_forward,imu_acc_left,imu_acc_up,imu_gyro_x,imu_gyro_y,imu_gyro_z,imu_yaw_rate10,imu_fresh,imu_bias_ready\r\n";
}
int steering_log_csv(char *dst, size_t size, const steering_log_record_t *r) {
    return snprintf(dst,size,"%lu,%lu,%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%d,%u,%u,%u,%u,%d,%d,%d,%d,%d,%d,%d,%u,%u\r\n",
        (unsigned long)r->ms,(unsigned long)r->frame,(unsigned long)r->frame_age_ms,
        (int)r->error,(int)r->heading,(int)r->near_error,(int)r->near_heading,(int)r->bottom_error,
        (int)r->filtered_error,(int)r->filtered_heading,(int)r->servo10,
        (int)r->target_left10,(int)r->target_right10,(int)r->speed_left,(int)r->speed_right,
        (int)r->pwm_left,(int)r->pwm_right,(unsigned)r->running,(unsigned)r->ready,
        (unsigned)r->stop_reason,(unsigned)r->confidence,(unsigned)r->valid,
        (unsigned)r->ring_phase,(unsigned)r->ring_kind,(unsigned)r->protection,
        (unsigned)r->ring_stage,(unsigned)r->ring_guide,(unsigned)r->ring_source,(int)r->ring_direction,
        (unsigned)r->bridge_seen,(unsigned)r->bridge_passed,(unsigned)r->exit_armed,(unsigned)r->transition_frames,
        (int)r->imu_acc_forward,(int)r->imu_acc_left,(int)r->imu_acc_up,
        (int)r->imu_gyro_x,(int)r->imu_gyro_y,(int)r->imu_gyro_z,(int)r->imu_yaw_rate10,
        (unsigned)r->imu_fresh,(unsigned)r->imu_bias_ready);
}
