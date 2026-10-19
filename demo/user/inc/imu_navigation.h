#ifndef IMU_NAVIGATION_H
#define IMU_NAVIGATION_H

#include <stdint.h>

#define IMU_NAVIGATION_CALIBRATION_SAMPLES 200U
#define IMU_NAVIGATION_DPS_PER_LSB 0.070000f
#define IMU_NAVIGATION_RATE_DEADBAND_DPS 1.5f

typedef struct
{
    uint8_t device_ready;
    uint8_t bias_ready;
    uint16_t bias_samples;
    int64_t bias_sum;
    int16_t gyro_y_bias;
    int16_t acc_forward;
    int16_t acc_left;
    int16_t acc_up;
    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;
    float yaw_rate_dps;
    uint32_t sample_count;
} imu_navigation_state_t;

void imu_navigation_init(imu_navigation_state_t *state, uint8_t device_ready);
/* Confirmed body mapping: forward=-sensor Z, left=+sensor X, up=-sensor Y.
 * Confirmed yaw convention: left turn is positive and equals -sensor GY. */
void imu_navigation_observe(imu_navigation_state_t *state,
                            int16_t acc_x, int16_t acc_y, int16_t acc_z,
                            int16_t gyro_x, int16_t gyro_y, int16_t gyro_z,
                            uint8_t stationary);

#endif
