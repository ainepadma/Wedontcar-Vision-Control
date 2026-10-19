#include "imu_navigation.h"

#include <math.h>
#include <string.h>

static int16_t imu_negate_i16(int16_t value)
{
    return value == INT16_MIN ? INT16_MAX : (int16_t)-value;
}

void imu_navigation_init(imu_navigation_state_t *state, uint8_t device_ready)
{
    if (state == NULL)
        return;
    memset(state, 0, sizeof(*state));
    state->device_ready = device_ready != 0U;
}

void imu_navigation_observe(imu_navigation_state_t *state,
                            int16_t acc_x, int16_t acc_y, int16_t acc_z,
                            int16_t gyro_x, int16_t gyro_y, int16_t gyro_z,
                            uint8_t stationary)
{
    if (state == NULL || !state->device_ready)
        return;

    state->acc_forward = imu_negate_i16(acc_z);
    state->acc_left = acc_x;
    state->acc_up = imu_negate_i16(acc_y);
    state->gyro_x_raw = gyro_x;
    state->gyro_y_raw = gyro_y;
    state->gyro_z_raw = gyro_z;
    ++state->sample_count;

    if (!state->bias_ready)
    {
        if (!stationary)
        {
            state->bias_samples = 0U;
            state->bias_sum = 0;
            state->yaw_rate_dps = 0.0f;
            return;
        }
        state->bias_sum += gyro_y;
        if (state->bias_samples < IMU_NAVIGATION_CALIBRATION_SAMPLES)
            ++state->bias_samples;
        if (state->bias_samples >= IMU_NAVIGATION_CALIBRATION_SAMPLES)
        {
            const int64_t count = IMU_NAVIGATION_CALIBRATION_SAMPLES;
            const int64_t rounded = state->bias_sum >= 0
                ? state->bias_sum + count / 2 : state->bias_sum - count / 2;
            state->gyro_y_bias = (int16_t)(rounded / count);
            state->bias_ready = 1U;
        }
    }

    if (state->bias_ready)
    {
        float rate = -(float)(gyro_y - state->gyro_y_bias) *
                     IMU_NAVIGATION_DPS_PER_LSB;
        if (fabsf(rate) < IMU_NAVIGATION_RATE_DEADBAND_DPS)
            rate = 0.0f;
        state->yaw_rate_dps = rate;
    }
}
