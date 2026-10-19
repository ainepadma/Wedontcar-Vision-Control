#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "imu_navigation.h"

#define CHECK(c) do { if (!(c)) { fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #c); exit(EXIT_FAILURE); } } while (0)

int main(void)
{
    imu_navigation_state_t state;
    imu_navigation_init(&state, 1U);
    CHECK(state.device_ready && !state.bias_ready);

    for (uint16_t i = 0; i < 50U; ++i)
        imu_navigation_observe(&state, 11, -22, -33, 1, -10, 3, 1U);
    imu_navigation_observe(&state, 11, -22, -33, 1, -10, 3, 0U);
    CHECK(state.bias_samples == 0U && !state.bias_ready);
    for (uint16_t i = 0; i < IMU_NAVIGATION_CALIBRATION_SAMPLES; ++i)
        imu_navigation_observe(&state, 11, -22, -33, 1, -10, 3, 1U);
    CHECK(state.bias_ready && state.gyro_y_bias == -10);
    CHECK(state.acc_forward == 33 && state.acc_left == 11 && state.acc_up == 22);

    // Sensor -GY means body-left yaw, and +GY means body-right yaw.
    imu_navigation_observe(&state, 0, 0, 0, 0, -1010, 0, 0U);
    CHECK(fabsf(state.yaw_rate_dps - 70.0f) < 0.01f);
    imu_navigation_observe(&state, 0, 0, 0, 0, 990, 0, 0U);
    CHECK(fabsf(state.yaw_rate_dps + 70.0f) < 0.01f);
    imu_navigation_observe(&state, 0, 0, 0, 0, -20, 0, 0U);
    CHECK(state.yaw_rate_dps == 0.0f);

    imu_navigation_observe(&state, 0, INT16_MIN, INT16_MIN, 0, -10, 0, 0U);
    CHECK(state.acc_forward == INT16_MAX && state.acc_up == INT16_MAX);
    puts("All IMU navigation host tests passed.");
    return EXIT_SUCCESS;
}
