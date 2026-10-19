#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

// Only the portable decoding and unit-conversion API is exercised on the host.
#include "zf_device_imu660rb.h"

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); \
        exit(EXIT_FAILURE); \
    } \
} while (0)

static int near_float(float actual, float expected, float tolerance)
{
    return fabsf(actual - expected) <= tolerance;
}

int main(void)
{
    CHECK(imu660rb_decode_i16(0x00U, 0x00U) == 0);
    CHECK(imu660rb_decode_i16(0xFFU, 0x7FU) == INT16_MAX);
    CHECK(imu660rb_decode_i16(0x00U, 0x80U) == INT16_MIN);
    CHECK(imu660rb_decode_i16(0xFFU, 0xFFU) == -1);
    CHECK(imu660rb_decode_i16(0x34U, 0x12U) == 0x1234);

    CHECK(near_float(imu660rb_acc_to_g(4096), 0.999424f, 0.000001f));
    CHECK(near_float(imu660rb_acc_to_g(-4096), -0.999424f, 0.000001f));
    CHECK(near_float(imu660rb_gyro_to_dps(1000), 70.0f, 0.0001f));
    CHECK(near_float(imu660rb_gyro_to_dps(-1000), -70.0f, 0.0001f));

    puts("IMU660RB portable tests passed.");
    return EXIT_SUCCESS;
}
