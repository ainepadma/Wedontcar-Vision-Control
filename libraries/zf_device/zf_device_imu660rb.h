#ifndef _zf_device_imu660rb_h_
#define _zf_device_imu660rb_h_

#include <stdint.h>

// IMU660RB is based on ST LSM6DSR. These are the motherboard IMU socket pins.
#define IMU660RB_SPI_SPEED             (10U * 1000U * 1000U)
#define IMU660RB_SPI                   (SPI_4)
#define IMU660RB_SPC_PIN               (SPI4_SCK_C23)
#define IMU660RB_SDI_PIN               (SPI4_MOSI_C22)
#define IMU660RB_SDO_PIN               (SPI4_MISO_C21)
#define IMU660RB_CS_PIN                (C20)

#define IMU660RB_EXPECTED_ID           (0x6BU)
#define IMU660RB_ACC_G_PER_LSB         (0.000244f) // +/-8 g
#define IMU660RB_GYRO_DPS_PER_LSB      (0.070000f) // +/-2000 dps

typedef enum
{
    IMU660RB_RESULT_OK = 0,
    IMU660RB_RESULT_ERROR = 1,
    IMU660RB_RESULT_NOT_READY = 2
} imu660rb_result_t;

typedef struct
{
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
} imu660rb_sample_t;

static inline int16_t imu660rb_decode_i16(uint8_t low, uint8_t high)
{
    return (int16_t)((uint16_t)low | ((uint16_t)high << 8));
}

static inline float imu660rb_acc_to_g(int16_t raw)
{
    return (float)raw * IMU660RB_ACC_G_PER_LSB;
}

static inline float imu660rb_gyro_to_dps(int16_t raw)
{
    return (float)raw * IMU660RB_GYRO_DPS_PER_LSB;
}

// Initialization is bounded and non-fatal: an absent sensor returns ERROR.
imu660rb_result_t imu660rb_init(void);
// One status read plus one 12-byte burst; this function never waits for data.
imu660rb_result_t imu660rb_read_sample(imu660rb_sample_t *sample);
uint8_t imu660rb_last_identity(void);

#endif
