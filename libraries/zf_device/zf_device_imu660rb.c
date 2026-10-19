/*
 * IMU660RB (LSM6DSR) support for the SeekFree RT1064 library.
 * Register layout follows ST's LSM6DSR datasheet and the SeekFree RT1064
 * IMU660RB driver. Initialization and sampling are intentionally bounded so
 * an absent or miswired sensor cannot stop the existing vision controller.
 */

#include "zf_driver_delay.h"
#include "zf_driver_gpio.h"
#include "zf_driver_spi.h"
#include "zf_device_imu660rb.h"

#define IMU660RB_SPI_READ             (0x80U)
#define IMU660RB_REG_WHO_AM_I         (0x0FU)
#define IMU660RB_REG_CTRL1_XL         (0x10U)
#define IMU660RB_REG_CTRL2_G          (0x11U)
#define IMU660RB_REG_CTRL3_C          (0x12U)
#define IMU660RB_REG_CTRL4_C          (0x13U)
#define IMU660RB_REG_CTRL9_XL         (0x18U)
#define IMU660RB_REG_STATUS           (0x1EU)
#define IMU660RB_REG_OUTX_L_G         (0x22U)

#define IMU660RB_CTRL1_XL_208HZ_8G    (0x5CU)
#define IMU660RB_CTRL2_G_208HZ_2000   (0x5CU)
#define IMU660RB_CTRL3_BDU_IF_INC     (0x44U)
#define IMU660RB_CTRL4_LPF1_G         (0x02U)
#define IMU660RB_CTRL9_I3C_DISABLE    (0x02U)
#define IMU660RB_STATUS_XL_G_READY    (0x03U)

static uint8_t imu660rb_initialized;
static uint8_t imu660rb_identity;

static void imu660rb_select(uint8_t selected)
{
    if (selected)
        gpio_low(IMU660RB_CS_PIN);
    else
        gpio_high(IMU660RB_CS_PIN);
}

static void imu660rb_write_register(uint8_t reg, uint8_t value)
{
    imu660rb_select(1U);
    spi_write_8bit_register(IMU660RB_SPI, reg, value);
    imu660rb_select(0U);
}

static uint8_t imu660rb_read_register(uint8_t reg)
{
    uint8_t value;
    imu660rb_select(1U);
    value = spi_read_8bit_register(IMU660RB_SPI, reg | IMU660RB_SPI_READ);
    imu660rb_select(0U);
    return value;
}

static void imu660rb_read_registers(uint8_t reg, uint8_t *data, uint32_t length)
{
    imu660rb_select(1U);
    spi_read_8bit_registers(IMU660RB_SPI, reg | IMU660RB_SPI_READ, data, length);
    imu660rb_select(0U);
}

uint8_t imu660rb_last_identity(void)
{
    return imu660rb_identity;
}

imu660rb_result_t imu660rb_init(void)
{
    uint8_t reset_cleared = 0U;
    imu660rb_initialized = 0U;
    imu660rb_identity = 0U;

    system_delay_ms(10U);
    spi_init(IMU660RB_SPI, SPI_MODE0, IMU660RB_SPI_SPEED,
             IMU660RB_SPC_PIN, IMU660RB_SDI_PIN, IMU660RB_SDO_PIN,
             SPI_CS_NULL);
    gpio_init(IMU660RB_CS_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);

    // A bounded reset replaces the reference driver's multi-second loop.
    imu660rb_write_register(IMU660RB_REG_CTRL3_C, 0x01U);
    for (uint8_t attempt = 0U; attempt < 10U; ++attempt)
    {
        if (!(imu660rb_read_register(IMU660RB_REG_CTRL3_C) & 0x01U))
        {
            reset_cleared = 1U;
            break;
        }
        system_delay_ms(1U);
    }
    if (!reset_cleared)
        return IMU660RB_RESULT_ERROR;

    for (uint8_t attempt = 0U; attempt < 5U; ++attempt)
    {
        imu660rb_identity = imu660rb_read_register(IMU660RB_REG_WHO_AM_I);
        if (imu660rb_identity == IMU660RB_EXPECTED_ID)
            break;
        system_delay_ms(2U);
    }
    if (imu660rb_identity != IMU660RB_EXPECTED_ID)
        return IMU660RB_RESULT_ERROR;

    // BDU + auto increment make the 12-byte gyro/accelerometer burst coherent.
    imu660rb_write_register(IMU660RB_REG_CTRL3_C, IMU660RB_CTRL3_BDU_IF_INC);
    // ST defines I3C_disable as CTRL9_XL bit 1 (0x02), not bit 0.
    imu660rb_write_register(IMU660RB_REG_CTRL9_XL, IMU660RB_CTRL9_I3C_DISABLE);
    imu660rb_write_register(IMU660RB_REG_CTRL4_C, IMU660RB_CTRL4_LPF1_G);
    imu660rb_write_register(IMU660RB_REG_CTRL1_XL, IMU660RB_CTRL1_XL_208HZ_8G);
    imu660rb_write_register(IMU660RB_REG_CTRL2_G, IMU660RB_CTRL2_G_208HZ_2000);

    if (imu660rb_read_register(IMU660RB_REG_CTRL1_XL) != IMU660RB_CTRL1_XL_208HZ_8G ||
        imu660rb_read_register(IMU660RB_REG_CTRL2_G) != IMU660RB_CTRL2_G_208HZ_2000 ||
        imu660rb_read_register(IMU660RB_REG_CTRL3_C) != IMU660RB_CTRL3_BDU_IF_INC ||
        imu660rb_read_register(IMU660RB_REG_CTRL9_XL) != IMU660RB_CTRL9_I3C_DISABLE)
        return IMU660RB_RESULT_ERROR;

    imu660rb_initialized = 1U;
    return IMU660RB_RESULT_OK;
}

imu660rb_result_t imu660rb_read_sample(imu660rb_sample_t *sample)
{
    uint8_t data[12];
    if (!imu660rb_initialized || sample == 0)
        return IMU660RB_RESULT_ERROR;
    if ((imu660rb_read_register(IMU660RB_REG_STATUS) & IMU660RB_STATUS_XL_G_READY) !=
        IMU660RB_STATUS_XL_G_READY)
        return IMU660RB_RESULT_NOT_READY;

    imu660rb_read_registers(IMU660RB_REG_OUTX_L_G, data, sizeof(data));
    sample->gyro_x = imu660rb_decode_i16(data[0], data[1]);
    sample->gyro_y = imu660rb_decode_i16(data[2], data[3]);
    sample->gyro_z = imu660rb_decode_i16(data[4], data[5]);
    sample->acc_x = imu660rb_decode_i16(data[6], data[7]);
    sample->acc_y = imu660rb_decode_i16(data[8], data[9]);
    sample->acc_z = imu660rb_decode_i16(data[10], data[11]);
    return IMU660RB_RESULT_OK;
}
