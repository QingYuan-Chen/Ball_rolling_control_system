/**
 * @file single_axis_gyro.h
 * @brief Portable UART protocol driver for the MCU single-axis gyro.
 */

#ifndef SINGLE_AXIS_GYRO_H_
#define SINGLE_AXIS_GYRO_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SINGLE_AXIS_GYRO_DEFAULT_RATE_FULL_SCALE_DPS 2000.0f
#define SINGLE_AXIS_GYRO_VALID_RATE                  0x01U
#define SINGLE_AXIS_GYRO_VALID_YAW                   0x02U

typedef enum {
    SINGLE_AXIS_GYRO_STATUS_OK = 0,
    SINGLE_AXIS_GYRO_STATUS_INVALID_ARGUMENT = -1,
    SINGLE_AXIS_GYRO_STATUS_IO_ERROR = -2
} SingleAxisGyro_Status_t;

typedef enum {
    SINGLE_AXIS_GYRO_FRAME_NONE = 0,
    SINGLE_AXIS_GYRO_FRAME_RATE,
    SINGLE_AXIS_GYRO_FRAME_YAW,
    SINGLE_AXIS_GYRO_FRAME_CHECKSUM_ERROR
} SingleAxisGyro_FrameResult_t;

typedef enum {
    SINGLE_AXIS_GYRO_RATE_0_1_HZ = 0x00,
    SINGLE_AXIS_GYRO_RATE_0_2_HZ = 0x01,
    SINGLE_AXIS_GYRO_RATE_0_5_HZ = 0x02,
    SINGLE_AXIS_GYRO_RATE_1_HZ = 0x03,
    SINGLE_AXIS_GYRO_RATE_2_HZ = 0x04,
    SINGLE_AXIS_GYRO_RATE_5_HZ = 0x05,
    SINGLE_AXIS_GYRO_RATE_10_HZ = 0x06,
    SINGLE_AXIS_GYRO_RATE_20_HZ = 0x07,
    SINGLE_AXIS_GYRO_RATE_50_HZ = 0x08,
    SINGLE_AXIS_GYRO_RATE_100_HZ = 0x09,
    SINGLE_AXIS_GYRO_RATE_125_HZ = 0x0A,
    SINGLE_AXIS_GYRO_RATE_200_HZ = 0x0B,
    SINGLE_AXIS_GYRO_RATE_250_HZ = 0x0C,
    SINGLE_AXIS_GYRO_RATE_500_HZ = 0x0D,
    SINGLE_AXIS_GYRO_RATE_1000_HZ = 0x0E
} SingleAxisGyro_OutputRate_t;

typedef bool (*SingleAxisGyro_Write_t)(void *context,
    const uint8_t *data, uint32_t length);
typedef void (*SingleAxisGyro_DelayMs_t)(void *context, uint32_t delay_ms);

typedef struct {
    SingleAxisGyro_Write_t write;
    SingleAxisGyro_DelayMs_t delay_ms;
    void *context;
} SingleAxisGyro_IO_t;

typedef struct {
    int16_t raw_angular_rate;
    int16_t raw_yaw;
    float angular_rate_dps;
    float yaw_deg;
    uint32_t valid_frame_count;
    uint32_t checksum_error_count;
    uint8_t valid_mask;
} SingleAxisGyro_Sample_t;

typedef struct {
    SingleAxisGyro_IO_t io;
    float angular_rate_full_scale_dps;
    uint8_t frame[5];
    uint8_t frame_index;
    volatile uint32_t update_sequence;
    volatile int16_t raw_angular_rate;
    volatile int16_t raw_yaw;
    volatile uint32_t valid_frame_count;
    volatile uint32_t checksum_error_count;
    volatile uint8_t valid_mask;
} SingleAxisGyro_t;

void SingleAxisGyro_Init(SingleAxisGyro_t *driver,
    const SingleAxisGyro_IO_t *io, float rate_full_scale_dps);

SingleAxisGyro_FrameResult_t SingleAxisGyro_FeedByte(
    SingleAxisGyro_t *driver, uint8_t byte);

bool SingleAxisGyro_GetSample(const SingleAxisGyro_t *driver,
    SingleAxisGyro_Sample_t *sample);

SingleAxisGyro_Status_t SingleAxisGyro_WriteRegister(
    SingleAxisGyro_t *driver, uint8_t reg, uint16_t value);

SingleAxisGyro_Status_t SingleAxisGyro_Unlock(SingleAxisGyro_t *driver);
SingleAxisGyro_Status_t SingleAxisGyro_Save(SingleAxisGyro_t *driver);
SingleAxisGyro_Status_t SingleAxisGyro_ZeroYaw(
    SingleAxisGyro_t *driver, bool save);
SingleAxisGyro_Status_t SingleAxisGyro_SetOutputRate(
    SingleAxisGyro_t *driver, SingleAxisGyro_OutputRate_t rate);
SingleAxisGyro_Status_t SingleAxisGyro_CalibrateBiasBlocking(
    SingleAxisGyro_t *driver);

#ifdef __cplusplus
}
#endif

#endif /* SINGLE_AXIS_GYRO_H_ */
