#include "single_axis_gyro.h"

#include <stddef.h>
#include <string.h>

#define GYRO_RX_HEADER             0x5AU
#define GYRO_FRAME_RATE            0xAAU
#define GYRO_FRAME_YAW             0xBBU
#define GYRO_TX_HEADER_0           0x55U
#define GYRO_TX_HEADER_1           0xAAU

#define GYRO_REG_SAVE              0x00U
#define GYRO_REG_OUTPUT_RATE       0x02U
#define GYRO_REG_CALIBRATION       0x0AU
#define GYRO_REG_UNLOCK            0x13U
#define GYRO_REG_YAW_ZERO          0x15U

#define GYRO_UNLOCK_VALUE          0x5F8EU
#define GYRO_CALIBRATE_BIAS_VALUE  0x0001U
#define GYRO_COMMAND_DELAY_MS      100U
#define GYRO_BIAS_WAIT_MS          21000U
#define GYRO_YAW_ZERO_TIMEOUT_MS   1000U
#define GYRO_YAW_ZERO_STABLE_COUNT 3U
#define GYRO_YAW_ZERO_MAX_ABS_RAW  365L

static bool DeadlineReached(uint32_t now_ms, uint32_t deadline_ms)
{
    return (int32_t) (now_ms - deadline_ms) >= 0;
}

static SingleAxisGyro_Status_t SendSettingAndSave(
    SingleAxisGyro_t *driver, uint8_t reg, uint16_t value)
{
    SingleAxisGyro_Status_t status;

    if ((driver == NULL) || (driver->io.delay_ms == NULL)) {
        return SINGLE_AXIS_GYRO_STATUS_INVALID_ARGUMENT;
    }

    status = SingleAxisGyro_Unlock(driver);
    if (status != SINGLE_AXIS_GYRO_STATUS_OK) {
        return status;
    }
    driver->io.delay_ms(driver->io.context, GYRO_COMMAND_DELAY_MS);

    status = SingleAxisGyro_WriteRegister(driver, reg, value);
    if (status != SINGLE_AXIS_GYRO_STATUS_OK) {
        return status;
    }
    driver->io.delay_ms(driver->io.context, GYRO_COMMAND_DELAY_MS);

    return SingleAxisGyro_Save(driver);
}

void SingleAxisGyro_Init(SingleAxisGyro_t *driver,
    const SingleAxisGyro_IO_t *io, float rate_full_scale_dps)
{
    if (driver == NULL) {
        return;
    }

    (void) memset(driver, 0, sizeof(*driver));
    if (io != NULL) {
        driver->io = *io;
    }
    driver->angular_rate_full_scale_dps =
        (rate_full_scale_dps > 0.0f) ? rate_full_scale_dps :
        SINGLE_AXIS_GYRO_DEFAULT_RATE_FULL_SCALE_DPS;
}

SingleAxisGyro_FrameResult_t SingleAxisGyro_FeedByte(
    SingleAxisGyro_t *driver, uint8_t byte)
{
    uint8_t checksum;
    uint8_t type;
    int16_t raw;

    if (driver == NULL) {
        return SINGLE_AXIS_GYRO_FRAME_NONE;
    }

    if (driver->frame_index == 0U) {
        if (byte == GYRO_RX_HEADER) {
            driver->frame[0] = byte;
            driver->frame_index = 1U;
        }
        return SINGLE_AXIS_GYRO_FRAME_NONE;
    }

    if (driver->frame_index == 1U) {
        if ((byte == GYRO_FRAME_RATE) || (byte == GYRO_FRAME_YAW)) {
            driver->frame[1] = byte;
            driver->frame_index = 2U;
        } else if (byte != GYRO_RX_HEADER) {
            driver->frame_index = 0U;
        }
        return SINGLE_AXIS_GYRO_FRAME_NONE;
    }

    driver->frame[driver->frame_index++] = byte;
    if (driver->frame_index < sizeof(driver->frame)) {
        return SINGLE_AXIS_GYRO_FRAME_NONE;
    }

    checksum = (uint8_t) (driver->frame[0] + driver->frame[1] +
                          driver->frame[2] + driver->frame[3]);
    if (checksum != driver->frame[4]) {
        driver->update_sequence++;
        driver->checksum_error_count++;
        driver->update_sequence++;
        driver->frame_index = (byte == GYRO_RX_HEADER) ? 1U : 0U;
        if (driver->frame_index == 1U) {
            driver->frame[0] = GYRO_RX_HEADER;
        }
        return SINGLE_AXIS_GYRO_FRAME_CHECKSUM_ERROR;
    }

    type = driver->frame[1];
    raw = (int16_t) (((uint16_t) driver->frame[3] << 8U) |
                     driver->frame[2]);

    driver->update_sequence++;
    if (type == GYRO_FRAME_RATE) {
        driver->raw_angular_rate = raw;
        driver->valid_mask |= SINGLE_AXIS_GYRO_VALID_RATE;
    } else {
        driver->raw_yaw = raw;
        driver->yaw_frame_count++;
        driver->valid_mask |= SINGLE_AXIS_GYRO_VALID_YAW;
    }
    driver->valid_frame_count++;
    driver->update_sequence++;
    driver->frame_index = 0U;

    return (type == GYRO_FRAME_RATE) ? SINGLE_AXIS_GYRO_FRAME_RATE :
                                      SINGLE_AXIS_GYRO_FRAME_YAW;
}

bool SingleAxisGyro_GetSample(const SingleAxisGyro_t *driver,
    SingleAxisGyro_Sample_t *sample)
{
    uint32_t sequence_before;
    uint32_t sequence_after;

    if ((driver == NULL) || (sample == NULL)) {
        return false;
    }

    for (;;) {
        sequence_before = driver->update_sequence;
        if ((sequence_before & 1U) != 0U) {
            continue;
        }

        sample->raw_angular_rate = driver->raw_angular_rate;
        sample->raw_yaw = driver->raw_yaw;
        sample->valid_frame_count = driver->valid_frame_count;
        sample->yaw_frame_count = driver->yaw_frame_count;
        sample->checksum_error_count = driver->checksum_error_count;
        sample->valid_mask = driver->valid_mask;
        sequence_after = driver->update_sequence;
        if ((sequence_before == sequence_after) &&
            ((sequence_after & 1U) == 0U)) {
            break;
        }
    }

    sample->angular_rate_dps =
        ((float) sample->raw_angular_rate / 32768.0f) *
        driver->angular_rate_full_scale_dps;
    sample->yaw_deg =
        ((float) sample->raw_yaw / 32768.0f) * 180.0f;

    return sample->valid_mask != 0U;
}

SingleAxisGyro_Status_t SingleAxisGyro_WriteRegister(
    SingleAxisGyro_t *driver, uint8_t reg, uint16_t value)
{
    uint8_t command[5];

    if ((driver == NULL) || (driver->io.write == NULL)) {
        return SINGLE_AXIS_GYRO_STATUS_INVALID_ARGUMENT;
    }

    command[0] = GYRO_TX_HEADER_0;
    command[1] = GYRO_TX_HEADER_1;
    command[2] = reg;
    command[3] = (uint8_t) value;
    command[4] = (uint8_t) (value >> 8U);

    return driver->io.write(driver->io.context, command, sizeof(command)) ?
        SINGLE_AXIS_GYRO_STATUS_OK : SINGLE_AXIS_GYRO_STATUS_IO_ERROR;
}

SingleAxisGyro_Status_t SingleAxisGyro_Unlock(SingleAxisGyro_t *driver)
{
    return SingleAxisGyro_WriteRegister(
        driver, GYRO_REG_UNLOCK, GYRO_UNLOCK_VALUE);
}

SingleAxisGyro_Status_t SingleAxisGyro_Save(SingleAxisGyro_t *driver)
{
    return SingleAxisGyro_WriteRegister(driver, GYRO_REG_SAVE, 0x0000U);
}

SingleAxisGyro_Status_t SingleAxisGyro_ZeroYaw(
    SingleAxisGyro_t *driver, bool save)
{
    SingleAxisGyro_Status_t status;

    if (!save) {
        if ((driver == NULL) || (driver->io.delay_ms == NULL)) {
            return SINGLE_AXIS_GYRO_STATUS_INVALID_ARGUMENT;
        }
        status = SingleAxisGyro_Unlock(driver);
        if (status != SINGLE_AXIS_GYRO_STATUS_OK) {
            return status;
        }
        driver->io.delay_ms(driver->io.context, GYRO_COMMAND_DELAY_MS);
        return SingleAxisGyro_WriteRegister(
            driver, GYRO_REG_YAW_ZERO, 0x0000U);
    }

    return SendSettingAndSave(driver, GYRO_REG_YAW_ZERO, 0x0000U);
}

bool SingleAxisGyro_StartYawZero(
    SingleAxisGyro_t *driver, uint32_t now_ms)
{
    SingleAxisGyro_Status_t status;

    if ((driver == NULL) || (driver->io.write == NULL) ||
        (driver->yaw_zero_state ==
         SINGLE_AXIS_GYRO_YAW_ZERO_WAIT_COMMAND_DELAY) ||
        (driver->yaw_zero_state ==
         SINGLE_AXIS_GYRO_YAW_ZERO_WAIT_RESULT_DELAY) ||
        (driver->yaw_zero_state ==
         SINGLE_AXIS_GYRO_YAW_ZERO_WAIT_VALID_SAMPLES)) {
        return false;
    }

    status = SingleAxisGyro_Unlock(driver);
    if (status != SINGLE_AXIS_GYRO_STATUS_OK) {
        driver->yaw_zero_state = SINGLE_AXIS_GYRO_YAW_ZERO_FAILED;
        return false;
    }

    driver->yaw_zero_deadline_ms = now_ms + GYRO_COMMAND_DELAY_MS;
    driver->yaw_zero_evaluated_frame_count = driver->yaw_frame_count;
    driver->yaw_zero_stable_sample_count = 0U;
    driver->yaw_zero_state =
        SINGLE_AXIS_GYRO_YAW_ZERO_WAIT_COMMAND_DELAY;
    return true;
}

SingleAxisGyro_YawZeroState_t SingleAxisGyro_ProcessYawZero(
    SingleAxisGyro_t *driver, uint32_t now_ms)
{
    if (driver == NULL) {
        return SINGLE_AXIS_GYRO_YAW_ZERO_FAILED;
    }

    if (driver->yaw_zero_state ==
        SINGLE_AXIS_GYRO_YAW_ZERO_WAIT_COMMAND_DELAY) {
        if (!DeadlineReached(now_ms, driver->yaw_zero_deadline_ms)) {
            return driver->yaw_zero_state;
        }
        if (SingleAxisGyro_WriteRegister(
                driver, GYRO_REG_YAW_ZERO, 0x0000U) !=
            SINGLE_AXIS_GYRO_STATUS_OK) {
            driver->yaw_zero_state =
                SINGLE_AXIS_GYRO_YAW_ZERO_FAILED;
            return driver->yaw_zero_state;
        }
        driver->yaw_zero_deadline_ms =
            now_ms + GYRO_COMMAND_DELAY_MS;
        driver->yaw_zero_state =
            SINGLE_AXIS_GYRO_YAW_ZERO_WAIT_RESULT_DELAY;
    }

    if (driver->yaw_zero_state ==
        SINGLE_AXIS_GYRO_YAW_ZERO_WAIT_RESULT_DELAY) {
        if (!DeadlineReached(now_ms, driver->yaw_zero_deadline_ms)) {
            return driver->yaw_zero_state;
        }
        driver->yaw_zero_evaluated_frame_count =
            driver->yaw_frame_count;
        driver->yaw_zero_stable_sample_count = 0U;
        driver->yaw_zero_deadline_ms =
            now_ms + GYRO_YAW_ZERO_TIMEOUT_MS;
        driver->yaw_zero_state =
            SINGLE_AXIS_GYRO_YAW_ZERO_WAIT_VALID_SAMPLES;
    }

    if (driver->yaw_zero_state ==
        SINGLE_AXIS_GYRO_YAW_ZERO_WAIT_VALID_SAMPLES) {
        if (driver->yaw_frame_count !=
            driver->yaw_zero_evaluated_frame_count) {
            int32_t raw_yaw = driver->raw_yaw;

            driver->yaw_zero_evaluated_frame_count =
                driver->yaw_frame_count;
            if ((raw_yaw >= -GYRO_YAW_ZERO_MAX_ABS_RAW) &&
                (raw_yaw <= GYRO_YAW_ZERO_MAX_ABS_RAW)) {
                driver->yaw_zero_stable_sample_count++;
                if (driver->yaw_zero_stable_sample_count >=
                    GYRO_YAW_ZERO_STABLE_COUNT) {
                    driver->yaw_zero_state =
                        SINGLE_AXIS_GYRO_YAW_ZERO_COMPLETE;
                    return driver->yaw_zero_state;
                }
            } else {
                driver->yaw_zero_stable_sample_count = 0U;
            }
        }
        if (DeadlineReached(now_ms, driver->yaw_zero_deadline_ms)) {
            driver->yaw_zero_state =
                SINGLE_AXIS_GYRO_YAW_ZERO_FAILED;
        }
    }

    return driver->yaw_zero_state;
}

SingleAxisGyro_YawZeroState_t SingleAxisGyro_GetYawZeroState(
    const SingleAxisGyro_t *driver)
{
    return (driver == NULL) ? SINGLE_AXIS_GYRO_YAW_ZERO_FAILED :
        driver->yaw_zero_state;
}

SingleAxisGyro_Status_t SingleAxisGyro_SetOutputRate(
    SingleAxisGyro_t *driver, SingleAxisGyro_OutputRate_t rate)
{
    if ((uint32_t) rate > (uint32_t) SINGLE_AXIS_GYRO_RATE_1000_HZ) {
        return SINGLE_AXIS_GYRO_STATUS_INVALID_ARGUMENT;
    }
    return SendSettingAndSave(
        driver, GYRO_REG_OUTPUT_RATE, (uint16_t) rate);
}

SingleAxisGyro_Status_t SingleAxisGyro_CalibrateBiasBlocking(
    SingleAxisGyro_t *driver)
{
    SingleAxisGyro_Status_t status;

    if ((driver == NULL) || (driver->io.delay_ms == NULL)) {
        return SINGLE_AXIS_GYRO_STATUS_INVALID_ARGUMENT;
    }

    status = SingleAxisGyro_Unlock(driver);
    if (status != SINGLE_AXIS_GYRO_STATUS_OK) {
        return status;
    }
    driver->io.delay_ms(driver->io.context, GYRO_COMMAND_DELAY_MS);

    status = SingleAxisGyro_WriteRegister(
        driver, GYRO_REG_CALIBRATION, GYRO_CALIBRATE_BIAS_VALUE);
    if (status != SINGLE_AXIS_GYRO_STATUS_OK) {
        return status;
    }
    driver->io.delay_ms(driver->io.context, GYRO_BIAS_WAIT_MS);

    return SingleAxisGyro_Save(driver);
}
